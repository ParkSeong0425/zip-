#include "save.h"
#include "motor.h"
#include "rot_test.h"
#include "fram.h"
#include "tcp.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ===================== 모터 설정 ===================== */

#define ROT_RPM       1000   /* MKS first test: actual motor RPM */
#define Y_RPM         1000
#define MOVE_TIMEOUT  60000
#define Y_ARRIVE_GAP  100
#define ROT_ARRIVE_GAP 20
#define MOVE_WAIT     500

/* ===================== 기존 X/Y FRAM ===================== */

#define DATA_VER      0x0003
#define DATA_ADDR     0x0000

/* ===================== 회전축 FRAM ===================== */

#define ROT_ADDR      0x0200
#define ROT_MAGIC     0x54494C54UL
#define ROT_VERSION   3   /* MKS axis 단위로 변경 */

#define REV_MM        160
#define REV_CNT       131072
#define MM(v)         ((v) * REV_MM / REV_CNT)

/* ===================== 저장 구조체 ===================== */

typedef struct {
	int ver;

	int x[4];
	int y[4];

	int home_x;
	int home_y;

	int last_x;
	int last_y;

	int home_ok;
} Data;

typedef struct {
	uint32_t magic;
	int32_t distance;   /* C에서 R까지 거리 */
	uint16_t version;
	uint16_t sum;
} TiltSave;

/* ===================== 저장 변수 ===================== */

static Data data;
static TiltSave save;

static int save_ok = 0;

/* 회전축 위치
 *
 * C = 전원을 켰을 때 수직 위치
 * R = DI1 원점센서 위치
 * L = C 기준 R 반대편 위치
 */
static int c_pos = 0;
static int r_pos = 0;
static int l_pos = 0;

static int homed = 0;
static int y_homed = 0;
static int y_save_n = 0;

/* RUN 반복에 필요한 변수만 추가 */
static int run_on = 0;
static int run_n = 1;

/* ===================== FRAM ===================== */

static uint16_t save_sum(const TiltSave *p)
{
	const uint8_t *b = (const uint8_t *)p;
	uint16_t s = 0;

	for (uint16_t i = 0;
			i < (uint16_t)offsetof(TiltSave, sum);
			i++) {
		s += b[i];
	}

	return s;
}

static void rot_write(void)
{
	save.magic = ROT_MAGIC;
	save.version = ROT_VERSION;
	save.sum = save_sum(&save);

	fram_write(
			ROT_ADDR,
			&save,
			sizeof(save));
}

static void rot_read(void)
{
	fram_read(
			ROT_ADDR,
			&save,
			sizeof(save));

	save_ok =
			(save.magic == ROT_MAGIC) &&
			(save.version == ROT_VERSION) &&
			(save.distance != 0) &&
			(save.sum == save_sum(&save));

	if (!save_ok) {
		memset(
				&save,
				0,
				sizeof(save));
	}
}

static void data_write(void)
{
	fram_write(
			DATA_ADDR,
			&data,
			sizeof(data));
}

static void data_read(void)
{
	fram_read(
			DATA_ADDR,
			&data,
			sizeof(data));

	if (data.ver != DATA_VER) {
		memset(
				&data,
				0,
				sizeof(data));

		data.ver = DATA_VER;

		data_write();
	}
}

/* ===================== 모터 이동 ===================== */

/* Y 시흥모터 목표 위치 도착 대기 */
static int wait_y(int target)
{
	uint32_t t0 = HAL_GetTick();
	int now;

	while (HAL_GetTick() - t0 < MOVE_TIMEOUT) {
		if (motor_pos(&motorY, &now)) {
			int gap = now - target;

			if (gap < 0) {
				gap = -gap;
			}

			if (gap <= Y_ARRIVE_GAP) {
				return 1;
			}
		}

		HAL_Delay(50);
	}

	motor_stop(&motorY);
	return 0;
}

/* Y 시흥모터 절대 위치 이동 */
static int move_y(int rpm, int target)
{
	if (!motor_move(&motorY, rpm, target)) {
		return 0;
	}

	return wait_y(target);
}

/* MKS 회전축 목표 위치 도착 대기 */
static int wait_rot(int target)
{
	uint32_t t0 = HAL_GetTick();
	int now;

	while (HAL_GetTick() - t0 < MOVE_TIMEOUT) {
		if (mks_pos(&now)) {
			int gap = now - target;

			if (gap < 0) {
				gap = -gap;
			}

			if (gap <= ROT_ARRIVE_GAP) {
				return 1;
			}
		}

		HAL_Delay(50);
	}

	mks_stop();
	return 0;
}

/* MKS 회전축 절대 Axis 이동 */
static int move_rot(int rpm, int target)
{
	if (!mks_move_abs(rpm, target)) {
		return 0;
	}

	return wait_rot(target);
}

/* ===================== 회전축 홈 ===================== */

/* 전원 ON에서 MKS의 IN_1 원점센서를 찾는다.
 *
 * MKS GoHome은 센서 위치의 Axis를 0으로 만든다.
 * 그래서 C->R 실제 이동량은 zero 영향을 받지 않는 RAW encoder로 측정한다.
 *
 * 최초 1회:
 * 1. 전원을 켠 C 위치의 RAW를 읽음
 * 2. MKS GoHome으로 R 센서를 찾음
 * 3. R 위치의 RAW를 읽음
 * 4. distance = R_RAW - C_RAW를 FRAM에 저장
 *
 * 이후:
 * R = 0
 * C = -distance
 * L = -distance * 2
 */
static int rot_home(void)
{
	int raw_start = 0;
	int raw_hit = 0;

	homed = 0;

	/* 최초 부팅은 반드시 실제 C 위치에서 시작해야 한다. */
	if (!mks_raw(&raw_start) &&
		!mks_raw(&raw_start)) {
		return 0;
	}

	/* 센서가 이미 눌린 상태여도 MKS가 반대로 빠졌다가 다시 홈을 잡는다. */
	if (!mks_home()) {
		return 0;
	}

	if (!mks_raw(&raw_hit) &&
		!mks_raw(&raw_hit)) {
		return 0;
	}

	/* 홈 위치를 MKS Axis 0으로 확실하게 맞춘다. */
	if (!mks_zero()) {
		return 0;
	}

	r_pos = 0;

	if (!save_ok) {
		save.distance = raw_hit - raw_start;

		if (save.distance == 0) {
			print("ROT FIRST BOOT MUST START C\r\n");
			return 0;
		}

		rot_write();
		save_ok = 1;
	}

	c_pos = -save.distance;
	l_pos = c_pos - save.distance;
	homed = 1;

	return 1;
}

/* MKS 회전축 C 이동 후 도착까지 대기 */
static int rot_c_wait(void)
{
	if (!homed) {
		return 0;
	}

	return move_rot(ROT_RPM, c_pos);
}

/* ===================== Y축 홈 ===================== */

/* TCP home에서만 실행
 *
 * 1. MKS 회전축을 C로 이동
 * 2. MKS의 IN_1은 이 동작에서 사용하지 않음
 * 3. ID1 Y모터만 JOG+ 100rpm으로 이동
 * 4. ID1 Y모터의 DI1이 들어오면 정지
 * 5. 현재 위치를 Y=0으로 설정
 */
static int y_home(void)
{
	int now = 0;

	if (y_save_n != 0) {
		return 0;
	}

	/* 회전축은 C로만 이동 */
	if (!rot_c_wait()) {
		return 0;
	}

	y_homed = 0;

	int ok;

	/* 기능을 바꾸기 전에 DI3 가상입력 OFF */
	motor_write(
			&motorY,
			R_DI3L,
			0);

	HAL_Delay(20);

	/* HOME 동안만 DI3 = 19 */
	if (!motor_write(
			&motorY,
			R_DI3,
			19)) {
		return 0;
	}

	HAL_Delay(20);

	/* DI3L로 홈 실행 */
	ok = motor_home(
			&motorY,
			R_DI3L);

	/* 확실하게 JOG OFF */
	motor_write(
			&motorY,
			R_DI3L,
			0);

	HAL_Delay(20);

	/* HOME 완료 후 기본 기능 DI3 = 18 복원 */
	if (!motor_write(
			&motorY,
			R_DI3,
			18)) {
		return 0;
	}

	HAL_Delay(20);

	if (!ok) {
		return 0;
	}
	if (!motor_pos(&motorY, &now) &&
		!motor_pos(&motorY, &now)) {
		return 0;
	}

	/* 현재 센서 위치를 Y축 0으로 지정 */
	motorY.off -= now;

	y_homed = 1;

	data.home_y = 0;
	data.last_y = 0;
	data.home_ok |= 0x02;

	data_write();

	return 1;
}

/* ===================== Y1~Y4 저장 ===================== */

/* y1 save ~ y4 save
 *
 * 회전축을 C로 이동한 뒤
 * Y축을 JOG- 방향으로 이동시킨다.
 */
static int y_save_start(int n)
{
	if (!y_homed) {
		return 0;
	}

	if (y_save_n != 0) {
		return 0;
	}

	/* 회전축 C 위치 이동 */
	if (!rot_c_wait()) {
		return 0;
	}

	/* 기능 변경 전에 DI3L OFF */
	if (!motor_write(
			&motorY,
			R_DI3L,
			0)) {
		return 0;
	}

	HAL_Delay(20);

	/* Y SAVE 동안 DI3 = 18 */
	if (!motor_write(
			&motorY,
			R_DI3,
			18)) {
		return 0;
	}

	HAL_Delay(20);

	/* JOG 속도 100rpm */
	if (!motor_write(
			&motorY,
			JOG_SPEED,
			JOG_RPM)) {
		return 0;
	}

	HAL_Delay(20);

	/* Y 저장 방향 이동 시작 */
	if (!motor_write(
			&motorY,
			R_DI3L,
			1)) {
		return 0;
	}

	y_save_n = n;

	return 1;
}
/* s 명령
 *
 * Y축 JOG를 정지하고 현재 위치를 FRAM에 저장한다.
 */
static int y_save_stop(void)
{
	int now = 0;
	int n = y_save_n;

	if (n == 0) {
		return 0;
	}

	motor_write(
			&motorY,
			R_DI3L,
			0);

	HAL_Delay(100);

	motor_stop(&motorY);

	y_save_n = 0;

	if (!motor_pos(&motorY, &now) &&
		!motor_pos(&motorY, &now)) {
		return 0;
	}

	data.y[n - 1] = now;
	data.last_y = now;

	data_write();

	return 1;
}

/* ===================== GO ===================== */

/* 기존 go 함수의 순서 그대로 유지
 *
 * 1. 회전축 R 이동
 * 2. 500ms 대기
 * 3. 회전축 C 이동
 * 4. Y축 Yn 이동
 * 5. 500ms 대기
 * 6. 회전축 L 이동
 * 7. 500ms 대기
 * 8. 회전축 C 복귀
 */
static int go(int n)
{
	int yt;

	if (!homed) {
		print("GO FAIL HOMED\r\n");
		return 0;
	}

	if (!y_homed) {
		print("GO FAIL Y HOME\r\n");
		return 0;
	}

	if (y_save_n != 0) {
		print("GO FAIL Y SAVE\r\n");
		return 0;
	}

	if (n < 1 || n > 4) {
		print("GO FAIL NUMBER\r\n");
		return 0;
	}

	yt = data.y[n - 1];

	if (yt == 0) {
		print("GO FAIL Y POS\r\n");
		return 0;
	}

	/* 회전축 R */
	print("GO R\r\n");

	if (!move_rot(
			ROT_RPM,
			r_pos)) {

		print("GO FAIL R\r\n");
		return 0;
	}

	HAL_Delay(MOVE_WAIT);

	/* 회전축 C */
	print("GO C1\r\n");

	if (!rot_c_wait()) {
		print("GO FAIL C1\r\n");
		return 0;
	}

	/* Y축 Yn */
	print("GO Y\r\n");

	if (!move_y(
			Y_RPM,
			yt)) {

		print("GO FAIL Y\r\n");
		return 0;
	}

	data.last_y = yt;
	data_write();

	HAL_Delay(MOVE_WAIT);

	/* 회전축 L */
	print("GO L\r\n");

	if (!move_rot(
			ROT_RPM,
			l_pos)) {

		print("GO FAIL L\r\n");
		return 0;
	}

	HAL_Delay(MOVE_WAIT);

	/* 회전축 C 복귀 */
	print("GO C2\r\n");

	if (!rot_c_wait()) {
		print("GO FAIL C2\r\n");
		return 0;
	}

	print("GO OK\r\n");

	return 1;
}

/* ===================== 통신 유지 ===================== */

static void keepalive(void)
{
	static uint32_t last = 0;

	if (HAL_GetTick() - last < 1000) {
		return;
	}

	last = HAL_GetTick();

	mks_check();
	motor_check(&motorY);
}

/* ===================== 명령 문자열 ===================== */

/* 기존 reply 이름 유지 */
static void reply(const char *s)
{
	tcp_reply(s);
}

/* 명령 뒤쪽 공백과 개행 제거 */
static void trim(char *s)
{
	size_t n = strlen(s);

	while (n > 0) {
		char c = s[n - 1];

		if (c != '\r' &&
			c != '\n' &&
			c != ' ' &&
			c != '\t') {
			break;
		}

		s[--n] = '\0';
	}
}

/* 문자열 숫자 변환 */
static int parse_int(
		const char *s,
		int *out)
{
	char *end;
	long v;

	while (*s == ' ' ||
		   *s == '\t') {
		s++;
	}

	v = strtol(
			s,
			&end,
			10);

	if (end == s) {
		return 0;
	}

	while (*end == ' ' ||
		   *end == '\t') {
		end++;
	}

	if (*end != '\0') {
		return 0;
	}

	*out = (int)v;

	return 1;
}

/* Y 저장 위치 출력 */
static void show(void)
{
	char buf[128];

	sprintf(
			buf,
			"y %dmm %dmm %dmm %dmm\r\n",
			MM(data.y[0]),
			MM(data.y[1]),
			MM(data.y[2]),
			MM(data.y[3]));

	reply(buf);
}

/* ===================== 명령 처리 ===================== */

/* 기존 command 함수 이름과 구조 유지 */
static void command(char *cmd)
{
	int v;

	trim(cmd);
	/* RUN 중에는 stop만 허용 */
	if (run_on &&
		strcmp(cmd, "stop") != 0) {

		reply("ERR RUNNING\r\n");
		return;
	}

	/* 현재 상태 */
	if (strcmp(cmd, "status") == 0) {
		char buf[192];
		int rn = 0;
		int yn = 0;

		mks_pos(&rn);

		motor_pos(
				&motorY,
				&yn);

		sprintf(
				buf,
				"homed=%d y_homed=%d "
				"rot_saved=%d dist=%ld "
				"c=%ld r=%ld l=%ld "
				"rot=%d y=%d\r\n",
				homed,
				y_homed,
				save_ok,
				(long)save.distance,
				(long)c_pos,
				(long)r_pos,
				(long)l_pos,
				rn,
				yn);

		reply(buf);

		return;
	}

	/* Y 저장 위치 확인 */
	if (strcmp(cmd, "show") == 0) {
		show();

		return;
	}

	/* 회전축 C 위치 */
	if (strcmp(cmd, "c") == 0) {
		reply(
				homed &&
				mks_move_abs(
						ROT_RPM,
						c_pos) ?
				"OK C\r\n" :
				"ERR C\r\n");

		return;
	}

	/* 회전축 R 위치 */
	if (strcmp(cmd, "r") == 0) {
		reply(
				y_save_n == 0 &&
				homed &&
				mks_move_abs(
						ROT_RPM,
						r_pos) ?
				"OK R\r\n" :
				"ERR R\r\n");

		return;
	}

	/* 회전축 L 위치 */
	if (strcmp(cmd, "l") == 0) {
		reply(
				y_save_n == 0 &&
				homed &&
				mks_move_abs(
						ROT_RPM,
						l_pos) ?
				"OK L\r\n" :
				"ERR L\r\n");

		return;
	}

	/* TCP HOME
	 *
	 * rot_home()은 호출하지 않는다.
	 * 회전축은 C로만 이동하고
	 * Y축만 DI1 센서를 찾는다.
	 */
	if (strcmp(cmd, "home") == 0) {
		reply(
				y_home() ?
				"OK HOME\r\n" :
				"ERR HOME\r\n");

		return;
	}

	/* 기존 yhome 명령도 동일하게 유지 */
	if (strcmp(cmd, "yhome") == 0) {
		reply(
				y_home() ?
				"OK YHOME\r\n" :
				"ERR YHOME\r\n");

		return;
	}

	/* y1 save ~ y4 save */
	if ((sscanf(
			cmd,
			"y%d save",
			&v) == 1 ||

		 sscanf(
			cmd,
			"y%dsave",
			&v) == 1) &&

		v >= 1 &&
		v <= 4) {

		reply(
				y_save_start(v) ?
				"OK, PRESS S\r\n" :
				"ERR Y SAVE\r\n");

		return;
	}

	/* 저장 중인 Y 위치 정지 및 저장 */
	if (strcmp(cmd, "s") == 0) {
		reply(
				y_save_stop() ?
				"OK SAVE\r\n" :
				"ERR SAVE\r\n");

		return;
	}

	/* go 1 ~ go 4 한 번 동작 */
	if (strncmp(
			cmd,
			"go ",
			3) == 0 &&

		parse_int(
			cmd + 3,
			&v)) {

		reply(
				go(v) ?
				"OK GO\r\n" :
				"ERR GO\r\n");

		return;
	}

	/* Y1 → Y2 → Y3 → Y4 반복 시작 */
	if (strcmp(cmd, "run") == 0) {
		int ok =
				homed &&
				y_homed &&
				y_save_n == 0 &&
				data.y[0] != 0 &&
				data.y[1] != 0 &&
				data.y[2] != 0 &&
				data.y[3] != 0;

		if (ok) {
			run_n = 1;
			run_on = 1;
		}

		reply(
				ok ?
				"OK RUN\r\n" :
				"ERR RUN\r\n");

		return;
	}

	/* 두 모터 정지 및 RUN 종료 */
	if (strcmp(cmd, "stop") == 0) {
		run_on = 0;
		run_n = 1;

		if (y_save_n != 0) {
			y_save_n = 0;
		}

		mks_stop();
		motor_stop(&motorY);

		reply("OK STOP\r\n");

		return;
	}

	/* p1000, p-1000
	 *
	 * MKS 현재 위치 기준 상대 Axis 이동
	 * 16384 = 모터축 1회전
	 */
	if ((cmd[0] == 'p' ||
		 cmd[0] == 'P') &&

		parse_int(
			cmd + 1,
			&v)) {

		int ok =
				y_save_n == 0 &&
				homed &&
				mks_move_rel(
						ROT_RPM,
						v);

		reply(
				ok ?
				"OK PULSE\r\n" :
				"ERR PULSE\r\n");

		return;
	}

	reply("ERR CMD\r\n");
}

/* tcp.c가 호출하는 외부 함수 */
void save_cmd(char *cmd)
{
	command(cmd);
}

/* ===================== 초기화 ===================== */

void save_init(void)
{
	rot_read();
	data_read();

	print(
			"\r\n"
			"=== rot + y start ===\r\n");

	HAL_Delay(1000);

	/* MKS 회전축: UART5, 주소 2, FA/FB 프로토콜 (Mb_RTU=Disable) */
	if (!mks_init()) {
		print("MKS INIT FAIL\r\n");
	}

	HAL_Delay(200);

	/* Y 시흥모터: ZIP x_y의 UART5, ID1 설정 그대로 */
	motor_init(
			&motorY,
			1);

	HAL_Delay(200);

	/* 전원 ON에서는 MKS만 IN_1 원점센서를 찾음 */
	if (rot_home()) {
		/* R을 설정한 뒤 C로 복귀 */
		rot_c_wait();
	}

	print(
			"tcp cmd: home, c, r, l, "
			"y1~4 save, s, go 1~4, "
			"run, stop, show, status\r\n");
}

/* ===================== 반복 실행 ===================== */

void save_run(void)
{
	keepalive();

	if (!run_on) {
		return;
	}

	/* 기존 go()를 그대로 사용 */
	if (!go(run_n)) {
		run_on = 0;
		run_n = 1;

		return;
	}

	run_n++;

	if (run_n > 4) {
		run_n = 1;
	}
}

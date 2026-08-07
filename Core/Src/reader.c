///*
// * reader.c
// *
// *  Created on: Aug 7, 2026
// *      Author: HWNOT
// */
///**
//  ****************************************************************************************************
//  * @ File      reader.c
//  *
//  * @ Project   RSS_3DS_RFID
//  * @ Author    KOTECH, MR
//  ****************************************************************************************************
//  */
//
///* Includes ------------------------------------------------------------------*/
//#include <stdio.h>
//#include <string.h>
//#include <reader.h>
//#include "main.h"
//#include "can.h"
//#include "TRH033MS_ISO14443A.h"
//
///* Private variables ---------------------------------------------------------*/
//__IO s_RFIDR_Info RFIDR;
//
//
//
///* Private function prototypes -----------------------------------------------*/
//
//
///* Private functions ---------------------------------------------------------*/
//
//void RDIDR_Timer(void)
//{
//	//if(RFIDR.Time.UnDetect > 1) RFIDR.Time.UnDetect--;
//}
//
//void RFIDR_Init(void)
//{
//	RFIDR_Get_ID();
//	memset(RFIDR.UID, 0, sizeof(RFIDR.UID));
//
//	TRH033M_Init();
//}
//
//void RFIDR_Get_ID(void)
//{
//	uint8_t id;
//
//	id = HAL_GPIO_ReadPin(ID1_GPIO_Port, ID1_Pin);
//	id |= (HAL_GPIO_ReadPin(ID2_GPIO_Port, ID2_Pin)) << 1;
//	id |= (HAL_GPIO_ReadPin(ID3_GPIO_Port, ID3_Pin)) << 2;
//	id |= (HAL_GPIO_ReadPin(ID4_GPIO_Port, ID4_Pin)) << 3;
//
//	id = (~id)&0x0F;
//
//	if(id == 0)
//		RFIDR.ID = 10;
//	else
//		RFIDR.ID = id;
//}
//
//void RFIDR_Get_UID(void)
//{
//	// while(cmd_loop)
//	{
//		ISO14443A_Read();
//		TRH033M_Reset2Active();
//	}
//}
//
//void RFIDR_Tx_UID(void)
//{
//	uint8_t txdata[10];
//
//	txdata[0] = RFIDR.ID+CAN_RFID_ID_BASE;
//	//txdata[1] = NetCMD_Tx_Data;
//	txdata[1] = RFIDR.UID[0];
//	txdata[2] = RFIDR.UID[1];
//	txdata[3] = RFIDR.UID[2];
//	txdata[4] = RFIDR.UID[3];
//	CAN1_Tx_Data(txdata[0], &txdata[1], 4);
//}
//
//void RFIDR_Parser(void)
//{
//	uint8_t txdata[10];
//
//	if(CAN1_Info.RxData[0] == NetCMD_Rx_Check)
//	{
//		RFIDR_Tx_UID();
///*
//		txdata[0] = RFIDR.ID+CAN_RFID_ID_BASE;
//		txdata[1] = NetCMD_Tx_Data;
//		txdata[2] = RFIDR.UID[0];
//		txdata[3] = RFIDR.UID[1];
//		txdata[4] = RFIDR.UID[2];
//		txdata[5] = RFIDR.UID[3];
//		CAN1_Tx_Data(txdata[0], &txdata[1], 5);
//*/
//	}
///*
//	else
//	{
//		txdata[0] = RFIDR.ID;
//		txdata[1] = NetCMD_Tx_NAK;
//		CAN1_Tx_Data(txdata[0], &txdata[1], 1);
//	}
//*/
//}
//
////
//void RFIDR_Process(void)
//{
//	RFIDR_Get_ID();
//	RFIDR_Get_UID();
//
//	if(RFIDR.Status.New)
//	{
//		RFIDR.Status.New = 0;
//		RFIDR_Tx_UID();
//	}
//
//}
//
//
//
//

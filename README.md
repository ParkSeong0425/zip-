# TCP 코드 

while($true){$cmd=Read-Host "STM";if($cmd.Trim().ToLower() -eq "exit"){break};if([string]::IsNullOrWhiteSpace($cmd)){continue};$c=[Net.Sockets.TcpClient]::new();try{$c.Connect("172.20.0.101",2500);$s=$c.GetStream();$s.ReadTimeout=90000;$b=[Text.Encoding]::ASCII.GetBytes($cmd+"`r`n");$s.Write($b,0,$b.Length);$r=New-Object byte[] 1024;$n=$s.Read($r,0,$r.Length);if($n -gt 0){[Text.Encoding]::ASCII.GetString($r,0,$n).TrimEnd("`r","`n")}else{"NO RESPONSE"}}catch{"ERR: $($_.Exception.Message)"}finally{$c.Close()}}

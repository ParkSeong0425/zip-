# zip 코드 입니다 
##
 while($true){$cmd=Read-Host "STM";if($cmd.Trim().ToLower() -eq "exit"){exit};if([string]::IsNullOrWhiteSpace($cmd)){continue};$c=[Net.Sockets.TcpClient]::new();try{$c.Connect("172.20.0.101",2500);$s=$c.GetStream();$s.ReadTimeout=90000;$w=[IO.StreamWriter]::new($s,[Text.Encoding]::ASCII);$w.NewLine="`r`n";$w.AutoFlush=$true;$r=[IO.StreamReader]::new($s,[Text.Encoding]::ASCII);$w.WriteLine($cmd);$res=$r.ReadLine();Write-Host $res}catch{Write-Host "ERR: $($_.Exception.Message)"}finally{$c.Close()}}

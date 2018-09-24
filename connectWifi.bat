netsh wlan disconnect
timeout /t 1 /nobreak
netsh wlan refresh
timeout /t 1 /nobreak
netsh wlan show networks
netsh wlan connect name=%1
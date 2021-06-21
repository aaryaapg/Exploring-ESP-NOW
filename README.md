# Exploring-ESP-NOW
## ESP MAC Addresses
Code used to get ESP MAC Address:
```
#include "WiFi.h"
 
void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
}
 
void loop(){

}
```

Receiver/User Board: `3C:61:05:12:94:18`

Sender/Sensor Board: `AC:67:B2:3C:91:CC`
## References
1. https://learn.circuit.rocks/esp-now-the-fastest-esp8266-protocol
2. https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/
3. https://www.survivingwithandroid.com/esp-now-esp32-esp8266/
4. https://www.instructables.com/ESP32-With-ESP-Now-Protocol/

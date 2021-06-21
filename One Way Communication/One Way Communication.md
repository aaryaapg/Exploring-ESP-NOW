# ESP-NOW One-way Point to Point Communication
One ESP32 will be the “sender” and the other ESP32 will be the “receiver”. Refer: https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/

## Sender 
* DHT 11 Connected to pin 32 of ESP32
* This device sends temperature and humidity data to the receiver via ESP-NOW

## Receiver
* Receives temperature and humidity data from sender

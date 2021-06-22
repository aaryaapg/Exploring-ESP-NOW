# ESP-NOW Two-way Communication
We have two ESP32 boards. One board is connected to a push button (to pin 22 of ESP32) & the other board is connected to DHT11 (pin 32). Each board needs to know the other board's MAC address in order to communicate.

## Board 1: Button Board
* Push button is connected tp pin 22
* Receives temperature and humidity data from the sensor board when the push button is pressed

## Board 2: Sensor Board
* DHT 11 Connected to pin 32 of ESP32
* This board receives the button state from the button board
* When the button is pressed on the button board, it sends temperature and humidity data to it via ESP-NOW

## Serial Monitor
![2Way DHT11 ESP-NOW](https://user-images.githubusercontent.com/61982410/122963752-314a9200-d3a4-11eb-934a-1719c2177c1f.jpg)

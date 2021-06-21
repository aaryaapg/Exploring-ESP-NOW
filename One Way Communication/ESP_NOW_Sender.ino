/* ______________________________________Libraries______________________________________ */
//ESP-NOW and ESP
#include <esp_now.h>
#include <WiFi.h>
//DHT
#include <DHT.h>        // DHT11 temperature and humidity sensor Library

/* ______________________________________Macros______________________________________ */
//ESP-NOW
uint8_t broadcastAddress[] = {0x3C, 0x61, 0x05, 0x12, 0x94, 0x18}; //RECEIVER MAC Address
//DHT
#define DHTPIN 32      
#define DHTTYPE DHT11  // We are using DHT11. (Can use DHT22.. or any other from this series)

/* ______________________________________Declarations and Variables______________________________________ */
//DHT
DHT dht = DHT(DHTPIN, DHTTYPE);   // DHT dht = DHT(DHTPIN,Sensor_type), "dht" is the name of the sensor (variable)
//ESP-NOW
typedef struct struct_message { // Structure example to send data. Must match the receiver structure
  int h;
  int tc;
  float tf;
  /*
  char a[32];
  int b;
  float c;
  String d;
  bool e;
  */
} struct_message;
struct_message myData; // Create a struct_message called myData

/* ______________________________________Loop______________________________________ */ 
void setup() {
  Serial.begin(115200); //Init Serial Monitor 
  //ESP-NOW 
  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station  
  //ESP-NOW: Init ESP-NOW
  if (esp_now_init() != ESP_OK) { 
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  //ESP-NOW: Once ESPNow is successfully Init, we will register for Send CB to get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);  
  //ESP-NOW: Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;  
  //ESP-NOW: Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  //DHT
  dht.begin(); //Initialize the DHT sensor
}

/* ______________________________________Loop______________________________________ */
void loop() {
  // Set values to send
  myData.h = dht.readHumidity();
  myData.tc = dht.readTemperature();
  myData.tf = dht.readTemperature(true);   // Get Temperature in F
  //myData.hif = dht.computeHeatIndex(tf, h); // Heat index in F, Default
  //myData.hic = dht.computeHeatIndex(tc, h, false);  // Heat index in C
  
  /*
  //General Structure for String, int, float, String, boolean
  strcpy(myData.a, "THIS IS A CHAR");
  myData.b = random(1,20);
  myData.c = 1.2;
  myData.d = "Hello";
  myData.e = false;
  */
  
  //Send message via ESP-NOW every 2 seconds
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(2000);
}

/* ______________________________________Functions______________________________________ */
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

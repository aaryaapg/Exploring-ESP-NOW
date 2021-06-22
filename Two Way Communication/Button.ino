/* ______________________________________Libraries______________________________________ */
//ESP-NOW and ESP
#include <esp_now.h>
#include <WiFi.h>

/* ______________________________________Macros______________________________________ */
//ESP-NOW
uint8_t broadcastAddress[] = {0xAC, 0x67, 0xB2, 0x3C, 0x91, 0xCC}; 

/* ______________________________________Declarations and Variables______________________________________ */
String success;
int flag=0;
typedef struct userMessage { // Structure example to send data. Must match the receiver structure (Sensor Board). 
  int buttonState;
} userMessage;
userMessage myButtonData; // Create a struct_message called myButtonData

typedef struct SensorMessage { // Structure example to receive data. Must match the sender structure.
  int h;
  int tc;
  float tf;
} SensorMessage;
SensorMessage mySensorData; // Create a struct_message called myData
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
void loop() {
  
  // Set values to send
  //while(1){
  myButtonData.buttonState = digitalRead(22);
  //while(1){
  if(flag==LOW){
  if(myButtonData.buttonState==HIGH){
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myButtonData, sizeof(myButtonData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
    flag = 1;
  }
  else {
    Serial.println("Error sending the data");
  }
  //break;
  }
  else{
    return;
  }
  }
  //break;
  //}
}
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&mySensorData, incomingData, sizeof(mySensorData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Humidity (%): ");
  Serial.println(mySensorData.h);
  Serial.print("Temp in C: ");
  Serial.println(mySensorData.tc);
  Serial.print("Temp in F: ");
  Serial.println(mySensorData.tf);
  Serial.println();
}
/* ______________________________________Libraries______________________________________ */
//ESP-NOW and ESP
#include <esp_now.h>
#include <WiFi.h>
//DHT
#include <DHT.h> // DHT11 temperature and humidity sensor Library

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
typedef struct userMessage { // Structure example to receive data. Must match the sender structure (User Board). 
  int buttonState;
} userMessage;
userMessage myButtonData; // Create a struct_message called myButtonData
typedef struct SensorMessage { // Structure example to send data. Must match the receiver structure.
  int h;
  int tc;
  float tf;
} SensorMessage;
SensorMessage mySensorData; // Create a struct_message called myData
String success; // Variable to store if sending data was successful
//Button
int b;

/* ______________________________________Setup______________________________________ */
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  //DHT
  dht.begin(); //Initialize the DHT sensor
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

/* ______________________________________Loop______________________________________ */
void loop() {
  if(b==HIGH){ 
    // Set values to send if the button was high
    mySensorData.h =  dht.readHumidity();
    mySensorData.tc = dht.readTemperature();
    mySensorData.tf = dht.readTemperature(true);
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &mySensorData, sizeof(mySensorData));   
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
    b=0;
  }  
}

/* ______________________________________Functions______________________________________ */
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
  memcpy(&myButtonData, incomingData, sizeof(myButtonData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  b = myButtonData.buttonState;
}

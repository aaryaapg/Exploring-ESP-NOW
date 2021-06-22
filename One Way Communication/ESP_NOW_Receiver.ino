/* ______________________________________Libraries______________________________________ */
//ESP-NOW and ESP
#include <esp_now.h>
#include <WiFi.h>

/* ______________________________________Declarations and Variables______________________________________ */
typedef struct struct_message { // Structure example to receive data. Must match the sender structure
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

/* ______________________________________Setup______________________________________ */ 
void setup() {
  Serial.begin(115200); // Initialize Serial Monitor
  //ESP-NOW
  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station
  //ESP-NOW: Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }    
  esp_now_register_recv_cb(OnDataRecv); // Once ESPNow is successfully Init, we will register for recv CB to get recv packer info
}

/* ______________________________________Loop______________________________________ */ 
void loop() {

}

/* ______________________________________Functions______________________________________ */
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Humidity (%): ");
  Serial.println(myData.h);
  Serial.print("Temp in C: ");
  Serial.println(myData.tc);
  Serial.print("Temp in F: ");
  Serial.println(myData.tf);
  Serial.println();
  /*
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(myData.a);
  Serial.print("Int: ");
  Serial.println(myData.b);
  Serial.print("Float: ");
  Serial.println(myData.c);
  Serial.print("String: ");
  Serial.println(myData.d);
  Serial.print("Bool: ");
  Serial.println(myData.e);
  Serial.println();
  */
}

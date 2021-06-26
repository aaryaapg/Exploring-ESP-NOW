/* ______________________________________Libraries______________________________________ */
//ESP-NOW and ESP
#include <esp_now.h>
#include <WiFi.h>
//MPU6050
#include <Wire.h> //For communication between MPU6050 and ESP32 (I2C)
#include <Math.h>

/* ______________________________________Macros______________________________________ */
//ESP-NOW
uint8_t broadcastAddress[] = {0x3C, 0x61, 0x05, 0x12, 0x94, 0x18}; //RECEIVER MAC Address
//For MPU6050 - threshod for number of rotations
#define Upper_Threshold 0.85
//MPU6050 I2C Pins
#define SCL 18
#define SDA 19
//Define sensitivity scale factors for accelerometer and gyro based on the GYRO_CONFIG and ACCEL_CONFIG register values
#define aScaleFactor 16384
#define gScaleFactor 131

/* ______________________________________Declarations and Variables______________________________________ */

//ESP-NOW
typedef struct userMessage { // Structure example to receive data. Must match the sender structure (User Board). 
  int buttonState;
} userMessage;
userMessage myButtonData; // Create a struct_message called myButtonData
typedef struct SensorMessage { // Structure example to send data. Must match the receiver structure.
  int C;
} SensorMessage;
SensorMessage mySensorData; // Create a struct_message called myData
String success; // Variable to store if sending data was successful
//Button
int b;
//MPU6050: Since pin AD0 is Grounded, address of the device is b1101000 (0x68) [Check Sec 9.2 of Datasheet]
const uint8_t MPU6050SlaveAddress = 0x68;
//MPU6050: Few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B; //Register 59 (14 Registers (59 to 72) contain accel, temp and gyro data)
//Accelerometer and Gyroscope Variabels
int16_t AX_raw, AY_raw,AZ_raw, GX_raw, GY_raw, GZ_raw, Temp_raw;
double Ax, Ay, Az, T, Gx, Gy, Gz;
//Counter Variables for Accelerometer Rotations
int state; //State Variable
int c_state = 0; //current state
int p_state = 0; //previous state
int count = 0; //counting all edges
int realcount; //Count for just low to high edges

/* ______________________________________Setup______________________________________ */
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

  //Setup MPU6050
  Wire.begin(SDA, SCL);
  MPU6050_Init(); //Setup the MPU6050 registers
}

/* ______________________________________Loop______________________________________ */
void loop() {
  readSensors();
  if(b==HIGH){ 
    // Set values to send if the button was high    
    mySensorData.C = realcount;
    
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
//Read MPU6050 Data
void readSensors(){
  //Read sensors here
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  //Divide each with their sensitivity scale factor
  Ax = (double)AX_raw / aScaleFactor;
  Ay = (double)AY_raw / aScaleFactor;
  Az = (double)AZ_raw / aScaleFactor;
  T = (double)Temp_raw / 340 + 36.53; //temperature formula from datasheet
  Gx = (double)GX_raw / gScaleFactor;
  Gy = (double)GY_raw / gScaleFactor;
  Gz = (double)GZ_raw / gScaleFactor; 
  //Comparator
  if(Az > Upper_Threshold) {
    state = 1;
  }
  else {
    state = 0;
  }
  c_state = state;
  // compare the current state to its previous state
  if (c_state != p_state) {
      // if the state has changed, increment the counter
      count++;
      // Delay a little bit to avoid bouncing
  }
  else{
    count = count;
  }
  // save the current state as the last state, for next time through the loop
  p_state = c_state;
  realcount = count / 2;
  //Serial.print(" State: "); Serial.print(state);
  //Serial.print(" Count: "); Serial.println(realcount);
  delay(500);
}
//Configure and setup MPU6050 Registers
void MPU6050_Init() {
  delay(150);
  //Step 1: Set sample rate divider to get the desired sampling rate of 1kHz based on the formula given in datasheet
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  //Step 2: Set PLL with X axis gyroscope as the clock reference for improved stability.
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  //Step 3: This functionality is not required. Disable it.
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  //Step 4: Disable external Frame Synchronization and disable DLPF so that Gyroscope Output Rate = 8kHz
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  //Step 5: Set gyroscope full range to +- 250 dps, so that the gyroscope sensitivity scale factor is 131
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);
  //Step 6: Set accelerometer full range to +- 2g, so that the accelerometer sensitivity scale factor is 16384
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);
  //Step 7: This functionality is not required. Disable it.
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  //Step 8: Enable the Data Ready interrupt
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  //Step 9: Do not reset signal paths
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  //Step 10: The functionalities provided by the bits of this register are not requires now. Disable them
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}
//14 Registers (59 to 72) contain accel, temp and gyro data. We need to access it
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress) {
  Wire.beginTransmission(deviceAddress); //Get the slave's attention, tell it we're sending a command byte. Slave = MPU6050
  Wire.write(regAddress); //The command byte sets pointer to register whose address is given
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14); //Used by the master to request bytes from a slave device
  AX_raw = (((int16_t)Wire.read() << 8) | Wire.read());
  AY_raw = (((int16_t)Wire.read() << 8) | Wire.read());
  AZ_raw = (((int16_t)Wire.read() << 8) | Wire.read());
  Temp_raw = (((int16_t)Wire.read() << 8) | Wire.read());
  GX_raw = (((int16_t)Wire.read() << 8) | Wire.read());
  GY_raw = (((int16_t)Wire.read() << 8) | Wire.read());
  GZ_raw = (((int16_t)Wire.read() << 8) | Wire.read());
}
//A function that lets us write data to the slave's registers easily
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data) {
  Wire.beginTransmission(deviceAddress); //Get the slave's attention, tell it we're sending a command byte. Slave = MPU6050
  Wire.write(regAddress); //The command byte sets pointer to register whose address is given
  Wire.write(data); //Now that the pointer is ‘pointing’ at the specific register you wanted, this command will replace the byte stored in that register with the given data.
  Wire.endTransmission(); //This tells the slave that you’re done giving it instructions for now
}

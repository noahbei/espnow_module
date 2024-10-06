// sender -- module
// uint8_t slave1Address[] = {0xFC, 0xE8, 0xC0, 0x7C, 0xCC, 0x10}; // MAC address for slave1
// uint8_t slave2Address[] = {0xAC, 0x15, 0x18, 0xC0, 0x02, 0x8C}; // MAC address for slave2

 
// Include Libraries
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include "main.h"

// Define Servo Pins
const int influxServoPin = 14;  // Replace with your actual pin number
const int outfluxServoPin = 27; // Replace with your actual pin number

// Create Servo Objects
Servo influxServo;
Servo outfluxServo;
 
// flow rate sensor stuff
const int sensorInterrupt = 0;  // interrupt 0 (GPIO0)
const int sensorPin = 16;       // GPIO16 (can be modified to suit your setup)
const int solenoidValve = 5;    // GPIO5
const unsigned int SetPoint = 400; // 400 milliliters
/* The hall-effect flow sensor outputs pulses per second per litre/minute of flow. */
const float calibrationFactor = 90; // Change according to your datasheet
volatile byte pulseCount = 0;
// sensor data
float flowRate = 0.0;
unsigned int flowMilliLitres = 0;
unsigned long totalMilliLitres = 0;
unsigned long oldTime = 0;

 
// MAC Address of main esp32
uint8_t mainAddress[] = {0xCC, 0xDB, 0xA7, 0x2F, 0x49, 0x4C};

// Define a data structure -- maybe call this send data structure or module send data structure
// main 0, modules 1 - 5
#define MODULE 1
#if MODULE == 1
uint8_t module_number = 1; // FC:E8:C0:7C:CC:10
#else
uint8_t module_number = 2; // AC:15:18:C0:02:8C
#endif

typedef struct struct_module_message {
  // module number
  uint8_t module = module_number;

  // flow rate data
  float flowRate;
  unsigned int flowMilliLitres;
  unsigned long totalMilliLitres;

  // system status
  bool influxOpen; //-- maybe only have this in received data struct
  bool outfluxOpen;
} module_message;
module_message myModuleData;

typedef struct struct_main_message {
  // system status
  bool influxOpen;
  bool outfluxOpen;
} main_message;
main_message myMainData;

// Peer info
esp_now_peer_info_t peerInfo;
 
// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myMainData, incomingData, sizeof(myMainData));
  Serial.println("ON RECEIVE");
  // Print the MAC address of the sender
  Serial.print("MAC Address: ");
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i < 5) {
      Serial.print(":");
    }
  }
  Serial.println();

  Serial.println();
  Serial.print("Data received: ");
  Serial.println(len);
  Serial.print("Influx open: ");
  Serial.println(myMainData.influxOpen);
  Serial.print("Outflux open: ");
  Serial.println(myMainData.outfluxOpen);
  Serial.println();

  controlServos();
}
 
void setup() {
  
  // Set up Serial Monitor
  Serial.begin(115200);
 
  influxServo.attach(influxServoPin);
  outfluxServo.attach(outfluxServoPin);

  flowRateSensorSetup();
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
 
  // Register the callback functions
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, mainAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
 
  // load data from flow rate sensors here
  readFlowRateSensor();
  
  // Format structured data
  myModuleData.flowMilliLitres = flowMilliLitres;
  myModuleData.flowRate = flowRate;
  myModuleData.totalMilliLitres = totalMilliLitres;

  myModuleData.influxOpen = true;//isIfluxOpen();
  myModuleData.outfluxOpen = false;//isOutfluxOpen();
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(mainAddress, (uint8_t *) &myModuleData, sizeof(myModuleData));
   
  if (result == ESP_OK) {
    Serial.println("Sending confirmed");
  }
  else {
    Serial.println("Sending error");
  }
  delay(2000);
}

void readFlowRateSensor() {
  if ((millis() - oldTime) > 500) { // Only process counters twice per second
    // Disable the interrupt while calculating flow rate and sending the value to the host
    detachInterrupt(digitalPinToInterrupt(sensorPin));

    // Calculate flow rate and volume
    if (millis() != oldTime) {  // Prevent division by zero
      flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
    }

    oldTime = millis();

    flowMilliLitres = (flowRate / 60) * 1000;
    totalMilliLitres += flowMilliLitres;

    // Print the flow rate and total volume
    Serial.print("Flow rate: ");
    Serial.print(flowMilliLitres);
    Serial.print(" mL/Second\t");
    Serial.print("Output Liquid Quantity: ");
    Serial.print(totalMilliLitres);
    Serial.println(" mL");

    if (totalMilliLitres > SetPoint) {
      SetSolenoidValve();
    }

    // Reset the pulse counter
    pulseCount = 0;

    // Enable the interrupt again
    attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, FALLING);
  }
}

void controlServos() {
  // Control influx servo
  if (myMainData.influxOpen) {
    influxServo.write(0); // Open position
  } else {
    influxServo.write(90); // Closed position
  }

  // Control outflux servo
  if (myMainData.outfluxOpen) {
    outfluxServo.write(0); // Open position
  } else {
    outfluxServo.write(90); // Closed position
  }
}

void flowRateSensorSetup() {
  // Initialize a serial connection for reporting values to the host
  pinMode(solenoidValve, OUTPUT);
  digitalWrite(solenoidValve, HIGH);
  pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);

  /* The Hall-effect sensor is connected to GPIO16 using interrupt 0.
     Configured to trigger on a FALLING state change (transition from HIGH to LOW state) */
  attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, FALLING);
}

void pulseCounter() {
  // Increment the pulse counter
  pulseCount++;
}

void SetSolenoidValve() {
  digitalWrite(solenoidValve, LOW);
}
/*
 ESPNOW communication between a master(receivers) ESP8266 at the ground floor LED panel, with the other slave(senders) ESP8266's in the trash bins.
 This program setups the receiver ESP to listen for messages from the sender ESPs, and then act upon the message by lighting up LEDs.
 Here, when the trash bin threshold distance is exceeded at the sender, the receiver will light up its LED.
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    bool full;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Int: ");
  Serial.println(myData.full);
  Serial.println();
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
}

void loop() {
  if (myData.full) {
    digitalWrite(12, HIGH);
  }
  else{
    digitalWrite(12, LOW);
  }
}

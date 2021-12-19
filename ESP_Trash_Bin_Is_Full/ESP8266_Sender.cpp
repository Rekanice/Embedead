/*
 ESPNOW communication between a master(receivers) ESP8266 at the ground floor LED panel, with the other slave(senders) ESP32's in the trash bins.
 This program setups the sender ESP to listen to its ultrasonic distance sensors signals and send a boolean flag to indicate that the threshold distance is exceeded
 & that the trash bin is full. The sender ESPs, and then act upon the message by lighting up LEDs.
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

// - - - - - - - - - - - - - - - - - - -
// ESP NOW communication stuff

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xCC, 0x50, 0xE3, 0xDE, 0x11, 0xFE};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  bool full = false;
} struct_message;

// Create a struct_message called myData
struct_message myData;

unsigned long lastTime = 0;  
unsigned long timerDelay = 2000;  // send readings timer

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}

// - - - - - - - - - - - - - - - - - - -
// Ultrasonic sensor stuff

const int trigPin = 5;
const int echoPin = 4;

//define sound velocity in cm/uS
#define SOUND_SPEED 0.034
#define TRASHOLD 10

long duration;
float distanceCm;
bool binFull = false;

// - - - - - - - - - - - - - - - - - - -

void setup() {
  // Starts the serial communication
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, set this ESP as the sender (controller)
  // & register the function to run when it sent data (to get the status of Transmitted packet)
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);


  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

}

// - - - - - - - - - - - - - - - - - - - 

void loop() {

  // Emit ultrasonic wave
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Detect the echo when pulse is HIGH, and get the time lapsed
  duration = pulseIn(echoPin, HIGH);
  // Measure distance
  distanceCm = duration * SOUND_SPEED/2;
  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  if (int(distanceCm) <= TRASHOLD) {
    binFull = true;
  }
  else{
    binFull = false;
  }

  if ((millis() - lastTime) > timerDelay) {
    // Set values to send
    // strcpy(myData.a, "THIS IS A CHAR");
    myData.full = binFull;
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    lastTime = millis();
  }
}

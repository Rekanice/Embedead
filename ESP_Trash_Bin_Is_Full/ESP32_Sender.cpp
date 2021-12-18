#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// - - - - - - - - - - - - - - - - - - -
// ESP NOW communication stuff

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xAC, 0x0B, 0xFB, 0xC2, 0xB8, 0xF9};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  bool full = false;
} struct_message;

// Create a struct_message called myData
struct_message myData;

unsigned long lastTime = 0;  
unsigned long timerDelay = 5000;  // send readings timer

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t sendStatus) {
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

const int trigPin1 = 26;
const int echoPin1 = 13;
const int trigPin2 = 14;
const int echoPin2 = 21;
const int trigPin3 = 16;
const int echoPin3 = 17;
const int trigPin4 = 18;
const int echoPin4 = 19;

//define sound velocity in cm/uS
#define SOUND_SPEED 0.034
#define TRASHOLD 10

long duration;
bool binFull = false;


uint getDistance(uint8_t trigPin, uint8_t echoPin){
  // Emit ultrasonic wave
  // 1. Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // 2. Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 3. Detect the echo when pulse is HIGH, and get the time lapsed
  duration = pulseIn(echoPin, HIGH);
  
  // 4. Measure distance
  uint distanceCm = duration * SOUND_SPEED/2;

  if(distanceCm > 1)
  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  return distanceCm;
}

bool isBinFull(){

  uint totalDistance = 
    getDistance(trigPin1,echoPin1) + 
    getDistance(trigPin2,echoPin2) + 
    getDistance(trigPin3,echoPin3) + 
    getDistance(trigPin4,echoPin4);
  
  uint avgDistance = totalDistance / 4;
  
  if (avgDistance <= TRASHOLD){
    binFull = true;
  }else{
    binFull = false;
  }

  return binFull;
}


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

  // Set the ultrasonic sensor signal send & signal receive pins
  pinMode(trigPin1, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin1,  INPUT);  // Sets the echoPin as an Input
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2,  INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3,  INPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4,  INPUT);
  
}

// - - - - - - - - - - - - - - - - - - - 

void loop() {

  // Set values to send
  myData.full = isBinFull();

  // Send message via ESP-NOW
  esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  delay(5000);
}

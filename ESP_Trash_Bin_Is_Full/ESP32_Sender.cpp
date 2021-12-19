#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// - - - - - - - - - - - - - - - - - - -
// ESP-NOW communication stuff

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xAC, 0x0B, 0xFB, 0xC2, 0xB8, 0xF9};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  char floor;
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
    Serial.println("Delivery success \n\n");
  }
  else{
    Serial.println("Delivery fail \n\n");
  }
}

// - - - - - - - - - - - - - - - - - - -
// Ultrasonic sensor stuff

const int trigPin1 = 18;
const int echoPin1 = 17;
const int trigPin2 = 26;
const int echoPin2 = 25;
const int trigPin3 = 13;
const int echoPin3 = 27;
const int trigPin4 =  4;
const int echoPin4 = 16;

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
  Serial.print("Avg Distance: ");
  Serial.print(avgDistance);
  Serial.println(" cm");

  if (avgDistance <= TRASHOLD){
    binFull = true;
  }else{
    binFull = false;
  }

  return binFull;
}


// - - - - - - - - - - - - - - - - - - -
// DIP Switch stuff

#define dip1 23 
#define dip2 22
#define dip3 21
#define dip4 19

char whatFloor(){
  // Read the dip switch pin values as a binary value, for better targeting the one-hot encoding states.
  // How it works: Bitwise OR (Add) the pin values after left-shifting the pin value to its appropriate binary place value.
  byte dipByte = (
    digitalRead(dip1) | 
    (digitalRead(dip2) << 1) |
    (digitalRead(dip3) << 2) |
    (digitalRead(dip4) << 3) 
  );
  
  Serial.print("Dip Byte: ");
  Serial.print(dipByte);
  Serial.print("\t");

  char floorNum = 0;

  switch(int(dipByte)){
    case 0 : Serial.println("GRD Floor is ON."); floorNum=0; break;
    case 1 : Serial.println("Floor 1 is ON.");   floorNum=1; break;
    case 2 : Serial.println("Floor 2 is ON.");   floorNum=2; break;
    case 4 : Serial.println("Floor 3 is ON.");   floorNum=3; break;
    case 8 : Serial.println("Floor 4 is ON.");   floorNum=4; break;
    default: Serial.println("Invalid floor. Not one-hot encoding binary. OR switch bounce occurred (just wait awhile)."); break;
  }

  return floorNum;
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

  // Set internal pullup resistors for the input pins reading the dip switch pin
  pinMode(dip1, INPUT_PULLUP);
  pinMode(dip2, INPUT_PULLUP);
  pinMode(dip3, INPUT_PULLUP);
  pinMode(dip4, INPUT_PULLUP);
  
}

// - - - - - - - - - - - - - - - - - - - 

void loop() {

  // Set values to send
  myData.full = isBinFull();
  myData.floor = whatFloor();

  // Send message via ESP-NOW
  esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  delay(5000);
}

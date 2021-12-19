#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>



// - - - - - - - - - ESP-NOW communication stuff - - - - - - - - - -

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xAC, 0x0B, 0xFB, 0xC2, 0xB8, 0xF9};

// Structure example to send data. Must match the receiver structure
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

void initEspNow() {
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
}



// - - - - - - - - - Ultrasonic sensor stuff - - - - - - - - - -

const int trigPin1 = 18;
const int echoPin1 = 17;
const int trigPin2 = 26;
const int echoPin2 = 25;
const int trigPin3 = 13;
const int echoPin3 = 27;
const int trigPin4 = 16;
const int echoPin4 = 4;

#define SOUND_SPEED 0.034   // cm/uS
#define TRASHOLD 10         // cm
#define MAX_BIN_DEPTH 50    // cm

long duration;
bool binFull = false;

void initUltrasonicSensors() {
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

  // Set a max bin depth & check if the distance measured by an ultrasonic sensor exceeds this depth. 
  // This means the measured distance is not valid & the extreme distance is probably due to uneevn reflected surface of the trash, 
  // causing the echo to travel a longer distance.
  char validCount = 0;  // Only average the valid distances.
  uint totalDistance = 0;

  uint dist1 = getDistance(trigPin1,echoPin1);
  if (dist1 <= MAX_BIN_DEPTH) {
    ++validCount;
    totalDistance += dist1;
  }
  uint dist2 = getDistance(trigPin2,echoPin2);
  if (dist2 <= MAX_BIN_DEPTH) {
    ++validCount;
    totalDistance += dist2;
  }
  uint dist3 = getDistance(trigPin3,echoPin3);
  if (dist3 <= MAX_BIN_DEPTH) {
    ++validCount;
    totalDistance += dist3;
  }
  uint dist4 = getDistance(trigPin4,echoPin4);
  if (dist4 <= MAX_BIN_DEPTH) {
    ++validCount;
    totalDistance += dist4;
  }
  
  uint avgDistance = totalDistance / validCount;
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



// - - - - - - - - - DIP Switch stuff - - - - - - - - - -

#define dip1 23 
#define dip2 22
#define dip3 21
#define dip4 19

void initDipSwitch() {
  // Set internal pullup resistors for the input pins reading the dip switch pin
  pinMode(dip1, INPUT_PULLUP);
  pinMode(dip2, INPUT_PULLUP);
  pinMode(dip3, INPUT_PULLUP);
  pinMode(dip4, INPUT_PULLUP);
}

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



// - - - - - - - - Deep Sleep Stuff - - - - - - - - - - -

#define S_TO_uS_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

// Save Data on RTC Memories
RTC_DATA_ATTR int wakeCount = 0;

void initDeepSleep() {
  //Increment wake number and print it every reboot
  ++wakeCount;
  Serial.println("WakeUp number: " + String(wakeCount));

  // Configure the wake up source. Set our ESP32 to wake up every 5 seconds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * S_TO_uS_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
}

void goToSleep() {

  Serial.println("Going to sleep now");
  delay(1000);
  Serial.flush(); 
  
  // esp goes to sleep
  esp_deep_sleep_start();
}



// - - - - - - - - The routine task done by the sytem when awake - - - - - - - - - - -
void mainRoutine() {

  // Set values to send
  myData.full = isBinFull();
  myData.floor = whatFloor();

  // Send message via ESP-NOW
  esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

}

// - - - - - - - - - - - - - - - - - - -
void setup() {
  // Starts the serial communication
  Serial.begin(115200);
 
  initEspNow();
  initUltrasonicSensors();
  initDipSwitch();
  initDeepSleep();

  mainRoutine();

  goToSleep();
  
}

void loop() {
}

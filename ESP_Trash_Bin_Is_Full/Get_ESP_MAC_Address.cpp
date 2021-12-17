/*
Complete Instructions to Get and Change ESP MAC Address: https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/

STEPS:
1. Build, compile, & upload code to ESP
2. Open Serial Monitor
3. Press RST button on ESP, MAC Address should pop up
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>

void setup(){
  Serial.begin(115200);
  Serial.println();
  Serial.print("ESP8266 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
}
 
void loop(){

}

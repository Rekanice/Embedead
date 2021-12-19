#include <Arduino.h>

#define dip1 23 
#define dip2 22
#define dip3 27
#define dip4 25

void setup(){

  Serial.begin(115200);
  // Set internal pullup resistors for the input ppins reading the dip switch pin
  pinMode(dip1, INPUT_PULLUP);
  pinMode(dip2, INPUT_PULLUP);
  pinMode(dip3, INPUT_PULLUP);
  pinMode(dip4, INPUT_PULLUP);
  
}

void loop(){
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

  switch(int(dipByte)){
    case 1 : Serial.println("Floor 1 is ON."); break;
    case 2 : Serial.println("Floor 2 is ON."); break;
    case 4 : Serial.println("Floor 3 is ON."); break;
    case 8 : Serial.println("Floor 4 is ON."); break;
    case 0 : Serial.println("GRD Floor is ON."); break;
    default: Serial.println("Invalid floor. Not one-hot encoding binary. OR switch bounce occurred (just wait awhile)."); break;
  }

  delay(3000);
}

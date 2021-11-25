#include <Arduino.h>

#define ENCA 2 
#define ENCB 3

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int a = digitalRead(ENCA);
  int b = digitalRead(ENCB);

  Serial.print(a*5);
  Serial.print(" ");
  Serial.print(b*5);
  Serial.println();
}
#include <Arduino.h>

// Pins used on Arduino (Name is defined by what they are connected to)
#define ENCA 2 
#define ENCB 3
#define M1A 5
#define M1B 6

// globals
long currT = 0;
long prevT = 0;
float deltaT = 0;

// Use the "volatile" directive for variables used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;
volatile long currT_i = 0;
volatile float deltaT_i = 0;

float vFilt = 0;
float vPrev = 0;

float eintegral = 0;

// custom functions' declarations
void setMotor(int pwm, int ma, int mb);
void readEncoder();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(M1A,OUTPUT);
  pinMode(M1B,OUTPUT);

  digitalWrite(M1A, LOW);
  digitalWrite(M1B, LOW);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

}


void loop() {
  // put your main code here, to run repeatedly:
  
  // Read the position and velocity
  // int pos = 0;
  float velocity = 0;
  noInterrupts();   // disable interrupts temporarily while reading
  // pos = pos_i;
  velocity = velocity_i;
  currT = currT_i;
  deltaT = deltaT_i;
  interrupts();     // turn interrupts back on

  // Convert velocity from count/s to RPM
  float v = velocity / 210.0 * 60.0 ;   // X1 counting & Gear ratio (30:1)

  // Low pass filter (cutoff freq = 25Hz)
  vFilt = 0.854*vFilt + 0.0728*v + 0.0728*vPrev;
  vPrev = v;

  // PID speed control part
  // Set a target velocity
  float vt = 255 * (sin(currT/1e6) > 0);    // Note the boolean condition to ensure it's zero at start time

  // Compute the control signal u
  float kp = 5;
  float ki = 10;
  float e = vt - vFilt;
  eintegral = eintegral + e*deltaT;
  
  float u = kp*e + ki*eintegral;

  // Set the motor speed and direction
  setMotor(u,M1A,M1B);

  Serial.print(vt);
  Serial.print(" ");
  Serial.print(vFilt);
  Serial.println();
  delay(1);
}

void setMotor(int pwm, int ma, int mb){

  // Make sure the speed is within the limit.
  if (pwm > 255) {
    pwm = 255;
  } else if (pwm < -255) {
    pwm = -255;
  }

  // For direction, functions used here cater to my motor driver (MDD3A) 
  // MDD3A has dual PWM pins for a single motor channel. 
  // Direction is specified in signage of pwm value.
  if (pwm >= 0) {
    analogWrite(ma, pwm);
    analogWrite(mb, 0);
  } else {
    analogWrite(ma, 0);
    analogWrite(mb, -pwm);
  }
}

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2
  currT_i = micros();
  deltaT_i = ((float) (currT_i - prevT_i))/1.0e6;
  velocity_i = increment/deltaT_i;
  prevT_i = currT_i;
}
#include <Arduino.h>

// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

// Defined states of the encoder channels AB (in this order)
#define START 0x0
#define CCW_BEGIN 0x1
#define CW_BEGIN 0x2
#define START_M 0x3
#define CW_BEGIN_M 0x4
#define CCW_BEGIN_M 0x5

// State table - tells what is the next possible states for a given current state
const unsigned char ttable[6][4] = {
  // START (00)
  {START_M,           CW_BEGIN,     CCW_BEGIN,  START},
  // CCW_BEGIN
  {START_M | DIR_CCW, START,        CCW_BEGIN,  START},
  // CW_BEGIN
  {START_M | DIR_CW,  CW_BEGIN,     START,      START},
  // START_M (11)
  {START_M,           CCW_BEGIN_M,  CW_BEGIN_M, START},
  // CW_BEGIN_M
  {START_M,           START_M,      CW_BEGIN_M, START | DIR_CW},
  // CCW_BEGIN_M
  {START_M,           CCW_BEGIN_M,  START_M,    START | DIR_CCW},
};

// Input pins connected to encoder outputs
#define ENCA 2 
#define ENCB 3
// Output pins connected to motor driver inputs
#define M1A 5
#define M1B 6

// GLOBALS
// Interrupt variables are volatile
// encoder count
volatile long counter = 0;

// encoder state
unsigned char state;

//custom functions' declarations
void readEncoder();
void setMotor(int pwm, int ma, int mb);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  // Initialise state.
  state = START;

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  
  pinMode(M1A,OUTPUT);
  pinMode(M1B,OUTPUT);

  digitalWrite(M1A, LOW);
  digitalWrite(M1B, LOW);
  
  attachInterrupt(0, readEncoder, CHANGE); // ENCA
  attachInterrupt(1, readEncoder, CHANGE); // ENCB
}

void loop() {
  setMotor(-100,M1A,M1B);
}

void readEncoder(){
  // Grab state of input pins.
  unsigned char pinstate = (digitalRead(ENCB) << 1) | digitalRead(ENCA);
  // Determine new state from the pins and state table.
  state = ttable[state & 0xf][pinstate];
  // Return the first two bits of the upper word in the state byte, 
  // ie the direction determined by the state machine.
  unsigned char result = state & 0x30;
  
  // Only after direction is determined in one phase cycle, count up / down accordingly
  if (result == DIR_CW) { // It's counting up smoothly in sync with my rotating of the wheel
    counter++;
    Serial.println(counter);
  } 
  else if (result == DIR_CCW) { // It's counting down slowly out of sync with my rotating of the wheel
    counter--;
    Serial.println(counter);
  }
}

void setMotor(int pwm, int ma, int mb){
  // Make sure the speed is within the limit.
  if (pwm > 255) {
    pwm = 255;
  } 
  else if (pwm < -255) {
    pwm = -255;
  }

  // For direction, functions used here cater to my motor driver (MDD3A) 
  // MDD3A has dual PWM pins for a single motor channel. 
  // Direction is specified in signage of pwm value.
  if (pwm >= 0) {
    analogWrite(ma, pwm);
    analogWrite(mb, 0);
  } 
  else {
    analogWrite(ma, 0);
    analogWrite(mb, -pwm);
  }
}


/* 
Findings:
  The encoder count was ok in syncing at low speed, but seems stuck at high speeds (PWM=200).
  This apparently is due to counting one pulse per phase, so I have to go thru each state in the state table 
  b4 I can read a count. So, this result in a lot of stuck state bcos fast rotation cause the encoder to not 
  catch the pulses, and therefore waits a lot for the right next state.

  Solution:
  By using half step (counting when I reach half of the state table 00 & 11), this reduces the waiting if I 
  MCU already knows it's moving clockwise or ACW by knowing the right transition before 00 & 11. This counting 
  works well counting up & down.
*/
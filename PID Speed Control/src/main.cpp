#include <Arduino.h>
#include <math.h>

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

#define POT 0

// Global Interrupt variables are volatile
volatile bool newIsr = false;
volatile unsigned long isrTime;
volatile unsigned long isrCount = 0;  // encoder count

// Global Non-Interrupt variables
unsigned char state;      // encoder state
unsigned long currTime = 0;   // current revolution time
unsigned long prevTime = 0;   // previous revolution time
float elapsedTime = 0; // change in time
long currCount = 0;
long prevCount = 0;
float setRpm = 0;
double cps = 0;
double rpm = 0;
double error = 0;
double prevError = 0;
double cumError = 0;
double rateError = 0;
float pidPwm;

//custom functions' declarations
void readEncoder();
void setMotor(int pwm, int ma, int mb);
void getIsrData();
void pidControl(float kp, float ki, float kd);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  // Initialise
  state = START;

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  
  pinMode(M1A,OUTPUT);
  pinMode(M1B,OUTPUT);

  digitalWrite(M1A, LOW);
  digitalWrite(M1B, LOW);
  
  attachInterrupt(0, readEncoder, CHANGE); // ENCA
  attachInterrupt(1, readEncoder, CHANGE); // ENCB
  
  setMotor(0,M1A,M1B);
  pidControl(0.5, 0, 1); // I put one here bcos without it, we need a count to run. But this is not possible when starting from stationary.
}

void loop() {
  setRpm = analogRead(POT);
  getIsrData();
  pidControl(1, 0.1, 0.01);
  setMotor((int)fabs(pidPwm), M1A, M1B);

  Serial.print("counts: "); Serial.print(currCount-prevCount); Serial.print("  "); Serial.print("dT: "); Serial.print(elapsedTime); Serial.print("  ");  Serial.print("cps: "); Serial.print(cps); Serial.print("  "); Serial.print("rpm: "); Serial.println(rpm);
  delay(500);
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
    isrTime = micros();
    isrCount++;
    newIsr = true;
    // Serial.println(isrCount);
  } 
  else if (result == DIR_CCW) { // It's counting down slowly out of sync with my rotating of the wheel
    isrTime = micros();
    isrCount--;
    newIsr = true;
    // Serial.println(isrCount);
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

void getIsrData(){
  if (newIsr == true){
    // Save the previous value
    prevCount = currCount;

    // Read parameter values from ISR, by pausing interrupts
    noInterrupts();
      currCount = isrCount;
      newIsr = false;
    interrupts();   
  }
}

void pidControl(float kp, float ki, float kd){
  prevTime = currTime; 
  currTime = micros();
  elapsedTime = ((float)(currTime - prevTime)) / 1.0e6;   // Time change in seconds
  cps = (currCount - prevCount) / elapsedTime;      // Velocity in count per second
  rpm = cps / 420.0 * 60;     // Velocity in RPM (X2 counting, 420 counts per revolution)
  
  // Scale rpm to pwm scale (0 - 255)
  rpm = rpm / 60 * 255;
  setRpm = setRpm / 1023 * 255;

  // Compute the control signal, pidPwm
  error = setRpm - rpm;
  cumError += error*elapsedTime;
  rateError = (error - prevError) / elapsedTime;
  pidPwm = kp*error + ki*cumError + kd*rateError;
  prevError = error;
}

/*
Finally, a working PID prototype
There are some configuration issues, which I think is affecting the timing of evrything.
TODO:
- Make flowchart and work from there to troubleshoot
- How to start motor without counting, and only use counting when the the motor is running
- How to fix the jitters in the motor speed up
- How to fix the timing issues in reading & calcultaing the parameters
- Try out different PID values. 
*/
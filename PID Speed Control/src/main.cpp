#include <Arduino.h>

// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

// Defined states of the encoder channels AB (in this order)
#define START 0x0
#define CW_FINAL 0x1
#define CW_BEGIN 0x2
#define CW_NEXT 0x3
#define CCW_BEGIN 0x4
#define CCW_FINAL 0x5
#define CCW_NEXT 0x6

// State table - tells what is the next possible states for a given current state
const unsigned char ttable[7][4] = {
  // START
  {START,    CW_BEGIN,  CCW_BEGIN, START},
  // CW_FINAL
  {CW_NEXT,  START,     CW_FINAL,  START | DIR_CW}, 
  // CW_BEGIN 
  {CW_NEXT,  CW_BEGIN,  START,     START},
  // CW_NEXT
  {CW_NEXT,  CW_BEGIN,  CW_FINAL,  START},
  // CCW_BEGIN
  {CCW_NEXT, START,     CCW_BEGIN, START},
  // CCW_FINAL
  {CCW_NEXT, CCW_FINAL, START,     START | DIR_CCW},
  // CCW_NEXT
  {CCW_NEXT, CCW_FINAL, CCW_BEGIN, START},
};

// IO pins connected to encoder outputs
#define ENCA 2 
#define ENCB 3

// encoder count
volatile int counter = 0;
// encoder state
unsigned char state;

//custom functions' declarations
void readEncoder();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  // Initialise state.
  state = START;

  attachInterrupt(0, readEncoder, CHANGE); // ENCA
  attachInterrupt(1, readEncoder, CHANGE); // ENCB
}

void loop() {}

void readEncoder(){
  // Grab state of input pins.
  unsigned char pinstate = (digitalRead(ENCB) << 1) | digitalRead(ENCA);
  // Determine new state from the pins and state table.
  state = ttable[state & 0xf][pinstate];
  // Return the first two bits of the upper word in the state byte, 
  // ie the direction determined by the state machine.
  unsigned char result = state & 0x30;
  
  if (result == DIR_CW) { // It's counting up smoothly in sync with my rotating of the wheel
    counter++;
    Serial.print(state); Serial.print(" ");  Serial.println(counter);
  } 
  else if (result == DIR_CCW) { // It's counting down slowly out of sync with my rotating of the wheel
    counter--;
    Serial.print(state); Serial.print(" ");  Serial.println(counter);
  }
}
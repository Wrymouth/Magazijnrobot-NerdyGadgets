#define speedPin 11
#define directionPin 13
#define brakePin 8
#define encoder 5
#include <Wire.h>

// the end position of the motor on the z axes
const int end = 0;

// used for communication between arduinos
bool x = false;
bool y = true;

int direction = 0;

// speed of motor
int speed = 50;

// variables for reading encoder
int encoderState;
int LastState;
int counter;

void setup() {
  // put your setup code here, to run once:

  // writes PWM frequency to be used by motor
  TCCR2B = TCCR2B & B11111000 | B00000111; // for PWM frequency of 30.64 Hz

  pinMode(directionPin, OUTPUT);
  pinMode(brakePin, OUTPUT);
  pinMode(speedPin, OUTPUT);

  // Starts connection to other arduino and recieves data on address 9
  Wire.begin(9); 
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);

  // interrupts evrytime encoder A/B pulses and executes readEncoder functions accordingly. this makes sure no pulse is missed
  attachInterrupt(digitalPinToInterrupt(encoder),readEncoder,RISING);
}


void receiveEvent(int bytes) {
  x = Wire.read();    // read one character from the I2C
}

// reads encoder and adds/ subtracts 1, based on direction, from counter everytime encoder pulses
void readEncoder(){
  encoderState = digitalRead(encoder);
  
  if (encoderState != LastState){      
       if (direction == 1){
       counter ++;  
       }
       else if (direction == -1){
       counter --; 
      }
   }
  LastState = encoderState;
  Serial.println("counter: " + counter);
}

void loop() {
  // put your main code here, to run repeatedly:

// moves z motor forward if joystick button is pressed
  if (x){
    direction = 1;
    digitalWrite(brakePin, LOW);
    digitalWrite(directionPin, HIGH);
    analogWrite(speedPin, speed);

// if end is reached stop motor and send for the other arduino to move up a bit
    if (counter >= end){
      direction = 0;
      digitalWrite(brakePin, HIGH);
      analogWrite(speedPin, 0);
      Wire.beginTransmission(8); // transmit to device #9
      Wire.write(y);              // sends y
      Wire.endTransmission();    // stop transmitting
    }
  }

// if y axes has stopped moving to pickup an item, move motor backwards until z motor is back at starting position
  if (!x){
    if (counter <= 0){
      direction = 0;
      digitalWrite(brakePin, HIGH);
      analogWrite(speedPin, 0);
      } 
      else{
      direction = -1;
      digitalWrite(brakePin, LOW);
      digitalWrite(directionPin, LOW);
      analogWrite(speedPin, speed);
    } 
  }
}

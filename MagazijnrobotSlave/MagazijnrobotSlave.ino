#define speedPin 11
#define directionPin 13
#define brakePin 8
#define encoder 5
#include <Wire.h>

const int end = 0;

bool x = false;
bool y = true;

int speed = 50;

// variables for reading encoder
int encoderState;
int LastState;
int counter;

void setup() {
  // put your setup code here, to run once:
  pinMode(directionPin, OUTPUT);
  pinMode(brakePin, OUTPUT);
  pinMode(speedPin, OUTPUT);

  // Start the I2C Bus as Slave on address 9
  Wire.begin(9); 
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
}

void receiveEvent(int bytes) {
  x = Wire.read();    // read one character from the I2C
}

void readEncoder(int dir){
  encoderState = digitalRead(encoder);
  
  if (encoderState != LastState){      
       if (dir == 1){
       counter ++;  
       }
       else if (dir == -1){
       counter --; 
      }
   }
  LastState = encoderState;
  Serial.println("counter: " + counter);
}

void loop() {
  // put your main code here, to run repeatedly:
  readEncoder();
  
  if (x){
    digitalWrite(brakePin, LOW);
    digitalWrite(directionPin, HIGH);
    analogWrite(speedPin, speed);

    if (counter >= end){
      digitalWrite(brakePin, HIGH);
      analogWrite(speedPin, 0);
      Wire.beginTransmission(8); // transmit to device #9
      Wire.write(y);              // sends y
      Wire.endTransmission();    // stop transmitting
    }
  }

  if (!x){
    if (counter <= 0){
      digitalWrite(brakePin, HIGH);
      analogWrite(speedPin, 0);
      } 
      else{
      digitalWrite(brakePin, LOW);
      digitalWrite(directionPin, LOW);
      analogWrite(speedPin, speed);
    } 
  }
}

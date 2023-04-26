#define speedPinA 3
#define speedPinB 11
#define directionPinA 12
#define directionPinB 13
#define brakePinA 9
#define brakePinB 8
#define encoderA 7
#define encoderB 5
#define SwPin 4
#include <Wire.h>

const int pickupDistance = 0;

// joystick
xDirection = analogRead(VrxPin);
yDirection = analogRead(VryPin);

// int pos = 0;
const int speed = 50;

// change value based on joystick or HMI input
int directionA;
int directionB;

// variables for reading encoderA
int encoderAState;
int aLastState;
int counterA = 0;

// variables for reading encoderB
int encoderBState;
int bLastState;
int counterB = 0;

//for connection between arduinos
bool x = true;
bool y = false;

int a = 0;

int pickupDistance = 0;

void setup() {
  Serial.begin(9600);

  TCCR2B = TCCR2B & B11111000 | B00000111; // for PWM frequency of 30.64 Hz

  //Setup Motor A vertical
  pinMode(directionPinA, OUTPUT);
  pinMode(brakePinA, OUTPUT); 
  pinMode(encoderA, INPUT);

  //Setup Motor B horizontal
  pinMode(directionPinB, OUTPUT);
  pinMode(brakePinB, OUTPUT);
  pinMode(encoderB, INPUT);

    // Start the I2C Bus as Slave on address 9
  Wire.begin(8); 
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);

  // attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {
// read joystick input
// if joystick pressed up, call: setMotorA(directionA); directionA being 1 for up, setMotorB(directionB); directionB being 0 for standing still.
// etc.
//if joystick is untouched motorA + B stop moving   
if (xDirection == 509 && yDirection == 528 ){
     setMotorA(0);
     setMotorB(0);
     readEncoderA(0);
     readEncoderB(0);
     //Serial.println("STOP");
   }
//if joystick is pointed left motorA goes left
if (xDirection < 200) {
      //Serial.println("Left");
      setMotorA(-1);
      readEncoderA(-1);
      readEncoderB(directionB);
   } 
//if joystick is pointed right motorA goes right  
   else if (xDirection > 700) {
      //Serial.println("Right");
      setMotorA(1);
      readEncoderA(1);
     
   }
//if joystick is pointed down motorB goes down
if (yDirection < 200) {
       //Serial.println("Down");
       setMotorB(-1);
       readEncoderB(-1);
   } 
//if joystick is pointed up motorB goes up
   else if (yDirection > 700) {
       //Serial.println("Up");
       setMotorB(1);
       readEncoderB(1);
   }

   if (y){
     if (a == 0){
       counterStart = counterB;
       a++;
     }
     setMotorB(1);
     readEncoderB(1);
     if(counterB - counterStart >= pickupDistance){
       setMotor(0);
       wire.beginTransmission(9);
       wire.write(false);
       wire.endTransmission();
     }
   }

  // transmission between arduinos
  readButton();
}

void receiveEvent(int bytes) {
  y = Wire.read();    // read one character from the I2C
}

void readButton(){
  switchState = digitalRead(SwPin);
  if (!switchState) {
       Serial.println("Switch pressed");
       Wire.beginTransmission(9); // transmit to device #9
      Wire.write(x);              // sends x 
      Wire.endTransmission();    // stop transmitting
      a = 0;
   }
}

// based on direction order motorA to move at predetermined speed in given direction, also disables brake if direction unless no direction is given
// motorA is for vertical movement
void setMotorA(int dir){
  if(dir == 1){
    digitalWrite(directionPinA, HIGH);
    digitalWrite(brakePinA, LOW);
    analogWrite(speedPinA, speed);
  }
  else if(dir == -1){
    digitalWrite(directionPinA, LOW);
    digitalWrite(brakePinA, LOW);
    analogWrite(speedPinA, speed);
  }
  else{
    digitalWrite(brakePinA, HIGH);
    analogWrite(speedPinA, 0);
  }
}

// functions the same as setMotorA but for motorB
// motorB is for horizontal movement
void setMotorB(int dir){
  if(dir == 1){
    digitalWrite(directionPinB, HIGH);
    digitalWrite(brakePinB, LOW);
    analogWrite(speedPinB, speed);
  }
  else if(dir == -1){
    digitalWrite(directionPinB, LOW);
    digitalWrite(brakePinB, LOW);
    analogWrite(speedPinB, speed);
  }
  else{
    digitalWrite(brakePinB, HIGH);
    analogWrite(speedPinB, 0);
  }
}

void readEncoderA(int dir){
  encoderAState = digitalRead(encoderA);
  
  if (encoderAState != aLastState){      
       if (dir == 1){
       counterA ++;  
       }
       else if (dir == -1){
       counterA --; 
      }
   }
  aLastState = encoderAState;
  Serial.println("counterA: " + counterA);
}

void readEncoderB(int dir){
  encoderBState = digitalRead(encoderB);
  
  if (encoderBState != bLastState){      
       if (dir == 1){
       counterB ++;  
       }
       else if (dir == -1){
       counterB --; 
      }
   }
  bLastState = encoderBState;
  Serial.println("counterB: " + counterB);
}

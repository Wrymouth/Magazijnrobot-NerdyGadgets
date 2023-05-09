#include <Wire.h>
#include <ezButton.h>
// Motor pins
#define speedPinA 3
#define speedPinB 11
#define directionPinA 12
#define directionPinB 13
#define brakePinA 9
#define brakePinB 8
#define encoderA 7
#define encoderB 5
//Joystick pins
#define VrxPin A3
#define VryPin A2
#define SwPin 4

ezButton button(SwPin);

// int pos = 0;
const int speed = 150;

// change value based on joystick or HMI input
int directionA;
int directionB;

// variables for direction joystick
int xDirection = 0;
int yDirection = 0;
bool x = true;
bool y;

// variables for reading encoderA
int aLastState = 0;
int encoderAState;
int counterA = 0;

// variables for reading encoderB
int bLastState;
int encoderBState;
int counterB = 0;
int counter;
bool prevA = 1, prevB = 1;



void setup() {
  Serial.begin(9600);


   //Setup Motor A vertical
  pinMode(directionPinA, OUTPUT);
  pinMode(brakePinA, OUTPUT); 
  pinMode(encoderA, INPUT);

  //Setup Motor B horizontal
  pinMode(directionPinB, OUTPUT);
  pinMode(brakePinB, OUTPUT);
  pinMode(encoderB, INPUT);

  pinMode(SwPin, INPUT);
  digitalWrite(SwPin, HIGH);
  
TCCR2B = TCCR2B & B11111000 | B00000110; // for PWM frequency of 122.55 Hz


   //attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {


xDirection = analogRead(VrxPin);
yDirection = analogRead(VryPin);

 // print data to Serial Monitor on Arduino IDE
  //Serial.print("x = ");
  //Serial.print(xDirection);
  //Serial.print(", y = ");
  //Serial.println(yDirection);
   //Serial.print("Switch:  ");
  //Serial.println(digitalRead(SwPin));
  //delay(200);

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
   else if (yDirection > 700 || y == true) {
       //Serial.println("Up");
       setMotorB(1);
       readEncoderB(1);
       
   } 


Serial.println()
readButton();



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
  bool A = digitalRead(encoderA);
 
  if (encoderAState > aLastState){      
    
       if (dir == 1){
       counterA++;
         
       }
       else if (dir == -1){
       counterA--; 
      }
   }
  //Serial.print("CounterA: ");
  //Serial.println(counterA);
 aLastState = encoderAState;
 prevA = A;
prevB = B;
 
}

void readEncoderB(int dir){
  encoderBState = digitalRead(encoderB);
  bool B = digitalRead(encoderB);
  if (encoderBState > bLastState){      
       if (dir == 1){ 
       counterB++;
     
       }
       else if (dir == -1){
       counterB--; 
      }

   }

  //Serial.print("CounterB: ");
  //Serial.println(counterB);
   bLastState = encoderBState; 
}


void readButton(){
  int buttonState = digitalRead(SwPin);
  if (buttonState == 0) {
       Serial.println("Switch pressed");
       Wire.beginTransmission(9); // transmit to device #9
      Wire.write(x);              // sends x 
      Wire.endTransmission();    // stop transmitting
      buttonState == 1;
      
   } 
}
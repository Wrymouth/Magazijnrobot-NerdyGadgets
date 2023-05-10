#define speedPinA 3
#define speedPinB 11
#define directionPinA 12
#define directionPinB 13
#define brakePinA 9
#define brakePinB 8
#define encoderA 2
#define encoderB 5
#define VrxPin A3
#define VryPin A2
#define SwPin 4
#include <Wire.h>
#include <PinChangeInterrupt.h>

// bool that changes to true when emergency button is pressed
bool emergency = false;

// distance motor y needs to move up to pickup an item
const int pickupDistance = -30;

// reads y and x direction on the joystick and save it in variable
int xDirection = analogRead(VrxPin);
int yDirection = analogRead(VryPin);

// button
byte lastButtonState = LOW;
unsigned long debounceDuration = 50; // millis
unsigned long lastTimeButtonStateChanged = 0;

// int pos = 0;
const int speed = 150;

// change value based on joystick or HMI input
int directionA = 0;
int directionB = 0;

// variables for reading encoderA
int counterStart;
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

// used to get startposition of y motor during item pickup
int a = 0;

void setup() {
  // starts serial communication
  Serial.begin(9600);

  // writes PWM frequency to be used by motors
  TCCR2B = TCCR2B & B11111000 | B00000111; // for PWM frequency of 30.64 Hz

  //Setup Motor A vertical
  pinMode(directionPinA, OUTPUT);
  pinMode(brakePinA, OUTPUT); 
  pinMode(encoderA, INPUT);

  //Setup Motor B horizontal
  pinMode(directionPinB, OUTPUT);
  pinMode(brakePinB, OUTPUT);
  pinMode(encoderB, INPUT);
  //Setup for button
   pinMode(SwPin, INPUT);
  digitalWrite(SwPin, HIGH);
    // Starts connection to other arduino and recieves data on address 8
  Wire.begin(8); 
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
  
  // interrupts evrytime encoder A/B pulses and executes readEncoder functions accordingly. this makes sure no pulse is missed
attachInterrupt(digitalPinToInterrupt(encoderA), readEncoderA, CHANGE);
attachPCINT(digitalPinToPCINT(encoderB), readEncoderB, RISING);
}

void loop() {
// read joystick input
// if joystick pressed up, call: setMotorA(directionA); directionA being 1 for up, setMotorB(directionB); directionB being 0 for standing still.
// etc.
 // print data to Serial Monitor on Arduino IDE

xDirection = analogRead(VrxPin);
yDirection = analogRead(VryPin);


//Serial.println(directionA);
  //Serial.print("x = ");
  //Serial.print(xDirection);
  //Serial.print(", y = ");
  //Serial.println(yDirection);
  //delay(200);
if (!emergency){
  
  readButton();
  setMotorA(directionA);
  setMotorB(directionB);
  readEncoderA();
  readEncoderB();
  if (!y){
  //if joystick is untouched motorA + B stop moving   
    if (xDirection == 509 && yDirection == 528 ){
     directionA = 0;
     directionB = 0;
     //Serial.println("STOP");
   }
  //if joystick is pointed left motorA goes left
    if (xDirection < 200) {
      //Serial.println("Left");
      directionA = -1;
    
   } 
  //if joystick is pointed right motorA goes right  
      else if (xDirection > 700) {
      //Serial.println("Right");
      directionA = 1;
      
   }
  //if joystick is pointed down motorB goes down
    if (yDirection < 200) {
       //Serial.println("Down");
       directionB = -1;
   } 
  //if joystick is pointed up motorB goes up
      else if (yDirection > 700) {
       //Serial.println("Up");
       directionB = 1;
   }
  }

   // if recieved data in variable y is true, determines start position of motor y and moves motor y up until pickupDistance is achieved.
   // then motor stops and sends for the other arduino to begin retracting motor z
   if (y){
     //Serial.println("kom op zeg");
     if (a == 0){
      counterStart = counterA;
      
         a++;
     }
     directionA = -1;
     directionB = 0;
     

     if(counterA - counterStart < pickupDistance){
     //Serial.println("check");
       directionA = 0;
       directionB = 0;
       digitalWrite(brakePinA, HIGH);
       digitalWrite(brakePinB, HIGH);
       Wire.beginTransmission(9);
       Wire.write(y);
       Wire.endTransmission();
       
     }
   }
   
}
}

// code to be executed on wire.onRecieve event
void receiveEvent(int bytes) {
  y = Wire.read();    // read one character from the I2C

}

// checks if joystick button is pressed, if true sends for the other arduino to begin pickup process
// set a to 0 to be able to determine starting position of motor y

void readButton(){
  int buttonState = digitalRead(SwPin);
  if (millis() - lastTimeButtonStateChanged > debounceDuration) {
    
   
    if (buttonState == LOW && lastButtonState == HIGH) {
      
      lastTimeButtonStateChanged = millis();
      
     
         Serial.println("Switch pressed");
       Wire.beginTransmission(9); // transmit to device #9
      Wire.write(x);              // sends x 
      Wire.endTransmission();    // stop transmitting
  
      
    }
    lastButtonState = buttonState;
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

// reads encoder from motor A and adds/ subtracts 1, based on direction, from counter everytime encoder pulses
void readEncoderA(){
  encoderAState = digitalRead(encoderA);
  
  if (encoderAState != aLastState){      
   counterA += directionA;

   }
  aLastState = encoderAState;
  Serial.print("CounterA: ");
  Serial.println(counterA);
  
}

// reads encoder from motor B and adds/ subtracts 1, based on direction, from counter everytime encoder pulses
void readEncoderB(){
  encoderBState = digitalRead(encoderB);
  
  if (encoderBState != bLastState){      
      counterB += directionB;
   }
  bLastState = encoderBState;
  Serial.println("---");
  Serial.print("CounterB: ");
  Serial.println(counterB);
 
}

// if emergency button is pressed set emegerency to true, code in loop won't be executed as long as emergency is true
void emergencyBrake(){
  emergency = true;
}

// void pciSetup(byte pin)
// {
//     *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
//     PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
//     PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
// }
// ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
//  {
//      digitalWrite(13,digitalRead(7) and digitalRead(5));
//  }  

#define speedPinA 3
#define speedPinB 11
#define directionPinA 12
#define directionPinB 13
#define brakePinA 9
#define brakePinB 8

// int pos = 0;
const int speed = 50;

// change value based on joystick or HMI input
int directionA;
int directionB;

void setup() {
  Serial.begin(9600);

  //Setup Motor A
  pinMode(directionPinA, OUTPUT);
  pinMode(brakePinA, OUTPUT); 
  pinMode(speedPinA, OUTPUT);

  //Setup Motor B
  pinMode(directionPinB, OUTPUT);
  pinMode(brakePinB, OUTPUT);
  pinMode(speedPinB, OUTPUT);

  // attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {
// read joystick input
// if joystick pressed up, call: setMotorA(directionA); directionA being 0 for not moving, setMotorB(directionB); directionB being 1 for up.
// etc.

setMotorA(directionA);
setMotorB(directionB);
}

// based on direction order motorA to move at predetermined speed in given direction, also disables brake if direction unless no direction is given
// motorA is for horizontal movement
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
    digitalWrite(speedPinA, LOW);
    analogWrite(speedPinA, 0);
  }
}

// functions the same as setMotorA but for motorB
// motorB is for vertical movement
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
    digitalWrite(speedPinB, LOW);
    analogWrite(speedPinB, 0);
  }
}

// void readEncoder(){
//   int b = digitalRead(ENCB);
//   if(b > 0){
//     pos++;
//   }
//   else{
//     pos--;
//   }
// }


#define speedPinA 3
#define speedPinB 11
#define directionPinA 12
#define directionPinB 13
#define brakePinA 9
#define brakePinB 8
#define encoderA 1
#define encoderB 2


// int pos = 0;
const int speed = 50;

// change value based on joystick or HMI input
int directionA;
int directionB;
<<<<<<< HEAD
const int swPin = 1;
const int VrxPin = A1;
const int VryPin = A0;

int xDirection = 0;
int yDirection = 0;
int switchState = 1;


// variables for reading encoderA
int encoderAState;
int aLastState;
int counterA;

// variables for reading encoderB
int encoderBState;
int bLastState;
int counterB;
>>>>>>> origin/motors

void setup() {
  Serial.begin(9600);


   //Setup Motor A vertical
  pinMode(directionPinA, OUTPUT);
  pinMode(brakePinA, OUTPUT); 
  pinMode(speedPinA, OUTPUT);
  pinMode(encoderA, INPUT);

  //Setup Motor B horizontal
  pinMode(directionPinB, OUTPUT);
  pinMode(brakePinB, OUTPUT);
  pinMode(speedPinB, OUTPUT);
  pinMode(encoderB, INPUT);

  //Setup joystick
  pinMode(swPin, INPUT);
  digitalWrite(swPin, HIGH);
  
  TCCR2B = TCCR2B & B11111000 | B00000111; // for PWM frequency of 30.64 Hz


 


  // attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {

xDirection = analogRead(VryPin);
yDirection = analogRead(VrxPin);
// if joystick pressed up, call: setMotorA(directionA); directionA being 0 for not moving, setMotorB(directionB); directionB being 1 for up.
// etc.
if (xDirection < 480) {
      Serial.println("Left");
      setMotorA(1);
   } else if (xDirection > 520) {
      Serial.println("Right");
      setMotorA(-1);
   }
 
   if (yDirection < 480) {
       Serial.println("Down");
       setMotorB(-1);
   } else if (yDirection > 520) {
       Serial.println("Up");
       setMotorB(1);
   }

readEncoderA(directionA);
readEncoderB(directionB);
setMotorA(directionA);
setMotorB(directionB);


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
    digitalWrite(speedPinA, LOW);
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
    digitalWrite(speedPinB, LOW);
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



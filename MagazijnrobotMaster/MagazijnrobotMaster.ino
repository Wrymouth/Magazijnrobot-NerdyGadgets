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
#include <PinChangeInterrupt.h>
#include <Wire.h>

// bool that changes to true when emergency button is pressed
bool emergency = false;

// distance motor y needs to move up to pickup an item
const int pickupDistance = -200;

// reads y and x direction on the joystick and save it in variable
int xDirection = analogRead(VrxPin);
int yDirection = analogRead(VryPin);
bool readJoystick = true;

// button
byte lastButtonState = LOW;
unsigned long debounceDuration = 50;  // millis
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

// for connection between arduinos
bool x = true;
bool y = false;


enum MasterSignals {
    MASTER_INITIAL,
    MASTER_JOYSTICK_PRESSED,
    MASTER_MOVE_FINISHED,
};

enum SlaveSignals {
    SLAVE_INITIAL,
    SLAVE_AT_END,
    SLAVE_AT_START,
};

MasterSignals masterSignal = MASTER_INITIAL;
SlaveSignals slaveSignal = SLAVE_INITIAL;

//array for coordinates, contains 3 elements
String coordinates[3];


// used to get startposition of y motor during item pickup
int a = 0;

enum robot {
  automatic,
  joystick,
  emergencyState
};

robot currentRobotState;

void setup() {
    // starts serial communication
    Serial.begin(9600);

    // writes PWM frequency to be used by motors
    TCCR2B = TCCR2B & B11111000 | B00000111;  // for PWM frequency of 30.64 Hz

    // Setup Motor A vertical
    pinMode(directionPinA, OUTPUT);
    pinMode(brakePinA, OUTPUT);
    pinMode(encoderA, INPUT);

    // Setup Motor B horizontal
    pinMode(directionPinB, OUTPUT);
    pinMode(brakePinB, OUTPUT);
    pinMode(encoderB, INPUT);
    // Setup for button
    pinMode(SwPin, INPUT);
    digitalWrite(SwPin, HIGH);
    // Starts connection to other arduino and recieves data on address 8
    Wire.begin(8);
    // Attach a function to trigger when something is received.
    Wire.onReceive(receiveEvent);

    // interrupts evrytime encoder A/B pulses and executes readEncoder functions
    // accordingly. this makes sure no pulse is missed
    attachInterrupt(digitalPinToInterrupt(encoderA), readEncoderA, CHANGE);
    attachPCINT(digitalPinToPCINT(encoderB), readEncoderB, CHANGE);
}

void loop() {
  emergencyBrake(); //Constantly check for emergency

  if(emergency) {
    currentRobotState = emergencyState; //Change case to emergency
  }
    // read joystick input
    // if joystick pressed up, call: setMotorA(directionA); directionA being 1
    // for up, setMotorB(directionB); directionB being 0 for standing still.
    // etc.
    // print data to Serial Monitor on Arduino IDE


  readButton();
  setMotorA(directionA);
  setMotorB(directionB);
  xDirection = analogRead(VrxPin);
  yDirection = analogRead(VryPin);

  readEncoderA();
  readEncoderB();

  switch(currentRobotState) {
    case automatic:
      //all functions for automatic
      if(emergency){
        currentRobotState = emergencyState;
      }

      readSerial();

      for(String coordinate: coordinates){
        int commaIndex = coordinate.indexOf(',');
        String xCoordinate = coordinate.substring(0, commaIndex);
        String yCoordinate = coordinate.substring(commaIndex + 1);

        Serial.println(xCoordinate);
        Serial.println(yCoordinate);
      }

      


      break;
    
    case joystick:
      if (emergency){
        currentRobotState = emergencyState;
      }
      else if (!emergency && readJoystick){

        if (!y){
        //if joystick is untouched motorA + B stop moving
          if (xDirection == 509){
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
          Serial.print("CounterY: ");
          Serial.println(counterA);
          Serial.println("---");
          Serial.print("CounterX: ");
          Serial.println(counterB);
        // if recieved data in variable y is true, determines start position of motor y and moves motor y up until pickupDistance is achieved.
        // then motor stops and sends for the other arduino to begin retracting motor z
        if (y){
          
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
            Wire.write(false);
            Wire.endTransmission();
            y = false;
            a--;
            
          }
        }
      }

      break;
    
    case emergencyState:
      Serial.println("Emergency Pressed");

      if(Serial.available() > 0) {
        int emergencyValue = Serial.read(); //If any value is read, change emergency back to false

        if(emergencyValue == 1) {
          emergency = false;
        }
      }
      break;

    default:
      //functions default

      break;
  }
// read joystick input
// if joystick pressed up, call: setMotorA(directionA); directionA being 1 for up, setMotorB(directionB); directionB being 0 for standing still.
// etc.
 // print data to Serial Monitor on Arduino IDE


    // Serial.println(directionA);
    // Serial.print("x = ");
    // Serial.print(xDirection);
    // Serial.print(", y = ");
    // Serial.println(yDirection);
    // delay(200);


    if (!emergency && readJoystick) {
        readButton();
        setMotorA(directionA);
        setMotorB(directionB);
        xDirection = analogRead(VrxPin);
        yDirection = analogRead(VryPin);
        readEncoderA();
        readEncoderB();
        if (slaveSignal == SLAVE_INITIAL && masterSignal == MASTER_INITIAL) {
            // if joystick is untouched motorA + B stop moving
            if (xDirection == 509 && yDirection == 528) {
                directionA = 0;
                directionB = 0;
                // Serial.println("STOP");
            }
            // if joystick is pointed left motorA goes left
            if (xDirection < 200) {
                // Serial.println("Left");
                directionA = -1;

            }
            // if joystick is pointed right motorA goes right
            else if (xDirection > 700) {
                // Serial.println("Right");
                directionA = 1;
            }
            // if joystick is pointed down motorB goes down
            if (yDirection < 200) {
                // Serial.println("Down");
                directionB = -1;
            }
            // if joystick is pointed up motorB goes up
            else if (yDirection > 700) {
                // Serial.println("Up");
                directionB = 1;
            }
        }
        Serial.print("CounterY: ");
        Serial.println(counterA);
        Serial.println("---");
        Serial.print("CounterX: ");
        Serial.println(counterB);
        // if recieved data in variable y is true, determines start position of
        // motor y and moves motor y up until pickupDistance is achieved. then
        // motor stops and sends for the other arduino to begin retracting motor
        // z
        if (slaveSignal == SLAVE_AT_END) {
            if (a == 0) {
                counterStart = counterA;

                a++;
            }

            directionA = -1;
            directionB = 0;

            if (counterA - counterStart < pickupDistance) {
                // Serial.println("check");
                directionA = 0;
                directionB = 0;
                digitalWrite(brakePinA, HIGH);
                digitalWrite(brakePinB, HIGH);
                Wire.beginTransmission(9);
                masterSignal = MASTER_MOVE_FINISHED;
                Wire.write(masterSignal);
                Wire.endTransmission();
                y = false;
            }
        }
        if (slaveSignal == SLAVE_AT_START) {
          slaveSignal = SLAVE_INITIAL;
          masterSignal = MASTER_INITIAL;
        }
    }
}

//Emergency check function
void emergencyBrake(){
  if(Serial.available() > 0) {
    int emergencyValue = Serial.read(); //Returns an int either 0 or 1

    if(emergencyValue == 1) {
      emergency = true;
    } else {
      emergency = false;      
    }  
  }
}

// code to be executed on wire.onRecieve event
void receiveEvent(int bytes) {
    // read one character from the I2C
    slaveSignal = static_cast<SlaveSignals>(Wire.read());

    readJoystick = true;
}

// checks if joystick button is pressed, if true sends for the other arduino to
// begin pickup process set a to 0 to be able to determine starting position of
// motor y

void readButton() {
    int buttonState = digitalRead(SwPin);
    if (millis() - lastTimeButtonStateChanged > debounceDuration) {
        if (buttonState == LOW && lastButtonState == HIGH) {
            lastTimeButtonStateChanged = millis();
            a = 0;
            Serial.println("Switch pressed");
            readJoystick = false;
            Wire.beginTransmission(9);  // transmit to device #9
            masterSignal = MASTER_JOYSTICK_PRESSED;
            Wire.write(masterSignal);           // sends x
            Wire.endTransmission();     // stop transmitting
        }
        lastButtonState = buttonState;
    }
}

// based on direction order motorA to move at predetermined speed in given
// direction, also disables brake if direction unless no direction is given
// motorA is for vertical movement
void setMotorA(int dir) {
    if (dir == 1) {
        digitalWrite(directionPinA, HIGH);
        digitalWrite(brakePinA, LOW);
        analogWrite(speedPinA, 50);
    } else if (dir == -1) {
        digitalWrite(directionPinA, LOW);
        digitalWrite(brakePinA, LOW);
        analogWrite(speedPinA, speed);
    } else {
        digitalWrite(brakePinA, HIGH);
        analogWrite(speedPinA, 0);
    }
}

// functions the same as setMotorA but for motorB
// motorB is for horizontal movement
void setMotorB(int dir) {
    if (dir == 1) {
        digitalWrite(directionPinB, HIGH);
        digitalWrite(brakePinB, LOW);
        analogWrite(speedPinB, speed);
    } else if (dir == -1) {
        digitalWrite(directionPinB, LOW);
        digitalWrite(brakePinB, LOW);
        analogWrite(speedPinB, speed);
    } else {
        digitalWrite(brakePinB, HIGH);
        analogWrite(speedPinB, 0);
    }
}

// reads encoder from motor A and adds/ subtracts 1, based on direction, from
// counter everytime encoder pulses
void readEncoderA() {
    encoderAState = digitalRead(encoderA);

    if (encoderAState != aLastState) {
        counterA += directionA;
    }
    aLastState = encoderAState;
}

// reads encoder from motor B and adds/ subtracts 1, based on direction, from
// counter everytime encoder pulses
void readEncoderB() {
    encoderBState = digitalRead(encoderB);

    if (encoderBState != bLastState) {
        counterB += directionB;
    }
    bLastState = encoderBState;
}

void readSerial(){
  if (Serial.available() >  0){
      String instructions = "12,3 35,300 46,69";
      // String instructions = Serial.readString(); // reads input from HMI
      int spaceIndex = instructions.indexOf(' '); // saves space position in variable
      int secondSpaceIndex = instructions.indexOf(' ', spaceIndex + 1); // saves second space position variable

      // x + y from 3 products from an order
      String firstCoordinate = instructions.substring(0,spaceIndex);
      String secondCoordinate = instructions.substring(spaceIndex + 1, secondSpaceIndex);
      String thirdCoordinate = instructions.substring(secondSpaceIndex + 1);

      //converts string to integer and stores it in coordinates array
      coordinates[0] = firstCoordinate;
      coordinates[1] = secondCoordinate;
      coordinates[2] = thirdCoordinate;


      // int commaIndex = firstCoordinate.indexOf(',');
      // String xFirstCoordinate = firstCoordinate.substring(0, commaIndex);
      // String yFirstCoordinate = firstCoordinate.substring(commaIndex + 1);

      // int commaSecondIndex = secondCoordinate.indexOf(',');
      // String xSecondCoordinate = secondCoordinate.substring(0, commaSecondIndex);
      // String ySecondCoordinate = secondCoordinate.substring(commaSecondIndex + 1);

      // int commaThirdIndex = thirdCoordinate.indexOf(',');
      // String xThirdCoordinate = thirdCoordinate.substring(0, commaThirdIndex);
      // String yThirdCoordinate = thirdCoordinate.substring(commaThirdIndex + 1);

      }
  }



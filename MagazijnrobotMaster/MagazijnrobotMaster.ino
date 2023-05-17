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
int joystickX = analogRead(VrxPin);
int joystickY = analogRead(VryPin);
bool readJoystick = true;

// button
byte lastButtonState = LOW;
unsigned long debounceDuration = 50;  // millis
unsigned long lastTimeButtonStateChanged = 0;

// int pos = 0;
const int speed = 150;

// change value based on joystick or HMI input
int directionY = 0;
int directionX = 0;

// variables for reading encoderA
int counterStart;
int encoderAState;
int aLastState;
int counterY = 0;

// variables for reading encoderB
int encoderBState;
int bLastState;
int counterX = 0;

// for connection between arduinos
bool x = true;
bool y = false;

int goalX = 0;
int goalY = 0;

int coordinateIndex = 0;


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

  // read joystick input
  // if joystick pressed up, call: setMotorA(directionY); directionY being 1
  // for up, setMotorB(directionX); directionX being 0 for standing still.
  // etc.
  // print data to Serial Monitor on Arduino IDE


  readButton();
  setMotorA(directionY);
  setMotorB(directionX);
  joystickX = analogRead(VrxPin);
  joystickY = analogRead(VryPin);

  readEncoderA();
  readEncoderB();

  switch (currentRobotState) {
    case automatic:
      //all functions for automatic
      if (emergency) {
        currentRobotState = emergencyState;
      }

      readSerial();


      int commaIndex = coordinate.indexOf(',');
      String xCoordinate = coordinate.substring(0, commaIndex);
      String yCoordinate = coordinate.substring(commaIndex + 1);

      goalX = (int)xCoordinate;
      goalY = (int)yCoordinate;


      break;

    case joystick:
      if (emergency) {
        currentRobotState = emergencyState;
      } else if (!emergency && readJoystick) {

        if (!y) {
          //if joystick is untouched motorA + B stop moving
          if (joystickX == 509) {
            directionY = 0;
            directionX = 0;
            //Serial.println("STOP");
          }
          //if joystick is pointed left motorA goes left
          if (joystickX < 200) {
            //Serial.println("Left");
            directionY = -1;

          }
          //if joystick is pointed right motorA goes right
          else if (joystickX > 700) {
            //Serial.println("Right");
            directionY = 1;
          }
          //if joystick is pointed down motorB goes down
          if (joystickY < 200) {
            //Serial.println("Down");
            directionX = -1;
          }
          //if joystick is pointed up motorB goes up
          else if (joystickY > 700) {
            //Serial.println("Up");
            directionX = 1;
          }
        }
        Serial.print("CounterY: ");
        Serial.println(counterY);
        Serial.println("---");
        Serial.print("CounterX: ");
        Serial.println(counterX);
        // if recieved data in variable y is true, determines start position of motor y and moves motor y up until pickupDistance is achieved.
        // then motor stops and sends for the other arduino to begin retracting motor z
        if (y) {

          if (a == 0) {
            counterStart = counterY;

            a++;
          }

          directionY = -1;
          directionX = 0;

          if (counterY - counterStart < pickupDistance) {
            //Serial.println("check");
            directionY = 0;
            directionX = 0;
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
      //all functions emergency

      break;

    default:
      //functions default

      break;
  }

// read joystick input
// if joystick pressed up, call: setMotorA(directionY); directionY being 1 for up, setMotorB(directionX); directionX being 0 for standing still.
// etc.
// print data to Serial Monitor on Arduino IDE


// Serial.println(directionY);
// Serial.print("x = ");
// Serial.print(joystickX);
// Serial.print(", y = ");
// Serial.println(joystickY);
// delay(200);


if (!emergency && readJoystick) {
  readButton();
  setMotorA(directionY);
  setMotorB(directionX);
  joystickX = analogRead(VrxPin);
  joystickY = analogRead(VryPin);
  readEncoderA();
  readEncoderB();
  if (slaveSignal == SLAVE_INITIAL && masterSignal == MASTER_INITIAL) {
    // if joystick is untouched motorA + B stop moving
    if (joystickX == 509 && joystickY == 528) {
      directionY = 0;
      directionX = 0;
      // Serial.println("STOP");
    }
    // if joystick is pointed left motorA goes left
    if (joystickX < 200) {
      // Serial.println("Left");
      directionY = -1;

    }
    // if joystick is pointed right motorA goes right
    else if (joystickX > 700) {
      // Serial.println("Right");
      directionY = 1;
    }
    // if joystick is pointed down motorB goes down
    if (joystickY < 200) {
      // Serial.println("Down");
      directionX = -1;
    }
    // if joystick is pointed up motorB goes up
    else if (joystickY > 700) {
      // Serial.println("Up");
      directionX = 1;
    }
  }
  Serial.print("CounterY: ");
  Serial.println(counterY);
  Serial.println("---");
  Serial.print("CounterX: ");
  Serial.println(counterX);
  // if recieved data in variable y is true, determines start position of
  // motor y and moves motor y up until pickupDistance is achieved. then
  // motor stops and sends for the other arduino to begin retracting motor
  // z
  if (slaveSignal == SLAVE_AT_END) {
    if (a == 0) {
      counterStart = counterY;

      a++;
    }

    directionY = -1;
    directionX = 0;

    if (counterY - counterStart < pickupDistance) {
      // Serial.println("check");
      directionY = 0;
      directionX = 0;
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
      Wire.write(masterSignal);  // sends x
      Wire.endTransmission();    // stop transmitting
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
    counterY += directionY;
  }
  aLastState = encoderAState;
}

// reads encoder from motor B and adds/ subtracts 1, based on direction, from
// counter everytime encoder pulses
void readEncoderB() {
  encoderBState = digitalRead(encoderB);

  if (encoderBState != bLastState) {
    counterX += directionX;
  }
  bLastState = encoderBState;
}

// if emergency button is pressed set emegerency to true, code in loop won't be
// executed as long as emergency is true
void emergencyBrake() {
  emergency = true;
}

// if emergency button is pressed set emegerency to true, code in loop won't be executed as long as emergency is true
void emergencyBrake() {
  emergency = true;
}

void readSerial() {
  if (Serial.available() > 0) {
    String instructions = "12,3 35,300 46,69";
    // String instructions = Serial.readString(); // reads input from HMI
    int spaceIndex = instructions.indexOf(' ');                        // saves space position in variable
    int secondSpaceIndex = instructions.indexOf(' ', spaceIndex + 1);  // saves second space position variable

    // x + y from 3 products from an order
    String firstCoordinate = instructions.substring(0, spaceIndex);
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

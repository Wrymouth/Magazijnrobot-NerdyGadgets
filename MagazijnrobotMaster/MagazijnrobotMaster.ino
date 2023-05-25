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
const int pickupDistance = 200;

// interval between Serial messages sent to HMI
const int printInterval = 100;
unsigned long previousPrintTime = 0;

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

// array for coordinates, contains 3 elements
String coordinates[3];

// used to get startposition of y motor during item pickup
int a = 0;

enum RobotState { AUTOMATIC, JOYSTICK, EMERGENCY };

RobotState currentRobotState = JOYSTICK;

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
    emergencyBrake(); //Check if emergency value is being received

    if(emergency) {
        currentRobotState = EMERGENCY;  // Change case to emergency
    }
    // read joystick input
    // if joystick pressed up, call: setMotorA(directionY); directionY being 1
    // for up, setMotorB(directionX); directionX being 0 for standing still.
    // etc.
    // print data to Serial Monitor on Arduino IDE

    // readButton();
    setMotorA(directionY);
    setMotorB(directionX);
    // joystickX = analogRead(VrxPin);
    // joystickY = analogRead(VryPin);

    readEncoderA();
    readEncoderB();

    readSerial();

    if (millis() - previousPrintTime >= printInterval) {
        previousPrintTime = millis();
        Serial.print(counterX);
        Serial.print(",");
        Serial.println(counterY);
    }

    switch (currentRobotState) {
        case AUTOMATIC: {
            // all functions for automatic
            if(emergency) {
                currentRobotState = EMERGENCY;
            }

            int commaIndex = coordinates[coordinateIndex].indexOf(',');
            String xCoordinate =
                coordinates[coordinateIndex].substring(0, commaIndex);
            String yCoordinate =
                coordinates[coordinateIndex].substring(commaIndex + 1);

            goalX = xCoordinate.toInt();
            goalY = yCoordinate.toInt();

            if (slaveSignal == SLAVE_INITIAL &&
                masterSignal == MASTER_INITIAL) {
                if (counterX < goalX) {
                    directionX = 1;
                } else if (counterX > goalX) {
                    directionX = -1;
                } else {
                    directionX = 0;
                }

                if (counterY < goalY) {
                    directionY = -1;
                } else if (counterY > goalY) {
                    directionY = 1;
                } else {
                    directionY = 0;
                }

                if (counterX == goalX && counterY == goalY) {
                    a = 0;
                    masterSignal = MASTER_JOYSTICK_PRESSED;
                    wireSendSignal();
                    coordinateIndex++;
                    if (coordinateIndex > 2 ||
                        coordinates[coordinateIndex] == "") {
                        coordinateIndex = 0;
                        currentRobotState = JOYSTICK;
                    }
                }
            }

            if (slaveSignal == SLAVE_AT_END) {
                if (a == 0) {
                    counterStart = counterY;
                    a++;
                }

                directionY = -1;
                directionX = 0;

                if (counterY - counterStart > pickupDistance) {
                    directionY = 0;
                    directionX = 0;
                    digitalWrite(brakePinA, HIGH);
                    digitalWrite(brakePinB, HIGH);
                    masterSignal = MASTER_MOVE_FINISHED;
                    wireSendSignal();
                    y = false;
                }
            }
            if (slaveSignal == SLAVE_AT_START) {
                slaveSignal = SLAVE_INITIAL;
                masterSignal = MASTER_INITIAL;
            }
            break;
        }

        case JOYSTICK: {
            if (emergency) {
                currentRobotState = EMERGENCY;
            } else if (!emergency && readJoystick) {
                readButton();
                // setMotorA(directionY);
                // setMotorB(directionX);
                joystickX = analogRead(VrxPin);
                joystickY = analogRead(VryPin);
                // readEncoderA();
                // readEncoderB();
                if (slaveSignal == SLAVE_INITIAL &&
                    masterSignal == MASTER_INITIAL) {
                    // if joystick is untouched motorA + B stop moving
                    if (joystickX == 510 && joystickY == 528) {
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

                // if recieved data in variable y is true, determines start
                // position of motor y and moves motor y up until pickupDistance
                // is achieved. then motor stops and sends for the other arduino
                // to begin retracting motor
                // z
                if (slaveSignal == SLAVE_AT_END) {
                    if (a == 0) {
                        counterStart = counterY;

                        a++;
                    }

                    directionY = -1;
                    directionX = 0;

                    if (counterY - counterStart > pickupDistance) {
                        directionY = 0;
                        directionX = 0;
                        digitalWrite(brakePinA, HIGH);
                        digitalWrite(brakePinB, HIGH);
                        masterSignal = MASTER_MOVE_FINISHED;
                        wireSendSignal();
                        y = false;
                    }
                }
                if (slaveSignal == SLAVE_AT_START) {
                    slaveSignal = SLAVE_INITIAL;
                    masterSignal = MASTER_INITIAL;
                }
            }

            break;
        }
        case EMERGENCY: {
          Serial.println("Emergency Pressed");
            
          if(Serial.available() > 0) {
            char emergencyValue = Serial.read(); //If any value is read, change emergency back to false

            if(emergencyValue ==  "E") {
              emergency = false;
              break;
            }

          break;
        }

        default: {
            // functions default

            Serial.println("default");

            break;
        }
    }
}

// Emergency check function
void emergencyBrake() {
    if (Serial.available() > 0) {
        char emergencyValue = Serial.read();  //If char "E" is received, the emergency brake will be activated

        if (emergencyValue == "E") {
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
            directionY = 0;
            directionX = 0;
            lastTimeButtonStateChanged = millis();
            a = 0;
            Serial.println("Switch pressed");
            readJoystick = false;
            masterSignal = MASTER_JOYSTICK_PRESSED;
            wireSendSignal();
        }
        lastButtonState = buttonState;
    }
}

void wireSendSignal() {
    Wire.beginTransmission(9);  // transmit to device #9
    Wire.write(masterSignal);   // sends x
    Wire.endTransmission();     // stop transmitting
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
        counterY -= directionY;
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

void readSerial() {
    if (Serial.available() > 0) {
        String instructions = "12,3 35,300 46,69";
        // String instructions = Serial.readString(); // reads input from HMI
        int spaceIndex =
            instructions.indexOf(' ');  // saves space position in variable
        int secondSpaceIndex = instructions.indexOf(
            ' ', spaceIndex + 1);  // saves second space position variable

        // x + y from 3 products from an order
        String firstCoordinate = instructions.substring(0, spaceIndex);
        String secondCoordinate =
            instructions.substring(spaceIndex + 1, secondSpaceIndex);
        String thirdCoordinate = instructions.substring(secondSpaceIndex + 1);

        // converts string to integer and stores it in coordinates array
        coordinates[0] = firstCoordinate;
        coordinates[1] = secondCoordinate;
        coordinates[2] = thirdCoordinate;

        // int commaIndex = firstCoordinate.indexOf(',');
        // String xFirstCoordinate = firstCoordinate.substring(0, commaIndex);
        // String yFirstCoordinate = firstCoordinate.substring(commaIndex + 1);

        // int commaSecondIndex = secondCoordinate.indexOf(',');
        // String xSecondCoordinate = secondCoordinate.substring(0,
        // commaSecondIndex); String ySecondCoordinate =
        // secondCoordinate.substring(commaSecondIndex + 1);

        // int commaThirdIndex = thirdCoordinate.indexOf(',');
        // String xThirdCoordinate = thirdCoordinate.substring(0,
        // commaThirdIndex); String yThirdCoordinate =
        // thirdCoordinate.substring(commaThirdIndex + 1);
    }
}       

void readSerial() {
    if (Serial.available() > 0) {
        currentRobotState = AUTOMATIC;
        String instructions = Serial.readString();  // reads input from HMI
        int spaceIndex =
            instructions.indexOf(' ');  // saves space position in variable
        int secondSpaceIndex = instructions.indexOf(
            ' ', spaceIndex + 1);  // saves second space position variable

        // x + y from 3 products from an order
        String firstCoordinate = instructions.substring(0, spaceIndex);
        String secondCoordinate = "";
        String thirdCoordinate = "";
        if (spaceIndex != -1) {
            secondCoordinate =
                instructions.substring(spaceIndex + 1, secondSpaceIndex);
        }
        if (secondSpaceIndex != -1) {
            thirdCoordinate = instructions.substring(secondSpaceIndex + 1);
        }

        // converts string to integer and stores it in coordinates array
        coordinates[0] = firstCoordinate;
        coordinates[1] = secondCoordinate;
        coordinates[2] = thirdCoordinate;
    }
}

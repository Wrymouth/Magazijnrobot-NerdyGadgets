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
#include <ezButton.h>
// pins for limitswitches on X and Y axis
ezButton limitSwitch1(10);  // create ezButton object that attach to pin 10;
ezButton limitSwitch2(7);   // create ezButton object that attach to pin 7;
ezButton limitSwitch3(6);   // create ezButton object that attach to pin 6;

// bool that changes to true when emergency button is pressed
bool emergency = false;
// bools to check for limitswitches direction
bool SwitchYup = false;
bool SwitchDown = false;
bool SwitchRight = false;
bool SwitchLeft = false;

// distance motor y needs to move up to pickup an item
const int pickupDistance = 200;

// interval between Serial messages sent to HMI
const int printInterval = 500;
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
const int speed = 220;

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

enum RobotState { AUTOMATIC, JOYSTICK, RESET, PICKUP, EMERGENCY, SWITCHY };

RobotState currentRobotState = JOYSTICK;
RobotState previousRobotState = JOYSTICK;

bool pickupReset = false;

void setup() {
    // starts serial communication
    Serial.begin(9600);

    // set debounce time to 50 milliseconds
    limitSwitch1.setDebounceTime(50);
    limitSwitch2.setDebounceTime(50);
    limitSwitch3.setDebounceTime(50);

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
    // print data to Serial Monitor on Arduino IDE
    if (emergency) {
        currentRobotState = EMERGENCY;
    }
    setMotorA(directionY);
    setMotorB(directionX);

    readEncoderA();
    readEncoderB();

    readSerial();

    switchY1();
    switchY2();
    switchX2();

    if (millis() - previousPrintTime >= printInterval) {
        previousPrintTime = millis();
        Serial.print(counterX);
        Serial.print(",");
        Serial.println(counterY);
    }

    switch (currentRobotState) {
        case AUTOMATIC: {
            // all functions for automatic

            if (coordinateIndex > 2 || coordinates[coordinateIndex] == "") {
                previousRobotState = currentRobotState;
                Serial.print("p");
                Serial.println(coordinateIndex);
                currentRobotState = RESET;
                coordinateIndex = 0;
                break;
            }

            int commaIndex = coordinates[coordinateIndex].indexOf(',');
            String xCoordinate =
                coordinates[coordinateIndex].substring(0, commaIndex);
            String yCoordinate =
                coordinates[coordinateIndex].substring(commaIndex + 1);

            goalX = xCoordinate.toInt();
            goalY = yCoordinate.toInt();

            if (counterX < goalX) {
                directionX = 1;
            } else if (counterX > goalX) {
                directionX = -1;
            } else {
                directionX = 0;
                // if recieved data in variable y is true, determines start
                // position of motor y and moves motor y up until pickupDistance
                // is achieved. then motor stops and sends for the other arduino
                // to begin retracting motor
                // z
            }

            if (counterY < goalY) {
                directionY = 1;
            } else if (counterY > goalY) {
                directionY = -1;
            } else {
                directionY = 0;
            }

            if (counterX == goalX && counterY == goalY) {
                a = 0;
                coordinateIndex++;
                previousRobotState = currentRobotState;
                currentRobotState = PICKUP;
            }

            break;
        }
        case JOYSTICK: {
            if (emergency) {
                currentRobotState = EMERGENCY;
                break;
            }

            if (readJoystick) {
                readButton();
                joystickY = analogRead(VrxPin);
                joystickX = analogRead(VryPin);

                // if joystick is pointed up motorA goes up
                if (joystickY < 200 && !SwitchYup) {
                    directionY = 1;

                }
                // if joystick is pointed down motorA goes down
                else if (joystickY > 700 && !SwitchDown) {
                    directionY = -1;
                } else {
                    // if joystick is untouched motor A stops moving
                    directionY = 0;
                }

                // if joystick is pointed left motorB goes left
                if (joystickX < 200 && !SwitchLeft) {
                    directionX = -1;
                }
                // if joystick is pointed right motorB goes right
                else if (joystickX > 700 && !SwitchRight) {
                    directionX = 1;
                }
                // if joystick is untouched motor B stops moving
                else {
                    directionX = 0;
                }
                // if recieved data in variable y is true, determines start
                // position of motor y and moves motor y up until pickupDistance
                // is achieved. then motor stops and sends for the other arduino
                // to begin retracting motor
                // z
            }
            break;
        }
        case RESET: {
            if (slaveSignal == SLAVE_INITIAL &&
                masterSignal == MASTER_INITIAL) {
                moveToOrigin();
            }
            break;
        }
        case PICKUP: {
            if (masterSignal == MASTER_INITIAL &&
                slaveSignal == SLAVE_INITIAL) {
                masterSignal = MASTER_JOYSTICK_PRESSED;
                wireSendSignal();
            }
            if (slaveSignal == SLAVE_AT_END &&
                masterSignal != MASTER_MOVE_FINISHED) {
                if (a == 0) {
                    counterStart = counterY;
                    a++;
                }
                directionY = 1;
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
                currentRobotState = previousRobotState;
            }
            break;
        }
        case EMERGENCY: {
            // Emergency functionality
            Serial.println("Emergency Pressed");

            directionY = 0;
            directionX = 0;

            break;
        }
        default: {
            Serial.println("default");
        }
    }
}

// code to be executed on wire.onRecieve event
void receiveEvent(int bytes) {
    // read one character from the I2C
    slaveSignal = static_cast<SlaveSignals>(Wire.read());

    readJoystick = true;
}

// checks if joystick button is pressed, if true sends for the other arduino
// to begin pickup process set a to 0 to be able to determine starting
// position of motor y

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
            previousRobotState = currentRobotState;
            currentRobotState = PICKUP;
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
        analogWrite(speedPinA, 120);
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
    if (dir == -1) {
        digitalWrite(directionPinB, HIGH);
        digitalWrite(brakePinB, LOW);
        analogWrite(speedPinB, speed);
    } else if (dir == 1) {
        digitalWrite(directionPinB, LOW);
        digitalWrite(brakePinB, LOW);
        analogWrite(speedPinB, speed);
    } else {
        digitalWrite(brakePinB, HIGH);
        analogWrite(speedPinB, 0);
    }
}

// reads encoder from motor A and adds/ subtracts 1, based on direction,
// from counter everytime encoder pulses
void readEncoderA() {
    encoderAState = digitalRead(encoderA);

    if (encoderAState != aLastState) {
        counterY += directionY;
    }
    aLastState = encoderAState;
}

// reads encoder from motor B and adds/ subtracts 1, based on direction,
// from counter everytime encoder pulses
void readEncoderB() {
    encoderBState = digitalRead(encoderB);

    if (encoderBState != bLastState) {
        counterX += directionX;
    }
    bLastState = encoderBState;
}

void readSerial() {
    if (Serial.available() > 0) {
        currentRobotState = AUTOMATIC;
        String instructions =
            Serial.readStringUntil('\n');  // reads input from HMI

        if (instructions.charAt(0) == 'E') {  // Check for emergency signal
            emergency = !emergency;
            return;
        }

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

void moveToOrigin() {
    if (counterY > 0) {
        directionY = -1;
    } else if (counterY < 0) {
        directionY = 1;
    } else {
        directionY = 0;
    }

    if (counterX > 0) {
        directionX = -1;
    } else if (counterX < 0) {
        directionX = 1;
    } else {
        directionX = 0;
    }

    if (counterX == 0 && counterY == 0) {
        currentRobotState = JOYSTICK;
    }
}

void switchY1() {
    limitSwitch1.loop();

    // //Get state of limit switch on X-axis and do something
    int stateY1 = limitSwitch1.getState();
    if (stateY1 == LOW) {
        // Serial.println("unactivated");
        SwitchDown = false;

    } else {
        counterY = 0;
        SwitchDown = true;
        // Serial.println("activated.");
    }
}

void switchY2() {
    limitSwitch2.loop();

    // //Get state of limit switch on X-axis and do something
    int stateY2 = limitSwitch2.getState();
    if (stateY2 == LOW) {
        // Serial.println("unactivated");
        SwitchRight = false;

    } else {
        // Serial.println("activated.");
        counterX = 0;
        SwitchRight = true;
    }
}

void switchX2() {
    limitSwitch1.loop();

    // //Get state of limit switch on X-axis and do something
    int stateX2 = limitSwitch3.getState();
    if (stateX2 == LOW) {
        // Serial.println("unactivated");
        SwitchYup = false;

    } else {
        // Serial.println("The limit switch on X-Axis is: UNTOUCHED");
        SwitchYup = true;
    }
}
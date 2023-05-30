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

enum RobotState { AUTOMATIC, JOYSTICK, RESET, PICKUP, EMERGENCY };

RobotState currentRobotState = JOYSTICK;
RobotState previousRobotState = JOYSTICK;

bool pickupReset = false;

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
    if (emergency) {
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
            if (coordinateIndex > 2 || coordinates[coordinateIndex] == "") {
                coordinateIndex = 0;
                previousRobotState = currentRobotState;
                currentRobotState = RESET;
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
                joystickX = analogRead(VrxPin);
                joystickY = analogRead(VryPin);
                // if joystick is untouched motorA + B stop moving
                if (joystickX == 510) {
                    directionX = 0;
                }
                if (joystickY == 520) {
                    directionY = 0;
                }
                // Serial.println("STOP");
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

                // if recieved data in variable y is true, determines start
                // position of motor y and moves motor y up until pickupDistance
                // is achieved. then motor stops and sends for the other arduino
                // to begin retracting motor
                // z
            }

            break;
        }
        case RESET: {
            if (emergency) {
                currentRobotState = EMERGENCY;
                break;
            }
            if (slaveSignal == SLAVE_INITIAL &&
                masterSignal == MASTER_INITIAL) {
                moveToOrigin();
            }
            break;
        }
        case PICKUP: {
            if (emergency) {
                currentRobotState = EMERGENCY;
                break;
            }
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
            // functions default

            Serial.println("default");

            break;
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

// reads encoder from motor A and adds/ subtracts 1, based on direction,
// from counter everytime encoder pulses
void readEncoderA() {
    encoderAState = digitalRead(encoderA);

    if (encoderAState != aLastState) {
        counterY -= directionY;
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
        directionY = 1;
    } else if (counterY < 0) {
        directionY = -1;
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

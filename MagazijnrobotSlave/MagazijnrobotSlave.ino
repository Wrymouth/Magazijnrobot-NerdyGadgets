#define speedPin 11
#define directionPin 13
#define brakePin 8
#define encoder 2

#include <Wire.h>

// the end position of the motor on the z axes
const int end = 700;

// used for communication between arduinos
bool x = false;
bool y = true;

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

int direction = 0;

// speed of motor
int speed = 150;

// variables for reading encoder
int encoderState;
int LastState;
int counter;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    // writes PWM frequency to be used by motor
    TCCR2B = TCCR2B & B11111000 | B00000111;  // for PWM frequency of 30.64 Hz

    pinMode(directionPin, OUTPUT);
    pinMode(brakePin, OUTPUT);
    pinMode(speedPin, OUTPUT);
    pinMode(encoder, INPUT_PULLUP);

    // Starts connection to other arduino and recieves data on address 9
    Wire.begin(9);
    // Attach a function to trigger when something is received.
    Wire.onReceive(receiveEvent);

    // interrupts evrytime encoder A/B pulses and executes readEncoder functions
    // accordingly. this makes sure no pulse is missed
    attachInterrupt(digitalPinToInterrupt(encoder), readEncoder, RISING);
}

void receiveEvent(int bytes) {
    masterSignal = static_cast<MasterSignals>(Wire.read());
}

// reads encoder and adds/ subtracts 1, based on direction, from counter
// everytime encoder pulses
void readEncoder() {
    encoderState = digitalRead(encoder);

    if (encoderState != LastState) {
        if (direction == 1) {
            counter++;
        } else if (direction == -1) {
            counter--;
        }
    }
    LastState = encoderState;
    // Serial.println("counter: " + counter);
}

void loop() {
    // put your main code here, to run repeatedly:
    readEncoder();
    // moves z motor forward if joystick button is pressed
    if (masterSignal == MASTER_JOYSTICK_PRESSED && slaveSignal == SLAVE_INITIAL) {
        direction = 1;
        digitalWrite(brakePin, LOW);
        digitalWrite(directionPin, LOW);
        analogWrite(speedPin, speed);

        // if end is reached stop motor and send for the other arduino to move
        // up a bit
        if (counter == end) {
            direction = 0;
            digitalWrite(brakePin, HIGH);
            analogWrite(speedPin, 0);
            Wire.beginTransmission(8);  // transmit to device #9
            slaveSignal = SLAVE_AT_END;
            Wire.write(slaveSignal);           // sends true
            Wire.endTransmission();     // stop transmitting
        }
    }

    // if y axes has stopped moving to pickup an item, move motor backwards
    // until z motor is back at starting position
    if (masterSignal == MASTER_MOVE_FINISHED) {
        if (counter == 0) {
            direction = 0;
            digitalWrite(brakePin, HIGH);
            analogWrite(speedPin, 0);
            Wire.beginTransmission(8);  // transmit to device #9
            slaveSignal = SLAVE_AT_START;
            Wire.write(slaveSignal);           // sends false
            Wire.endTransmission();     // stop transmitting
        } else {
            direction = -1;
            digitalWrite(brakePin, LOW);
            digitalWrite(directionPin, HIGH);
            analogWrite(speedPin, speed);
        }
    }
    if (slaveSignal == SLAVE_AT_START) {
        slaveSignal = SLAVE_INITIAL;
        masterSignal = MASTER_INITIAL;
    }
}

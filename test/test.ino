#define LED_RED 2
#define LED_GREEN 3

// millis
unsigned long previousMillis = 0;
const int interval = 500;

// Blink
unsigned long previousMillisBlink = 0;
const int intervalBlink = 1000;

bool startBlinkGreen = false;
bool blinkGreen = false;

String test = "Hello, World";

void setup() {
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    readSerial();
    if (millis() - previousMillis > interval) {
        previousMillis = millis();
        Serial.println(test);
    }
    if (startBlinkGreen) {
        if (millis() - previousMillisBlink > intervalBlink) {
            previousMillisBlink = millis();
            blinkGreen = !blinkGreen;
            if (blinkGreen) {
                digitalWrite(LED_GREEN, HIGH);
            } else {
                digitalWrite(LED_GREEN, LOW);
            }
        }
    }
}

void readSerial() {
    if (Serial.available() > 0) {
        digitalWrite(LED_RED, HIGH);
        String inByte = Serial.readStringUntil('\n');
        test = inByte;
        if (inByte.charAt(0) == 'E') {
            startBlinkGreen = !startBlinkGreen;
        } else {
            startBlinkGreen = false;
            digitalWrite(LED_GREEN, HIGH);
        }
    } else {
        digitalWrite(LED_RED, LOW);
    }
}

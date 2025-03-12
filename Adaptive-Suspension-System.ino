#include <Wire.h>
#include <Servo.h>

#define HEIGHT_SENSOR A0
#define PRESSURE_SENSOR A1
#define BUTTON_PIN 2
#define RELAY_COILOVERS 3
#define RELAY_BYPASS 4
#define PNEUMATIC_ACTUATOR 5

Servo bypassServo;
int heightValue, pressureValue;
bool suspensionMode = false;

void setup() {
    Serial.begin(9600);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(RELAY_COILOVERS, OUTPUT);
    pinMode(RELAY_BYPASS, OUTPUT);
    pinMode(PNEUMATIC_ACTUATOR, OUTPUT);
    bypassServo.attach(6);
}

void loop() {
    heightValue = analogRead(HEIGHT_SENSOR);
    pressureValue = analogRead(PRESSURE_SENSOR);

    Serial.print("Height: ");
    Serial.print(heightValue);
    Serial.print(" | Pressure: ");
    Serial.println(pressureValue);

    if (digitalRead(BUTTON_PIN) == LOW) {
        suspensionMode = !suspensionMode;
        delay(500);
    }

    if (suspensionMode) {
        activateAdaptiveSuspension();
    } else {
        normalSuspension();
    }

    delay(100);
}

void activateAdaptiveSuspension() {
    digitalWrite(RELAY_COILOVERS, HIGH);
    digitalWrite(RELAY_BYPASS, HIGH);
    digitalWrite(PNEUMATIC_ACTUATOR, HIGH);
    bypassServo.write(90);
    Serial.println("Adaptive Suspension: ACTIVE");
}

void normalSuspension() {
    digitalWrite(RELAY_COILOVERS, LOW);
    digitalWrite(RELAY_BYPASS, LOW);
    digitalWrite(PNEUMATIC_ACTUATOR, LOW);
    bypassServo.write(0);
    Serial.println("Adaptive Suspension: NORMAL");
}

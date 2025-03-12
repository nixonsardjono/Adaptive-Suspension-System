#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Sensor and Actuator Pins
#define HEIGHT_SENSOR A0
#define PRESSURE_SENSOR A1
#define GYRO_MPU6050 A2

// Buttons
#define BUTTON_AUTO_HEIGHT 2
#define BUTTON_SOFT 3
#define BUTTON_NORMAL 4
#define BUTTON_HARD 5
#define BUTTON_RESET_DAMPING 6
#define BUTTON_RESET_HEIGHT 7
#define BUTTON_MANUAL_HEIGHT 8

// 3-Way Switch for Manual Height
#define SWITCH_HEIGHT_UP 9
#define SWITCH_HEIGHT_DOWN 10

// Actuators
#define AIR_TANK_ACTUATOR 11
#define AIR_RELEASE_VALVE 12
#define RELAY_COILOVERS 13
#define RELAY_BYPASS A3
#define PNEUMATIC_ACTUATOR A4

// LED Indicators
#define LED_GREEN A5
#define LED_YELLOW A6
#define LED_RED A7

Servo bypassServo;
Adafruit_MPU6050 mpu;
int heightValue, pressureValue;
bool systemOn = false;
bool manualHeightMode = false;
int suspensionMode = 1; // 0 = Soft, 1 = Normal, 2 = Stiff
bool autoHeightEnabled = false;
unsigned long inclineStartTime = 0; // To track incline duration

void setup() {
    Serial.begin(9600);
    Wire.begin();

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("MPU6050 initialization failed!");
        digitalWrite(LED_RED, HIGH);
        while (1);
    }

    // Button Inputs
    pinMode(BUTTON_AUTO_HEIGHT, INPUT_PULLUP);
    pinMode(BUTTON_SOFT, INPUT_PULLUP);
    pinMode(BUTTON_NORMAL, INPUT_PULLUP);
    pinMode(BUTTON_HARD, INPUT_PULLUP);
    pinMode(BUTTON_RESET_DAMPING, INPUT_PULLUP);
    pinMode(BUTTON_RESET_HEIGHT, INPUT_PULLUP);
    pinMode(BUTTON_MANUAL_HEIGHT, INPUT_PULLUP);

    // 3-Way Switch
    pinMode(SWITCH_HEIGHT_UP, INPUT_PULLUP);
    pinMode(SWITCH_HEIGHT_DOWN, INPUT_PULLUP);

    // Actuators & Relays
    pinMode(AIR_TANK_ACTUATOR, OUTPUT);
    pinMode(AIR_RELEASE_VALVE, OUTPUT);
    pinMode(RELAY_COILOVERS, OUTPUT);
    pinMode(RELAY_BYPASS, OUTPUT);
    pinMode(PNEUMATIC_ACTUATOR, OUTPUT);
    bypassServo.attach(A8);

    // LED Indicators
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_RED, OUTPUT);
}

void loop() {
    heightValue = analogRead(HEIGHT_SENSOR);
    pressureValue = analogRead(PRESSURE_SENSOR);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float pitch = a.acceleration.y;
    float roll = a.acceleration.x;

    Serial.print("Height: "); Serial.print(heightValue);
    Serial.print(" | Pressure: "); Serial.print(pressureValue);
    Serial.print(" | Pitch: "); Serial.print(pitch);
    Serial.print(" | Roll: "); Serial.println(roll);

    // Button Handlers
    if (digitalRead(BUTTON_AUTO_HEIGHT) == LOW) {
        autoHeightEnabled = !autoHeightEnabled;
        delay(500);
    }
    
    if (digitalRead(BUTTON_SOFT) == LOW) {
        setSuspensionMode(0);
    }
    if (digitalRead(BUTTON_NORMAL) == LOW) {
        setSuspensionMode(1);
    }
    if (digitalRead(BUTTON_HARD) == LOW) {
        setSuspensionMode(2);
    }
    if (digitalRead(BUTTON_RESET_DAMPING) == LOW) {
        resetSuspension();
    }
    if (digitalRead(BUTTON_RESET_HEIGHT) == LOW) {
        resetHeight();
    }
    if (digitalRead(BUTTON_MANUAL_HEIGHT) == LOW) {
        manualHeightMode = !manualHeightMode;
        autoHeightEnabled = false; // Auto Mode OFF when Manual is ON
        delay(500);
    }

    // Height Control
    if (manualHeightMode) {
        if (digitalRead(SWITCH_HEIGHT_UP) == LOW) {
            adjustHeight(true);
        } else if (digitalRead(SWITCH_HEIGHT_DOWN) == LOW) {
            adjustHeight(false);
        }
    } else if (autoHeightEnabled) {
        handleAutoHeight(pitch, roll);
    }

    delay(100);
}

void setSuspensionMode(int mode) {
    if (mode == 0) {
        // Soft Mode
        digitalWrite(AIR_TANK_ACTUATOR, HIGH);  // Send air to shocks
        digitalWrite(RELAY_COILOVERS, LOW);
        digitalWrite(RELAY_BYPASS, LOW);
        bypassServo.write(10);
        Serial.println("Soft Mode: 20 PSI | 1 Turn");
    } else if (mode == 1) {
        // Normal Mode
        digitalWrite(AIR_TANK_ACTUATOR, LOW);
        digitalWrite(RELAY_COILOVERS, HIGH);
        digitalWrite(RELAY_BYPASS, LOW);
        bypassServo.write(30);
        Serial.println("Normal Mode: 50 PSI | 3 Turns");
    } else if (mode == 2) {
        // Stiff Mode
        digitalWrite(AIR_TANK_ACTUATOR, HIGH);
        digitalWrite(AIR_RELEASE_VALVE, LOW);
        digitalWrite(RELAY_COILOVERS, HIGH);
        digitalWrite(RELAY_BYPASS, HIGH);
        bypassServo.write(50);
        Serial.println("Stiff Mode: 80 PSI | 5 Turns");
    }
}

void handleAutoHeight(float pitch, float roll) {
    if (abs(pitch) > 10) {
        if (inclineStartTime == 0) {
            inclineStartTime = millis();
        } else if (millis() - inclineStartTime > 4000) {
            Serial.println("Incline Detected for 4s! Adjusting Height...");
            if (pitch > 10) {
                adjustHeight(true); // Raise Front
            } else {
                adjustHeight(false); // Lower Front
            }
            inclineStartTime = 0;
        }
    } else {
        inclineStartTime = 0;
    }
}

void adjustHeight(bool increase) {
    if (increase) {
        digitalWrite(PNEUMATIC_ACTUATOR, HIGH);
        Serial.println("Height Increasing...");
    } else {
        digitalWrite(PNEUMATIC_ACTUATOR, LOW);
        Serial.println("Height Decreasing...");
    }
}

void resetSuspension() {
    digitalWrite(AIR_TANK_ACTUATOR, LOW);
    digitalWrite(AIR_RELEASE_VALVE, HIGH);
    Serial.println("Suspension Reset to Normal");
}

void resetHeight() {
    digitalWrite(PNEUMATIC_ACTUATOR, LOW);
    Serial.println("Height Reset to Normal");
}

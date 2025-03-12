#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>

// ** OLED Display Config **
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ** Sensors & Actuators **
Adafruit_MPU6050 mpu; // Gyroscope

#define HEIGHT_SENSOR A0
#define PRESSURE_SENSOR A1
#define BUTTON_MODE 2
#define BUTTON_MANUAL_UP 3
#define BUTTON_MANUAL_DOWN 4
#define BUTTON_STIFFNESS 5
#define BUTTON_CALIBRATE 6
#define RELAY_COILOVERS 7
#define RELAY_BYPASS 8
#define PNEUMATIC_ACTUATOR 9
#define SERVO_DAMPING 10
#define LED_STATUS_GREEN 11
#define LED_STATUS_YELLOW 12
#define LED_STATUS_RED 13

Servo bypassServo;
int heightValue, pressureValue;
bool adaptiveMode = false;
int suspensionMode = 0; // 0 = Normal, 1 = Adaptive, 2 = Off-Road, 3 = Manual, 4 = Calibration
float tiltAngle = 0, slopeAngle = 0;

// Pressure & Height Constants
const int MIN_PRESSURE = 200;  
const int MAX_PRESSURE = 800;  
const int TARGET_HEIGHT = 600;
const float MAX_TILT_ANGLE = 15.0;  // If tilt is more than 15°, auto-correct
const float MAX_SLOPE_ANGLE = 10.0; // Only adjust height if slope >10°

// PID Control Variables
float Kp = 2.5, Ki = 0.1, Kd = 1.2;
float prevError = 0, integral = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // ** Initialize Display **
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 initialization failed"));
    } else {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 10);
        display.println("Adaptive Suspension Ready");
        display.display();
    }

    // ** Initialize Gyroscope (MPU6050) **
    if (!mpu.begin()) {
        Serial.println("MPU6050 failed to initialize!");
        digitalWrite(LED_STATUS_RED, HIGH); // Indicate Error
        while (1);
    }

    // ** Pin Modes **
    pinMode(BUTTON_MODE, INPUT_PULLUP);
    pinMode(BUTTON_MANUAL_UP, INPUT_PULLUP);
    pinMode(BUTTON_MANUAL_DOWN, INPUT_PULLUP);
    pinMode(BUTTON_STIFFNESS, INPUT_PULLUP);
    pinMode(BUTTON_CALIBRATE, INPUT_PULLUP);
    
    pinMode(RELAY_COILOVERS, OUTPUT);
    pinMode(RELAY_BYPASS, OUTPUT);
    pinMode(PNEUMATIC_ACTUATOR, OUTPUT);
    
    pinMode(LED_STATUS_GREEN, OUTPUT);
    pinMode(LED_STATUS_YELLOW, OUTPUT);
    pinMode(LED_STATUS_RED, OUTPUT);

    bypassServo.attach(SERVO_DAMPING);
    digitalWrite(LED_STATUS_GREEN, LOW);
    digitalWrite(LED_STATUS_YELLOW, LOW);
    digitalWrite(LED_STATUS_RED, LOW);

    Serial.println("System Initialized.");
}

void loop() {
    // Read Sensors
    heightValue = analogRead(HEIGHT_SENSOR);
    pressureValue = analogRead(PRESSURE_SENSOR);

    // ** Read Gyroscope Data **
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    slopeAngle = atan2(a.acceleration.x, a.acceleration.z) * 180 / PI;
    tiltAngle = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;

    Serial.print("Height: ");
    Serial.print(heightValue);
    Serial.print(" | Pressure: ");
    Serial.print(pressureValue);
    Serial.print(" | Slope: ");
    Serial.print(slopeAngle);
    Serial.print(" | Tilt: ");
    Serial.println(tiltAngle);

    // ** Switch Mode on Button Press **
    if (digitalRead(BUTTON_MODE) == LOW) {
        suspensionMode = (suspensionMode + 1) % 5; // Cycle through 5 modes
        digitalWrite(LED_STATUS_GREEN, suspensionMode > 0 ? HIGH : LOW);
        displayMode();
        delay(500);  // Debounce
    }

    // ** Apply Selected Suspension Mode **
    if (suspensionMode == 1) adaptiveSuspension();
    else if (suspensionMode == 2) offRoadMode();
    else if (suspensionMode == 3) manualControl();
    else if (suspensionMode == 4) calibrateSystem();
    else normalSuspension();

    delay(100);
}

// ** Adaptive Suspension Mode **
void adaptiveSuspension() {
    // ** Auto Leveling on Slope **
    if (slopeAngle > MAX_SLOPE_ANGLE) {
        analogWrite(PNEUMATIC_ACTUATOR, 200); // Lift
    } else {
        analogWrite(PNEUMATIC_ACTUATOR, 0);
    }

    // ** Side Tilt Correction **
    if (tiltAngle > MAX_TILT_ANGLE) {
        digitalWrite(RELAY_BYPASS, HIGH); // Raise left side
    } else if (tiltAngle < -MAX_TILT_ANGLE) {
        digitalWrite(RELAY_COILOVERS, HIGH); // Raise right side
    }

    // ** PID-based height control **
    float pidOutput = pidControl(TARGET_HEIGHT, heightValue);
    analogWrite(PNEUMATIC_ACTUATOR, constrain(pidOutput, 0, 255));

    Serial.println("✅ Adaptive Suspension: ACTIVE");
}

// ** Off-Road Mode - Higher Suspension Height, Softer Ride **
void offRoadMode() {
    Serial.println("🚜 Switching to OFF-ROAD mode...");
    digitalWrite(RELAY_COILOVERS, LOW);
    digitalWrite(RELAY_BYPASS, HIGH);
    bypassServo.write(120);
    analogWrite(PNEUMATIC_ACTUATOR, 180);
}

// ** Manual Mode - User Controlled **
void manualControl() {
    if (digitalRead(BUTTON_MANUAL_UP) == LOW) {
        analogWrite(PNEUMATIC_ACTUATOR, 255); // Lift Up
    } 
    if (digitalRead(BUTTON_MANUAL_DOWN) == LOW) {
        analogWrite(PNEUMATIC_ACTUATOR, 0); // Lower Down
    }
    if (digitalRead(BUTTON_STIFFNESS) == LOW) {
        bypassServo.write(180); // Stiffer Suspension
    }
}

// ** Calibration Mode - Checks Pressure & Sensors **
void calibrateSystem() {
    digitalWrite(LED_STATUS_YELLOW, HIGH);
    Serial.println("⚙ Calibration Mode: Checking Sensors...");
    if (pressureValue < MIN_PRESSURE) {
        Serial.println("⚠ Pressure too LOW!");
    } else if (pressureValue > MAX_PRESSURE) {
        Serial.println("⚠ Pressure too HIGH!");
    }
    delay(3000);
    digitalWrite(LED_STATUS_YELLOW, LOW);
}

// ** Normal Suspension Mode **
void normalSuspension() {
    digitalWrite(RELAY_COILOVERS, LOW);
    digitalWrite(RELAY_BYPASS, LOW);
    digitalWrite(PNEUMATIC_ACTUATOR, LOW);
    bypassServo.write(0);
    Serial.println("🚗 Normal Suspension: ACTIVE");
}

// ** PID Control Function for Height Regulation **
float pidControl(float setpoint, float current) {
    float error = setpoint - current;
    integral += error;
    float derivative = error - prevError;
    prevError = error;
    return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

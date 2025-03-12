#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>

// Sensor dan Actuator
const int imuAddress = 0x68; // Alamat I2C IMU Sensor
Servo bypassShock; 
int pneumaticValve = 9; // Kontrol aktuator pneumatic

// PID Control untuk damping dan ride height
double Setpoint, Input, Output;
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
    Serial.begin(9600);
    Wire.begin();
    bypassShock.attach(6); // PWM pin untuk motorized bypass shock

    // Setup PID Controller
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 180); // Untuk kontrol servo damping
    pinMode(pneumaticValve, OUTPUT);
}

void loop() {
    // Baca data IMU sensor
    Wire.beginTransmission(imuAddress);
    Wire.write(0x3B);  
    Wire.endTransmission(false);
    Wire.requestFrom(imuAddress, 6, true);  

    int16_t accelX = Wire.read() << 8 | Wire.read();
    int16_t accelY = Wire.read() << 8 | Wire.read();
    int16_t accelZ = Wire.read() << 8 | Wire.read();

    // Konversi data IMU menjadi g-force
    float gForceX = accelX / 16384.0;
    float gForceY = accelY / 16384.0;
    float gForceZ = accelZ / 16384.0;

    // Hitung kemiringan kendaraan
    float tiltAngle = atan2(gForceY, gForceZ) * 180.0 / PI;

    // Setpoint untuk PID Controller
    Setpoint = 0; 
    Input = tiltAngle;
    myPID.Compute();

    // Kontrol bypass shock
    bypassShock.write(Output);

    // Kontrol suspensi pneumatic
    if (tiltAngle > 10) {
        digitalWrite(pneumaticValve, HIGH); // Naikkan suspensi
    } else {
        digitalWrite(pneumaticValve, LOW); // Turunkan suspensi
    }

    delay(100);
}

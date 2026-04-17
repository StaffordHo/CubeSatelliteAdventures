// THIS PROGRAM uses PID control to point towards the light using Sun Sensors,
// tracks 3D orientation via a BNO055 IMU, and forces a spin if shaken!

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Define pins
#define MOTOR_IN1 9   // Swapped to 9
#define MOTOR_IN2 10  // Swapped to 10
#define LEFT_SENSOR A2
#define BACK_SENSOR A3
#define RIGHT_SENSOR A1

// Shake Threshold
const int shakeThreshold = 10; 

// PID Tuning Parameters
float K_P = 1.5;  
float K_I = 0.01; 
float K_D = 0.5;  

// Initialize the BNO055 
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// ---------------------------------------------------------
// 1. Motor Struct
// ---------------------------------------------------------
struct Motor {
  int pin1;
  int pin2;

  void begin(int p1, int p2) {
    pin1 = p1;
    pin2 = p2;
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
  }

  void drive(int speed) {
    if (speed > 0) {
      analogWrite(pin1, speed);
      analogWrite(pin2, 0);
    } else if (speed < 0) {
      analogWrite(pin1, 0);
      // We use (speed * -1) because abs() can sometimes act weird with integers in Arduino
      analogWrite(pin2, (speed * -1)); 
    } else {
      stopMotor();
    }
  }

  void stopMotor() {
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
  }
};

// ---------------------------------------------------------
// 2. Sun Sensor Struct
// ---------------------------------------------------------
struct SunSensors {
  int leftPin, backPin, rightPin;
  int leftLight, backLight, rightLight;

  void begin(int l, int b, int r) {
    leftPin = l;
    backPin = b;
    rightPin = r;
  }

  void readAll() {
    leftLight = analogRead(leftPin);
    backLight = analogRead(backPin);
    rightLight = analogRead(rightPin);
  }
};

// ---------------------------------------------------------
// 3. PID Struct
// ---------------------------------------------------------
struct PIDController {
  float kp, ki, kd;
  float integral;
  float prevError;
  unsigned long lastTime;

  void begin(float p, float i, float d) {
    kp = p; ki = i; kd = d;
    integral = 0; prevError = 0;
    lastTime = millis();
  }

  int compute(float error) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0; 
    if (dt <= 0) return 0; 

    float pTerm = error * kp;
    integral += (error * dt);
    float iTerm = integral * ki;
    float dTerm = ((error - prevError) / dt) * kd;

    prevError = error;
    lastTime = now;

    float output = pTerm + iTerm + dTerm;

    if (output > 255) output = 255;
    if (output < -255) output = -255;
    return (int)output;
  }
  
  void reset() { integral = 0; }
};

// ---------------------------------------------------------
// 4. IMU Struct 
// ---------------------------------------------------------
struct IMUSensor {
  float yaw, pitch, roll;
  float accelX, accelY, accelZ;

  void begin() {
    if (!bno.begin()) {
      Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1); // Freezes here if IMU isn't found
    }
    delay(100);
  }

  void readAll() {
    // Read Orientation (yaw/pitch/roll)
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    yaw = orientationData.orientation.x;
    pitch = orientationData.orientation.y;
    roll = orientationData.orientation.z;

    // Read Linear Acceleration (ignores gravity so it rests perfectly at 0!)
    sensors_event_t accelData;
    bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    accelX = accelData.acceleration.x;
    accelY = accelData.acceleration.y;
    accelZ = accelData.acceleration.z;
  }

  // A custom function we can use to ask the IMU if it is currently shaking
  bool isShaking(float threshold) {
    if (abs(accelX) > threshold || abs(accelY) > threshold || abs(accelZ) > threshold) {
      return true;
    }
    return false;
  }
};

// =========================================================
// Create Global Instances
// =========================================================
Motor reactionWheel;
SunSensors sensors;
PIDController pid;
IMUSensor myIMU; 

void setup() {
  Serial.begin(9600); 
  
  reactionWheel.begin(MOTOR_IN1, MOTOR_IN2);
  sensors.begin(LEFT_SENSOR, BACK_SENSOR, RIGHT_SENSOR);
  pid.begin(K_P, K_I, K_D); 
  myIMU.begin(); 
}

void loop() {
  sensors.readAll();
  myIMU.readAll(); 

  // Priority 1: SHAKE OVERRIDE
  if (myIMU.isShaking(shakeThreshold)) {
    Serial.println(">> SHAKE DETECTED! Spinning motor... <<");
    reactionWheel.drive(127); // Spin clockwise
    delay(500); // Wait half a second
    reactionWheel.stopMotor();
    
    // CRITICAL: We must reset the PID memory. Otherwise, the 500ms delay causes 
    // the PID's "dt" math to spike and create unpredictable motor bursting!
    pid.reset(); 
  }
  
  // Priority 2: Back sensor logic
  else if (sensors.backLight > sensors.leftLight && sensors.backLight > sensors.rightLight) {
    reactionWheel.stopMotor();
    pid.reset(); 
    Serial.print("Motor: Stopped | ");
  } 
  
  // Priority 3: Normal PID tracking
  else {
    float error = sensors.rightLight - sensors.leftLight;
    int motorSpeed = pid.compute(error);
    reactionWheel.drive(motorSpeed);

    Serial.print("Error: "); Serial.print(error);
    Serial.print(" | PWM: "); Serial.print(motorSpeed);
    Serial.print(" | ");
  }

  // Print the IMU Orientation & Acceleration
  Serial.print("Yaw: "); Serial.print(myIMU.yaw);
  // Just picking the X axis to display so the screen doesn't get too cluttered
  Serial.print("\tAccel X: "); Serial.println(myIMU.accelX); 

  delay(50); 
}

// THIS PROGRAM uses PID control to point towards the light using Sun Sensors,
// tracks 3D orientation via a BNO055 IMU, forces a spin if shaken,
// and reports Temperature over SoftwareSerial when requested!

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>

// Define pins
#define MOTOR_IN1 9   
#define MOTOR_IN2 10  
#define LEFT_SENSOR A2
#define BACK_SENSOR A3
#define RIGHT_SENSOR A1
#define PIN_ADCSTEMP A6

// Shake Threshold
const int shakeThreshold = 10; 

// PID Tuning Parameters
float K_P = 1.5;  
float K_I = 0.01; 
float K_D = 0.5;  

// Hardware Initializations
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
SoftwareSerial OBCSerial(3, 2);

// ---------------------------------------------------------
// 1. Motor Struct
// ---------------------------------------------------------
struct Motor {
  int pin1, pin2;

  void begin(int p1, int p2) {
    pin1 = p1; pin2 = p2;
    pinMode(pin1, OUTPUT); pinMode(pin2, OUTPUT);
  }

  void drive(int speed) {
    if (speed > 0) {
      analogWrite(pin1, speed);
      analogWrite(pin2, 0);
    } else if (speed < 0) {
      analogWrite(pin1, 0);
      analogWrite(pin2, (speed * -1)); 
    } else {
      stopMotor();
    }
  }

  void stopMotor() {
    analogWrite(pin1, 0); analogWrite(pin2, 0);
  }
};

// ---------------------------------------------------------
// 2. Sun Sensor Struct
// ---------------------------------------------------------
struct SunSensors {
  int leftPin, backPin, rightPin;
  int leftLight, backLight, rightLight;

  void begin(int l, int b, int r) {
    leftPin = l; backPin = b; rightPin = r;
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
  float integral, prevError;
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
      while (1); 
    }
    delay(100);
  }

  void readAll() {
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    yaw = orientationData.orientation.x;
    pitch = orientationData.orientation.y;
    roll = orientationData.orientation.z;

    sensors_event_t accelData;
    bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    accelX = accelData.acceleration.x;
    accelY = accelData.acceleration.y;
    accelZ = accelData.acceleration.z;
  }

  bool isShaking(float threshold) {
    if (abs(accelX) > threshold || abs(accelY) > threshold || abs(accelZ) > threshold) {
      return true;
    }
    return false;
  }
};

// ---------------------------------------------------------
// 5. Temperature Sensor Struct 
// ---------------------------------------------------------
struct TemperatureSensor {
  int pin;
  
  void begin(int p) { pin = p; }
  
  float readCelsius() {
    float reading = analogRead(pin);
    float temp = (reading * (5100.00 / 1024.00)) / 10.00;
    return temp;
  }
};

// =========================================================
// Create Global Instances
// =========================================================
Motor reactionWheel;
SunSensors sensors;
PIDController pid;
IMUSensor myIMU; 
TemperatureSensor adcsTemp;

void setup() {
  Serial.begin(9600); 
  OBCSerial.begin(9600); // Start communication link with OBC
  
  reactionWheel.begin(MOTOR_IN1, MOTOR_IN2);
  sensors.begin(LEFT_SENSOR, BACK_SENSOR, RIGHT_SENSOR);
  pid.begin(K_P, K_I, K_D); 
  myIMU.begin(); 
  adcsTemp.begin(PIN_ADCSTEMP);
}

void loop() {
  
  // --- 1. HANDLE REQUESTS FROM THE OBC ---
  // This smoothly checks for a command without blocking the motor control
  if (OBCSerial.available() > 0) {
    String command = OBCSerial.readStringUntil('\n');
    command.trim(); // Remove invisible characters like \r
    
    if (command == "TEMPADCS") {
      float currentTemp = adcsTemp.readCelsius();
      
      // Reply to the OBC
      OBCSerial.print("TEMPADCS: ");
      OBCSerial.print(currentTemp);
      OBCSerial.println(" Celsius");

      // Print to your local PC screen just so you know it was triggered
      Serial.println(">> Responded to OBC with Temperature! <<");
    }
  }

  // --- 2. GATHER SENSOR DATA ---
  sensors.readAll();
  myIMU.readAll(); 

  // --- 3. EXECUTE CONTROL LOGIC ---
  
  // Priority 1: SHAKE OVERRIDE
  if (myIMU.isShaking(shakeThreshold)) {
    Serial.println(">> SHAKE DETECTED! Spinning motor... <<");
    reactionWheel.drive(127); 
    delay(500); 
    reactionWheel.stopMotor();
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

  // Debug Print the IMU Orientation & Acceleration
  Serial.print("Yaw: "); Serial.print(myIMU.yaw);
  Serial.print("\tAccel X: "); Serial.println(myIMU.accelX); 

  // Wait 50 milliseconds before the next loop
  delay(50); 
}

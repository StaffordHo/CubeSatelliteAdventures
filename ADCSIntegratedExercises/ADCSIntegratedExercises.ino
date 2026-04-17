#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>

// Pins
#define R_ADCS_T_OBC 3
#define T_ADCS_R_OBC 2
#define ENABLE_GPS 4
#define R_GPS_T_ADCS 7
#define T_GPS_R_ADCS 8

#define MOTOR_IN1 9
#define MOTOR_IN2 10
#define SUN_SENSOR_1 A0 
#define SUN_SENSOR_2 A1 
#define SUN_SENSOR_3 A2 
#define SUN_SENSOR_4 A3 
#define ADCS_TEMP A6

// Hardwares
SoftwareSerial GPSSerial(8,7);
SoftwareSerial OBCSerial(3,2);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_GPS GPS(&GPSSerial);

// System State
String secretKey = "STAFFORDPWROX";
uint8_t ADCSStatus = 0;
bool pidActive = false;
bool spinActive = false;

// ---------------------------------------------------------
// 1. MODULE STRUCTS (Highly Encapsulated Hardware Logic)
// ---------------------------------------------------------
struct Motor {
  int p1, p2;
  void begin(int pin1, int pin2) {
    p1 = pin1; p2 = pin2;
    pinMode(p1, OUTPUT); pinMode(p2, OUTPUT);
  }
  void drive(int speed) {
    // Arduino PWM is max 255. Do not use 1023!
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    if (speed > 0) {
      analogWrite(p1, speed);
      analogWrite(p2, 0);
    } else if (speed < 0) {
      analogWrite(p1, 0);
      analogWrite(p2, (speed * -1)); 
    } else { stopMotor(); }
  }
  void stopMotor() { analogWrite(p1, 0); analogWrite(p2, 0); }
};

struct PIDController {
  float kp = 1.5, ki = 0.01, kd = 0.5;
  float integral = 0, prevError = 0;
  unsigned long lastTime = 0;

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
    return (int)(pTerm + iTerm + dTerm);
  }
  void reset() { integral = 0; }
};

struct SunSensors {
  int s1, s2, s3, s4;
  void readAll() {
    s1 = analogRead(SUN_SENSOR_1);
    s2 = analogRead(SUN_SENSOR_2);
    s3 = analogRead(SUN_SENSOR_3);
    s4 = analogRead(SUN_SENSOR_4);
  }
};

struct IMUData {
  float yaw, pitch, roll, accelX, accelY, accelZ;
  bool isOnline = false;

  void begin() {
    if (bno.begin()) isOnline = true;
  }
  void readAll() {
    if (!isOnline) return;
    sensors_event_t oData, aData;
    bno.getEvent(&oData, Adafruit_BNO055::VECTOR_EULER);
    yaw = oData.orientation.x; pitch = oData.orientation.y; roll = oData.orientation.z;
    bno.getEvent(&aData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    accelX = aData.acceleration.x; accelY = aData.acceleration.y; accelZ = aData.acceleration.z;
  }
  bool isShaking(float threshold) {
    return (abs(accelX) > threshold || abs(accelY) > threshold || abs(accelZ) > threshold);
  }
};

struct TempSensor {
  float readCelsius() {
    return (analogRead(ADCS_TEMP) * (5100.00 / 1024.00)) / 10.00;
  }
};

// Global Instances
Motor reactionWheel;
PIDController pid;
SunSensors sun;
IMUData myIMU;
TempSensor adcsTemp;


// ---------------------------------------------------------
// 2. SETUP & MAIN LOOP
// ---------------------------------------------------------
void setup() {
  Serial.begin(9600);
  OBCSerial.begin(9600);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  reactionWheel.begin(MOTOR_IN1, MOTOR_IN2);
  myIMU.begin();

  Serial.println("ADCS Initialized & Waiting...");
}

void loop() {
  // --- 1. SENSOR POLLING ---
  sun.readAll();
  myIMU.readAll();

  // --- 2. COMMAND LISTENER ---
  OBCSerial.listen(); // Ensure we are listening to OBC by default!
  
  if (OBCSerial.available() > 0) {
    String content = OBCSerial.readStringUntil('\n');
    content.trim();
    
    // Strip security key if present
    if (content.substring(0, secretKey.length()) == secretKey) {
      content.remove(0, secretKey.length());
    }

    // Process State Changes
    if (content.equals("ENADCS")) { ADCSStatus = 1; }
    else if (content.equals("DISADCS")) { ADCSStatus = 0; } // Bonus: turn it back off!
    
    // Execute Commands Only if Enabled
    if (ADCSStatus == 1) {
      if (content.equals("ALL"))       { sendAll(); }
      else if (content.equals("IMU"))  { sendIMU(); }
      else if (content.equals("SUN"))  { sendSun(); }
      else if (content.equals("TEMPADCS")) { sendTemp(); }
      else if (content.equals("GPS"))  { sendGPS(); }  
      
      // Motor Toggles
      else if (content.equals("NIC")) { 
        spinActive = true; pidActive = false; // "Spin forever" mode
      }
      else if (content.equals("GYRO")) { 
        pidActive = !pidActive; spinActive = false; // Toggle PID tracking ON or OFF!
        if(!pidActive) reactionWheel.stopMotor();
      }
    } 
    else {
      OBCSerial.print(secretKey);
      OBCSerial.println("ADCS is disabled. Send ENADCS.");
    }
  }

  // --- 3. HARDWARE CONTROL EXECUTION ---
  if (myIMU.isShaking(10)) {
    // SHAKE OVERRIDE (Safety event)
    spinActive = false; pidActive = false; 
    reactionWheel.drive(127);
    delay(500); 
    reactionWheel.stopMotor();
    pid.reset();
  }
  else if (ADCSStatus == 1) {
    if (spinActive) {
      reactionWheel.drive(255); // Full power spin
    } 
    else if (pidActive) {
      // Classic PID Logic 
      if (sun.s3 > sun.s1 && sun.s3 > sun.s4) { // Modified based on your new sensor count!
         reactionWheel.stopMotor();
         pid.reset();
      } else {
         float error = sun.s4 - sun.s1; // Error calculates offset
         reactionWheel.drive( pid.compute(error) );
      }
    } else {
      reactionWheel.stopMotor(); // Idling
    }
  } else {
    // If Disabled, kill everything
    reactionWheel.stopMotor();
    pidActive = false; spinActive = false;
  }

  delay(20); // Fast 50Hz stabilization loop
}


// ---------------------------------------------------------
// 3. TELEMETRY SENDERS (Non-blocking format)
// ---------------------------------------------------------

void sendSun() {
  OBCSerial.print(secretKey); OBCSerial.print("SUN1:"); OBCSerial.print(sun.s1);
  OBCSerial.print(" SUN2:"); OBCSerial.print(sun.s2);
  OBCSerial.print(" SUN3:"); OBCSerial.print(sun.s3);
  OBCSerial.print(" SUN4:"); OBCSerial.println(sun.s4);
}

void sendIMU() {
  OBCSerial.print(secretKey); 
  OBCSerial.print("YAW:"); OBCSerial.print(myIMU.yaw);
  OBCSerial.print(" PCH:"); OBCSerial.print(myIMU.pitch);
  OBCSerial.print(" ROL:"); OBCSerial.println(myIMU.roll);
}

void sendTemp() {
  OBCSerial.print(secretKey); 
  OBCSerial.print("ADCS_TEMP:"); OBCSerial.print(adcsTemp.readCelsius());
  OBCSerial.println(" Celsius"); // Safely removed degree symbol!
}

void sendAll() {
  sendIMU();
  sendTemp();
  sendSun();
}

void sendGPS() {
  // CRITICAL FIX: The SoftwareSerial Limitation Escape-Hatch
  
  GPSSerial.listen(); // Switch our ears away from the OBC to listen to the GPS
  unsigned long timeout = millis();
  
  // Wait for GPS, but give up after 2000 milliseconds if it fails!
  while (!GPS.newNMEAreceived()) {
    char c = GPS.read();
    if (millis() - timeout > 2000) {
      OBCSerial.println(secretKey + "GPS_ERROR: NO_FIX_OR_TIMEOUT");
      OBCSerial.listen(); // VERY IMPORTANT: Re-connect to OBC!
      return;
    }
  }
  
  GPS.parse(GPS.lastNMEA());
  OBCSerial.listen(); // Re-connect our ears to the OBC
  
  // Send the data we just parsed
  OBCSerial.print(secretKey);
  OBCSerial.println(GPS.lastNMEA());  
}

// THIS PROGRAM uses PID control to point towards the light.
// It tries to balance the left and right sensors.
// If the back sensor has the most light, it stops the motor.

// Define pins
#define MOTOR_IN1 9
#define MOTOR_IN2 10
#define LEFT_SENSOR A2
#define BACK_SENSOR A3
#define RIGHT_SENSOR A1

// PID Tuning Parameters (YOU MUST TUNE THESE FOR YOUR CUBESAT!)
float K_P = 1.5;  // Proportional: How strongly it reacts to being off-center
float K_I = 0.01; // Integral: Helps overcome friction over time
float K_D = 0.5;  // Derivative: Prevents overshooting/oscillations

// ---------------------------------------------------------
// 1. Define a Struct for the Motor control
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

  // Drive takes a speed from -255 to 255
  void drive(int speed) {
    if (speed > 0) {
      analogWrite(pin1, speed);
      analogWrite(pin2, 0);
    } else if (speed < 0) {
      analogWrite(pin1, 0);
      analogWrite(pin2, abs(speed));
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
// 2. Define a Struct to hold all Sun Sensor data
// ---------------------------------------------------------
struct SunSensors {
  int leftPin;
  int backPin;
  int rightPin;
  
  int leftLight;
  int backLight;
  int rightLight;

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
// 3. Define a Struct for PID Control
// ---------------------------------------------------------
struct PIDController {
  float kp, ki, kd;
  float integral;
  float prevError;
  unsigned long lastTime;

  void begin(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
    integral = 0;
    prevError = 0;
    lastTime = millis();
  }

  // Calculate the motor PWM (-255 to 255) based on the error
  int compute(float error) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0; // Time step in seconds
    if (dt <= 0) return 0; // Prevent division by zero

    // Proportional Term
    float pTerm = error * kp;
    
    // Integral Term (with basic anti-windup bounds)
    integral += (error * dt);
    float iTerm = integral * ki;
    
    // Derivative Term
    float dTerm = ((error - prevError) / dt) * kd;

    // Save for next loop
    prevError = error;
    lastTime = now;

    // Sum it all up
    float output = pTerm + iTerm + dTerm;

    // Constrain to motor PWM limits
    if (output > 255) output = 255;
    if (output < -255) output = -255;

    return (int)output;
  }
  
  // Resets the integral build-up when stopped
  void reset() {
    integral = 0;
  }
};

// =========================================================
// Create Global Instances
// =========================================================
Motor reactionWheel;
SunSensors sensors;
PIDController pid;

void setup() {
  Serial.begin(9600);
  
  reactionWheel.begin(MOTOR_IN1, MOTOR_IN2);
  sensors.begin(LEFT_SENSOR, BACK_SENSOR, RIGHT_SENSOR);
  pid.begin(K_P, K_I, K_D); // Send tuning parameters to the PID controller
}

void loop() {
  sensors.readAll();

  // Condition 1: The back sensor is facing the light. 
  // We maintain your original logic to stop the motor in this case.
  if (sensors.backLight > sensors.leftLight && sensors.backLight > sensors.rightLight) {
    reactionWheel.stopMotor();
    pid.reset(); // Reset PID memory so it doesn't build up while stopped
    Serial.println("Back sensor brightest. Motor stopped.");
  } 
  
  // Condition 2: The light is on the front sensors. Let PID aggressively track it.
  else {
    // The Error is the difference. If it's perfectly centered, error = 0.
    // If right is brighter, error is positive (spins one way).
    // If left is brighter, error is negative (spins the other way).
    float error = sensors.rightLight - sensors.leftLight;

    // Pass the error into the math algorithm
    int motorSpeed = pid.compute(error);

    // Apply the resulting speed to the hardware
    reactionWheel.drive(motorSpeed);

    // Print for debugging
    Serial.print("Error: "); Serial.print(error);
    Serial.print(" | PID PWM Output: "); Serial.println(motorSpeed);
  }

  // VERY IMPORTANT: PID needs fast feedback to adjust calculations!
  // We cannot use delay(5000) anymore. We run it rapidly 20 times a second.
  delay(50); 
}

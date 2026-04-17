#include <Servo.h>

#define TRIG_PIN 7
#define ECHO_PIN 6
#define SERVO_PIN 9

Servo servo;
const int thresholdDistance = 15; // Stop if object is within 15cm
bool obstacleDetected = false; // Track activation state

void setup() {
    SerialUSB.begin(115200);  // USB Monitor
    Serial1.begin(115200);    // UART to TurtleBot3

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    servo.detach(); // Keep the servo motor OFF initially

    SerialUSB.println("System Initialized. Sending MOVE command...");
    Serial1.println("MOVE");  // Send MOVE command at startup
    Serial1.flush();
}

long getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout at 30ms
    if (duration == 0) {
        return -1; // No echo detected
    }
    
    long distance = duration * 0.034 / 2; // Convert to cm
    return distance;
}

void stopTurtleBot() {
    SerialUSB.println("🚨 Obstacle detected! Stopping TurtleBot3.");
    Serial1.println("STOP"); // Send STOP command
    Serial1.flush();
}

void moveTurtleBot() {
    SerialUSB.println("✅ Obstacle cleared. Resuming movement.");
    Serial1.println("MOVE"); // Send MOVE command
    Serial1.flush();
}

void activateServo() {
    if (!servo.attached()) {
        SerialUSB.println("🔄 Activating Servo...");
        servo.attach(SERVO_PIN); // Attach servo only when needed
    }
    servo.write(90); // Move servo to 90 degrees
    delay(500);
    servo.write(0); // Return to 0 degrees
    delay(500);
}

void deactivateServo() {
    if (servo.attached()) {
        SerialUSB.println("🔌 Deactivating Servo...");
        servo.detach(); // Save power and avoid jittering
    }
}

void loop() {
    long distance = getDistance();

    SerialUSB.print("📏 Distance: ");
    SerialUSB.print(distance);
    SerialUSB.println(" cm");

    // If object is within the threshold distance
    if (distance > 0 && distance < thresholdDistance) {
        if (!obstacleDetected) {
            stopTurtleBot(); // Stop TurtleBot3 once on first detection
            activateServo(); // Activate the servo
            obstacleDetected = true;
        } else {
            // Keep running the servo if the obstacle remains
            activateServo();
        }
    } else {
        if (obstacleDetected) {
            moveTurtleBot(); // Resume TurtleBot3 movement
            deactivateServo(); // Turn off the servo motor
            obstacleDetected = false;
        }
    }

    delay(100); // Reduced delay for faster reaction time
}

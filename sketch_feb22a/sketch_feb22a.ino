#include <Servo.h>

#define TRIG_PIN 7
#define ECHO_PIN 6
#define SERVO_PIN 9

Servo servo;
const int thresholdDistance = 15; // Stop if object is within 15cm
bool obstacleDetected = false; // Track activation state

void setup() {
    SerialUSB.begin(115200);  // Native USB Serial Monitor
    Serial1.begin(115200);    // UART for TurtleBot3 communication

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    servo.detach(); // Keep the servo motor OFF initially

    SerialUSB.println("System Initialized. Waiting for obstacle detection...");
}

long getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout at 30ms (prevents false readings)
    
    if (duration == 0) {
        SerialUSB.println("⚠️ Warning: No echo detected, assuming no obstacle.");
        return -1; // Return -1 to indicate no valid reading
    }
    
    long distance = duration * 0.034 / 2; // Convert to cm
    return distance;
}

void stopTurtleBot() {
    SerialUSB.println("🚨 Obstacle detected! Stopping TurtleBot3.");
    Serial1.println("STOP"); // Send stop command to TurtleBot3
}

void moveTurtleBot() {
    SerialUSB.println("✅ Obstacle cleared. Resuming movement.");
    Serial1.println("MOVE"); // Send move command to TurtleBot3
}

void activateServo() {
    SerialUSB.println("🔄 Activating Servo...");
    servo.attach(SERVO_PIN); // Attach servo only when needed
    servo.write(90); // Move servo to 90 degrees
    delay(500);
    servo.write(0); // Move back to 0 degrees
    delay(500);
}

void loop() {
    long distance = getDistance();
    
    if (distance == -1) { // No echo detected, assume no obstacle
        SerialUSB.println("🚀 No valid distance detected. TurtleBot continues moving.");
        moveTurtleBot();  // Ensure TurtleBot3 keeps moving
        obstacleDetected = false; // Reset detection to prevent false activation
        servo.detach(); // Keep the servo OFF when no obstacle is detected
        delay(500);
        return;
    }

    SerialUSB.print("📏 Distance: ");
    SerialUSB.print(distance);
    SerialUSB.println(" cm");

    if (distance > 0 && distance < thresholdDistance) {
        if (!obstacleDetected) { // Only stop once when first detecting the obstacle
            stopTurtleBot();
            servo.attach(SERVO_PIN); // Attach the servo when obstacle is detected
            obstacleDetected = true; // Mark that the obstacle has been handled
        }
        activateServo(); // Keep servo running while obstacle remains
    } else {
        if (obstacleDetected) {
            SerialUSB.println("✅ Obstacle removed. Ready for next detection.");
        }
        obstacleDetected = false; // Reset when object is gone
        servo.detach(); // Stop the servo when obstacle is removed
    }

    delay(500); // Short delay for stability
}

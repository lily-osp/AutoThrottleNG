/**
 * @file servo_control.ino
 * @brief Real-world example: Servo Position Control with Feedback
 *
 * This example demonstrates precise servo position control using AutoThrottleNG
 * with potentiometer feedback for closed-loop control. It includes:
 * - Potentiometer-based position feedback
 * - PID-based position control
 * - Input filtering for noisy position signals
 * - Output smoothing to prevent servo jitter
 * - Failsafe mechanisms for position safety
 *
 * Hardware Requirements:
 * - Arduino board (Uno, Mega, etc.)
 * - Servo motor (standard RC servo or continuous rotation servo)
 * - Potentiometer for position feedback
 * - Servo power supply (separate from Arduino if using multiple servos)
 *
 * Connections:
 * - Servo signal: Digital pin 9 (PWM capable)
 * - Position feedback potentiometer: Analog pin A0
 * - Servo power: 5V/GND (or external supply)
 */

#include <Arduino.h>
#include <Servo.h>
#include <AutoThrottleNG.h>
#include <PID_v1.h>

// --- Hardware Configuration ---
const uint8_t SERVO_PIN = 9;           // Servo signal pin
const uint8_t POSITION_SENSOR_PIN = A0; // Analog pin for position feedback

// --- Servo Configuration ---
const int SERVO_MIN_ANGLE = 0;         // Minimum servo angle (degrees)
const int SERVO_MAX_ANGLE = 180;       // Maximum servo angle (degrees)
const int SERVO_MIN_PULSE = 544;       // Minimum pulse width (microseconds)
const int SERVO_MAX_PULSE = 2400;      // Maximum pulse width (microseconds)

// --- Position Sensor Configuration ---
const double POSITION_MIN = 0.0;       // Minimum position reading (degrees)
const double POSITION_MAX = 180.0;     // Maximum position reading (degrees)

// --- Control Configuration ---
const double TARGET_POSITION = 90.0;   // Target position (degrees)
const double POSITION_TOLERANCE = 2.0; // Acceptable position error (degrees)

// PID tuning values (adjust based on your servo and feedback system)
double Kp = 1.2, Ki = 0.05, Kd = 0.8;

// --- AutoThrottleNG Configuration ---
AutoThrottleNG servoController(POSITION_MIN, POSITION_MAX, Kp, Ki, Kd, P_ON_E, DIRECT);

// --- Global Variables ---
Servo servoMotor;                      // Servo object
double currentPosition = 0.0;          // Current position reading (degrees)
unsigned long lastPositionUpdate = 0;  // Last position measurement time

/**
 * @brief Read position from potentiometer
 * @return Position in degrees (0-180)
 */
double readPosition() {
    // Read analog value and convert to degrees
    int adcValue = analogRead(POSITION_SENSOR_PIN);
    double position = map(adcValue, 0, 1023, POSITION_MIN, POSITION_MAX);

    // Ensure valid range
    return constrain(position, POSITION_MIN, POSITION_MAX);
}

/**
 * @brief Set servo position
 * @param angle Target angle in degrees (0-180)
 */
void setServoPosition(double angle) {
    // Constrain angle to servo limits
    angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    // Set servo position
    servoMotor.write((int)angle);
}

/**
 * @brief Check if servo is at target position within tolerance
 * @param target Target position
 * @param current Current position
 * @param tolerance Acceptable tolerance
 * @return true if within tolerance
 */
bool isAtPosition(double target, double current, double tolerance) {
    return abs(target - current) <= tolerance;
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("AutoThrottleNG - Servo Position Control");
    Serial.println("=====================================");

    // Configure servo
    servoMotor.attach(SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

    // Move to center position initially
    setServoPosition(90);
    delay(1000);  // Allow servo to reach position

    // Configure AutoThrottleNG
    servoController.setTarget(TARGET_POSITION);
    servoController.setSampleTime(50);  // Fast response for position control

    // Enable filtering and smoothing
    servoController.setInputFilterAlpha(0.4);  // Moderate filtering for position noise
    servoController.enableSmoothing(true, 90.0); // Smooth movements (90°/sec max)

    // Configure failsafes
    servoController.setFailsafeValue(90.0);    // Center position on failsafe
    servoController.setInputTimeout(1000);     // 1 second sensor timeout
    servoController.setStabilityParams(POSITION_TOLERANCE, 5000); // Position tolerance, 5 second timeout

    Serial.println("Configuration:");
    Serial.print("Target Position: "); Serial.print(TARGET_POSITION); Serial.println("°");
    Serial.print("Position Range: "); Serial.print(POSITION_MIN);
    Serial.print(" - "); Serial.print(POSITION_MAX); Serial.println("°");
    Serial.print("PID Gains: Kp="); Serial.print(Kp);
    Serial.print(", Ki="); Serial.print(Ki);
    Serial.print(", Kd="); Serial.println(Kd);
    Serial.println("Input Filter: Alpha=0.4");
    Serial.println("Output Smoothing: 90°/sec max rate");
    Serial.println("Failsafe: Center position (90°)");
    Serial.println();
    Serial.println("Time | Target | Current | Output | Error | At Target");
    Serial.println("--------------------------------------------------");
}

void loop() {
    unsigned long currentTime = millis();

    // Read position every 20ms
    static unsigned long lastReadTime = 0;
    if (currentTime - lastReadTime >= 20) {
        currentPosition = readPosition();
        lastReadTime = currentTime;

        // Update controller with current position
        servoController.updateInput(currentPosition);
    }

    // Compute servo control output
    double servoCommand = servoController.compute();

    // Apply servo command
    setServoPosition(servoCommand);

    // Print status every 200ms
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= 200) {
        Serial.print(currentTime / 1000.0, 1);
        Serial.print(" | ");
        Serial.print(servoController.getSetpoint(), 1);
        Serial.print(" | ");
        Serial.print(currentPosition, 1);
        Serial.print(" | ");
        Serial.print(servoCommand, 1);
        Serial.print(" | ");

        // Show error state
        AutoThrottleNG::Error err = servoController.getErrorState();
        if (err == AutoThrottleNG::Error::NONE) {
            Serial.print("OK");
        } else {
            Serial.print("ERROR");
        }
        Serial.print(" | ");

        // Show if at target
        Serial.print(isAtPosition(TARGET_POSITION, currentPosition, POSITION_TOLERANCE) ? "YES" : "NO");
        Serial.println();

        lastPrintTime = currentTime;
    }

    // Dynamic target changes for demonstration
    static int targetIndex = 0;
    static double targets[] = {45.0, 135.0, 0.0, 180.0, 90.0};
    static unsigned long lastTargetChange = 0;

    if (currentTime - lastTargetChange > 8000) {  // Change target every 8 seconds
        targetIndex = (targetIndex + 1) % 5;
        double newTarget = targets[targetIndex];
        servoController.setTarget(newTarget);
        Serial.print("*** New Target: "); Serial.print(newTarget); Serial.println("° ***");
        lastTargetChange = currentTime;
    }

    // Handle failsafe recovery
    if (servoController.isInErrorState()) {
        static unsigned long errorStartTime = 0;
        if (errorStartTime == 0) {
            errorStartTime = currentTime;
            Serial.println("*** SERVO FAILSAFE ACTIVE ***");
        }

        // Attempt recovery after 5 seconds
        if (currentTime - errorStartTime > 5000) {
            Serial.println("*** ATTEMPTING SERVO RECOVERY ***");
            servoController.clearErrorState();
            errorStartTime = 0;
        }
    }

    // Handle servo overload protection
    static unsigned long lastMovementTime = 0;
    if (abs(servoCommand - currentPosition) > 5.0) {
        lastMovementTime = currentTime;
    }

    // If servo hasn't moved for too long while receiving commands, it might be stuck
    if (currentTime - lastMovementTime > 10000 && !isAtPosition(servoCommand, currentPosition, 10.0)) {
        Serial.println("*** SERVO STUCK DETECTED - RESETTING ***");
        servoMotor.detach();
        delay(100);
        servoMotor.attach(SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
        setServoPosition(90);  // Reset to center
        delay(500);
        lastMovementTime = currentTime;
    }

    // Small delay for stability
    delay(10);
}

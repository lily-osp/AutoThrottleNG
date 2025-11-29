/**
 * @file motor_control.ino
 * @brief Real-world example: DC Motor Speed Control with Encoder Feedback
 *
 * This example demonstrates controlling a DC motor's speed using AutoThrottleNG
 * with encoder feedback for closed-loop control. It includes:
 * - Encoder-based speed measurement
 * - PID-based speed control
 * - Input filtering for noisy encoder signals
 * - Output smoothing to prevent motor jerking
 * - Failsafe mechanisms
 *
 * Hardware Requirements:
 * - Arduino board (Uno, Mega, etc.)
 * - DC motor with encoder
 * - Motor driver (L298N, TB6612, etc.)
 * - Encoder connected to interrupt pins
 *
 * Connections:
 * - Motor driver IN1, IN2: Digital pins 9, 10
 * - Motor driver PWM: Digital pin 11
 * - Encoder A, B: Digital pins 2, 3 (interrupt capable)
 */

#include <Arduino.h>
#include <AutoThrottleNG.h>
#include <PID_v1.h>

// --- Hardware Configuration ---
const uint8_t MOTOR_IN1 = 9;      // Motor driver input 1
const uint8_t MOTOR_IN2 = 10;     // Motor driver input 2
const uint8_t MOTOR_PWM = 11;     // Motor driver PWM pin
const uint8_t ENCODER_A = 2;      // Encoder channel A (interrupt pin)
const uint8_t ENCODER_B = 3;      // Encoder channel B (interrupt pin)

// --- Motor & Encoder Configuration ---
const float COUNTS_PER_REV = 360.0;    // Encoder counts per revolution
const float GEAR_RATIO = 1.0;          // Gear reduction ratio
const unsigned long SAMPLE_TIME_MS = 50; // PID sample time

// --- Control Configuration ---
const double MIN_SPEED = 0.0;          // Minimum speed (RPM)
const double MAX_SPEED = 100.0;        // Maximum speed (RPM)
const double TARGET_SPEED = 60.0;      // Target speed (RPM)

// PID tuning values (adjust based on your motor)
double Kp = 0.8, Ki = 0.2, Kd = 0.05;

// --- AutoThrottleNG Configuration ---
AutoThrottleNG motorController(MIN_SPEED, MAX_SPEED, Kp, Ki, Kd, P_ON_E, DIRECT);

// --- Global Variables ---
volatile long encoderCount = 0;        // Encoder pulse count
unsigned long lastEncoderTime = 0;     // Last encoder measurement time
double currentSpeedRPM = 0.0;          // Current motor speed in RPM
unsigned long lastSpeedUpdate = 0;     // Last speed calculation time

/**
 * @brief Encoder interrupt service routine
 * Handles encoder pulses and maintains direction awareness
 */
void encoderISR() {
    // Read both channels to determine direction
    static bool lastA = false;
    bool currentA = digitalRead(ENCODER_A);
    bool currentB = digitalRead(ENCODER_B);

    // Simple quadrature decoding
    if (currentA != lastA) {
        if (currentB != currentA) {
            encoderCount++;
        } else {
            encoderCount--;
        }
    }
    lastA = currentA;
}

/**
 * @brief Set motor direction and speed
 * @param speed Speed value (-255 to 255, negative for reverse)
 */
void setMotorSpeed(double speed) {
    // Constrain speed to safe PWM range
    speed = constrain(speed, -255.0, 255.0);

    if (speed >= 0) {
        // Forward direction
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_PWM, (int)speed);
    } else {
        // Reverse direction
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        analogWrite(MOTOR_PWM, (int)-speed);
    }
}

/**
 * @brief Calculate current motor speed from encoder
 * @return Speed in RPM
 */
double calculateSpeedRPM() {
    static long lastEncoderCount = 0;
    unsigned long currentTime = millis();

    // Calculate time delta
    unsigned long timeDelta = currentTime - lastSpeedUpdate;
    if (timeDelta == 0) return currentSpeedRPM;

    // Calculate encoder delta
    long encoderDelta = encoderCount - lastEncoderCount;

    // Calculate RPM
    // RPM = (counts/sec) / (counts/rev) * (60 sec/min) / gear_ratio
    double countsPerSecond = (double)encoderDelta / (timeDelta / 1000.0);
    double rpm = (countsPerSecond / COUNTS_PER_REV) * 60.0 / GEAR_RATIO;

    // Update tracking variables
    lastEncoderCount = encoderCount;
    lastSpeedUpdate = currentTime;
    currentSpeedRPM = abs(rpm); // Store absolute speed

    return currentSpeedRPM;
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("AutoThrottleNG - DC Motor Speed Control");
    Serial.println("=====================================");

    // Configure motor control pins
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_PWM, OUTPUT);

    // Configure encoder pins
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    // Attach encoder interrupt
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

    // Stop motor initially
    setMotorSpeed(0);

    // Configure AutoThrottleNG
    motorController.setTarget(TARGET_SPEED);
    motorController.setSampleTime(SAMPLE_TIME_MS);

    // Enable filtering and smoothing for stable control
    motorController.setInputFilterAlpha(0.3);  // Moderate input filtering
    motorController.enableSmoothing(true, 50.0); // Smooth output changes

    // Configure failsafes
    motorController.setFailsafeValue(0.0);     // Stop motor on failsafe
    motorController.setInputTimeout(1000);     // 1 second input timeout
    motorController.setStabilityParams(5.0, 3000); // 5 RPM tolerance, 3 second timeout

    // Initialize timing
    lastSpeedUpdate = millis();

    Serial.println("Configuration:");
    Serial.print("Target Speed: "); Serial.print(TARGET_SPEED); Serial.println(" RPM");
    Serial.print("PID Gains: Kp="); Serial.print(Kp);
    Serial.print(", Ki="); Serial.print(Ki);
    Serial.print(", Kd="); Serial.println(Kd);
    Serial.println("Input Filter: Alpha=0.3");
    Serial.println("Output Smoothing: 50 RPM/sec max rate");
    Serial.println("Failsafe: Stop motor on error");
    Serial.println();
    Serial.println("Time | Target | Speed | Output | Error");
    Serial.println("-------------------------------------");
}

void loop() {
    unsigned long currentTime = millis();

    // Calculate current speed every 100ms
    static unsigned long lastCalcTime = 0;
    if (currentTime - lastCalcTime >= 100) {
        currentSpeedRPM = calculateSpeedRPM();
        lastCalcTime = currentTime;

        // Update controller with current speed
        motorController.updateInput(currentSpeedRPM);
    }

    // Compute motor control output
    double motorOutput = motorController.compute();

    // Apply motor output (scale from RPM to PWM)
    // Assuming linear relationship - adjust scaling factor based on your motor
    double pwmValue = map(motorOutput, MIN_SPEED, MAX_SPEED, 0, 255);
    setMotorSpeed(pwmValue);

    // Print status every 500ms
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= 500) {
        Serial.print(currentTime / 1000.0, 1);
        Serial.print(" | ");
        Serial.print(motorController.getSetpoint(), 1);
        Serial.print(" | ");
        Serial.print(currentSpeedRPM, 1);
        Serial.print(" | ");
        Serial.print(motorOutput, 1);
        Serial.print(" | ");

        // Show error state
        AutoThrottleNG::Error err = motorController.getErrorState();
        if (err == AutoThrottleNG::Error::NONE) {
            Serial.print("OK");
        } else {
            Serial.print("ERROR");
        }
        Serial.println();

        lastPrintTime = currentTime;
    }

    // Handle failsafe recovery
    if (motorController.isInErrorState()) {
        // In a real application, you might want to:
        // - Log the error
        // - Attempt recovery procedures
        // - Alert the user/operator
        // - Emergency stop other systems

        // For this example, we automatically attempt recovery after 5 seconds
        static unsigned long errorStartTime = 0;
        if (errorStartTime == 0) {
            errorStartTime = currentTime;
            Serial.println("*** MOTOR FAILSAFE ACTIVE ***");
        }

        if (currentTime - errorStartTime > 5000) {
            // Attempt recovery
            Serial.println("*** ATTEMPTING MOTOR RECOVERY ***");
            motorController.clearErrorState();
            errorStartTime = 0;
        }
    }

    // Allow some time for other processes
    delay(10);
}

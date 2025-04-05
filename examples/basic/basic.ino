/**
 * @file 1_BasicControlNG.ino
 * @brief Basic demo for AutoThrottleNG library.
 */
#include <Arduino.h>
#include <AutoThrottleNG.h> // Include the new library
#include <PID_v1.h> // Include standard PID library

// --- Config ---
const double MIN_OUTPUT = 0.0;
const double MAX_OUTPUT = 255.0;
double Kp = 2.0, Ki = 0.5, Kd = 0.1;
int direction = DIRECT; // Use global constant

// --- Create Instance ---
AutoThrottleNG throttle(MIN_OUTPUT, MAX_OUTPUT, Kp, Ki, Kd, P_ON_E, direction);

// --- Simulation ---
double currentSpeed = 0.0;
double targetSpeed = 100.0;
double simMotorEffect = 0.0;

void setup()
{
    Serial.begin(115200);
    Serial.println("AutoThrottleNG - Basic Demo");
    throttle.setTarget(targetSpeed);
    throttle.setSampleTime(100); // Optional: Set PID sample time
    Serial.println("Target | Input | Output | Mode");
    Serial.println("-------------------------------");
}

void loop()
{
    // 1. Simulate Sensor Reading
    double effectiveness = 0.005;
    double drag = 0.01;
    currentSpeed += (simMotorEffect * effectiveness) - (currentSpeed * drag);
    currentSpeed = max(0.0, currentSpeed);

    // 2. Update Controller
    throttle.updateInput(currentSpeed);

    // 3. Compute Output
    double output = throttle.compute();

    // 4. Apply Output (Simulation)
    simMotorEffect = output;
    // applyToMotor(output); // Real hardware

    // 5. Print Status
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 500) {
        Serial.print(throttle.getSetpoint(), 0);
        Serial.print("    | ");
        Serial.print(throttle.getFilteredInput(), 1);
        Serial.print("  | "); // Show filtered input
        Serial.print(output, 1);
        Serial.print("   | ");
        Serial.print(throttle.getMode() == AUTOMATIC ? "AUTO" : "MANUAL");
        Serial.println();
        lastPrint = millis();
    }
    delay(10);
}

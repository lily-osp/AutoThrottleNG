/**
 * @file 2_SmoothingFilteringNG.ino
 * @brief Demo for AutoThrottleNG smoothing and input filtering.
 */
#include <Arduino.h>
#include <AutoThrottleNG.h>
#include <PID_v1.h>

// --- Config ---
const double MIN_OUTPUT = 0.0;
const double MAX_OUTPUT = 100.0; // Percentage
double Kp = 3.0, Ki = 0.7, Kd = 0.15;
int direction = DIRECT;

// --- Create Instance ---
AutoThrottleNG throttle(MIN_OUTPUT, MAX_OUTPUT, Kp, Ki, Kd, P_ON_E, direction);

// --- Simulation ---
double rawSpeed = 0.0; // Simulate noisy sensor
double targetSpeed = 40.0;
double simMotorEffect = 0.0;

void setup()
{
    Serial.begin(115200);
    Serial.println("AutoThrottleNG - Smoothing & Filtering Demo");

    throttle.setTarget(targetSpeed);
    throttle.setSampleTime(50);

    // --- Enable Features ---
    throttle.setInputFilterAlpha(0.2); // Apply moderate input filtering (lower alpha = more filtering)
    throttle.enableSmoothing(true, 25.0); // Enable smoothing, max 25 units/sec

    Serial.println("Input Filter Alpha: 0.2");
    Serial.println("Smoothing Enabled: Rate=25.0");
    Serial.println("Target | RawIn | FiltIn | RawPID | SmoothOut");
    Serial.println("-----------------------------------------------");
}

void loop()
{
    // 1. Simulate Noisy Sensor
    double effectiveness = 0.008;
    double drag = 0.015;
    rawSpeed += (simMotorEffect * effectiveness) - (rawSpeed * drag);
    rawSpeed = max(0.0, rawSpeed);
    // Add noise
    double noisySpeed = rawSpeed + (random(-100, 101) / 20.0); // +/- 5.0 noise

    // 2. Update Controller with RAW noisy value
    throttle.updateInput(noisySpeed);

    // 3. Compute Output
    double output = throttle.compute();

    // 4. Apply Output (Simulation)
    simMotorEffect = output;

    // 5. Print Status
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 250) {
        Serial.print(throttle.getSetpoint(), 0);
        Serial.print("    | ");
        Serial.print(throttle.getRawInput(), 1);
        Serial.print("   | "); // Show raw noisy input
        Serial.print(throttle.getFilteredInput(), 1);
        Serial.print("    | "); // Show filtered input
        Serial.print(throttle.getRawPIDOutput(), 1);
        Serial.print("    | "); // Show raw PID output
        Serial.print(output, 1); // Show final smoothed output
        Serial.println();
        lastPrint = millis();
    }

    // --- Dynamic Target Change ---
    static bool targetChanged = false;
    unsigned long currentTime = millis();
    if (!targetChanged && currentTime > 8000) {
        targetSpeed = 80.0;
        throttle.setTarget(targetSpeed);
        Serial.println("----> New Target: 80.0");
        targetChanged = true;
    } else if (targetChanged && currentTime > 16000) {
        targetSpeed = 20.0;
        throttle.setTarget(targetSpeed);
        Serial.println("----> New Target: 20.0");
        targetChanged = false;
    }
    delay(10);
}

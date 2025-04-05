/**
 * @file 3_FailsafeDemoNG.ino
 * @brief Demo for AutoThrottleNG failsafe features.
 */
#include <Arduino.h>
#include <AutoThrottleNG.h>
#include <PID_v1.h>
#include <math.h> // For NAN

// --- Config ---
const double MIN_OUTPUT = 0.0;
const double MAX_OUTPUT = 100.0;
double Kp = 2.8, Ki = 0.6, Kd = 0.1;
int direction = DIRECT;

// --- Create Instance ---
AutoThrottleNG throttle(MIN_OUTPUT, MAX_OUTPUT, Kp, Ki, Kd, P_ON_E, direction);

// --- Failsafe Config ---
const double FAILSAFE_VAL = 5.0; // Low idle on failsafe
const unsigned long INPUT_TIMEOUT = 2000; // 2 seconds
const double STABILITY_TOL = 5.0; // +/- 5 units
const unsigned long STABILITY_TIME = 5000; // Unstable for 5 seconds

// --- Simulation ---
double currentSpeed = 0.0;
double targetSpeed = 60.0;
double simMotorEffect = 0.0;
unsigned long lastGoodUpdate = 0;
bool simInputTimeout = false;
bool simInputInvalid = false;
bool simUnstable = false;
unsigned long errorActiveStart = 0;

void setup()
{
    Serial.begin(115200);
    Serial.println("AutoThrottleNG - Failsafe Demo");

    throttle.setTarget(targetSpeed);
    throttle.setSampleTime(50);
    throttle.enableSmoothing(true, 30.0); // Optional smoothing

    // --- Configure Failsafes ---
    throttle.setFailsafeValue(FAILSAFE_VAL);
    throttle.setInputTimeout(INPUT_TIMEOUT);
    throttle.setStabilityParams(STABILITY_TOL, STABILITY_TIME);

    Serial.println("Failsafes Enabled:");
    Serial.print(" - Input Timeout: ");
    Serial.println(INPUT_TIMEOUT);
    Serial.print(" - Stability: Tol=");
    Serial.print(STABILITY_TOL);
    Serial.print(", Time=");
    Serial.println(STABILITY_TIME);
    Serial.print(" - Failsafe Output: ");
    Serial.println(FAILSAFE_VAL);
    Serial.println("Target| Input | Output | Mode | Error");
    Serial.println("--------------------------------------");
    lastGoodUpdate = millis();
}

void loop()
{
    unsigned long now = millis();
    double sensorValue = NAN; // Default to invalid

    // 1. Simulate Sensor / Failure Condition
    if (!simInputTimeout && !simInputInvalid) {
        double effectiveness = 0.007;
        double drag = 0.01;
        if (simUnstable) {
            currentSpeed = targetSpeed + 25.0 * sin(now / 600.0); // Force instability
        } else {
            currentSpeed += (simMotorEffect * effectiveness) - (currentSpeed * drag);
            currentSpeed = max(0.0, currentSpeed);
        }
        sensorValue = currentSpeed; // Valid reading
    } else if (simInputInvalid) {
        sensorValue = NAN; // Send invalid reading
    }
    // If simInputTimeout is true, we just don't call updateInput()

    // 2. Update Controller (if not simulating timeout)
    if (!simInputTimeout) {
        throttle.updateInput(sensorValue);
    }

    // 3. Compute Output
    double output = throttle.compute();

    // 4. Apply Output (Simulation)
    // If in error state, output is already failsafe value
    simMotorEffect = output;

    // 5. Monitor & Print Status / Failsafe
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 300) {
        AutoThrottleNG::Error err = throttle.getErrorState();
        Serial.print(throttle.getSetpoint(), 0);
        Serial.print("  | ");
        if (simInputTimeout || isnan(sensorValue)) {
            Serial.print("Stale ");
        } else {
            Serial.print(throttle.getFilteredInput(), 1);
        }
        Serial.print(" | ");
        Serial.print(output, 1);
        Serial.print("   | ");
        Serial.print(throttle.getMode() == AUTOMATIC ? "AUTO " : "MANUL");
        Serial.print("| ");
        switch (err) {
        case AutoThrottleNG::Error::NONE:
            Serial.print("OK");
            break;
        case AutoThrottleNG::Error::INPUT_INVALID:
            Serial.print("INV_IN");
            break;
        case AutoThrottleNG::Error::INPUT_TIMEOUT:
            Serial.print("TIMEOUT");
            break;
        case AutoThrottleNG::Error::STABILITY_TIMEOUT:
            Serial.print("UNSTBL");
            break;
        default:
            Serial.print("UNK");
            break;
        }
        Serial.println();
        lastPrint = millis();
    }

    // 6. Handle Failsafe State / Recovery Simulation
    if (throttle.isInErrorState()) {
        if (errorActiveStart == 0)
            errorActiveStart = now;
        // Try to recover after 6 seconds
        if (now - errorActiveStart > 6000) {
            Serial.println("----> Attempting Failsafe Recovery <----");
            simInputTimeout = false;
            simInputInvalid = false;
            simUnstable = false;
            // Provide a valid input before clearing if needed
            if (throttle.getErrorState() == AutoThrottleNG::Error::INPUT_INVALID || throttle.getErrorState() == AutoThrottleNG::Error::INPUT_TIMEOUT) {
                throttle.updateInput(targetSpeed); // Force a valid reading
                Serial.println("Forcing simulated input update.");
            }
            throttle.clearErrorState();
            errorActiveStart = 0;
        }
    } else {
        errorActiveStart = 0; // Reset timer if not in error
    }

    // --- Test Triggers ---
    static bool trig1 = false, trig2 = false, trig3 = false;
    if (!trig1 && now > 10000 && !throttle.isInErrorState()) {
        Serial.println("\n---> SIMULATING INPUT TIMEOUT <---\n");
        simInputTimeout = true;
        trig1 = true;
    }
    if (!trig2 && now > 20000 && !throttle.isInErrorState()) {
        Serial.println("\n---> SIMULATING INVALID INPUT <---\n");
        simInputInvalid = true;
        trig2 = true;
    }
    if (!trig3 && now > 30000 && !throttle.isInErrorState()) {
        Serial.println("\n---> SIMULATING INSTABILITY <---\n");
        simUnstable = true;
        trig3 = true;
    }

    delay(10);
}

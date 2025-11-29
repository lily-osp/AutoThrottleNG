/**
 * @file operational_modes.ino
 * @brief Real-world example: Operational Modes and Auto-Recovery
 *
 * This example demonstrates the new operational modes and auto-recovery features
 * of AutoThrottleNG, including:
 * - SAFE_MODE: Conservative operation for maximum safety
 * - LEARNING_MODE: Adaptive tuning for parameter optimization
 * - REVERSE_MODE: Reverse operation for specific applications
 * - MAINTENANCE_MODE: Diagnostic operations
 * - CALIBRATION_MODE: System calibration
 * - Auto-recovery: Automatic error recovery without manual intervention
 *
 * Hardware Requirements:
 * - Arduino board (Uno, Mega, etc.)
 * - Potentiometer for setpoint adjustment (optional)
 * - LED for status indication (optional)
 * - Any sensor/actuator for control demonstration
 */

#include <Arduino.h>
#include <AutoThrottleNG.h>
#include <PID_v1.h>

// --- Hardware Configuration ---
const int setpointPotPin = A0;    // Potentiometer for setpoint adjustment
const int ledPin = 13;            // Status LED
const int modeButtonPin = 2;      // Button to cycle through modes

// --- Control Configuration ---
const double MIN_OUTPUT = 0.0;
const double MAX_OUTPUT = 255.0;
const double TARGET_BASE = 127.0;  // Base setpoint

// PID tuning values (will be adjusted by operational modes)
double Kp = 2.0, Ki = 0.5, Kd = 0.1;

// --- AutoThrottleNG Configuration ---
AutoThrottleNG controller(MIN_OUTPUT, MAX_OUTPUT, Kp, Ki, Kd, P_ON_E, DIRECT);

// --- Global Variables ---
double currentSetpoint = TARGET_BASE;
unsigned long lastModeChange = 0;
int currentModeIndex = 0;

// Array of operational modes for easy cycling
AutoThrottleNG::OperationalMode modes[] = {
    AutoThrottleNG::OperationalMode::NORMAL,
    AutoThrottleNG::OperationalMode::SAFE_MODE,
    AutoThrottleNG::OperationalMode::LEARNING_MODE,
    AutoThrottleNG::OperationalMode::REVERSE_MODE,
    AutoThrottleNG::OperationalMode::MAINTENANCE_MODE,
    AutoThrottleNG::OperationalMode::CALIBRATION_MODE
};

const char* modeNames[] = {
    "NORMAL",
    "SAFE_MODE",
    "LEARNING_MODE",
    "REVERSE_MODE",
    "MAINTENANCE_MODE",
    "CALIBRATION_MODE"
};

const int NUM_MODES = sizeof(modes) / sizeof(modes[0]);

// Button debouncing
unsigned long lastButtonPress = 0;
const unsigned long DEBOUNCE_DELAY = 200;

// Error simulation for testing auto-recovery
bool simulateErrors = false;
unsigned long lastErrorSimulation = 0;

/**
 * @brief Read setpoint from potentiometer
 * @return Setpoint value (0-255)
 */
double readSetpoint() {
    int potValue = analogRead(setpointPotPin);
    return map(potValue, 0, 1023, 0, 255);
}

/**
 * @brief Simulate a sensor reading (sine wave with some noise)
 * @return Simulated sensor value
 */
double readSimulatedSensor() {
    // Create a slowly varying sine wave
    double time = millis() / 1000.0; // Convert to seconds
    double baseValue = sin(time * 0.5) * 50 + 127; // ±50 around 127

    // Add some noise
    double noise = random(-10, 11); // ±10 noise

    return baseValue + noise;
}

/**
 * @brief Control LED based on operational mode
 * @param mode Current operational mode
 */
void updateStatusLED(AutoThrottleNG::OperationalMode mode) {
    static unsigned long lastBlink = 0;
    static bool ledState = false;

    switch (mode) {
        case AutoThrottleNG::OperationalMode::NORMAL:
            digitalWrite(ledPin, HIGH); // Steady on
            break;

        case AutoThrottleNG::OperationalMode::SAFE_MODE:
            // Slow blink
            if (millis() - lastBlink >= 1000) {
                ledState = !ledState;
                digitalWrite(ledPin, ledState);
                lastBlink = millis();
            }
            break;

        case AutoThrottleNG::OperationalMode::LEARNING_MODE:
            // Medium blink
            if (millis() - lastBlink >= 500) {
                ledState = !ledState;
                digitalWrite(ledPin, ledState);
                lastBlink = millis();
            }
            break;

        case AutoThrottleNG::OperationalMode::REVERSE_MODE:
            // Fast blink
            if (millis() - lastBlink >= 200) {
                ledState = !ledState;
                digitalWrite(ledPin, ledState);
                lastBlink = millis();
            }
            break;

        case AutoThrottleNG::OperationalMode::MAINTENANCE_MODE:
            // Very fast blink
            if (millis() - lastBlink >= 100) {
                ledState = !ledState;
                digitalWrite(ledPin, ledState);
                lastBlink = millis();
            }
            break;

        case AutoThrottleNG::OperationalMode::CALIBRATION_MODE:
            // Alternating pattern
            digitalWrite(ledPin, (millis() / 250) % 2 == 0 ? HIGH : LOW);
            break;
    }
}

/**
 * @brief Check for mode change button press
 */
void checkModeButton() {
    static bool lastButtonState = HIGH;

    bool buttonState = digitalRead(modeButtonPin);

    if (buttonState == LOW && lastButtonState == HIGH &&
        millis() - lastButtonPress >= DEBOUNCE_DELAY) {

        // Cycle to next mode
        currentModeIndex = (currentModeIndex + 1) % NUM_MODES;
        controller.setOperationalMode(modes[currentModeIndex]);

        Serial.print("Switched to mode: ");
        Serial.println(modeNames[currentModeIndex]);

        lastButtonPress = millis();
    }

    lastButtonState = buttonState;
}

/**
 * @brief Simulate errors for testing auto-recovery
 */
void simulateErrorConditions() {
    if (!simulateErrors) return;

    unsigned long currentTime = millis();

    // Simulate different error conditions
    if (currentTime - lastErrorSimulation >= 15000) { // Every 15 seconds
        static int errorType = 0;

        switch (errorType) {
            case 0:
                // Simulate invalid input
                Serial.println("SIMULATING: Invalid sensor input");
                controller.updateInput(NAN); // Invalid input
                break;

            case 1:
                // Simulate input timeout (don't call updateInput)
                Serial.println("SIMULATING: Input timeout (stopping updates)");
                // Don't call updateInput for a while
                break;

            case 2:
                // Simulate stability issues
                Serial.println("SIMULATING: Stability issues");
                // Set unrealistic setpoint to cause oscillations
                controller.setTarget(currentSetpoint + 200);
                break;
        }

        errorType = (errorType + 1) % 3;
        lastErrorSimulation = currentTime;
    }
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("AutoThrottleNG - Operational Modes & Auto-Recovery Demo");
    Serial.println("=======================================================");

    // Configure hardware pins
    pinMode(setpointPotPin, INPUT);
    pinMode(ledPin, OUTPUT);
    pinMode(modeButtonPin, INPUT_PULLUP);

    // Configure controller
    controller.setTarget(TARGET_BASE);
    controller.setSampleTime(100);

    // Enable auto-recovery with 3-second delay, max 5 attempts
    controller.enableAutoRecovery(true, 3000, 5);

    // Configure failsafes
    controller.setFailsafeValue(50.0);     // Safe middle value
    controller.setInputTimeout(5000);      // 5 second timeout
    controller.setStabilityParams(20.0, 8000); // 20 unit tolerance, 8 second timeout

    Serial.println("Configuration:");
    Serial.println("- Auto-recovery: ENABLED (3s delay, max 5 attempts)");
    Serial.println("- Failsafe value: 50.0");
    Serial.println("- Input timeout: 5 seconds");
    Serial.println("- Stability tolerance: 20 units, 8 second timeout");
    Serial.println();
    Serial.println("Controls:");
    Serial.println("- Potentiometer (A0): Adjust setpoint");
    Serial.println("- Button (pin 2): Cycle operational modes");
    Serial.println("- LED (pin 13): Mode status indicator");
    Serial.println();
    Serial.println("Available modes:");
    for (int i = 0; i < NUM_MODES; i++) {
        Serial.print("- ");
        Serial.println(modeNames[i]);
    }
    Serial.println();
    Serial.println("Time | Mode | Setpoint | Input | Output | Error | Recovery#");
    Serial.println("----------------------------------------------------------");

    // Enable error simulation for demonstration
    simulateErrors = true;
}

void loop() {
    unsigned long currentTime = millis();

    // Check for mode change button
    checkModeButton();

    // Read setpoint from potentiometer
    currentSetpoint = readSetpoint();
    controller.setTarget(currentSetpoint);

    // Read simulated sensor (normally this would be a real sensor)
    double sensorValue = readSimulatedSensor();

    // Simulate error conditions for demonstration
    simulateErrorConditions();

    // For input timeout simulation, skip updateInput occasionally
    static bool skipUpdate = false;
    if (simulateErrors && currentTime - lastErrorSimulation < 3000) {
        skipUpdate = true; // Skip updates during timeout simulation
    } else {
        skipUpdate = false;
    }

    if (!skipUpdate) {
        controller.updateInput(sensorValue);
    }

    // Compute control output
    double output = controller.compute();

    // Apply output (to PWM, motor, etc.)
    analogWrite(11, (int)output); // Example: PWM output on pin 11

    // Update status LED based on operational mode
    updateStatusLED(controller.getOperationalMode());

    // Print status every 500ms
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= 500) {
        Serial.print(currentTime / 1000.0, 1);
        Serial.print(" | ");

        // Show current mode
        Serial.print(modeNames[currentModeIndex]);
        Serial.print(" | ");

        // Show values
        Serial.print(controller.getSetpoint(), 0);
        Serial.print(" | ");
        Serial.print(controller.getFilteredInput(), 1);
        Serial.print(" | ");
        Serial.print(output, 1);
        Serial.print(" | ");

        // Show error state
        if (controller.isInErrorState()) {
            Serial.print("YES");
        } else {
            Serial.print("NO");
        }
        Serial.print(" | ");

        // Show recovery attempts
        Serial.println(controller.getRecoveryAttemptCount());

        lastPrintTime = currentTime;
    }

    // Small delay to prevent overwhelming the system
    delay(50);
}

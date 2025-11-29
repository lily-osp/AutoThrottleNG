/**
 * @file led_brightness.ino
 * @brief Real-world example: LED Brightness Control with Light Sensor Feedback
 *
 * This example demonstrates LED brightness control using AutoThrottleNG
 * with a light sensor (LDR) for closed-loop ambient light compensation.
 * It includes:
 * - Light sensor feedback for ambient light measurement
 * - PID-based brightness control
 * - Input filtering for noisy sensor readings
 * - Output smoothing to prevent LED flickering
 * - Failsafe mechanisms for lighting safety
 *
 * Hardware Requirements:
 * - Arduino board (Uno, Mega, etc.)
 * - LED (with appropriate current-limiting resistor)
 * - Light sensor (LDR/photoresistor)
 * - Resistors for voltage divider circuit
 *
 * Connections:
 * - LED: Digital pin 9 (PWM capable) with current-limiting resistor
 * - Light sensor: Analog pin A0 (voltage divider with 10k resistor)
 */

#include <Arduino.h>
#include <AutoThrottleNG.h>
#include <PID_v1.h>

// --- Hardware Configuration ---
const uint8_t LED_PIN = 9;             // PWM pin for LED control
const uint8_t LIGHT_SENSOR_PIN = A0;   // Analog pin for light sensor

// --- Light Sensor Configuration ---
const int ADC_MIN = 0;                 // Minimum ADC reading (dark)
const int ADC_MAX = 1023;              // Maximum ADC reading (bright)
const double LIGHT_MIN = 0.0;          // Minimum light level (lux equivalent)
const double LIGHT_MAX = 1000.0;       // Maximum light level (lux equivalent)

// --- Control Configuration ---
const double TARGET_BRIGHTNESS = 500.0; // Target brightness level
const double MIN_BRIGHTNESS = 0.0;      // Minimum LED brightness (PWM 0-255)
const double MAX_BRIGHTNESS = 255.0;    // Maximum LED brightness (PWM 0-255)

// PID tuning values (adjust based on your LED and sensor response)
double Kp = 0.5, Ki = 0.02, Kd = 0.1;

// --- AutoThrottleNG Configuration ---
// Using REVERSE direction because higher sensor reading = more ambient light = less LED brightness needed
AutoThrottleNG ledController(MIN_BRIGHTNESS, MAX_BRIGHTNESS, Kp, Ki, Kd, P_ON_E, REVERSE);

// --- Global Variables ---
double currentLightLevel = 0.0;        // Current ambient light level
unsigned long lastLightUpdate = 0;      // Last light measurement time

/**
 * @brief Read ambient light level from sensor
 * @return Light level (0-1000, higher = brighter)
 */
double readLightLevel() {
    // Read analog value and convert to light level
    int adcValue = analogRead(LIGHT_SENSOR_PIN);

    // Convert ADC to light level (inverse relationship for LDR)
    // Higher ADC = more resistance = darker environment
    double lightLevel = map(adcValue, ADC_MIN, ADC_MAX, LIGHT_MAX, LIGHT_MIN);

    // Ensure valid range
    return constrain(lightLevel, LIGHT_MIN, LIGHT_MAX);
}

/**
 * @brief Set LED brightness
 * @param brightness Brightness level (0-255)
 */
void setLEDBrightness(double brightness) {
    // Constrain brightness to safe PWM range
    brightness = constrain(brightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS);

    // Set LED brightness using PWM
    analogWrite(LED_PIN, (int)brightness);
}

/**
 * @brief Create breathing effect by modulating target brightness
 * @param time Current time in milliseconds
 * @return Modulated target brightness
 */
double getBreathingTarget(unsigned long time) {
    // Create a slow sine wave for breathing effect (8 second cycle)
    double breathingFactor = sin(2.0 * PI * time / 8000.0) * 0.3 + 0.7;

    // Modulate around base target
    return TARGET_BRIGHTNESS * breathingFactor;
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("AutoThrottleNG - LED Brightness Control");
    Serial.println("=====================================");

    // Configure LED pin
    pinMode(LED_PIN, OUTPUT);
    pinMode(LIGHT_SENSOR_PIN, INPUT);

    // Start with LED off
    setLEDBrightness(0);

    // Configure AutoThrottleNG
    ledController.setTarget(TARGET_BRIGHTNESS);
    ledController.setSampleTime(100);  // Moderate response time

    // Enable filtering and smoothing for stable lighting
    ledController.setInputFilterAlpha(0.3);  // Moderate filtering for sensor noise
    ledController.enableSmoothing(true, 25.0); // Smooth brightness changes (25 units/sec)

    // Configure failsafes
    ledController.setFailsafeValue(128.0);   // Medium brightness on failsafe
    ledController.setInputTimeout(2000);     // 2 second sensor timeout
    ledController.setStabilityParams(50.0, 5000); // 50 unit tolerance, 5 second timeout

    Serial.println("Configuration:");
    Serial.print("Target Brightness: "); Serial.print(TARGET_BRIGHTNESS); Serial.println(" (0-1000 scale)");
    Serial.print("LED PWM Range: "); Serial.print(MIN_BRIGHTNESS);
    Serial.print(" - "); Serial.print(MAX_BRIGHTNESS); Serial.println(" (0-255)");
    Serial.print("PID Gains: Kp="); Serial.print(Kp);
    Serial.print(", Ki="); Serial.print(Ki);
    Serial.print(", Kd="); Serial.println(Kd);
    Serial.println("Input Filter: Alpha=0.3");
    Serial.println("Output Smoothing: 25 units/sec max rate");
    Serial.println("Failsafe: Medium brightness (128)");
    Serial.println();
    Serial.println("Time | Target | Light | Output | Error | Breathing");
    Serial.println("-------------------------------------------------");
}

void loop() {
    unsigned long currentTime = millis();

    // Read light level every 50ms
    static unsigned long lastReadTime = 0;
    if (currentTime - lastReadTime >= 50) {
        currentLightLevel = readLightLevel();
        lastReadTime = currentTime;

        // Update controller with current light level
        ledController.updateInput(currentLightLevel);
    }

    // Update target with breathing effect
    static bool breathingEnabled = true;
    double currentTarget = breathingEnabled ? getBreathingTarget(currentTime) : TARGET_BRIGHTNESS;
    ledController.setTarget(currentTarget);

    // Compute LED control output
    double ledBrightness = ledController.compute();

    // Apply LED brightness
    setLEDBrightness(ledBrightness);

    // Print status every 500ms
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= 500) {
        Serial.print(currentTime / 1000.0, 1);
        Serial.print(" | ");
        Serial.print(currentTarget, 0);
        Serial.print(" | ");
        Serial.print(currentLightLevel, 0);
        Serial.print(" | ");
        Serial.print(ledBrightness, 0);
        Serial.print(" | ");

        // Show error state
        AutoThrottleNG::Error err = ledController.getErrorState();
        if (err == AutoThrottleNG::Error::NONE) {
            Serial.print("OK");
        } else {
            Serial.print("ERROR");
        }
        Serial.print(" | ");

        // Show breathing status
        Serial.print(breathingEnabled ? "ON" : "OFF");
        Serial.println();

        lastPrintTime = currentTime;
    }

    // Toggle breathing effect every 30 seconds
    static unsigned long lastToggleTime = 0;
    if (currentTime - lastToggleTime > 30000) {
        breathingEnabled = !breathingEnabled;
        Serial.print("*** Breathing Effect: ");
        Serial.println(breathingEnabled ? "ENABLED ***" : "DISABLED ***");
        lastToggleTime = currentTime;
    }

    // Simulate light level changes for testing (optional)
    static bool simulateChanges = false;
    if (simulateChanges) {
        // Gradually change light level for testing
        static double simLight = TARGET_BRIGHTNESS;
        static int simDirection = 1;

        simLight += simDirection * 2.0;
        if (simLight >= LIGHT_MAX || simLight <= LIGHT_MIN) {
            simDirection *= -1;
        }

        // Override sensor reading with simulation
        ledController.updateInput(simLight);
    }

    // Handle failsafe recovery
    if (ledController.isInErrorState()) {
        static unsigned long errorStartTime = 0;
        if (errorStartTime == 0) {
            errorStartTime = currentTime;
            Serial.println("*** LED FAILSAFE ACTIVE ***");
        }

        // Attempt recovery after 8 seconds
        if (currentTime - errorStartTime > 8000) {
            Serial.println("*** ATTEMPTING LED RECOVERY ***");
            ledController.clearErrorState();
            errorStartTime = 0;
        }
    }

    // Emergency brightness limits
    if (currentLightLevel > LIGHT_MAX * 0.9) {
        // Very bright environment - dim LED significantly
        setLEDBrightness(MIN_BRIGHTNESS);
        Serial.println("*** VERY BRIGHT ENVIRONMENT - LED DIMMED ***");
    } else if (currentLightLevel < LIGHT_MIN + 50.0) {
        // Very dark environment - allow full brightness
        // LED controller will handle this automatically
    }

    // Small delay for stability
    delay(10);
}

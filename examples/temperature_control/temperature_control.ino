/**
 * @file temperature_control.ino
 * @brief Real-world example: Temperature Control System
 *
 * This example demonstrates precise temperature control using AutoThrottleNG
 * with a thermistor or temperature sensor. It includes:
 * - Temperature measurement and filtering
 * - PID-based heating/cooling control
 * - Input filtering for noisy temperature readings
 * - Output smoothing to prevent thermal stress
 * - Failsafe mechanisms for temperature safety
 *
 * Hardware Requirements:
 * - Arduino board (Uno, Mega, etc.)
 * - Temperature sensor (thermistor, DS18B20, TMP117, etc.)
 * - Heating element or Peltier module
 * - Relay or solid-state relay for power control
 *
 * This example uses a simple thermistor circuit, but can be adapted
 * for other temperature sensors.
 */

#include <Arduino.h>
#include <AutoThrottleNG.h>
#include <PID_v1.h>

// --- Hardware Configuration ---
const uint8_t TEMP_SENSOR_PIN = A0;    // Analog pin for temperature sensor
const uint8_t HEATER_PIN = 9;          // PWM pin for heater control
const uint8_t COOLER_PIN = 10;         // PWM pin for cooler control (optional)

// --- Temperature Sensor Configuration ---
// Using a 10k NTC thermistor with 10k series resistor
const double THERMISTOR_NOMINAL = 10000.0;    // Resistance at 25°C
const double TEMPERATURE_NOMINAL = 25.0;      // Temperature for nominal resistance
const double B_COEFFICIENT = 3950.0;          // Beta coefficient of thermistor
const double SERIES_RESISTOR = 10000.0;       // Series resistor value

// --- Control Configuration ---
const double MIN_TEMPERATURE = 15.0;     // Minimum safe temperature (°C)
const double MAX_TEMPERATURE = 35.0;     // Maximum safe temperature (°C)
const double TARGET_TEMPERATURE = 25.0;  // Target temperature (°C)

// PID tuning values (adjust based on your system thermal characteristics)
double Kp = 8.0, Ki = 0.3, Kd = 1.2;

// --- AutoThrottleNG Configuration ---
// Using REVERSE direction because higher output = more heating = higher temperature
AutoThrottleNG tempController(MIN_TEMPERATURE, MAX_TEMPERATURE, Kp, Ki, Kd, P_ON_E, REVERSE);

// --- Global Variables ---
double currentTemperature = 0.0;        // Current temperature reading
unsigned long lastTempUpdate = 0;        // Last temperature measurement time

/**
 * @brief Read temperature from thermistor
 * @return Temperature in Celsius
 */
double readTemperature() {
    // Read analog value
    int adcValue = analogRead(TEMP_SENSOR_PIN);

    // Convert to resistance
    double resistance = SERIES_RESISTOR / ((1023.0 / adcValue) - 1.0);

    // Convert to temperature using Steinhart-Hart equation approximation
    double steinhart;
    steinhart = resistance / THERMISTOR_NOMINAL;     // (R/Ro)
    steinhart = log(steinhart);                      // ln(R/Ro)
    steinhart /= B_COEFFICIENT;                       // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                     // Invert
    steinhart -= 273.15;                             // Convert to Celsius

    return steinhart;
}

/**
 * @brief Control heating and cooling elements
 * @param output PID output value (negative = cooling, positive = heating)
 */
void controlTemperature(double output) {
    // Constrain output to safe range
    output = constrain(output, MIN_TEMPERATURE, MAX_TEMPERATURE);

    if (output >= TARGET_TEMPERATURE + 0.5) {
        // Need cooling
        double coolingPower = map(output, TARGET_TEMPERATURE, MAX_TEMPERATURE, 0, 255);
        analogWrite(COOLER_PIN, (int)coolingPower);
        analogWrite(HEATER_PIN, 0);
    } else if (output <= TARGET_TEMPERATURE - 0.5) {
        // Need heating
        double heatingPower = map(TARGET_TEMPERATURE - output, 0, TARGET_TEMPERATURE - MIN_TEMPERATURE, 0, 255);
        analogWrite(HEATER_PIN, (int)heatingPower);
        analogWrite(COOLER_PIN, 0);
    } else {
        // Maintain temperature - minimal power
        analogWrite(HEATER_PIN, 0);
        analogWrite(COOLER_PIN, 0);
    }
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("AutoThrottleNG - Temperature Control System");
    Serial.println("==========================================");

    // Configure hardware pins
    pinMode(TEMP_SENSOR_PIN, INPUT);
    pinMode(HEATER_PIN, OUTPUT);
    pinMode(COOLER_PIN, OUTPUT);

    // Initialize outputs to safe state (off)
    analogWrite(HEATER_PIN, 0);
    analogWrite(COOLER_PIN, 0);

    // Configure AutoThrottleNG
    tempController.setTarget(TARGET_TEMPERATURE);
    tempController.setSampleTime(1000);  // 1 second sample time for thermal systems

    // Enable filtering and smoothing for stable temperature control
    tempController.setInputFilterAlpha(0.2);  // Moderate filtering for temperature noise
    tempController.enableSmoothing(true, 2.0); // Slow output changes (2°C per second max)

    // Configure failsafes for temperature safety
    tempController.setFailsafeValue(TARGET_TEMPERATURE);  // Maintain target temp on failsafe
    tempController.setInputTimeout(5000);    // 5 second sensor timeout
    tempController.setStabilityParams(1.0, 10000); // 1°C tolerance, 10 second timeout

    Serial.println("Configuration:");
    Serial.print("Target Temperature: "); Serial.print(TARGET_TEMPERATURE); Serial.println(" °C");
    Serial.print("Safe Range: "); Serial.print(MIN_TEMPERATURE);
    Serial.print(" - "); Serial.print(MAX_TEMPERATURE); Serial.println(" °C");
    Serial.print("PID Gains: Kp="); Serial.print(Kp);
    Serial.print(", Ki="); Serial.print(Ki);
    Serial.print(", Kd="); Serial.println(Kd);
    Serial.println("Input Filter: Alpha=0.2");
    Serial.println("Output Smoothing: 2°C/sec max rate");
    Serial.println("Failsafe: Maintain target temperature");
    Serial.println();
    Serial.println("Time | Target | Current | Output | Error | Heater | Cooler");
    Serial.println("----------------------------------------------------------");
}

void loop() {
    unsigned long currentTime = millis();

    // Read temperature every 200ms
    static unsigned long lastReadTime = 0;
    if (currentTime - lastReadTime >= 200) {
        currentTemperature = readTemperature();
        lastReadTime = currentTime;

        // Validate temperature reading
        if (currentTemperature < -50.0 || currentTemperature > 100.0) {
            // Invalid reading - use previous valid value or target
            Serial.println("Warning: Invalid temperature reading");
        } else {
            // Update controller with current temperature
            tempController.updateInput(currentTemperature);
        }
    }

    // Compute temperature control output
    double controlOutput = tempController.compute();

    // Apply temperature control
    controlTemperature(controlOutput);

    // Print status every 2 seconds
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= 2000) {
        Serial.print(currentTime / 1000.0, 1);
        Serial.print(" | ");
        Serial.print(tempController.getSetpoint(), 1);
        Serial.print(" | ");
        Serial.print(currentTemperature, 1);
        Serial.print(" | ");
        Serial.print(controlOutput, 1);
        Serial.print(" | ");

        // Show error state
        AutoThrottleNG::Error err = tempController.getErrorState();
        if (err == AutoThrottleNG::Error::NONE) {
            Serial.print("OK");
        } else {
            Serial.print("ERROR");
        }
        Serial.print(" | ");

        // Show heater/cooler status
        int heaterPWM = (controlOutput <= TARGET_TEMPERATURE - 0.5) ?
                       (int)map(TARGET_TEMPERATURE - controlOutput, 0, TARGET_TEMPERATURE - MIN_TEMPERATURE, 0, 255) : 0;
        int coolerPWM = (controlOutput >= TARGET_TEMPERATURE + 0.5) ?
                       (int)map(controlOutput, TARGET_TEMPERATURE, MAX_TEMPERATURE, 0, 255) : 0;

        Serial.print(heaterPWM);
        Serial.print(" | ");
        Serial.print(coolerPWM);
        Serial.println();

        lastPrintTime = currentTime;
    }

    // Handle critical temperature safety
    if (currentTemperature > MAX_TEMPERATURE + 5.0) {
        // Critical over-temperature - emergency shutdown
        analogWrite(HEATER_PIN, 0);
        analogWrite(COOLER_PIN, 255);  // Maximum cooling
        Serial.println("*** CRITICAL: OVER-TEMPERATURE - EMERGENCY COOLING ***");

        // You might want to add alarm buzzer, system shutdown, etc.
    } else if (currentTemperature < MIN_TEMPERATURE - 5.0) {
        // Critical under-temperature - emergency heating
        analogWrite(HEATER_PIN, 255);  // Maximum heating
        analogWrite(COOLER_PIN, 0);
        Serial.println("*** CRITICAL: UNDER-TEMPERATURE - EMERGENCY HEATING ***");
    }

    // Handle failsafe recovery
    if (tempController.isInErrorState()) {
        static unsigned long errorStartTime = 0;
        if (errorStartTime == 0) {
            errorStartTime = currentTime;
            Serial.println("*** TEMPERATURE CONTROL FAILSAFE ACTIVE ***");
        }

        // Attempt recovery after 10 seconds
        if (currentTime - errorStartTime > 10000) {
            Serial.println("*** ATTEMPTING TEMPERATURE CONTROL RECOVERY ***");
            tempController.clearErrorState();
            errorStartTime = 0;
        }
    }

    // Small delay to prevent overwhelming the serial output
    delay(50);
}

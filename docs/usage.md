# AutoThrottleNG Usage Guide

## Table of Contents

- [Getting Started](#getting-started)
  - [Installation](#installation)
  - [Basic Usage](#basic-usage)
- [Configuration Guide](#configuration-guide)
  - [PID Tuning Parameters](#pid-tuning-parameters)
  - [Output Configuration](#output-configuration)
  - [Signal Processing](#signal-processing)
  - [Sample Time Configuration](#sample-time-configuration)
- [Failsafe Configuration](#failsafe-configuration)
- [Monitoring and Debugging](#monitoring-and-debugging)
- [Common Usage Patterns](#common-usage-patterns)
  - [Motor Speed Control](#motor-speed-control)
  - [Temperature Control](#temperature-control)
  - [Position Control](#position-control)
- [Performance Optimization](#performance-optimization)

## Getting Started

### Installation

1. **Install Arduino IDE** (version 1.8.0 or later)
2. **Install PID Library**:
   - Arduino IDE: `Sketch → Include Library → Manage Libraries`
   - Search for "PID" by Brett Beauregard
   - Click Install

3. **Install AutoThrottleNG**:
   - Download the library ZIP file
   - Arduino IDE: `Sketch → Include Library → Add .ZIP Library`
   - Select the downloaded AutoThrottleNG ZIP file

4. **Restart Arduino IDE**

### Basic Usage

#### 1. Include Libraries
```cpp
#include <Arduino.h>
#include <AutoThrottleNG.h>
#include <PID_v1.h>
```

#### 2. Declare Controller
```cpp
// Declare global controller instance
AutoThrottleNG throttle(minOutput, maxOutput, kp, ki, kd, pOn, direction);
```

#### 3. Initialize in setup()
```cpp
void setup() {
    // Configure controller
    throttle.setTarget(desiredValue);

    // Optional: Configure additional features
    throttle.setSampleTime(100);  // 100ms sample time
    throttle.setInputFilterAlpha(0.2);  // Moderate filtering
    throttle.enableSmoothing(true, 50.0);  // Smooth output changes
}
```

#### 4. Use in loop()
```cpp
void loop() {
    // 1. Read sensor
    double sensorValue = readSensor();

    // 2. Update controller with sensor reading
    throttle.updateInput(sensorValue);

    // 3. Compute control output
    double output = throttle.compute();

    // 4. Apply output to actuator
    applyToActuator(output);

    // 5. Optional: Monitor and debug
    printStatus();

    delay(50);  // Adjust loop timing as needed
}
```

## Configuration Guide

### PID Tuning Parameters

#### Choosing Initial Values

**Conservative Starting Values:**
```cpp
double Kp = 1.0;   // Proportional gain
double Ki = 0.05;  // Integral gain
double Kd = 0.1;   // Derivative gain
```

**Application-Specific Guidelines:**

| Application | Kp Range | Ki Range | Kd Range | Sample Time |
|-------------|----------|----------|----------|-------------|
| Motor Speed | 0.5-3.0 | 0.02-0.2 | 0.1-1.0 | 50-100ms |
| Temperature | 5-15 | 0.1-0.5 | 1-5 | 500-2000ms |
| Position | 1-5 | 0.01-0.1 | 0.5-2 | 20-50ms |
| Lighting | 0.2-1 | 0.01-0.05 | 0.05-0.2 | 100-500ms |

#### Runtime Tuning

```cpp
// Adjust PID gains during operation
throttle.setTunings(2.0, 0.1, 0.5);  // Kp=2.0, Ki=0.1, Kd=0.5

// Change proportional mode
throttle.setTunings(2.0, 0.1, 0.5, P_ON_M);  // Use P_ON_M instead of P_ON_E
```

### Output Configuration

#### Output Limits
```cpp
// PWM range (0-255)
throttle.setOutputLimits(0, 255);

// Percentage range (0-100)
throttle.setOutputLimits(0, 100);

// Custom range (-100 to 100)
throttle.setOutputLimits(-100, 100);
```

#### Controller Direction
```cpp
// Direct acting: increase output → increase input
throttle.setControllerDirection(DIRECT);

// Reverse acting: increase output → decrease input
throttle.setControllerDirection(REVERSE);
```

### Signal Processing

#### Input Filtering
```cpp
// No filtering (raw sensor values)
throttle.setInputFilterAlpha(0.0);

// Light filtering (responds quickly)
throttle.setInputFilterAlpha(0.8);

// Moderate filtering (good balance)
throttle.setInputFilterAlpha(0.3);

// Heavy filtering (very smooth)
throttle.setInputFilterAlpha(0.1);
```

#### Output Smoothing
```cpp
// Disable smoothing (immediate response)
throttle.enableSmoothing(false);

// Enable smoothing (50 units/second max change)
throttle.enableSmoothing(true, 50.0);

// Fast smoothing (200 units/second)
throttle.enableSmoothing(true, 200.0);

// Very slow smoothing (10 units/second)
throttle.enableSmoothing(true, 10.0);
```

### Sample Time Configuration

```cpp
// Fast response (20ms - good for position control)
throttle.setSampleTime(20);

// Standard response (100ms - good for most applications)
throttle.setSampleTime(100);

// Slow response (1000ms - good for temperature control)
throttle.setSampleTime(1000);
```

## Failsafe Configuration

### Basic Failsafe Setup

```cpp
// Set failsafe output value (safe state)
throttle.setFailsafeValue(0.0);  // Stop motor, turn off heater, etc.

// Configure input timeout (2 seconds)
throttle.setInputTimeout(2000);

// Configure stability monitoring
throttle.setStabilityParams(2.0, 5000);  // 2 unit tolerance, 5 second timeout
```

### Error Handling

```cpp
void loop() {
    // ... control code ...

    // Check for errors
    if (throttle.isInErrorState()) {
        AutoThrottleNG::Error err = throttle.getErrorState();

        switch(err) {
            case AutoThrottleNG::Error::INPUT_INVALID:
                Serial.println("ERROR: Invalid sensor reading");
                // Handle invalid input (check sensor wiring/power)
                break;

            case AutoThrottleNG::Error::INPUT_TIMEOUT:
                Serial.println("ERROR: Sensor timeout");
                // Handle communication issues
                break;

            case AutoThrottleNG::Error::STABILITY_TIMEOUT:
                Serial.println("ERROR: System unstable");
                // Handle control instability
                break;
        }

        // Attempt recovery
        // Fix the underlying issue first, then:
        throttle.clearErrorState();
    }
}
```

## Monitoring and Debugging

### Basic Status Monitoring

```cpp
void printStatus() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 1000) {  // Print every second
        Serial.print("Target: ");
        Serial.print(throttle.getSetpoint(), 1);

        Serial.print(" | Input: ");
        Serial.print(throttle.getFilteredInput(), 1);

        Serial.print(" | Output: ");
        Serial.print(throttle.getThrottle(), 1);

        Serial.print(" | Mode: ");
        Serial.print(throttle.getMode() == AUTOMATIC ? "AUTO" : "MANUAL");

        Serial.print(" | Error: ");
        if (throttle.isInErrorState()) {
            Serial.print("YES");
        } else {
            Serial.print("NO");
        }

        Serial.println();
        lastPrint = millis();
    }
}
```

### Extended Debugging

```cpp
void printExtendedDebug() {
    Serial.println("=== AutoThrottleNG Debug Info ===");

    // PID parameters
    Serial.print("PID Gains: Kp=");
    Serial.print(throttle.getKp(), 2);
    Serial.print(" Ki=");
    Serial.print(throttle.getKi(), 4);
    Serial.print(" Kd=");
    Serial.println(throttle.getKd(), 2);

    // Input signals
    Serial.print("Raw Input: ");
    Serial.println(throttle.getRawInput(), 2);
    Serial.print("Filtered Input: ");
    Serial.println(throttle.getFilteredInput(), 2);

    // Output signals
    Serial.print("Raw PID Output: ");
    Serial.println(throttle.getRawPIDOutput(), 2);
    Serial.print("Final Output: ");
    Serial.println(throttle.getThrottle(), 2);

    // System status
    Serial.print("Mode: ");
    Serial.println(throttle.getMode() == AUTOMATIC ? "AUTOMATIC" : "MANUAL");
    Serial.print("Saturated: ");
    Serial.println(throttle.isSaturated() ? "YES" : "NO");
    Serial.print("Stable: ");
    Serial.println(throttle.isStable() ? "YES" : "NO");

    // Error status
    Serial.print("Error State: ");
    Serial.print((int)throttle.getErrorState());
    Serial.print(" (");
    Serial.print(throttle.isInErrorState() ? "ACTIVE" : "CLEAR");
    Serial.println(")");

    // Timing
    Serial.print("Last Update: ");
    Serial.print(throttle.getLastUpdateTime());
    Serial.print("ms ago (current: ");
    Serial.print(millis());
    Serial.println(")");

    Serial.println("================================");
}
```

## Common Usage Patterns

### Motor Speed Control

```cpp
#include <AutoThrottleNG.h>

// Motor control pins
const int motorPin = 9;
const int encoderPinA = 2;
const int encoderPinB = 3;

// Controller configuration
AutoThrottleNG motorController(0, 255, 1.5, 0.1, 0.3, P_ON_E, DIRECT);

void setup() {
    // Configure motor
    pinMode(motorPin, OUTPUT);

    // Configure controller
    motorController.setTarget(100.0);  // 100 RPM target
    motorController.setSampleTime(50);
    motorController.setInputFilterAlpha(0.2);
    motorController.enableSmoothing(true, 100.0);

    // Configure encoder interrupts
    attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);
}

void loop() {
    // Read current speed (implement encoder-based speed calculation)
    double currentSpeed = calculateMotorSpeed();

    // Update controller
    motorController.updateInput(currentSpeed);

    // Compute PWM output
    double pwmValue = motorController.compute();

    // Apply to motor
    analogWrite(motorPin, (int)pwmValue);

    delay(50);
}
```

### Temperature Control

```cpp
#include <AutoThrottleNG.h>

// Hardware pins
const int tempSensorPin = A0;
const int heaterPin = 9;
const int coolerPin = 10;

// Temperature controller (reverse acting - more power = less heat)
AutoThrottleNG tempController(0, 255, 8.0, 0.2, 2.0, P_ON_E, REVERSE);

void setup() {
    // Configure pins
    pinMode(heaterPin, OUTPUT);
    pinMode(coolerPin, OUTPUT);

    // Configure controller
    tempController.setTarget(25.0);  // 25°C target
    tempController.setSampleTime(1000);  // 1 second for thermal systems
    tempController.setInputFilterAlpha(0.1);  // Heavy filtering for temperature
    tempController.enableSmoothing(true, 5.0);  // Slow changes for thermal mass

    // Configure failsafes
    tempController.setFailsafeValue(20.0);  // Safe temperature
    tempController.setInputTimeout(5000);   // 5 second sensor timeout
}

void loop() {
    // Read temperature
    double currentTemp = readTemperature();

    // Update controller
    tempController.updateInput(currentTemp);

    // Compute control output
    double controlOutput = tempController.compute();

    // Apply heating/cooling
    if (controlOutput > 128) {
        // Cooling needed
        int coolingPower = map(controlOutput, 128, 255, 0, 255);
        analogWrite(coolerPin, coolingPower);
        analogWrite(heaterPin, 0);
    } else {
        // Heating needed
        int heatingPower = map(128 - controlOutput, 0, 128, 0, 255);
        analogWrite(heaterPin, heatingPower);
        analogWrite(coolerPin, 0);
    }

    delay(1000);
}
```

### Position Control

```cpp
#include <AutoThrottleNG.h>
#include <Servo.h>

// Hardware setup
Servo myServo;
const int feedbackPin = A0;

// Position controller
AutoThrottleNG positionController(0, 180, 2.0, 0.05, 0.8, P_ON_E, DIRECT);

void setup() {
    // Attach servo
    myServo.attach(9);

    // Configure controller
    positionController.setTarget(90.0);  // Center position
    positionController.setSampleTime(50);
    positionController.setInputFilterAlpha(0.3);
    positionController.enableSmoothing(true, 90.0);  // 90°/second max speed

    // Configure failsafes
    positionController.setFailsafeValue(90.0);  // Safe center position
    positionController.setInputTimeout(1000);
    positionController.setStabilityParams(5.0, 3000);
}

void loop() {
    // Read current position
    int potValue = analogRead(feedbackPin);
    double currentPosition = map(potValue, 0, 1023, 0, 180);

    // Update controller
    positionController.updateInput(currentPosition);

    // Compute servo command
    double servoAngle = positionController.compute();

    // Apply to servo
    myServo.write((int)servoAngle);

    // Monitor status
    if (positionController.isInErrorState()) {
        // Handle errors - could center servo, flash LED, etc.
        myServo.write(90); // Force to center
    }

    delay(50);
}
```

### Advanced PID Tuning

#### Ziegler-Nichols Tuning Method

For systematic PID tuning, use the Ziegler-Nichols method:

1. **Set Ki = 0, Kd = 0**
2. Increase Kp until oscillation starts (Ku - ultimate gain)
3. Record oscillation period (Tu - ultimate period)
4. **Calculate PID values:**
   - Kp = 0.6 × Ku
   - Ki = 2 × Kp / Tu
   - Kd = Kp × Tu / 8

#### Manual Tuning Steps

1. **Start with Kp only**: Increase until system responds
2. **Add Ki**: Increase until steady-state error eliminated
3. **Add Kd**: Increase to reduce overshoot and oscillations
4. **Fine-tune**: Small adjustments to optimize performance

#### System-Specific Tuning Guidelines

**Fast Systems (DC motors, servos):**
- Sample time: 20-50ms
- Kp: 1.0-5.0, Ki: 0.01-0.1, Kd: 0.5-2.0

**Medium Systems (flow control, pressure):**
- Sample time: 50-200ms
- Kp: 0.5-2.0, Ki: 0.05-0.5, Kd: 0.1-1.0

**Slow Systems (temperature, level):**
- Sample time: 500-2000ms
- Kp: 5.0-20.0, Ki: 0.1-1.0, Kd: 1.0-10.0

## Performance Optimization

### Memory Optimization

```cpp
// Use appropriate data types
double setpoint = 100.0;  // Not float for precision
int output = (int)throttle.compute();  // Cast to int for PWM

// Minimize string operations in loop
static char buffer[64];
sprintf(buffer, "Output: %d", output);
Serial.println(buffer);
```

### CPU Optimization

```cpp
// Use appropriate delays
const unsigned long LOOP_DELAY = 50;
static unsigned long lastLoop = 0;

if (millis() - lastLoop >= LOOP_DELAY) {
    // Control code here
    lastLoop = millis();
}

// Avoid floating point in timing-critical sections
unsigned long currentTime = millis();
if (currentTime - lastTime >= interval) {
    // Use integer comparisons when possible
}
```

### Power Optimization

```cpp
// Sleep between operations
#include <avr/sleep.h>

// Configure watchdog for low-power timing
// Use appropriate sleep modes for your application
```

This usage guide provides the foundation for implementing AutoThrottleNG in your projects. Start with the basic examples and gradually add features as needed.

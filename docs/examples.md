# AutoThrottleNG Examples Guide

## Table of Contents

- [Overview](#overview)
- [Example Categories](#example-categories)
  - [Basic Examples](#basic-examples)
  - [Intermediate Examples](#intermediate-examples)
  - [Advanced Examples](#advanced-examples)
- [Real-World Application Examples](#real-world-application-examples)
  - [Motor Control](#motor-control)
  - [Temperature Control](#temperature-control)
  - [Servo Control](#servo-control)
  - [LED Brightness Control](#led-brightness-control)
- [Running the Examples](#running-the-examples)
- [Example Selection Guide](#example-selection-guide)
- [Extending the Examples](#extending-the-examples)

## Overview

AutoThrottleNG includes examples demonstrating various applications and features. Each example is designed to be educational, practical, and easily adaptable to real-world projects. The examples progress from basic concepts to advanced implementations.

## Example Categories

### 1. Basic Examples

#### Basic Control (`examples/basic/`)
**Purpose:** Introduces fundamental PID control concepts

**Key Features Demonstrated:**
- Basic PID controller setup and configuration
- Simple setpoint control
- Manual PID tuning
- Serial monitoring of control variables

**Hardware Requirements:**
- Arduino board (Uno, Mega, etc.)
- No additional hardware (simulation-based)

**Learning Objectives:**
- Understand PID controller initialization
- Learn basic setpoint management
- Monitor control system behavior
- Practice manual PID tuning

**Code Highlights:**
```cpp
// Initialize controller with conservative PID values
AutoThrottleNG throttle(0.0, 255.0, 2.0, 0.5, 0.1, P_ON_E, DIRECT);

// Set target value
throttle.setTarget(150.0);

// Main control loop
void loop() {
    // Simulate sensor reading
    double currentValue = readSensor();

    // Update controller
    throttle.updateInput(currentValue);

    // Compute output
    double output = throttle.compute();

    // Apply to actuator
    analogWrite(outputPin, (int)output);
}
```

### Intermediate Examples

#### Smoothing & Filtering (`examples/intermediate/`)
**Purpose:** Demonstrates signal processing features

**Key Features Demonstrated:**
- Input filtering (EMA) for noisy sensors
- Output smoothing for stable control
- Dynamic setpoint changes
- Comparison of raw vs filtered signals

**Hardware Requirements:**
- Arduino board
- Optional: Potentiometer for real sensor input

**Learning Objectives:**
- Understand input filtering benefits
- Learn output smoothing configuration
- Observe signal processing effects
- Handle dynamic target changes

**Configuration Example:**
```cpp
// Enable input filtering (moderate smoothing)
throttle.setInputFilterAlpha(0.2);

// Enable output smoothing (25 units/second max change)
throttle.enableSmoothing(true, 25.0);
```

### Advanced Examples

#### Failsafe Demonstration (`examples/advanced/`)
**Purpose:** Comprehensive failsafe system demonstration

**Key Features Demonstrated:**
- Input validation and timeout handling
- Stability monitoring and timeout
- Error state management
- Failsafe recovery procedures
- Serial error reporting

**Hardware Requirements:**
- Arduino board
- Optional: Sensor for real input validation

**Learning Objectives:**
- Understand failsafe mechanisms
- Learn error state management
- Practice recovery procedures
- Monitor system health

**Failsafe Configuration:**
```cpp
// Configure failsafe parameters
throttle.setFailsafeValue(10.0);           // Safe idle value
throttle.setInputTimeout(2000);            // 2 second timeout
throttle.setStabilityParams(5.0, 5000);   // 5 unit tolerance, 5 second timeout
```

## Real-World Application Examples

### Motor Control (`examples/motor_control/`)

**Application:** DC motor speed control with encoder feedback
**Complexity:** Advanced
**Real-world Use:** Robotics, conveyor systems, fans

#### Features Demonstrated:
- Encoder-based speed measurement
- Closed-loop speed control
- Interrupt-driven encoder reading
- Motor direction control
- Real-time speed monitoring

#### Hardware Requirements:
```
- Arduino board
- DC motor with encoder
- Motor driver (L298N, TB6612, etc.)
- Power supply for motor
- Encoder connections to interrupt pins
```

#### Key Code Concepts:
```cpp
// Encoder interrupt service routine
void encoderISR() {
    // Quadrature decoding logic
    static bool lastA = false;
    bool currentA = digitalRead(ENCODER_A);
    bool currentB = digitalRead(ENCODER_B);

    if (currentA != lastA) {
        if (currentB != currentA) {
            encoderCount++;
        } else {
            encoderCount--;
        }
    }
    lastA = currentA;
}

// Speed calculation
double calculateSpeedRPM() {
    long encoderDelta = encoderCount - lastEncoderCount;
    double countsPerSecond = (double)encoderDelta / (timeDelta / 1000.0);
    double rpm = (countsPerSecond / COUNTS_PER_REV) * 60.0 / GEAR_RATIO;
    return abs(rpm);
}
```

#### PID Tuning Guidelines:
- **Kp:** 0.5-2.0 (responsive but stable)
- **Ki:** 0.02-0.1 (minimal integral for steady-state accuracy)
- **Kd:** 0.1-1.0 (damping for overshoot reduction)

### Temperature Control (`examples/temperature_control/`)

**Application:** Heating/cooling system control
**Complexity:** Advanced
**Real-world Use:** Incubators, ovens, climate control

#### Features Demonstrated:
- Thermistor temperature measurement
- Heating and cooling control
- Safety limits and thermal protection
- Multiple output control
- Temperature monitoring

#### Hardware Requirements:
```
- Arduino board
- NTC thermistor (10k)
- Series resistor (10k)
- Heating element (resistor/heater)
- Cooling element (fan/TEC)
- Relay modules or MOSFET drivers
```

#### Thermistor Circuit:
```
Vcc ────[10k]─┬───[NTC 10k]──── GND
             │
             └─── Analog Input (A0)
```

#### Temperature Calculation:
```cpp
double readTemperature() {
    int adcValue = analogRead(TEMP_SENSOR_PIN);
    double resistance = SERIES_RESISTOR / ((1023.0 / adcValue) - 1.0);

    // Steinhart-Hart approximation
    double steinhart = resistance / THERMISTOR_NOMINAL;
    steinhart = log(steinhart);
    steinhart /= B_COEFFICIENT;
    steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15; // Convert to Celsius

    return steinhart;
}
```

#### PID Tuning Guidelines:
- **Kp:** 5.0-15.0 (aggressive for temperature control)
- **Ki:** 0.1-0.5 (moderate integral for offset correction)
- **Kd:** 1.0-5.0 (significant derivative for overshoot prevention)

### Servo Control (`examples/servo_control/`)

**Application:** Servo position control with feedback
**Complexity:** Intermediate
**Real-world Use:** Robotics, automation, positioning systems

#### Features Demonstrated:
- Potentiometer position feedback
- Precise servo positioning
- Dynamic target changes
- Position error monitoring
- Servo overload protection

#### Hardware Requirements:
```
- Arduino board
- RC servo or positioning servo
- Potentiometer (10k linear)
- Servo power supply (5-6V, 1A+)
- Feedback potentiometer connections
```

#### Servo Control Logic:
```cpp
void setServoPosition(double angle) {
    // Constrain to servo limits
    angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    // Convert angle to pulse width
    int pulseWidth = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

    // Send to servo
    servoMotor.writeMicroseconds(pulseWidth);
}
```

#### PID Tuning Guidelines:
- **Kp:** 1.0-3.0 (responsive positioning)
- **Ki:** 0.01-0.1 (minimal integral windup)
- **Kd:** 0.5-2.0 (damping for smooth movement)

### LED Brightness Control (`examples/led_brightness/`)

**Application:** Adaptive LED brightness control
**Complexity:** Intermediate
**Real-world Use:** Display backlighting, mood lighting, indicators

#### Features Demonstrated:
- Light sensor feedback
- Ambient light compensation
- Breathing effect simulation
- PWM brightness control
- Adaptive lighting control

#### Hardware Requirements:
```
- Arduino board
- LED with current-limiting resistor
- LDR (Light Dependent Resistor)
- Series resistor for LDR (10k)
```

#### Light Sensor Circuit:
```
Vcc ────[LDR]─┬───[10k]──── GND
             │
             └─── Analog Input (A0)
```

#### Adaptive Control Logic:
```cpp
double getBreathingTarget(unsigned long time) {
    // Create slow sine wave for breathing effect
    double breathingFactor = sin(2.0 * PI * time / 8000.0) * 0.3 + 0.7;
    return TARGET_BRIGHTNESS * breathingFactor;
}

// Dynamic target adjustment
double currentTarget = breathingEnabled ? getBreathingTarget(millis()) : TARGET_BRIGHTNESS;
ledController.setTarget(currentTarget);
```

#### PID Tuning Guidelines:
- **Kp:** 0.2-1.0 (gentle brightness changes)
- **Ki:** 0.01-0.05 (smooth adaptation)
- **Kd:** 0.05-0.2 (minimal oscillation)

## Running the Examples

### 1. Arduino IDE Setup
1. Install AutoThrottleNG library
2. Install PID library
3. Open desired example: `File → Examples → AutoThrottleNG`
4. Select appropriate board and port
5. Upload and monitor serial output

### 2. Serial Monitoring
All examples output debug information at 115200 baud:
```
Setpoint | Input | Output | Mode | Error
90.0     | 45.2  | 128.5  | AUTO | OK
```

### 3. Example Modification
Each example includes comments for customization:
- Adjust PID tuning values
- Modify hardware pin assignments
- Change control parameters
- Add custom features

## Example Selection Guide

| Application Type | Recommended Example | Key Features |
|------------------|-------------------|--------------|
| Learning PID | Basic | Simple, educational |
| Motor Control | Motor Control | Encoder feedback |
| Temperature | Temperature Control | Safety limits |
| Positioning | Servo Control | Precise control |
| Lighting | LED Brightness | Adaptive control |
| Signal Processing | Intermediate | Filtering/smoothing |
| Safety Systems | Advanced | Failsafe demonstration |

## Example Modification Guide

### Adapting Examples to Your Hardware

#### Sensor Integration
```cpp
// Generic sensor integration template
double readSensor() {
    // Replace with your sensor reading code
    int rawValue = analogRead(sensorPin);

    // Apply sensor-specific calibration
    double calibratedValue = map(rawValue, sensorMin, sensorMax, 0.0, 100.0);

    // Apply sensor-specific scaling/offset
    calibratedValue = calibratedValue * scaleFactor + offset;

    return calibratedValue;
}
```

#### Actuator Integration
```cpp
// Generic actuator control template
void controlActuator(double output) {
    // Constrain output to actuator limits
    output = constrain(output, actuatorMin, actuatorMax);

    // Apply actuator-specific control logic
    if (actuatorType == PWM) {
        int pwmValue = map(output, 0, 100, 0, 255);
        analogWrite(actuatorPin, pwmValue);
    } else if (actuatorType == RELAY) {
        digitalWrite(actuatorPin, output > 50.0 ? HIGH : LOW);
    } else if (actuatorType == SERVO) {
        int angle = map(output, 0, 100, 0, 180);
        servoMotor.write(angle);
    }
}
```

### Advanced Customization

#### Multi-Sensor Fusion
```cpp
// Combine multiple sensors for improved accuracy
double readFusedSensor() {
    double sensor1 = readSensor1();
    double sensor2 = readSensor2();
    double sensor3 = readSensor3();

    // Weighted average with reliability factors
    double fusedValue = (sensor1 * 0.5) + (sensor2 * 0.3) + (sensor3 * 0.2);

    // Apply outlier rejection
    if (abs(sensor1 - fusedValue) > threshold) {
        // Sensor1 is outlier, reduce its weight
        fusedValue = (sensor2 * 0.4) + (sensor3 * 0.6);
    }

    return fusedValue;
}
```

#### Dynamic PID Tuning
```cpp
// Adjust PID parameters based on operating conditions
void adaptiveTuning(double currentValue, double setpoint) {
    double error = abs(setpoint - currentValue);

    if (error > largeErrorThreshold) {
        // Aggressive tuning for large errors
        controller.setTunings(5.0, 0.1, 1.0);
    } else if (error > smallErrorThreshold) {
        // Moderate tuning for medium errors
        controller.setTunings(2.0, 0.05, 0.5);
    } else {
        // Conservative tuning for small errors
        controller.setTunings(1.0, 0.02, 0.2);
    }
}
```

#### Cascade Control Implementation
```cpp
// Primary controller (outer loop)
AutoThrottleNG primaryController(minPrimary, maxPrimary, kp1, ki1, kd1);

// Secondary controller (inner loop)
AutoThrottleNG secondaryController(minSecondary, maxSecondary, kp2, ki2, kd2);

void cascadeControl() {
    // Primary loop: position → velocity setpoint
    double primaryOutput = primaryController.compute();

    // Set secondary setpoint based on primary output
    secondaryController.setTarget(primaryOutput);

    // Secondary loop: velocity control
    double finalOutput = secondaryController.compute();

    // Apply final output to actuator
    controlActuator(finalOutput);
}
```

## Troubleshooting Example Modifications

### Common Integration Issues

#### Sensor Noise Problems
```cpp
// Problem: Noisy sensor causing erratic control
// Solution: Adjust input filtering

// Too little filtering - noisy response
controller.setInputFilterAlpha(0.9); // Light filtering

// Better: Moderate filtering for most sensors
controller.setInputFilterAlpha(0.3);

// Too much filtering - sluggish response
controller.setInputFilterAlpha(0.05); // Heavy filtering
```

#### Actuator Response Issues
```cpp
// Problem: Actuator too slow/jerky
// Solution: Adjust output smoothing

// No smoothing - immediate response
controller.enableSmoothing(false);

// Smooth response - reduce jerkiness
controller.enableSmoothing(true, 50.0); // 50 units/second

// Very smooth - slow response
controller.enableSmoothing(true, 10.0); // 10 units/second
```

#### System Oscillation
```cpp
// Problem: System oscillates around setpoint
// Solution: Reduce PID gains or add filtering

// Reduce proportional gain
controller.setTunings(controller.getKp() * 0.8, controller.getKi(), controller.getKd());

// Add derivative gain to damp oscillations
controller.setTunings(controller.getKp(), controller.getKi(), controller.getKd() * 1.2);

// Increase input filtering
controller.setInputFilterAlpha(0.1); // More filtering
```

### Performance Monitoring

#### Real-time Performance Analysis
```cpp
void monitorPerformance() {
    static unsigned long lastMonitor = 0;
    static unsigned long loopCount = 0;
    static double maxError = 0;

    loopCount++;

    // Track maximum error
    double currentError = abs(controller.getSetpoint() - controller.getFilteredInput());
    maxError = max(maxError, currentError);

    // Report every 5 seconds
    if (millis() - lastMonitor >= 5000) {
        Serial.print("Loops/sec: ");
        Serial.println(loopCount / 5.0);

        Serial.print("Max Error: ");
        Serial.println(maxError);

        Serial.print("Stability: ");
        Serial.println(controller.isStable() ? "Stable" : "Unstable");

        // Reset counters
        loopCount = 0;
        maxError = 0;
        lastMonitor = millis();
    }
}
```

### Safety Integration

#### Emergency Stop Implementation
```cpp
// Hardware emergency stop button
const int emergencyStopPin = 2;

// Software emergency stop flag
bool emergencyStop = false;

void checkEmergencyStop() {
    if (digitalRead(emergencyStopPin) == LOW) {
        emergencyStop = true;
        controller.setMode(MANUAL);
        controller.setFailsafeValue(0.0); // Safe stop position

        Serial.println("EMERGENCY STOP ACTIVATED");
    }
}

void resetEmergencyStop() {
    if (emergencyStop && digitalRead(emergencyStopPin) == HIGH) {
        // Require manual reset
        emergencyStop = false;
        controller.clearErrorState();
        controller.setMode(AUTOMATIC);

        Serial.println("EMERGENCY STOP RESET");
    }
}
```

## Best Practices for Example Modification

### Development Workflow
1. **Start Simple**: Begin with basic example, verify it works
2. **Incremental Changes**: Modify one parameter/feature at a time
3. **Test Each Change**: Verify system stability after each modification
4. **Document Changes**: Keep notes on what works and what doesn't

### Hardware Considerations
1. **Power Supply**: Ensure adequate power for all components
2. **Signal Integrity**: Use appropriate wiring and shielding
3. **Sensor Placement**: Position sensors for accurate measurements
4. **Actuator Loading**: Consider mechanical loads and power requirements

### Software Considerations
1. **Timing**: Maintain consistent loop timing
2. **Resource Usage**: Monitor memory and CPU usage
3. **Error Handling**: Implement appropriate error recovery
4. **Monitoring**: Add debugging output for troubleshooting

### Testing Methodology
1. **Unit Testing**: Test individual components separately
2. **Integration Testing**: Test combined system operation
3. **Edge Case Testing**: Test extreme conditions and error scenarios
4. **Long-term Testing**: Verify stability over extended periods

These examples provide a foundation for understanding and implementing AutoThrottleNG in real-world applications, with clear progression from basic concepts to advanced implementations. The modification guides and best practices ensure successful adaptation to specific project requirements.

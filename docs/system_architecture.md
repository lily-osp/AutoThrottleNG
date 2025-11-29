# AutoThrottleNG System Architecture

## Table of Contents

- [Overview](#overview)
- [Core Architecture Components](#core-architecture-components)
- [Signal Processing Pipeline](#signal-processing-pipeline)
- [Safety & Failsafe System](#safety--failsafe-system)
- [Class Structure](#class-structure)
- [Data Flow Architecture](#data-flow-architecture)
- [Memory Architecture](#memory-architecture)
- [Timing Architecture](#timing-architecture)
- [Error Handling Architecture](#error-handling-architecture)
- [Configuration Architecture](#configuration-architecture)
- [Integration Architecture](#integration-architecture)

## Overview

AutoThrottleNG is a PID-based control library designed for Arduino platforms, providing throttle/output control with safety and filtering features. The library integrates Arduino PID control with signal processing and safety mechanisms.

## Core Architecture Components

### 1. PID Controller Core
- **Library Integration**: Built on Arduino's standard PID library (PID_v1)
- **Algorithm**: Implements Proportional-Integral-Derivative control
- **Configuration**: Supports P_ON_E (Proportional on Error) and P_ON_M (Proportional on Measurement)

### 2. Signal Processing Pipeline

```
Raw Input → Input Filter → PID Algorithm → Output Smoothing → Final Output
     ↓           ↓             ↓             ↓              ↓
  Sensor     EMA Filter   PID Controller  Ramp Limiting   Actuator
  Reading    (Optional)   Computation     (Optional)     Control
```

### 3. Safety & Failsafe System
- **Error States**: NONE, INPUT_INVALID, INPUT_TIMEOUT, STABILITY_TIMEOUT
- **Failsafe Value**: Configurable safe output when errors occur
- **Automatic Recovery**: Manual intervention required to clear errors

## Class Structure

### AutoThrottleNG Class
```cpp
class AutoThrottleNG {
private:
    // PID Controller
    PID _pid;

    // Signal Processing
    double _pidInput;      // Filtered input to PID
    double _pidOutput;     // Raw PID output
    double _pidSetpoint;   // Target value

    // Control Parameters
    double _minOutputLimit;
    double _maxOutputLimit;
    double _currentSmoothedOutput;

    // Filtering & Smoothing
    double _inputFilterAlpha;
    bool _smoothingEnabled;
    double _smoothingRate;

    // Failsafe System
    Error _errorState;
    double _failsafeOutputValue;
    unsigned long _inputTimeoutMillis;
    unsigned long _stabilityTimeoutMillis;

    // Helper Methods
    double applyInputFilter(double);
    double applyOutputSmoothing(double);
    void updateFailsafeState(Error);
    void checkTimeouts(unsigned long);
};
```

## Data Flow Architecture

### 1. Input Processing
```
Sensor Reading → Validity Check → Input Filtering → PID Input
                     ↓
              Error: INPUT_INVALID
```

### 2. PID Computation
```
PID Input + Setpoint → PID Algorithm → Raw Output → Saturation Check
                                                         ↓
                                                  Output Saturated Flag
```

### 3. Output Processing
```
Raw PID Output → Output Smoothing → Range Limiting → Final Output
                    ↓                      ↓
         Rate Limiting         Min/Max Constraints
```

### 4. Failsafe Integration
```
All Stages → Error Detection → Failsafe Override → Safe Output
              ↓                        ↓
       Timeout Checks           Failsafe Value
       Stability Checks
```

## Memory Architecture

### Static Memory Allocation
- **Core Variables**: PID controller instance, control parameters
- **State Variables**: Current outputs, error states, timing variables
- **Configuration**: Limits, timeouts, filter parameters

### Dynamic Memory Usage (Arduino Uno)
- **Program Memory**: ~7-11KB depending on features used
- **Dynamic Memory**: ~450-984 bytes depending on example complexity
- **Stack Usage**: Minimal, suitable for resource-constrained environments

## Timing Architecture

### Sample Time Management
- **PID Sample Time**: Configurable via `setSampleTime()`
- **Compute Frequency**: Called in main loop, typically 50-1000ms intervals
- **Timeout Monitoring**: Millisecond-precision using `millis()`

### Execution Flow
```
Main Loop:
├── Read Sensors (updateInput())
├── Compute Control (compute())
├── Apply Output (to actuator)
├── Monitor Status (getThrottle(), etc.)
└── Handle Errors (isInErrorState(), clearErrorState())
```

## Error Handling Architecture

### Error State Machine
```
NORMAL OPERATION
       ↓ (Error Detected)
   ERROR STATE ACTIVE
       ↓ (clearErrorState())
NORMAL OPERATION (RESUMED)
```

### Error Types
1. **INPUT_INVALID**: NaN/Infinity sensor readings
2. **INPUT_TIMEOUT**: No sensor updates within timeout period
3. **STABILITY_TIMEOUT**: System unstable beyond tolerance duration

## Configuration Architecture

### Runtime Configuration
- **PID Tuning**: `setTunings(kp, ki, kd, pOn)`
- **Output Limits**: `setOutputLimits(min, max)`
- **Controller Direction**: `setControllerDirection(DIRECT/REVERSE)`
- **Sample Time**: `setSampleTime(millis)`

### Feature Configuration
- **Input Filtering**: `setInputFilterAlpha(alpha)`
- **Output Smoothing**: `enableSmoothing(enable, rate)`
- **Failsafe Settings**: `setFailsafeValue()`, `setInputTimeout()`, `setStabilityParams()`

## Integration Architecture

### Arduino Ecosystem Integration
- **Standard Libraries**: Uses Arduino PID library v1.2.0+, Arduino core libraries
- **Board Compatibility**: AVR (Uno, Mega, Leonardo), ESP32, ARM (Due, Zero), SAMD (MKR)
- **IDE Support**: Arduino IDE 1.8+, PlatformIO, VS Code with Arduino extension
- **Framework Compatibility**: Arduino framework on all supported platforms

### Hardware Abstraction Layer
- **Sensor Interface**: Generic double-precision input for any sensor type
- **Actuator Interface**: Generic double-precision output supporting PWM, DAC, serial protocols
- **Timer Interface**: Arduino `millis()` for consistent timing across all platforms
- **Interrupt Handling**: Compatible with Arduino interrupt system for encoder inputs

### Library Dependencies
- **Primary**: Arduino PID library (automatically managed)
- **Core**: Arduino.h, math.h for standard functions
- **Optional**: Servo.h for servo control examples
- **No External**: No additional libraries required for core functionality

### Platform-Specific Considerations
- **Memory Constraints**: Optimized for AVR (2KB RAM) to ESP32 (512KB+ RAM)
- **Timing Resolution**: Uses millisecond precision suitable for control applications
- **Interrupt Availability**: Supports encoder interfaces on interrupt-capable pins
- **PWM Resolution**: Adapts to available PWM resolution (8-bit AVR to 16-bit ESP32)

### Development Environment Integration
- **Library Manager**: Compatible with Arduino Library Manager installation
- **Include Structure**: Standard Arduino library include paths
- **Build System**: Integrates with Arduino build system and compiler flags
- **Debug Support**: Compatible with Serial debugging and monitoring tools

### Extensibility Architecture
- **Modular Design**: Core PID functionality separated from signal processing
- **Plugin Architecture**: Easy to extend with custom signal processing
- **Configuration Flexibility**: Runtime configuration without recompilation
- **Error Handling**: Comprehensive error reporting for integration debugging

This architecture ensures AutoThrottleNG is resource-efficient, suitable for a wide range of control applications while maintaining safety and reliability. The modular design allows for easy integration into existing projects and extension for specialized applications.

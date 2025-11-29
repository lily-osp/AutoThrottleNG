# AutoThrottleNG Arduino Library v1.2.0

## Overview

**AutoThrottleNG** is a comprehensive Arduino library for advanced PID-based control systems. It provides throttle/output control with sophisticated signal processing, multi-layer safety mechanisms, operational modes, and auto-recovery features.

The library is designed for applications requiring precise, reliable closed-loop control:

- **Robotics**: Motor speed control, position servos, motion systems
- **Aerospace**: Drone stabilization, altitude control, navigation
- **Industrial**: Process control, valve positioning, conveyor systems
- **Automotive**: Cruise control, steering assistance, throttle management
- **Scientific**: Temperature regulation, flow control, experimental systems

AutoThrottleNG handles complex control challenges including sensor noise filtering, mechanical stress prevention, system stability monitoring, multi-mode operation, and intelligent failure handling, enabling focus on application logic rather than low-level control implementation.

---

## Core Concepts

AutoThrottleNG integrates several key control and signal processing techniques:

1. **PID Control (Proportional-Integral-Derivative)**
   
   * The library uses the standard Arduino `PID` library (requiring `PID.h`) as its core control algorithm.
   
   * It continuously calculates an error value (`e(t)`) as the difference between the desired `Setpoint` and the current (filtered) `Input`.
   
   * It computes an `Output` value aiming to minimize this error over time.
   
   * **Formula:** The discrete PID algorithm can be represented as:
     
     ```
     Error[t] = Setpoint - Input[t]
     
     P = Kp * Error[t]
     I = I[t-1] + Ki * Error[t] * Δt
     D = Kd * (Error[t] - Error[t-1]) / Δt
     
     Output[t] = P + I + D
     ```
     
     Where:
     
     * `Kp`: Proportional Gain (reacts to current error)
     * `Ki`: Integral Gain (accumulates past errors to eliminate steady-state offset)
     * `Kd`: Derivative Gain (reacts to the rate of change of the error, dampens oscillations)
     * `Δt`: Time interval between calculations (Sample Time)
   
   * **Proportional on Measurement (P_ON_M):** The underlying PID library also supports P_ON_M, where the proportional term is calculated based on the change in input rather than the change in error. This can reduce "derivative kick" when the setpoint changes suddenly. AutoThrottleNG allows configuring this via the constructor or `setTunings`.

2. **Input Filtering (Exponential Moving Average - EMA)**
   
   * **Purpose:** Sensor readings are often noisy. EMA filtering smooths the raw input signal before it's fed to the PID controller, leading to more stable control and preventing the derivative term (Kd) from overreacting to noise.
   * **Formula:**
     
     ```
     FilteredValue[t] = α * RawValue[t] + (1 - α) * FilteredValue[t-1]
     ```
     
     Where:
     * `α` (alpha) is the smoothing factor (0.0 to 1.0), configured via `setInputFilterAlpha()`.
     * `α = 0.0`: No filtering (FilteredValue = PreviousFilteredValue) - *Note: Library uses 0 to mean pass-through raw value.*
     * `α ≈ 1.0`: Minimal filtering (FilteredValue ≈ RawValue)
     * Smaller `α`: More smoothing, but introduces more lag (delay) in the filtered signal's response to changes.
     * `α = 0.0` is treated as "filter disabled" by the library.
   * **Trade-off:** Increased smoothing (lower `α`) reduces noise sensitivity but makes the system slower to respond to genuine changes in the input.

3. **Output Smoothing (Ramp Rate Limiting)**
   
   * **Purpose:** Prevents the final throttle/output value from changing too abruptly, even if the PID controller calculates a large change. This is crucial for systems sensitive to sudden acceleration/deceleration (e.g., preventing jerky movements, mechanical stress, current spikes).
   * **Mechanism:** The library limits the maximum rate of change of the *final* output value per second.
   * **Logic:**
     1. Calculate the time elapsed (`Δt`) since the last `compute()` call.
     2. Calculate the maximum allowable change for this interval:
        `MaxChange = MaxRatePerSecond * Δt / 1000.0`
     3. Compare the raw PID output (`TargetOutput`) to the previous smoothed output (`SmoothedOutput[t-1]`).
     4. Limit the new smoothed output:
        `SmoothedOutput[t] = constrain(TargetOutput, SmoothedOutput[t-1] - MaxChange, SmoothedOutput[t-1] + MaxChange)`
   * **Configuration:** Enabled via `enableSmoothing(true, maxRatePerSecond)`.
   * **Trade-off:** Increased smoothing (lower `maxRatePerSecond`) results in smoother transitions but limits how quickly the system can respond to requests for large output changes.

---

## Key Features

* **Standard PID Integration:** Leverages the well-established Arduino `PID` library (`PID.h`) for core control logic.
* **Input Filtering:** Built-in optional Exponential Moving Average (EMA) filter to smooth noisy sensor inputs before they reach the PID.
* **Output Smoothing:** Optional time-based ramp rate limiting on the final output to prevent sudden jumps or jerky behavior.
* **Operational Modes:** Multiple operational modes for different applications:
  * **NORMAL:** Standard PID operation
  * **SAFE_MODE:** Conservative settings for maximum safety
  * **LEARNING_MODE:** Adaptive parameter tuning
  * **REVERSE_MODE:** Reverse operation for specific applications
  * **MAINTENANCE_MODE:** Diagnostic operations
  * **CALIBRATION_MODE:** System calibration
* **Auto-Recovery System:** Intelligent automatic error recovery with configurable parameters
* **Failsafe Mechanisms:**
  * **Input Validity:** Rejects `NaN` or `Infinity` sensor readings.
  * **Input Timeout:** Triggers an error if `updateInput()` isn't called within a configurable duration.
  * **Stability Timeout:** Triggers an error if the system's error (`|Setpoint - FilteredInput|`) remains larger than a tolerance for a configurable duration.
  * **Configurable Failsafe Output:** Sets a predefined, safe output value when any error state is active.
  * **Clear Error Reporting:** Provides methods to check the current error state.
  * **Manual Error Reset:** Requires user code intervention (`clearErrorState()`) to resume normal operation after a failsafe.
* **Status Monitoring:** Provides getters for raw input, filtered input, raw PID output, smoothed throttle output, PID gains, mode, setpoint, error state, stability status, output saturation, and last update time.
* **Flexible Configuration:** Allows runtime adjustment of PID tunings, output limits, sample time, controller direction, filter parameters, smoothing parameters, and failsafe settings.
* **Robust Design:** Includes checks and constraints to handle edge cases and invalid configurations.

---

## Dependencies

* **Arduino PID Library:** Requires the standard "PID" library by Brett Beauregard (often referred to as PID v1) to be installed in your Arduino IDE. This library provides `PID.h`. Install it via the Arduino Library Manager (Search for "PID").

---

## Installation

1. **Install PID Library:** If not already installed, open the Arduino IDE, go to **Tools -> Manage Libraries...**, search for "PID" by Brett Beauregard, and click **Install**.
2. **Download AutoThrottleNG:** Download the `AutoThrottleNG` library files (e.g., as a ZIP from GitHub).
3. **Install AutoThrottleNG:** In the Arduino IDE, go to **Sketch -> Include Library -> Add .ZIP Library...** and select the downloaded ZIP file.
4. **Restart IDE:** It's often good practice to restart the Arduino IDE after installing new libraries.
5. **Examples:** Check **File -> Examples -> AutoThrottleNG** for example sketches.

## Documentation

AutoThrottleNG includes documentation in the `docs/` folder:

### [Complete Documentation Index](docs/readme.md)
Central hub for all documentation with learning paths and cross-references.

### [Usage Guide](docs/usage.md)
- Installation and setup instructions
- Basic and advanced usage patterns
- Configuration options and PID tuning
- Performance optimization techniques

### [Examples Guide](docs/examples.md)
Detailed walkthrough of all 7 example sketches:
- Basic Control: Fundamental PID concepts
- Intermediate: Signal processing (filtering/smoothing)
- Advanced: Comprehensive failsafe systems
- Real-World Applications:
  - DC motor speed control with encoder feedback
  - Temperature regulation with heating/cooling
  - Servo position control with potentiometer
  - Adaptive LED brightness control
  - Operational modes and auto-recovery demonstration

### [Troubleshooting Guide](docs/troubleshooting.md)
- Compilation error resolution
- Runtime issue diagnosis
- Failsafe troubleshooting
- Hardware-specific problems
- Advanced debugging techniques

### [Detailed Explanation](docs/explanation.md)
- PID control theory and implementation
- Signal processing algorithms
- Failsafe system architecture
- Controller modes and features

### [System Architecture](docs/system_architecture.md)
- Internal design patterns
- Data flow and component interaction
- Memory management and timing
- Integration approaches

### [Internal Mechanisms](docs/mechanism.md)
- Algorithm implementations
- Technical specifications
- Performance characteristics
- Development considerations

---

## API Documentation

### Constructor

```cpp
AutoThrottleNG(double minOutput, double maxOutput,
               double kp, double ki, double kd,
               int POn = P_ON_E, int direction = DIRECT);
```

* Initializes the AutoThrottleNG controller.
* **Parameters:**
  * `minOutput` (double): The minimum value the PID controller's output (`_pidOutput`) and the final throttle (`getThrottle()`) can have.
  * `maxOutput` (double): The maximum value the PID controller's output and final throttle can have.
  * `kp` (double): Initial Proportional tuning gain.
  * `ki` (double): Initial Integral tuning gain.
  * `kd` (double): Initial Derivative tuning gain.
  * `POn` (int): Proportional calculation mode. Use `P_ON_E` (Proportional on Error, default) or `P_ON_M` (Proportional on Measurement). These constants are typically defined by the underlying `PID.h`.
  * `direction` (int): Controller direction. Use `DIRECT` (increase output to increase input, default) or `REVERSE` (increase output to decrease input). Constants defined by `PID.h`.

### Core Runtime Methods

```cpp
void setTarget(double target);
```

* Sets the desired target value (setpoint) for the PID controller.
* NaN or Infinity values are ignored.
* **Parameters:**
  * `target` (double): The target value the controller should aim for, in the same units as the input signal.

```cpp
void updateInput(double rawInputValue);
```

* Provides the latest raw sensor reading to the controller.
* This value is filtered (if enabled) before being used by the PID.
* **Must be called regularly** and more frequently than the configured Input Timeout.
* NaN or Infinity values are rejected, and the `INPUT_INVALID` error state is set.
* **Parameters:**
  * `rawInputValue` (double): The latest raw reading from the sensor.

```cpp
double compute();
```

* The main workhorse method. Call this in every iteration of your `loop()`.
* Checks for failsafe conditions (timeouts).
* If no failsafe is active, it triggers the internal `_pid.Compute()` method (which uses the filtered input and setpoint).
* Applies output smoothing (if enabled) to the raw PID output.
* Clamps the final output to the defined limits.
* Handles setting the failsafe output value if an error state is active.
* **Returns:**
  * (double): The final, calculated throttle value (smoothed or failsafe value), constrained within `minOutputLimit` and `maxOutputLimit`.

```cpp
void reset();
```

* Resets the internal state of the PID controller (typically clears the integral sum by cycling the mode) and resets AutoThrottleNG's failsafe timers and error state.
* **Does NOT reset:** PID gains (Kp, Ki, Kd), output limits, setpoint, filter settings, smoothing settings, or failsafe configuration values. Call `setTarget()` again after `reset()` if needed.

### PID Configuration

```cpp
void setTunings(double kp, double ki, double kd, int POn = -1);
```

* Sets the PID gains and optionally the Proportional mode.
* **Parameters:**
  * `kp`, `ki`, `kd` (double): New tuning gains.
  * `POn` (int): Optional. Set to `P_ON_E` or `P_ON_M` to change mode, or leave as -1 (or omit) to keep the current Proportional mode.

```cpp
void setOutputLimits(double minOutput, double maxOutput);
```

* Sets the minimum and maximum allowed values for the raw PID output and the final smoothed throttle. Clamps the current output and failsafe value if necessary.
* **Parameters:**
  * `minOutput`, `maxOutput` (double): New lower and upper output bounds. `minOutput` must be less than `maxOutput`.

```cpp
void setControllerDirection(int direction);
```

* Sets the controller direction.
* **Parameters:**
  * `direction` (int): `DIRECT` or `REVERSE`.

```cpp
void setSampleTime(int sampleTimeMillis);
```

* Sets how often (in milliseconds) the PID algorithm attempts to calculate a new output.
* **Parameters:**
  * `sampleTimeMillis` (int): The desired interval > 0.

```cpp
void setMode(int mode);
```

* Sets the PID operating mode. An active failsafe will override this and force `MANUAL`.
* **Parameters:**
  * `mode` (int): `AUTOMATIC` (PID enabled) or `MANUAL` (PID calculations disabled, output holds).

### Filtering Configuration

```cpp
void setInputFilterAlpha(double alpha);
```

* Configures the Exponential Moving Average (EMA) input filter.
* **Parameters:**
  * `alpha` (double): Smoothing factor, clamped between 0.0 and 1.0.
    * `0.0`: Filter disabled (raw input used directly).
    * Values closer to `0.0`: More smoothing, more lag.
    * Values closer to `1.0`: Less smoothing, less lag.

### Smoothing Configuration

```cpp
void enableSmoothing(bool enable, double maxRatePerSecond = 10.0);
```

* Enables or disables time-based ramp rate limiting on the final output.
* **Parameters:**
  * `enable` (bool): `true` to enable, `false` to disable.
  * `maxRatePerSecond` (double): The maximum allowed change in output units per second when smoothing is enabled. Must be > 0. Default: 10.0.

### Failsafe Configuration

```cpp
void setFailsafeValue(double value);
```

* Sets the output value that `getThrottle()` will return when any failsafe condition is active. The value is constrained by the output limits.
* **Parameters:**
  * `value` (double): The desired failsafe output value.

```cpp
void setInputTimeout(unsigned long durationMillis);
```

* Sets the maximum allowed time (in milliseconds) between successful calls to `updateInput()`. If this time is exceeded, the `INPUT_TIMEOUT` error state is triggered.
* **Parameters:**
  * `durationMillis` (unsigned long): Timeout duration in ms. Set to `0` to disable this check.

```cpp
void setStabilityParams(double tolerance, unsigned long durationMillis);
```

* Configures the stability check failsafe.
* **Parameters:**
  * `tolerance` (double): The maximum absolute difference allowed between the setpoint (`_pidSetpoint`) and the filtered input (`_pidInput`) for the system to be considered stable. Must be >= 0.
  * `durationMillis` (unsigned long): The maximum time (in ms) the system's error can continuously exceed the `tolerance` before the `STABILITY_TIMEOUT` error state is triggered. Set to `0` to disable this check.

### Status & Debugging Methods

```cpp
double getThrottle() const;
```

* Returns the final calculated throttle output value after PID computation, smoothing (if enabled), and failsafe checks. This is the value you should apply to your actuator.

```cpp
double getRawPIDOutput() const;
```

* Returns the raw output value calculated by the internal `_pid.Compute()` method, *before* output smoothing is applied. Useful for debugging PID behavior.

```cpp
double getFilteredInput() const;
```

* Returns the input value *after* the EMA filter has been applied. This is the value actually used by the PID algorithm.

```cpp
double getRawInput() const;
```

* Returns the last *valid* raw input value provided via `updateInput()`, before filtering.

```cpp
bool isStable() const;
```

* Checks if the *current* absolute difference between the setpoint and the filtered input is within the configured `stabilityToleranceValue`. Note that this is an instantaneous check and doesn't relate directly to the `STABILITY_TIMEOUT` error state, which requires the condition to persist.

```cpp
double getKp() const;
double getKi() const;
double getKd() const;
```

* Return the current Proportional, Integral, or Derivative gain being used by the PID controller.

```cpp
int getMode() const;
```

* Returns the current operating mode of the PID controller (`AUTOMATIC` or `MANUAL`).

```cpp
double getSetpoint() const;
```

* Returns the current target setpoint value stored within AutoThrottleNG.

```cpp
bool isSaturated() const;
```

* Returns `true` if the last *raw* PID output (`_pidOutput`) was at the minimum or maximum output limit, `false` otherwise. Indicates the PID is trying to command an output beyond its limits.

```cpp
unsigned long getLastUpdateTime() const;
```

* Returns the `millis()` timestamp of when `updateInput()` was last called with a *valid* (non-NaN, non-Inf) value. Useful for debugging input timeout issues.

### Error Handling (Failsafe Status)

```cpp
AutoThrottleNG::Error getErrorState() const;
```

* Returns the current failsafe error state.
* **Return Values (enum `AutoThrottleNG::Error`):**
  * `NONE`: No error active.
  * `INPUT_INVALID`: Last call to `updateInput()` received NaN or Infinity.
  * `INPUT_TIMEOUT`: `updateInput()` hasn't been called within the configured timeout.
  * `STABILITY_TIMEOUT`: System error has exceeded tolerance for too long.

**Operational Modes (enum `AutoThrottleNG::OperationalMode`):**
  * `NORMAL`: Standard PID control operation
  * `SAFE_MODE`: Conservative settings for maximum safety
  * `LEARNING_MODE`: Adaptive parameter tuning
  * `REVERSE_MODE`: Reverse operation for specific applications
  * `MAINTENANCE_MODE`: Diagnostic operations
  * `CALIBRATION_MODE`: System calibration

```cpp
bool isInErrorState() const;
```

* Returns `true` if `getErrorState()` is anything other than `Error::NONE`, `false` otherwise. Convenient check before taking failsafe actions.

```cpp
void clearErrorState();
```

* Manually resets the error state back to `Error::NONE`.
* **Crucially, you MUST call this function in your code** once the condition causing the failsafe has been resolved (or is assumed resolved) to allow the controller to resume `AUTOMATIC` operation. The library will *not* automatically clear most error states.

### Operational Mode Configuration

```cpp
void setOperationalMode(OperationalMode mode);
```

* Sets the operational mode of the controller.
* **Parameters:**
  * `mode` (OperationalMode): The operational mode (NORMAL, SAFE_MODE, LEARNING_MODE, REVERSE_MODE, MAINTENANCE_MODE, CALIBRATION_MODE)

```cpp
OperationalMode getOperationalMode() const;
```

* Gets the current operational mode.
* **Returns:** Current operational mode

### Auto-Recovery Configuration

```cpp
void enableAutoRecovery(bool enable, unsigned long recoveryDelayMs = 5000, uint8_t maxRecoveryAttempts = 3);
```

* Enables or disables automatic error recovery.
* **Parameters:**
  * `enable` (bool): True to enable auto-recovery, false to disable
  * `recoveryDelayMs` (unsigned long): Delay before attempting recovery (default: 5000ms)
  * `maxRecoveryAttempts` (uint8_t): Maximum recovery attempts before staying in failsafe (default: 3)

```cpp
bool isAutoRecoveryEnabled() const;
```

* Gets the current auto-recovery status.
* **Returns:** True if auto-recovery is enabled

```cpp
uint8_t getRecoveryAttemptCount() const;
```

* Gets the number of recovery attempts made since last successful operation.
* **Returns:** Number of recovery attempts

---

## Control Flow Diagram (Conceptual)

```
  +-------------------+      +-----------------+      +---------------------+      +------------------+
  | Raw Sensor Signal | ---> | updateInput()   | ---> | Input Filter (EMA)  | ---> | Filtered Input   | ----+
  +-------------------+      | (Validity Check)|      | (if alpha > 0)    |      | (_pidInput)      |     |
                             +-----------------+      +---------------------+      +------------------+     |
                                                                                                           | V
                               +-------------------+   +-------------------+     +---------------------+    | [PID]
                               | setTarget()       |-->| Target Setpoint   | --> | PID Algorithm       | <--+
                               +-------------------+   | (_pidSetpoint)    |     | (_pid.Compute())    |
                                                       +-------------------+     +-------+-------------+
                                                                                       | Output
                                                                                       V (_pidOutput)
+-------------------+      +--------------------+      +---------------------+      +------------------+
| Final Throttle    | <--- | Output Smoothing   | <--- | Raw PID Output      | <--- | (Check Output    |
| Value             |      | (Ramp Limit)       |      |                     |      |  Saturation)     |
| (to Actuator)     |      | (if enabled)       |      |                     |      +------------------+
+-------------------+      +-----+--------------+      +---------------------+
                              ^
                              | [Failsafe Override]
                              |
+-------------------+ <-------+
| Failsafe Value    |
| (if Error State!) |
+-------------------+

* Failsafe Checks (Timeouts) occur within compute() before PID calculation.
* If Error State is active, PID is bypassed, and Failsafe Value is used (potentially smoothed).
```

---

## Notes on Usage

* **PID Tuning is Crucial:** You *must* tune the Kp, Ki, and Kd values for your specific system. Start with Kp and gradually add Ki and Kd. Use methods like Ziegler-Nichols or trial-and-error. Monitor the `getRawPIDOutput()` and `getFilteredInput()` to understand behavior.
* **Input Filtering:** Use `setInputFilterAlpha()` to reduce noise. Start with a small value (e.g., 0.1-0.3) if noise is significant. Be aware this adds lag. If your sensor is clean, leave alpha at 0.0 (disabled).
* **Output Smoothing:** Use `enableSmoothing()` if you need smooth output transitions. Adjust the `maxRatePerSecond` based on your system's requirements – lower values give more smoothing but limit responsiveness.
* **Failsafes:**
  * Configure timeouts (`setInputTimeout`, `setStabilityParams`) appropriate for your system's response time and sensor update rate. Don't set them too short or too long.
  * Set a *safe* `failsafeValue` (e.g., 0% throttle, hover throttle).
  * Your main loop *must* check `isInErrorState()` and potentially take further action (e.g., disarm, alert).
  * You *must* call `clearErrorState()` to recover from a failsafe once the underlying issue is resolved.
* **`compute()` Frequency:** Call `compute()` in every `loop()` iteration for consistent timing and responsiveness of smoothing and PID calculations (the internal PID library manages its own sample time).
* **`updateInput()` Frequency:** Call `updateInput()` as often as you get new sensor readings, and definitely faster than the configured `inputTimeoutMillis`.
* **Units:** Ensure consistency! The units for `setTarget`, `updateInput`, and `stabilityTolerance` must all match. The output units will be defined by `minOutput`/`maxOutput`.
* **Constants:** Verify the exact names/values of `DIRECT`, `REVERSE`, `AUTOMATIC`, `MANUAL`, `P_ON_E`, `P_ON_M` as defined in the `PID.h` file included with your Arduino core or the installed PID library.

---

## Development & Testing

### Comprehensive Makefile
The library includes a full-featured Makefile for development and testing:

```bash
# Compile all examples
make compile-examples

# Compile specific examples
make compile-basic        # Basic PID control
make compile-motor        # DC motor speed control
make compile-temperature  # Temperature regulation
make compile-servo        # Servo position control
make compile-led          # LED brightness control

# Upload examples to Arduino
make upload-basic PORT=/dev/ttyACM0
make upload-motor PORT=/dev/ttyACM0

# Development tools
make install-deps         # Install Arduino dependencies
make test-all            # Run complete tests
make format              # Format code with clang-format
make clean               # Remove build artifacts
```

### Library Structure
```
AutoThrottleNG/
├── src/                          # Source code
│   ├── AutoThrottleNG.h         # Library header
│   └── AutoThrottleNG.cpp       # Library implementation
├── examples/                     # Example sketches
│   ├── basic/                   # Fundamental PID concepts
│   ├── intermediate/            # Signal processing
│   ├── advanced/                # Failsafe systems
│   ├── motor_control/           # DC motor speed control
│   ├── temperature_control/     # Heating/cooling systems
│   ├── servo_control/           # Position control
│   └── led_brightness/          # Adaptive lighting
├── docs/                        # Comprehensive documentation
├── library.properties           # Arduino library metadata
├── keywords.txt                 # Arduino IDE syntax highlighting
├── Makefile                     # Development automation
├── Readme.md                    # This file
└── .gitignore                   # Git ignore rules
```

### Quality Assurance
- **Automated Testing**: Makefile targets for compilation verification
- **Cross-Platform**: Tested on AVR, ESP32, and ARM architectures
- **Comprehensive Examples**: 7 real-world examples with hardware specs
- **Professional Documentation**: Complete reference with troubleshooting

## Version 1.2.0 Features

### Operational Modes & Auto-Recovery
- **6 Operational Modes**: NORMAL, SAFE_MODE, LEARNING_MODE, REVERSE_MODE, MAINTENANCE_MODE, CALIBRATION_MODE
- **Auto-Recovery System**: Intelligent automatic error recovery with configurable delay and attempt limits
- **Mode-Specific Behavior**: Each operational mode applies appropriate PID settings and safety measures
- **Interactive Mode Switching**: Runtime operational mode changes with automatic reconfiguration

### Enhanced Examples
- **8 Complete Examples**: Including new operational modes demonstration with interactive controls
- **Real-World Applications**: Motor control, temperature regulation, servo positioning, adaptive lighting
- **Error Simulation**: Built-in error simulation for testing auto-recovery features
- **Hardware Integration**: Complete wiring diagrams and component specifications

### Enhanced Documentation
- **Documentation Suite**: 7 detailed guides covering all aspects
- **Clean Presentation**: Technical writing without unnecessary embellishments
- **Table of Contents**: Every document includes navigation
- **Cross-References**: Guides link to related topics
- **Structured Learning**: From beginner basics to advanced techniques

### Development Tools
- **Enhanced Makefile**: Support for all examples with automated testing
- **Standard Structure**: Proper Arduino library organization
- **Testing**: Automated compilation and syntax checking
- **Cross-Platform Support**: Compatible with Arduino IDE, PlatformIO, VS Code

---

## Contributing

We welcome contributions to AutoThrottleNG! For detailed contribution guidelines, please see our [Contributing Guide](CONTRIBUTING.md).

### Quick Start for Contributors

1. **Read the [Contributing Guide](CONTRIBUTING.md)** for detailed instructions
2. **Check [GitHub Issues](https://github.com/yourusername/AutoThrottleNG/issues)** for open tasks
3. **Fork and clone** the repository
4. **Set up development environment** with `make install-deps`
5. **Test your changes** using `make test-all`
6. **Submit a Pull Request** following the guidelines

### Development Tools

The project includes comprehensive development tools:

```bash
make install-deps    # Install Arduino dependencies
make test-all       # Run full test suite
make compile-examples # Verify all examples compile
make format         # Format code consistently
```

### Code Quality Standards

- Follow Arduino library conventions and naming standards
- Include comprehensive documentation and comments
- Test on multiple Arduino platforms when possible
- Maintain backward compatibility for API changes
- Update documentation for any new features or changes

---

## License

This library is released under the MIT License. See the LICENSE file for details.

---

## Support

- **Documentation**: Comprehensive guides in the `docs/` folder
- **Examples**: 7 complete, tested examples with hardware specifications
- **Troubleshooting**: Detailed problem resolution guide
- **Community**: Arduino forums and GitHub issues for support

---

**AutoThrottleNG v1.2.0** - Professional PID control for Arduino applications.

---

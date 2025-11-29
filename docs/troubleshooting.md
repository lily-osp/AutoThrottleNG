# AutoThrottleNG Troubleshooting Guide

## Table of Contents

- [Common Issues and Solutions](#common-issues-and-solutions)
  - [Compilation Errors](#compilation-errors)
  - [Runtime Issues](#runtime-issues)
  - [Failsafe Issues](#failsafe-issues)
  - [Sensor Issues](#sensor-issues)
  - [PID Tuning Issues](#pid-tuning-issues)
  - [Memory Issues](#memory-issues)
  - [Hardware-Specific Issues](#hardware-specific-issues)
- [Debugging Tools](#debugging-tools)
- [Performance Optimization](#performance-optimization)
- [Getting Help](#getting-help)

## Common Issues and Solutions

### 1. Compilation Errors

#### "AutoThrottleNG.h: No such file or directory"
**Symptoms:**
```
fatal error: AutoThrottleNG.h: No such file or directory
```

**Solutions:**
1. **Library Installation**: Ensure AutoThrottleNG is properly installed in Arduino libraries folder
2. **Include Path**: Use `#include <AutoThrottleNG.h>` for installed libraries
3. **IDE Restart**: Restart Arduino IDE after library installation
4. **Library Location**: Check library is in `~/Arduino/libraries/` or `~/Documents/Arduino/libraries/`

#### "PID.h: No such file or directory"
**Symptoms:**
```
fatal error: PID.h: No such file or directory
```

**Solutions:**
1. Install PID library via Arduino IDE: `Sketch → Include Library → Manage Libraries → Search "PID"`
2. Or download from: https://github.com/br3ttb/Arduino-PID-Library
3. Ensure library is named "PID" (not "PID_v1")

### 2. Runtime Issues

#### Motor/Actuator Not Responding
**Symptoms:**
- Output remains at minimum value
- No movement despite setpoint changes

**Possible Causes:**
1. **PID Mode**: Check if controller is in MANUAL mode
   ```cpp
   if (controller.getMode() != AUTOMATIC) {
       controller.setMode(AUTOMATIC);
   }
   ```

2. **Output Limits**: Verify output limits are appropriate
   ```cpp
   controller.setOutputLimits(0, 255); // For PWM applications
   ```

3. **Failsafe Active**: Check for error states
   ```cpp
   if (controller.isInErrorState()) {
       Serial.println("Error: " + String((int)controller.getErrorState()));
       controller.clearErrorState();
   }
   ```

#### Unstable/Erratic Behavior
**Symptoms:**
- Output oscillates wildly
- System never reaches setpoint
- Derivative kick when setpoint changes

**Solutions:**
1. **PID Tuning**: Start with conservative values
   ```cpp
   // Conservative starting values
   AutoThrottleNG controller(0, 255, 1.0, 0.05, 0.1); // Much lower than default
   ```

2. **Input Filtering**: Enable input filtering to reduce noise
   ```cpp
   controller.setInputFilterAlpha(0.2); // Moderate filtering
   ```

3. **Output Smoothing**: Enable output smoothing
   ```cpp
   controller.enableSmoothing(true, 50.0); // 50 units/second max change
   ```

4. **Sample Time**: Ensure appropriate sample time
   ```cpp
   controller.setSampleTime(100); // 100ms sample time
   ```

#### System Too Slow to Respond
**Symptoms:**
- Slow response to setpoint changes
- Takes too long to reach target

**Solutions:**
1. **Increase PID Gains**: Gradually increase proportional gain
   ```cpp
   controller.setTunings(5.0, 0.1, 0.5); // Higher gains
   ```

2. **Reduce Filtering**: Lower input filter alpha
   ```cpp
   controller.setInputFilterAlpha(0.8); // Less filtering
   ```

3. **Increase Smoothing Rate**: Allow faster output changes
   ```cpp
   controller.enableSmoothing(true, 200.0); // Faster rate
   ```

4. **Shorter Sample Time**: More frequent updates
   ```cpp
   controller.setSampleTime(50); // 50ms sample time
   ```

### 3. Failsafe Issues

#### System Stuck in Failsafe
**Symptoms:**
- Output stuck at failsafe value
- `isInErrorState()` returns true
- Cannot resume normal operation

**Solutions:**
1. **Clear Error State**: Manually clear errors after fixing issues
   ```cpp
   if (controller.isInErrorState()) {
       // Fix the underlying problem first
       // Then clear the error
       controller.clearErrorState();
   }
   ```

2. **Check Error Type**: Identify specific error
   ```cpp
   AutoThrottleNG::Error err = controller.getErrorState();
   switch(err) {
       case AutoThrottleNG::Error::INPUT_INVALID:
           // Handle invalid sensor data
           break;
       case AutoThrottleNG::Error::INPUT_TIMEOUT:
           // Handle sensor communication issues
           break;
       case AutoThrottleNG::Error::STABILITY_TIMEOUT:
           // Handle unstable system
           break;
   }
   ```

#### INPUT_TIMEOUT Errors
**Symptoms:**
- `INPUT_TIMEOUT` error after some time
- System works initially but fails later

**Solutions:**
1. **Increase Timeout**: Set appropriate timeout for your sensor
   ```cpp
   controller.setInputTimeout(5000); // 5 second timeout
   ```

2. **Regular Updates**: Ensure `updateInput()` is called frequently
   ```cpp
   void loop() {
       // Read sensor
       double sensorValue = analogRead(sensorPin);

       // Update controller
       controller.updateInput(sensorValue);

       // Compute output
       double output = controller.compute();

       // Apply output
       delay(50); // Don't delay too long
   }
   ```

#### STABILITY_TIMEOUT Errors
**Symptoms:**
- System oscillates and triggers stability timeout
- Error occurs during normal operation

**Solutions:**
1. **Adjust Stability Parameters**: Set appropriate tolerance and timeout
   ```cpp
   controller.setStabilityParams(2.0, 10000); // 2 unit tolerance, 10 second timeout
   ```

2. **Better PID Tuning**: Improve system stability
3. **Increase Filtering**: Reduce noise sensitivity
4. **Check Setpoint**: Ensure setpoint is realistic

### 4. Sensor Issues

#### Invalid Sensor Readings
**Symptoms:**
- `INPUT_INVALID` errors
- System fails immediately

**Solutions:**
1. **Sensor Validation**: Check sensor output range
   ```cpp
   double sensorValue = analogRead(pin);
   if (isnan(sensorValue) || isinf(sensorValue)) {
       // Handle invalid reading
       return;
   }
   controller.updateInput(sensorValue);
   ```

2. **Sensor Calibration**: Ensure sensor outputs valid range
3. **Power Issues**: Check sensor power supply
4. **Connection Issues**: Verify wiring and pin assignments

#### Noisy Sensor Data
**Symptoms:**
- Erratic controller behavior
- Unstable output despite good PID tuning

**Solutions:**
1. **Enable Input Filtering**: Use EMA filtering
   ```cpp
   controller.setInputFilterAlpha(0.1); // Heavy filtering
   ```

2. **Hardware Filtering**: Add RC filter to sensor output
3. **Multiple Samples**: Average multiple readings
   ```cpp
   double readSensor() {
       double sum = 0;
       for(int i = 0; i < 10; i++) {
           sum += analogRead(pin);
           delay(1);
       }
       return sum / 10.0;
   }
   ```

### 5. Memory Issues

#### Out of Memory Errors
**Symptoms:**
- Compilation fails with memory errors
- Runtime crashes or unexpected behavior

**Solutions:**
1. **Check Memory Usage**: Monitor with `getFreeMemory()`
2. **Reduce Features**: Disable unused features
   ```cpp
   // Disable filtering if not needed
   controller.setInputFilterAlpha(0.0);

   // Disable smoothing if not needed
   controller.enableSmoothing(false);
   ```

3. **Optimize Code**: Remove unnecessary variables and functions
4. **Board Selection**: Use board with more memory if needed

### 6. PID Tuning Issues

#### Cannot Achieve Good Tuning
**Symptoms:**
- System either oscillates or responds too slowly
- Cannot find good PID values

**Solutions:**
1. **System Identification**: Understand system characteristics
   - **First Order Lag**: Use PI control
   - **Integrating Process**: Use high proportional gain
   - **Oscillatory System**: Add derivative gain

2. **Manual Tuning Method**:
   - Set Ki=0, Kd=0, increase Kp until oscillation
   - Add Ki for steady-state accuracy
   - Add Kd to reduce overshoot

3. ** Ziegler-Nichols Method**:
   - Find ultimate gain (Ku) and period (Tu)
   - Kp = 0.6 * Ku
   - Ki = 2 * Kp / Tu
   - Kd = Kp * Tu / 8

### 7. Hardware-Specific Issues

#### PWM Output Issues
**Symptoms:**
- PWM not working on certain pins
- Inconsistent output

**Solutions:**
1. **Pin Selection**: Use PWM-capable pins
   ```cpp
   // Arduino Uno PWM pins: 3, 5, 6, 9, 10, 11
   const int pwmPin = 9;
   ```

2. **Timer Conflicts**: Avoid pins used by other libraries
3. **analogWrite Resolution**: Ensure appropriate PWM range

#### Interrupt Conflicts
**Symptoms:**
- Encoder or sensor interrupts not working
- Timing issues

**Solutions:**
1. **Pin Selection**: Use interrupt-capable pins
   ```cpp
   // Arduino Uno interrupt pins: 2, 3
   const int encoderPinA = 2;
   const int encoderPinB = 3;
   ```

2. **ISR Optimization**: Keep interrupt service routines short
3. **Debouncing**: Add hardware/software debouncing for noisy signals

### 8. Debugging Tools

#### Serial Debugging
```cpp
void loop() {
    // Debug output
    Serial.print("Setpoint: "); Serial.print(controller.getSetpoint());
    Serial.print(" Input: "); Serial.print(controller.getFilteredInput());
    Serial.print(" Output: "); Serial.print(controller.getThrottle());
    Serial.print(" Mode: "); Serial.print(controller.getMode() == AUTOMATIC ? "AUTO" : "MANUAL");
    Serial.print(" Error: "); Serial.println((int)controller.getErrorState());

    // Your control code here
}
```

#### Error State Monitoring
```cpp
void checkErrors() {
    if (controller.isInErrorState()) {
        AutoThrottleNG::Error err = controller.getErrorState();
        Serial.print("Error detected: ");

        switch(err) {
            case AutoThrottleNG::Error::INPUT_INVALID:
                Serial.println("Invalid input");
                break;
            case AutoThrottleNG::Error::INPUT_TIMEOUT:
                Serial.println("Input timeout");
                break;
            case AutoThrottleNG::Error::STABILITY_TIMEOUT:
                Serial.println("Stability timeout");
                break;
            default:
                Serial.println("Unknown error");
        }

        // Log additional debug info
        Serial.print("Last update: "); Serial.println(controller.getLastUpdateTime());
        Serial.print("Current time: "); Serial.println(millis());
    }
}
```

### 9. Performance Optimization

#### Reduce CPU Usage
1. **Increase Sample Time**: Longer intervals reduce CPU load
2. **Disable Filtering**: If sensor is clean
3. **Optimize ISR**: Keep interrupt routines minimal

#### Improve Response Time
1. **Reduce Sample Time**: More frequent updates
2. **Disable Smoothing**: For applications needing fast response
3. **Optimize PID Gains**: Higher gains for faster response

## Advanced Diagnostics

### Platform-Specific Issues

#### ESP32-Specific Issues
**Symptoms:**
- Timing inconsistencies, watchdog resets

**Solutions:**
1. **FreeRTOS Compatibility**: Ensure proper task scheduling
   ```cpp
   // Use Arduino loop() instead of custom tasks for control
   void loop() {
       // Control code here
       delay(10); // Yield to RTOS
   }
   ```

2. **Watchdog Management**: Avoid blocking operations
   ```cpp
   // Feed watchdog regularly
   void loop() {
       yield(); // Allow RTOS scheduling

       // Control operations
       controller.updateInput(sensorValue);
       double output = controller.compute();

       delay(10);
   }
   ```

#### ARM Cortex-M (Due, Zero) Issues
**Symptoms:**
- Higher precision requirements, floating-point issues

**Solutions:**
1. **Floating Point**: Use double precision consistently
2. **Memory Alignment**: Ensure proper data alignment
3. **Interrupt Priorities**: Configure interrupt priorities appropriately

### Library Version Conflicts
**Symptoms:**
- Unexpected behavior with different PID library versions

**Solutions:**
1. **Version Checking**: Verify PID library version
   ```cpp
   // Check library version compatibility
   #if not defined(PID_H)
   #error "PID library not found"
   #endif
   ```

2. **Alternative Libraries**: Consider using PID_v2 if compatible
3. **Manual Installation**: Ensure clean library installation

### Integration Issues
**Symptoms:**
- Conflicts with other libraries, timer conflicts

**Solutions:**
1. **Timer Conflicts**: Use different PWM pins
   ```cpp
   // Avoid Timer 0 pins on AVR (pins 5, 6)
   const int pwmPin = 9; // Timer 1
   // const int pwmPin = 3; // Timer 2
   ```

2. **Library Conflicts**: Isolate conflicting libraries
3. **Resource Management**: Check for shared resource conflicts

## Advanced Diagnostics

### Oscilloscope Debugging
1. **Output Signal**: Monitor PWM output for expected behavior
2. **Sensor Signal**: Verify sensor signal stability
3. **Timing Analysis**: Check loop timing consistency

### Serial Plotter Analysis
```cpp
void loop() {
    // CSV format for Serial Plotter
    Serial.print(millis());
    Serial.print(",");
    Serial.print(controller.getSetpoint());
    Serial.print(",");
    Serial.print(controller.getFilteredInput());
    Serial.print(",");
    Serial.print(controller.getThrottle());
    Serial.print(",");
    Serial.println((int)controller.getErrorState());
}
```

### Memory Debugging
```cpp
// Check available memory
extern int __heap_start, *__brkval;
int freeMemory() {
    int v;
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

// Monitor memory usage
Serial.print("Free RAM: ");
Serial.println(freeMemory());
```

## Performance Profiling

### Execution Time Measurement
```cpp
unsigned long startTime = micros();

// Your control code
controller.updateInput(sensorValue);
double output = controller.compute();

unsigned long endTime = micros();
Serial.print("Execution time: ");
Serial.print(endTime - startTime);
Serial.println(" microseconds");
```

### Sample Rate Verification
```cpp
static unsigned long lastSample = 0;
static int sampleCount = 0;

if (millis() - lastSample >= 1000) { // Every second
    Serial.print("Samples per second: ");
    Serial.println(sampleCount);
    sampleCount = 0;
    lastSample = millis();
}
sampleCount++;
```

## Systematic Debugging Approach

### Step-by-Step Diagnosis
1. **Isolate Components**: Test each component individually
2. **Known Good State**: Start with working example, modify incrementally
3. **Single Variable**: Change one parameter at a time
4. **Document Changes**: Keep log of modifications and results

### Problem Classification
- **Immediate**: Compilation errors, crashes
- **Intermittent**: Occasional failures, timing issues
- **Performance**: Slow response, oscillations
- **Accuracy**: Steady-state error, offset

### Root Cause Analysis
1. **Hardware**: Sensor wiring, power supply, pin assignments
2. **Software**: Library versions, configuration, timing
3. **Environment**: Temperature, electrical noise, mechanical issues
4. **Interactions**: Library conflicts, resource sharing

## Getting Help

### Documentation Resources
1. **Examples**: Start with provided examples and modify gradually
2. **Usage Guide**: Review configuration options and best practices
3. **Detailed Explanation**: Understand algorithm behavior
4. **System Architecture**: Review design constraints

### Community Support
1. **Issue Reports**: Use GitHub issues for bugs and feature requests
2. **Forum Posts**: Arduino forums, Stack Overflow, Reddit r/arduino
3. **Code Reviews**: Share your code for peer review

### Professional Support
1. **Control Systems**: Consult control engineering resources
2. **Arduino Experts**: Local maker spaces, universities
3. **Commercial Support**: Professional embedded systems consultants

### Preventive Measures
1. **Version Control**: Track all changes and configurations
2. **Documentation**: Document your setup and modifications
3. **Testing**: Thoroughly test before deployment
4. **Monitoring**: Implement continuous system monitoring

Remember: PID control combines art, science, and experience. Start simple, understand the fundamentals, test thoroughly, and tune methodically. Most issues stem from incorrect tuning, noisy sensors, or improper system understanding rather than library bugs.

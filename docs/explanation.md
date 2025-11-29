# AutoThrottleNG Detailed Explanation

## Table of Contents

- [How AutoThrottleNG Works](#how-autothrottleng-works)
  - [PID Control Fundamentals](#pid-control-fundamentals)
  - [Signal Processing Pipeline](#signal-processing-pipeline)
  - [Failsafe System Architecture](#failsafe-system-architecture)
  - [Timing and Synchronization](#timing-and-synchronization)
  - [Controller Direction](#controller-direction)
  - [Proportional on Error vs Measurement](#proportional-on-error-vs-measurement)
  - [Saturation and Windup Protection](#saturation-and-windup-protection)
  - [Reset and Initialization](#reset-and-initialization)
  - [Thread Safety and Concurrency](#thread-safety-and-concurrency)

## How AutoThrottleNG Works

AutoThrottleNG is a control library that implements PID (Proportional-Integral-Derivative) control with safety and signal processing features. This document explains the internal workings and design philosophy.

## 1. PID Control Fundamentals

### What is PID Control?

PID control is a feedback mechanism that calculates an output value to minimize the error between a desired setpoint and a measured process variable.

**Mathematical Formula:**
```
Error(t) = Setpoint - Input(t)

P(t) = Kp × Error(t)
I(t) = I(t-1) + Ki × Error(t) × Δt
D(t) = Kd × (Error(t) - Error(t-1)) / Δt

Output(t) = P(t) + I(t) + D(t)
```

Where:
- **Kp** (Proportional Gain): Reacts to current error
- **Ki** (Integral Gain): Eliminates steady-state error by accumulating past errors
- **Kd** (Derivative Gain): Predicts future error by measuring error rate of change

### AutoThrottleNG PID Implementation

AutoThrottleNG uses the standard Arduino PID library but adds:
- **Input filtering** before PID computation
- **Output smoothing** after PID computation
- **Comprehensive failsafes**
- **Easy configuration and monitoring**

## 2. Signal Processing Pipeline

### Stage 1: Input Acquisition
```cpp
void updateInput(double rawInputValue) {
    // 1. Validate input
    if (isnan(rawInputValue) || isinf(rawInputValue)) {
        updateFailsafeState(Error::INPUT_INVALID);
        return;
    }

    // 2. Store raw value
    _lastRawInput = rawInputValue;

    // 3. Apply input filtering
    _pidInput = applyInputFilter(rawInputValue);

    // 4. Clear relevant errors
    if (_errorState == Error::INPUT_INVALID ||
        _errorState == Error::INPUT_TIMEOUT) {
        clearErrorState();
    }
}
```

### Stage 2: PID Computation
```cpp
double compute() {
    // 1. Check failsafes first
    checkTimeouts(currentMillis);

    if (isInErrorState()) {
        return _failsafeOutputValue;
    }

    // 2. Ensure PID is in automatic mode
    if (_pid.GetMode() == MANUAL && !isInErrorState()) {
        _pid.SetMode(AUTOMATIC);
    }

    // 3. Compute PID output
    bool computed = _pid.Compute();

    if (computed) {
        // 4. Check stability and timeouts
        checkTimeouts(currentMillis);
    }

    // 5. Apply output smoothing if enabled
    if (_smoothingEnabled) {
        _currentSmoothedOutput = applyOutputSmoothing(_pidOutput);
    } else {
        _currentSmoothedOutput = _pidOutput;
    }

    // 6. Apply output limits
    _currentSmoothedOutput = constrain(_currentSmoothedOutput,
                                     _minOutputLimit, _maxOutputLimit);

    return _currentSmoothedOutput;
}
```

## 3. Input Filtering Explained

### Exponential Moving Average (EMA)

AutoThrottleNG implements EMA filtering to reduce sensor noise:

**Formula:**
```
FilteredValue[n] = α × RawValue[n] + (1-α) × FilteredValue[n-1]
```

**Alpha (α) Values:**
- **α = 0.0**: No filtering (raw values pass through)
- **α = 0.1**: Heavy filtering (responds slowly to changes)
- **α = 0.5**: Moderate filtering (good balance)
- **α = 0.9**: Light filtering (responds quickly)

### When to Use Filtering

**Use filtering when:**
- Sensor readings are noisy
- PID derivative term causes oscillation
- System has high-frequency disturbances

**Don't use filtering when:**
- Sensor is already clean
- Fast response is critical
- System dynamics are slow

## 4. Output Smoothing Explained

### Ramp Rate Limiting

Output smoothing prevents abrupt changes that could:
- Damage mechanical components
- Cause current spikes
- Create jerky motion

**Algorithm:**
```cpp
double applyOutputSmoothing(double rawOutput) {
    unsigned long timeDelta = currentMillis - _lastComputeMillis;
    double maxChange = _smoothingRate * (double)timeDelta / 1000.0;
    double targetValue = constrain(rawOutput, _minOutputLimit, _maxOutputLimit);

    if (targetValue > _currentSmoothedOutput) {
        return min(targetValue, _currentSmoothedOutput + maxChange);
    } else if (targetValue < _currentSmoothedOutput) {
        return max(targetValue, _currentSmoothedOutput - maxChange);
    } else {
        return _currentSmoothedOutput;
    }
}
```

### Smoothing Rate Configuration

**MaxRatePerSecond** determines how fast the output can change:
- **Low values (10-50)**: Smooth, gradual changes (good for motors, servos)
- **High values (100-500)**: Faster response (good for LEDs, valves)
- **Very high values**: Minimal smoothing effect

## 5. Failsafe System Architecture

### Error State Machine

```
NORMAL_OPERATION
       ↓
   Error Detected
       ↓
  FAILSAFE_ACTIVE
       ↓
   Manual Recovery
       ↓
NORMAL_OPERATION
```

### Error Types and Triggers

1. **INPUT_INVALID**
   - **Trigger**: NaN or Infinity sensor reading
   - **Action**: Immediately switch to failsafe output
   - **Recovery**: Provide valid sensor reading + `clearErrorState()`

2. **INPUT_TIMEOUT**
   - **Trigger**: No `updateInput()` calls within timeout period
   - **Action**: Switch to failsafe output after timeout
   - **Recovery**: Call `updateInput()` + `clearErrorState()`

3. **STABILITY_TIMEOUT**
   - **Trigger**: Error exceeds tolerance for specified duration
   - **Action**: Switch to failsafe output
   - **Recovery**: System becomes stable + `clearErrorState()`

### Failsafe Output Logic

```cpp
if (isInErrorState()) {
    // Force PID to manual mode
    _pid.SetMode(MANUAL);

    // Apply failsafe value with smoothing if enabled
    if (_smoothingEnabled) {
        _currentSmoothedOutput = applyOutputSmoothing(_failsafeOutputValue);
    } else {
        _currentSmoothedOutput = _failsafeOutputValue;
    }

    return _currentSmoothedOutput;
}
```

## 6. Timing and Synchronization

### Sample Time Management

PID controllers need consistent timing. AutoThrottleNG handles this through:

1. **PID Library Timing**: Internal sample time management
2. **Compute Loop Timing**: `millis()` based execution
3. **Timeout Monitoring**: Millisecond precision

### Execution Timing Considerations

**Fast Loop (10-50ms):**
- Good for: Motor control, servo positioning
- Trade-off: Higher CPU usage

**Medium Loop (50-200ms):**
- Good for: Temperature control, process control
- Trade-off: Balanced performance

**Slow Loop (200-1000ms):**
- Good for: Slow thermal systems, level control
- Trade-off: Slower response

## 7. Memory Management

### Static Memory Layout

```cpp
// Core PID variables (required)
double _pidInput, _pidOutput, _pidSetpoint;

// Configuration (set at initialization)
double _minOutputLimit, _maxOutputLimit;
double _inputFilterAlpha, _smoothingRate;

// State tracking (changes during operation)
double _lastRawInput, _currentSmoothedOutput;
Error _errorState;
unsigned long _lastValidUpdateMillis;
```

### Memory Optimization Features

1. **No Dynamic Allocation**: All memory allocated at compile time
2. **Minimal RAM Usage**: ~450-1000 bytes depending on features
3. **Efficient Data Types**: Uses appropriate precision for calculations
4. **Shared Variables**: PID library shares variables with AutoThrottleNG

## 8. Controller Direction

### Direct vs Reverse Acting

**DIRECT Mode (default):**
- Increase in output → Increase in input
- Example: Heater control (more power = higher temperature)

**REVERSE Mode:**
- Increase in output → Decrease in input
- Example: Cooler control (more power = lower temperature)

```cpp
// Direct acting (heater)
AutoThrottleNG heater(0, 255, 2.0, 0.5, 0.1, P_ON_E, DIRECT);

// Reverse acting (cooler)
AutoThrottleNG cooler(0, 255, 2.0, 0.5, 0.1, P_ON_E, REVERSE);
```

## 9. Proportional on Error vs Measurement

### P_ON_E (Proportional on Error)
- Standard PID implementation
- Proportional term based on current error
- Can cause "derivative kick" on setpoint changes

### P_ON_M (Proportional on Measurement)
- Proportional term based on process variable
- Reduces derivative kick
- Better for systems with frequent setpoint changes

```cpp
// Standard PID
AutoThrottleNG controller(min, max, kp, ki, kd, P_ON_E, direction);

// Derivative kick reduction
AutoThrottleNG controller(min, max, kp, ki, kd, P_ON_M, direction);
```

## 10. Saturation and Windup Protection

### Output Saturation Detection

AutoThrottleNG detects when PID output hits limits:

```cpp
bool isSaturated = (_pidOutput <= _minOutputLimit || _pidOutput >= _maxOutputLimit);
```

### Integral Windup Protection

The underlying PID library includes:
- **Output limiting**: Prevents integral accumulation beyond limits
- **Reset on mode change**: Clears integral when switching modes
- **Manual mode handling**: Maintains output when in manual mode

## 11. Reset and Initialization

### Controller Reset Process

```cpp
void reset() {
    // Reset PID internal state
    _pid.SetMode(MANUAL);
    _pid.SetMode(AUTOMATIC);

    // Reset output values
    _pidOutput = _minOutputLimit;
    _currentSmoothedOutput = _minOutputLimit;

    // Reset state variables
    _pidInput = 0;
    _lastRawInput = 0;

    // Clear errors and reset timers
    clearErrorState();
}
```

### State Preservation

Reset maintains:
- PID tuning parameters (Kp, Ki, Kd)
- Output limits
- Filter and smoothing settings
- Failsafe configuration

Reset clears:
- Integral accumulation
- Previous error values
- Error states
- Timing variables

## 12. Thread Safety and Concurrency

### Arduino Environment Considerations

1. **Single-threaded**: Arduino is single-threaded, no concurrency issues
2. **Interrupt Safety**: Variables accessed in ISRs need atomic operations
3. **Timing Consistency**: All timing uses `millis()` for consistency

### Best Practices

1. **ISR Usage**: Keep ISRs short, avoid calling library methods
2. **Variable Access**: Use volatile for ISR-shared variables
3. **Timing**: Ensure consistent timing between `updateInput()` and `compute()`

This explanation covers the core algorithms, data flow, and design decisions that make AutoThrottleNG a reliable control library for Arduino projects.

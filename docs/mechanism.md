# AutoThrottleNG Internal Mechanisms

## Table of Contents

- [PID Control Algorithm](#pid-control-algorithm)
  - [Discrete PID Implementation](#discrete-pid-implementation)
  - [PID Library Integration](#pid-library-integration)
- [Signal Processing Mechanisms](#signal-processing-mechanisms)
  - [Input Filtering: Exponential Moving Average](#input-filtering-exponential-moving-average)
  - [Output Smoothing: Ramp Rate Limiting](#output-smoothing-ramp-rate-limiting)
- [Failsafe Mechanisms](#failsafe-mechanisms)
  - [Error State Machine](#error-state-machine)
  - [Failsafe Output Logic](#failsafe-output-logic)
- [Timing and Synchronization](#timing-and-synchronization)
  - [Sample Time Management](#sample-time-management)
  - [Timing Considerations](#timing-considerations)
- [Memory Management](#memory-management)
  - [Static Memory Layout](#static-memory-layout)
  - [Dynamic Memory Usage](#dynamic-memory-usage)
- [Controller Modes](#controller-modes)
  - [Automatic Mode](#automatic-mode-pid-control)
  - [Manual Mode](#manual-mode-direct-control)
  - [Mode Transitions](#mode-transitions)
- [Proportional Modes](#proportional-modes)
  - [Proportional on Error](#proportional-on-error)
  - [Proportional on Measurement](#proportional-on-measurement)
- [Saturation and Anti-Windup](#saturation-and-anti-windup)
  - [Output Saturation Detection](#output-saturation-detection)
  - [Integral Windup Protection](#integral-windup-protection)
- [Reset and Initialization](#reset-and-initialization)
  - [Controller Reset Process](#controller-reset-process)
  - [State Preservation vs Reset](#state-preservation-vs-reset)
- [Thread Safety and Concurrency](#thread-safety-and-concurrency)
  - [Arduino Execution Model](#arduino-execution-model)
  - [Interrupt Service Routine Considerations](#interrupt-service-routine-considerations)
- [Performance Characteristics](#performance-characteristics)
  - [Computational Complexity](#computational-complexity)
  - [Execution Time](#execution-time)
  - [Memory Efficiency](#memory-efficiency)

## PID Control Algorithm

### Discrete PID Implementation

AutoThrottleNG uses the standard Arduino PID library which implements the velocity form of the PID algorithm:

**Position Form:**
```
u(t) = Kp × e(t) + Ki × ∫e(τ)dτ + Kd × de(t)/dt
```

**Velocity Form (used by library):**
```
Δu(t) = Kp × Δe(t) + Ki × e(t) × Δt + Kd × (Δe(t) - Δe(t-1))/Δt
```

Where:
- **u(t)**: Control output at time t
- **e(t)**: Error at time t (Setpoint - Input)
- **Δt**: Sample time interval
- **Kp, Ki, Kd**: PID gains

### PID Library Integration

AutoThrottleNG maintains three key variables shared with the PID library:

```cpp
double _pidInput;     // Current filtered input (process variable)
double _pidOutput;    // Current PID output (control variable)
double _pidSetpoint;  // Desired target value
```

The PID library operates on these variables through pointers, allowing AutoThrottleNG to preprocess inputs and postprocess outputs.

## Signal Processing Mechanisms

### Input Filtering: Exponential Moving Average

#### Algorithm Implementation

```cpp
double applyInputFilter(double rawValue) {
    if (_inputFilterAlpha <= 0.0) {
        return rawValue;  // No filtering
    } else {
        // EMA: filtered = α × current + (1-α) × previous
        return (_inputFilterAlpha * rawValue) +
               (1.0 - _inputFilterAlpha) * _pidInput;
    }
}
```

#### Filter Characteristics

**Time Constant Analysis:**
- **α = 0.1**: Time constant ≈ 9 × sample_time
- **α = 0.5**: Time constant ≈ 1 × sample_time
- **α close to 1**: Minimal filtering, fast response

**Frequency Response:**
- **Cutoff frequency**: fc = (α × fs) / (2π × (1-α))
- **Phase lag**: θ = arctan(fc / f) where f is signal frequency

### Output Smoothing: Ramp Rate Limiting

#### Rate Limiting Algorithm

```cpp
double applyOutputSmoothing(double rawOutput) {
    unsigned long timeDelta = currentMillis - _lastComputeMillis;
    double maxChange = _smoothingRate * (double)timeDelta / 1000.0;

    double constrainedOutput = constrain(rawOutput, _minOutputLimit, _maxOutputLimit);

    if (constrainedOutput > _currentSmoothedOutput) {
        return min(constrainedOutput, _currentSmoothedOutput + maxChange);
    } else if (constrainedOutput < _currentSmoothedOutput) {
        return max(constrainedOutput, _currentSmoothedOutput - maxChange);
    } else {
        return _currentSmoothedOutput;
    }
}
```

#### Smoothing Dynamics

**Maximum Rate Calculation:**
```
max_change = rate_limit × (time_delta / 1000)
```

**Effective Smoothing:**
- **High rate_limit**: Minimal smoothing, fast response
- **Low rate_limit**: Heavy smoothing, slow response
- **Zero rate_limit**: Infinite smoothing (output frozen)

## Failsafe Mechanisms

### Error State Machine

#### State Transitions

```
NORMAL_OPERATION
       ↓ (Error Condition Met)
   ERROR_STATE_ACTIVE
   (PID → MANUAL, Failsafe Output)
       ↓ (clearErrorState() Called)
NORMAL_OPERATION_RESUMED
   (PID → AUTOMATIC)
```

#### Error Detection Logic

```cpp
void checkTimeouts(unsigned long currentMillis) {
    // Input Timeout Check
    if (_inputTimeoutMillis > 0) {
        if ((currentMillis - _lastValidUpdateMillis) > _inputTimeoutMillis) {
            updateFailsafeState(Error::INPUT_TIMEOUT);
        }
    }

    // Stability Timeout Check
    if (_stabilityTimeoutMillis > 0 && !isInErrorState()) {
        bool currentlyStable = isStable();
        if (!currentlyStable) {
            if (_currentlyStable) {
                // Just became unstable
                _currentlyStable = false;
                _unstableStartMillis = currentMillis;
            } else {
                // Check duration
                if ((currentMillis - _unstableStartMillis) > _stabilityTimeoutMillis) {
                    updateFailsafeState(Error::STABILITY_TIMEOUT);
                }
            }
        } else {
            _currentlyStable = true;
            _unstableStartMillis = 0;
        }
    }
}
```

### Failsafe Output Logic

```cpp
if (isInErrorState()) {
    _pid.SetMode(MANUAL);  // Disable PID computation

    // Apply failsafe with smoothing if enabled
    if (_smoothingEnabled) {
        _currentSmoothedOutput = applyOutputSmoothing(_failsafeOutputValue);
    } else {
        _currentSmoothedOutput = _failsafeOutputValue;
    }

    return _currentSmoothedOutput;
}
```

## Timing and Synchronization

### Sample Time Management

#### PID Library Timing

The underlying PID library uses its own timing mechanism:

```cpp
// In PID.Compute()
unsigned long now = millis();
unsigned long timeChange = now - _lastTime;

if (timeChange >= _sampleTime) {
    // Perform PID calculation
    _lastTime = now;
    return true;  // Output updated
} else {
    return false; // No update
}
```

#### AutoThrottleNG Timing Integration

AutoThrottleNG respects the PID library's timing while adding its own features:

```cpp
double compute() {
    // Always call PID.Compute() - it handles internal timing
    bool pidUpdated = _pid.Compute();

    if (pidUpdated) {
        // PID output was updated, apply post-processing
        applyFailsafeChecks();
        applyOutputSmoothing();
        applyOutputLimits();
    }

    // Return smoothed output regardless of PID update
    return _currentSmoothedOutput;
}
```

### Timing Considerations

**Sample Time Guidelines:**
- **Fast systems (motors, servos)**: 20-50ms
- **Medium systems (position, pressure)**: 50-200ms
- **Slow systems (temperature)**: 500-2000ms

**Timing Accuracy:**
- Uses Arduino `millis()` (accurate to ~1ms)
- Accounts for timer overflow (rolls over every 49.7 days)
- Consistent timing across all features

## Memory Management

### Static Memory Layout

#### Core PID Variables (16 bytes)
```cpp
double _pidInput;      // 8 bytes - filtered sensor input
double _pidOutput;     // 8 bytes - raw PID output
double _pidSetpoint;   // 8 bytes - target value
```

#### Configuration Variables (32 bytes)
```cpp
double _minOutputLimit;        // 8 bytes
double _maxOutputLimit;        // 8 bytes
double _inputFilterAlpha;      // 8 bytes
double _smoothingRate;         // 8 bytes
```

#### State Variables (28 bytes)
```cpp
double _lastRawInput;          // 8 bytes
double _currentSmoothedOutput; // 8 bytes
Error _errorState;             // 1 byte (enum)
bool _smoothingEnabled;        // 1 byte
unsigned long _lastComputeMillis;      // 4 bytes
unsigned long _lastValidUpdateMillis;  // 4 bytes
```

#### Failsafe Variables (20 bytes)
```cpp
double _failsafeOutputValue;           // 8 bytes
unsigned long _inputTimeoutMillis;     // 4 bytes
unsigned long _stabilityTimeoutMillis; // 4 bytes
unsigned long _unstableStartMillis;    // 4 bytes
```

**Total Static Memory:** ~96 bytes (varies by configuration)

### Dynamic Memory Usage

**Per Example Memory Usage (Arduino Uno):**
- **Basic**: 450 bytes RAM
- **Intermediate**: 591 bytes RAM
- **Advanced**: 780 bytes RAM
- **Motor Control**: 747 bytes RAM
- **Temperature**: 984 bytes RAM
- **Servo**: 923 bytes RAM
- **LED**: 906 bytes RAM

### Memory Optimization Features

1. **No Dynamic Allocation**: All memory allocated at compile time
2. **Shared Variables**: PID library shares variables with AutoThrottleNG
3. **Efficient Data Types**: Appropriate precision for embedded systems
4. **Minimal Overhead**: Small memory footprint compared to features

## Controller Modes

### Automatic Mode (PID Control)

```cpp
_pid.SetMode(AUTOMATIC);
// PID computations active
// Output follows PID algorithm
// Setpoint tracking enabled
```

### Manual Mode (Direct Control)

```cpp
_pid.SetMode(MANUAL);
// PID computations disabled
// Output holds current value
// Used during failsafe conditions
```

### Mode Transitions

**Automatic → Manual:**
- Triggered by failsafe activation
- PID output frozen at current value
- Controller waits for error clearance

**Manual → Automatic:**
- Triggered by `clearErrorState()` or `setMode(AUTOMATIC)`
- PID internal state reset
- Normal operation resumed

## Proportional Modes

### Proportional on Error (P_ON_E)

**Standard PID Implementation:**
```
P_term = Kp × (Setpoint - Input)
```

**Characteristics:**
- Proportional term based on current error
- Derivative kick when setpoint changes
- Traditional PID behavior

### Proportional on Measurement (P_ON_M)

**Modified Implementation:**
```
P_term = Kp × (Setpoint - Input) × derivative_gain_adjustment
```

**Characteristics:**
- Proportional term based on measurement changes
- Reduced derivative kick
- Better setpoint response
- Slight integral windup reduction

## Saturation and Anti-Windup

### Output Saturation Detection

```cpp
bool isSaturated() const {
    return (_pidOutput <= _minOutputLimit || _pidOutput >= _maxOutputLimit);
}
```

### Integral Windup Protection

The PID library implements automatic anti-windup:

```cpp
// In PID library
if (_outputSum > _outMax) _outputSum = _outMax;
if (_outputSum < _outMin) _outputSum = _outMin;
```

This prevents integral accumulation when output is saturated, avoiding overshoot when saturation clears.

## Reset and Initialization

### Controller Reset Process

```cpp
void reset() {
    // Reset PID internal state
    _pid.SetMode(MANUAL);
    _pid.SetMode(AUTOMATIC);  // This clears integral and derivative terms

    // Reset output values
    _pidOutput = _minOutputLimit;
    _currentSmoothedOutput = _minOutputLimit;

    // Reset input tracking
    _pidInput = 0;
    _lastRawInput = 0;

    // Reset timing
    _lastComputeMillis = millis();
    _lastValidUpdateMillis = millis();

    // Clear error state
    _errorState = Error::NONE;
    _unstableStartMillis = 0;
    _currentlyStable = true;
}
```

### State Preservation vs Reset

**Preserved During Reset:**
- PID tuning parameters (Kp, Ki, Kd)
- Output limits (_minOutputLimit, _maxOutputLimit)
- Filter settings (_inputFilterAlpha)
- Smoothing settings (_smoothingEnabled, _smoothingRate)
- Failsafe configuration (_failsafeOutputValue, timeouts)

**Reset During Reset:**
- Integral accumulation
- Previous error values
- Derivative state
- Output values
- Error states
- Timing variables

## Thread Safety and Concurrency

### Arduino Execution Model

AutoThrottleNG is designed for single-threaded Arduino execution:

1. **No Concurrency**: Arduino runs single-threaded
2. **ISR Compatibility**: Variables shared with ISRs use appropriate protection
3. **Timing Consistency**: All timing uses `millis()` for consistency

### Interrupt Service Routine Considerations

```cpp
// ISR-safe variable updates
volatile long encoderCount = 0;

void encoderISR() {
    // Atomic operations for shared variables
    encoderCount++;
}

// Main loop reads volatile variable
long safeCount;
noInterrupts();
safeCount = encoderCount;
interrupts();
```

## Performance Characteristics

### Computational Complexity

**Per compute() Call:**
- **PID Calculation**: O(1) - constant time
- **Input Filtering**: O(1) - single EMA calculation
- **Output Smoothing**: O(1) - min/max operations
- **Failsafe Checks**: O(1) - timeout comparisons

**Total Complexity:** O(1) - suitable for real-time control

### Execution Time

**Typical compute() Execution Times:**
- **Basic PID**: ~50-100μs
- **With Filtering**: ~60-110μs
- **With Smoothing**: ~70-120μs
- **With Failsafe Checks**: ~80-130μs

### Memory Efficiency

**Memory Usage Breakdown:**
- **Static Variables**: 96 bytes (constant)
- **Stack Usage**: < 32 bytes (local variables)
- **Dynamic Allocation**: 0 bytes (no malloc/new)

These mechanisms work together to provide a robust, efficient, and reliable PID control library suitable for resource-constrained embedded systems.

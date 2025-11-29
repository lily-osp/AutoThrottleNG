#include "AutoThrottleNG.h"
#include <Arduino.h>
#include <math.h> // For isnan, isinf

AutoThrottleNG::AutoThrottleNG(double minOutput, double maxOutput,
    double kp, double ki, double kd,
    int POn, int direction)
    // Initialize members before PID constructor
    : _minOutputLimit(minOutput)
    , _maxOutputLimit(maxOutput)
    , _pidInput(0.0)
    , // Initialize PID variables
    _pidOutput(minOutput)
    , _pidSetpoint(0.0)
    ,
    // Initialize PID instance, passing pointers to our variables
    _pid(&_pidInput, &_pidOutput, &_pidSetpoint, kp, ki, kd, POn, direction)
    ,
    // Initialize remaining AutoThrottle state
    _lastRawInput(0.0)
    , _currentSmoothedOutput(minOutput)
    , _smoothingEnabled(false)
    , _smoothingRate(10.0)
    , _inputFilterAlpha(0.0)
    , // Default: No input filter
    _lastComputeMillis(millis())
    , _errorState(Error::NONE)
    , _failsafeOutputValue(minOutput)
    , _inputTimeoutMillis(0)
    , _stabilityTimeoutMillis(0)
    , _stabilityTolerance(0.1)
    , _lastValidUpdateMillis(millis())
    , _unstableStartMillis(0)
    , _currentlyStable(true)
    , _outputSaturated(false)
{
    // Configure PID object after construction
    _pid.SetOutputLimits(_minOutputLimit, _maxOutputLimit);
    // Sample time is set automatically by PID library based on loop speed initially
    // Can be overridden using setSampleTime()
    _pid.SetMode(AUTOMATIC);
}

// --- Core Methods ---

void AutoThrottleNG::setTarget(double target)
{
    if (isnan(target) || isinf(target))
        return;
    // Update the variable PID uses via its pointer
    _pidSetpoint = target;
}

void AutoThrottleNG::updateInput(double rawInputValue)
{
    unsigned long currentMillis = millis();
    if (isnan(rawInputValue) || isinf(rawInputValue)) {
        updateFailsafeState(Error::INPUT_INVALID);
        return; // Don't process invalid input
    }

    _lastRawInput = rawInputValue; // Store the raw value
    _lastValidUpdateMillis = currentMillis; // Record time

    // Apply input filter before updating PID's input variable
    _pidInput = applyInputFilter(rawInputValue);

    // Clear relevant errors now that we have valid input
    if (_errorState == Error::INPUT_INVALID || _errorState == Error::INPUT_TIMEOUT) {
        clearErrorState();
    }
}

double AutoThrottleNG::compute()
{
    unsigned long currentMillis = millis();

    // 1. Check Failsafe Conditions
    checkTimeouts(currentMillis);

    // 2. Handle Active Failsafe
    if (isInErrorState()) {
        if (_pid.GetMode() == AUTOMATIC) {
            _pid.SetMode(MANUAL); // Stop PID calculations
        }
        _currentSmoothedOutput = _failsafeOutputValue; // Apply failsafe output
        // _pidOutput might hold last value, we ignore it
        _outputSaturated = (_currentSmoothedOutput <= _minOutputLimit || _currentSmoothedOutput >= _maxOutputLimit);
        _lastComputeMillis = currentMillis; // Update time for smoothing continuity
        return _currentSmoothedOutput;
    }

    // 3. Handle Normal Operation / PID Compute
    if (_pid.GetMode() == MANUAL && !isInErrorState()) {
        // If manually set to MANUAL (not by failsafe) and error cleared, resume AUTO
        _pid.SetMode(AUTOMATIC);
    }

    // Trigger PID computation (operates on _pidInput/_pidSetpoint, writes to _pidOutput)
    bool computed = _pid.Compute(); // Returns true if a new output was computed

    // _pidOutput variable now holds the latest raw PID output value

    if (computed) {
        // Update saturation status
        _outputSaturated = (_pidOutput <= _minOutputLimit || _pidOutput >= _maxOutputLimit);
        // Check stability timeout AFTER compute provides new error info
        checkTimeouts(currentMillis);
        if (isInErrorState()) { // Check if stability timeout just triggered
            _currentSmoothedOutput = _failsafeOutputValue;
            _outputSaturated = (_currentSmoothedOutput <= _minOutputLimit || _currentSmoothedOutput >= _maxOutputLimit);
            _lastComputeMillis = currentMillis;
            return _currentSmoothedOutput;
        }
    }

    // 4. Apply Smoothing (if enabled) to the raw PID output
    if (_smoothingEnabled) {
        _currentSmoothedOutput = applyOutputSmoothing(_pidOutput);
    } else {
        _currentSmoothedOutput = _pidOutput; // Use raw PID output
    }

    // 5. Final Output Clamping
    _currentSmoothedOutput = constrain(_currentSmoothedOutput, _minOutputLimit, _maxOutputLimit);

    // 6. Update Timestamp
    _lastComputeMillis = currentMillis;

    return _currentSmoothedOutput;
}

void AutoThrottleNG::reset()
{
    // Reset PID internal state by cycling mode
    _pid.SetMode(MANUAL);
    _pid.SetMode(AUTOMATIC);

    // Reset our state variables
    _pidOutput = _minOutputLimit; // Reset PID's output variable
    _currentSmoothedOutput = _minOutputLimit;
    _pidInput = 0; // Reset filtered input (optional, could keep last value)
    _lastRawInput = 0;
    // _pidSetpoint remains unchanged

    clearErrorState(); // Clear any error flags
    _lastComputeMillis = millis();
    _lastValidUpdateMillis = millis();
    _unstableStartMillis = 0;
    _currentlyStable = true;
    _outputSaturated = false;
}

// --- PID Configuration ---

void AutoThrottleNG::setTunings(double kp, double ki, double kd, int POn)
{
    if (POn == P_ON_E || POn == P_ON_M) {
        _pid.SetTunings(kp, ki, kd, POn);
    } else { // POn = -1 or invalid, only set Kp, Ki, Kd
        _pid.SetTunings(kp, ki, kd);
    }
}

void AutoThrottleNG::setOutputLimits(double minOutput, double maxOutput)
{
    if (minOutput >= maxOutput)
        return;
    _minOutputLimit = minOutput;
    _maxOutputLimit = maxOutput;
    _pid.SetOutputLimits(_minOutputLimit, _maxOutputLimit);

    // Adjust failsafe and current values
    _failsafeOutputValue = constrain(_failsafeOutputValue, _minOutputLimit, _maxOutputLimit);
    _currentSmoothedOutput = constrain(_currentSmoothedOutput, _minOutputLimit, _maxOutputLimit);
    _pidOutput = constrain(_pidOutput, _minOutputLimit, _maxOutputLimit);
}

void AutoThrottleNG::setControllerDirection(int direction)
{
    _pid.SetControllerDirection(direction);
}

void AutoThrottleNG::setSampleTime(int sampleTimeMillis)
{
    if (sampleTimeMillis > 0) {
        _pid.SetSampleTime(sampleTimeMillis);
    }
}

void AutoThrottleNG::setMode(int mode)
{
    if (isInErrorState())
        return; // Don't allow external mode change if in failsafe
    _pid.SetMode(mode);
    if (mode == MANUAL) {
        // Update PID's output variable when switching externally to MANUAL
        _pidOutput = _currentSmoothedOutput;
    }
}

// --- Filtering ---

void AutoThrottleNG::setInputFilterAlpha(double alpha)
{
    _inputFilterAlpha = constrain(alpha, 0.0, 1.0);
    // If disabling filter, ensure pidInput matches last raw input
    if (_inputFilterAlpha == 0.0) {
        _pidInput = _lastRawInput;
    }
}

// --- Smoothing ---

void AutoThrottleNG::enableSmoothing(bool enable, double maxRatePerSecond)
{
    _smoothingEnabled = enable;
    if (maxRatePerSecond > 0) {
        _smoothingRate = maxRatePerSecond;
    }
    _lastComputeMillis = millis(); // Reset timer
}

// --- Failsafe Configuration ---

void AutoThrottleNG::setFailsafeValue(double value)
{
    _failsafeOutputValue = constrain(value, _minOutputLimit, _maxOutputLimit);
}

void AutoThrottleNG::setInputTimeout(unsigned long durationMillis)
{
    _inputTimeoutMillis = durationMillis;
}

void AutoThrottleNG::setStabilityParams(double tolerance, unsigned long durationMillis)
{
    if (tolerance >= 0) {
        _stabilityTolerance = tolerance;
    }
    _stabilityTimeoutMillis = durationMillis;
}

// --- Status & Debugging ---

double AutoThrottleNG::getThrottle() const { return _currentSmoothedOutput; }
double AutoThrottleNG::getRawPIDOutput() const { return _pidOutput; }
double AutoThrottleNG::getFilteredInput() const { return _pidInput; } // Input used by PID
double AutoThrottleNG::getRawInput() const { return _lastRawInput; } // Last value from updateInput()

bool AutoThrottleNG::isStable() const
{
    // Compare PID setpoint with the FILTERED input PID is actually using
    return abs(_pidSetpoint - _pidInput) <= abs(_stabilityTolerance);
}

double AutoThrottleNG::getKp() const { return _pid.GetKp(); }
double AutoThrottleNG::getKi() const { return _pid.GetKi(); }
double AutoThrottleNG::getKd() const { return _pid.GetKd(); }
int AutoThrottleNG::getMode() const { return _pid.GetMode(); } // Returns int

double AutoThrottleNG::getSetpoint() const { return _pidSetpoint; }
// getInput() renamed to getFilteredInput() for clarity
// double AutoThrottleNG::getInput() const { return _pidInput; }

AutoThrottleNG::Error AutoThrottleNG::getErrorState() const { return _errorState; }
bool AutoThrottleNG::isInErrorState() const { return _errorState != Error::NONE; }

void AutoThrottleNG::clearErrorState()
{
    if (_errorState != Error::NONE) {
        Error oldError = _errorState;
        _errorState = Error::NONE;
        _unstableStartMillis = 0;
        _currentlyStable = true;
        _pid.SetMode(AUTOMATIC); // Resume automatic mode
        Serial.print(F("AutoThrottleNG: Error cleared (was ")); // F() saves RAM
        Serial.print((int)oldError);
        Serial.println(F("). Resuming AUTO mode."));
    }
}

bool AutoThrottleNG::isSaturated() const { return _outputSaturated; }
unsigned long AutoThrottleNG::getLastUpdateTime() const { return _lastValidUpdateMillis; }

// --- Private Helper Methods ---

double AutoThrottleNG::applyInputFilter(double rawValue)
{
    if (_inputFilterAlpha <= 0.0) { // No filter
        return rawValue;
    } else {
        // Apply Exponential Moving Average (EMA)
        // Filtered = alpha * CurrentRaw + (1 - alpha) * PreviousFiltered
        // Note: _pidInput holds the previous filtered value here
        return (_inputFilterAlpha * rawValue) + (1.0 - _inputFilterAlpha) * _pidInput;
        // This filtered value is then assigned back to _pidInput in updateInput()
    }
}

double AutoThrottleNG::applyOutputSmoothing(double rawOutput)
{
    unsigned long currentMillis = millis();
    // Use _lastComputeMillis used by compute() for accurate delta time
    unsigned long timeDelta = currentMillis - _lastComputeMillis;

    if (timeDelta == 0)
        return _currentSmoothedOutput; // Avoid division by zero

    double maxChange = _smoothingRate * (double)timeDelta / 1000.0;
    double targetValue = constrain(rawOutput, _minOutputLimit, _maxOutputLimit);
    double newSmoothedValue;

    if (targetValue > _currentSmoothedOutput) {
        newSmoothedValue = min(targetValue, _currentSmoothedOutput + maxChange);
    } else if (targetValue < _currentSmoothedOutput) {
        newSmoothedValue = max(targetValue, _currentSmoothedOutput - maxChange);
    } else {
        newSmoothedValue = _currentSmoothedOutput; // No change needed
    }
    // Final constraint happens in compute() after smoothing
    return newSmoothedValue;
}

void AutoThrottleNG::updateFailsafeState(Error newState)
{
    if (_errorState == Error::NONE && newState != Error::NONE) {
        _errorState = newState;
        _pid.SetMode(MANUAL); // Go to manual on error
        Serial.print(F("AutoThrottleNG: Failsafe triggered! State: "));
        Serial.println((int)newState);
    } else if (_errorState != Error::NONE && newState != Error::NONE && _errorState != newState) {
        _errorState = newState;
        _pid.SetMode(MANUAL); // Ensure still manual
        Serial.print(F("AutoThrottleNG: Failsafe state updated! State: "));
        Serial.println((int)newState);
    }
    // Setting state to NONE is handled by clearErrorState()
}

void AutoThrottleNG::checkTimeouts(unsigned long currentMillis)
{
    // Input Timeout Check
    if (_inputTimeoutMillis > 0) {
        if ((currentMillis - _lastValidUpdateMillis) > _inputTimeoutMillis) {
            updateFailsafeState(Error::INPUT_TIMEOUT);
            if (isInErrorState())
                return; // Stop checking if error was set
        }
    }

    // Stability Timeout Check (only if not already in another error state)
    if (_stabilityTimeoutMillis > 0 && !isInErrorState()) {
        if (!isStable()) { // isStable() uses filtered input (_pidInput)
            // Currently unstable
            if (_currentlyStable) {
                // Just became unstable
                _currentlyStable = false;
                _unstableStartMillis = currentMillis;
            } else {
                // Already unstable, check duration
                if ((currentMillis - _unstableStartMillis) > _stabilityTimeoutMillis) {
                    updateFailsafeState(Error::STABILITY_TIMEOUT);
                }
            }
        } else {
            // Currently stable
            if (!_currentlyStable) {
                // Just became stable
                _currentlyStable = true;
                _unstableStartMillis = 0; // Reset timer
                // No need to clear STABILITY_TIMEOUT here, failsafe check at start of compute() handles it
            }
        }
    }
}

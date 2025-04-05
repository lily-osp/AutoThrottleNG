#ifndef AUTOTHROTTLE_NG_H
#define AUTOTHROTTLE_NG_H

#include <Arduino.h>
#include <PID_v1.h> // Use standard Arduino PID library

// Ensure PID library constants are available (may be defined in PID.h or Arduino.h)
#ifndef DIRECT
#define DIRECT 0
#endif
#ifndef REVERSE
#define REVERSE 1
#endif
#ifndef MANUAL
#define MANUAL 0
#endif
#ifndef AUTOMATIC
#define AUTOMATIC 1
#endif

class AutoThrottleNG {
public:
    // --- Error States ---
    enum class Error : uint8_t {
        NONE = 0,
        INPUT_INVALID, // NaN or Inf sensor reading
        INPUT_TIMEOUT, // updateInput() not called recently
        STABILITY_TIMEOUT // Error outside tolerance for too long
    };

    /**
     * @brief Constructor.
     * @param minOutput Minimum allowable throttle output.
     * @param maxOutput Maximum allowable throttle output.
     * @param kp Initial Proportional gain.
     * @param ki Initial Integral gain.
     * @param kd Initial Derivative gain.
     * @param POn Proportional mode (P_ON_M or P_ON_E). Default: P_ON_E.
     * @param direction Controller direction (DIRECT or REVERSE). Default: DIRECT.
     */
    AutoThrottleNG(double minOutput, double maxOutput,
        double kp, double ki, double kd,
        int POn = P_ON_E, int direction = DIRECT);

    // --- Core Runtime Methods ---

    /** @brief Sets the desired target value (setpoint). Rejects NaN/Infinity. */
    void setTarget(double target);

    /**
     * @brief Updates the controller with the latest raw sensor reading.
     * Applies input filter before using value for PID. MUST be called regularly.
     * @param rawInputValue The current raw sensor value. Rejects NaN/Infinity.
     */
    void updateInput(double rawInputValue);

    /**
     * @brief Computes the new throttle value. Call this repeatedly in loop().
     * Handles failsafe checks, triggers PID computation, applies filters and smoothing.
     * @return The computed throttle value (smoothed or failsafe).
     */
    double compute();

    /** @brief Resets the PID controller's internal state and AutoThrottle timers/errors. */
    void reset();

    // --- PID Configuration ---

    /** @brief Sets new PID tuning parameters. */
    void setTunings(double kp, double ki, double kd, int POn = -1); // POn=-1 means don't change

    /** @brief Sets the minimum and maximum throttle output limits. */
    void setOutputLimits(double minOutput, double maxOutput);

    /** @brief Sets the controller direction (DIRECT or REVERSE). */
    void setControllerDirection(int direction);

    /** @brief Sets the PID computation interval. */
    void setSampleTime(int sampleTimeMillis);

    /** @brief Sets the PID mode (AUTOMATIC or MANUAL). Failsafe overrides MANUAL. */
    void setMode(int mode);

    // --- Filtering Configuration ---

    /**
     * @brief Sets the Input Filter (EMA) factor (alpha).
     * @param alpha 0.0 (no filter) to 1.0 (no change). Lower values give more smoothing. Default: 0.0.
     */
    void setInputFilterAlpha(double alpha);

    // --- Smoothing Configuration ---

    /**
     * @brief Enables/disables and configures throttle output smoothing.
     * @param enable True to enable smoothing, false to disable.
     * @param maxRatePerSecond Max change in throttle units per second. Must be positive. Default: 10.0.
     */
    void enableSmoothing(bool enable, double maxRatePerSecond = 10.0);

    // --- Failsafe Configuration ---

    /** @brief Sets the throttle value applied when a failsafe condition is active. */
    void setFailsafeValue(double value);

    /** @brief Sets max time (ms) between valid updateInput() calls. 0 disables. */
    void setInputTimeout(unsigned long durationMillis);

    /**
     * @brief Configures the stability check failsafe.
     * @param tolerance The maximum allowable absolute error (|Setpoint - Input|) to be considered stable.
     * @param durationMillis Max time (ms) the system can be unstable before failsafe. 0 disables timeout.
     */
    void setStabilityParams(double tolerance, unsigned long durationMillis);

    // --- Status & Debugging ---

    /** @brief Gets the final computed throttle output (smoothed or failsafe). */
    double getThrottle() const;

    /** @brief Gets the raw output value directly from the PID controller (before smoothing). */
    double getRawPIDOutput() const;

    /** @brief Gets the filtered input value used by the PID controller. */
    double getFilteredInput() const;

    /** @brief Checks if the filtered input is currently within tolerance of the setpoint. */
    bool isStable() const;

    /** @brief Gets the current Proportional gain. */
    double getKp() const;
    /** @brief Gets the current Integral gain. */
    double getKi() const;
    /** @brief Gets the current Derivative gain. */
    double getKd() const;
    /** @brief Gets the current PID mode (AUTOMATIC or MANUAL) as an int. */
    int getMode() const;

    /** @brief Gets the current target setpoint value. */
    double getSetpoint() const;
    /** @brief Gets the last valid raw input value provided via updateInput(). */
    double getRawInput() const;

    /** @brief Gets the current error state. */
    Error getErrorState() const;
    /** @brief Returns true if any failsafe condition is active. */
    bool isInErrorState() const;
    /** @brief Manually clears the current error state to allow resuming control. */
    void clearErrorState();

    /** @brief Returns true if the raw PID output is at its min or max limit. */
    bool isSaturated() const;
    /** @brief Gets the timestamp (millis()) of the last valid updateInput() call. */
    unsigned long getLastUpdateTime() const;

private:
    // --- Internal PID Instance & Shared Variables ---
    // These are the variables the standard Arduino PID library operates on via pointers
    double _pidInput; // Filtered input value passed to PID
    double _pidOutput; // Raw output value calculated by PID
    double _pidSetpoint; // Target value used by PID
    PID _pid; // The PID controller instance

    // --- Configuration & State ---
    double _minOutputLimit;
    double _maxOutputLimit;
    double _lastRawInput; // Last valid value from updateInput()
    double _currentSmoothedOutput; // Final output after smoothing/failsafe
    bool _smoothingEnabled;
    double _smoothingRate; // Max rate per second
    double _inputFilterAlpha; // EMA factor for input filter (0=off)
    unsigned long _lastComputeMillis;

    // --- Failsafe ---
    Error _errorState;
    double _failsafeOutputValue;
    unsigned long _inputTimeoutMillis;
    unsigned long _stabilityTimeoutMillis;
    double _stabilityTolerance;
    unsigned long _lastValidUpdateMillis;
    unsigned long _unstableStartMillis;
    bool _currentlyStable;
    bool _outputSaturated;

    // --- Helper Methods ---
    double applyInputFilter(double rawValue);
    double applyOutputSmoothing(double rawOutput);
    void updateFailsafeState(Error newState);
    void checkTimeouts(unsigned long currentMillis);
};

#endif // AUTOTHROTTLE_NG_H

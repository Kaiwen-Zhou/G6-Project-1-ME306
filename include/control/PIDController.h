#pragma once

/*
 * Reusable P/PI controller for the MECHENG 306 X-Y plotter.
 *
 * The class is called PIDController to match the project file names. The
 * current implementation uses proportional and integral control only because
 * a derivative term is not required unless hardware testing shows a need for
 * it. Set Ki to zero for proportional-only control.
 *
 * Each object contains its own gains, integral state, and limits. Therefore,
 * the same class can be used for two separate controller instances.
 */
class PIDController
{
public:
    PIDController(float proportionalGain,
                  float integralGain,
                  float minimumOutput,
                  float maximumOutput,
                  float minimumIntegralOutput,
                  float maximumIntegralOutput);

    // Calculate one control update from the current error.
    // timeStepSeconds should be the control-loop period in seconds.
    float update(float error, float timeStepSeconds);

    // Clear the stored integral output.
    void reset();

    // Change P and I gains without changing the stored integral output.
    // Call reset() before starting a new tuning test.
    void setGains(float proportionalGain, float integralGain);

    // Change the final controller-output limits.
    // minimumOutput must be less than or equal to maximumOutput.
    void setOutputLimits(float minimumOutput, float maximumOutput);

    // Change the limits applied to the integral contribution.
    // These values use the same units as the controller output.
    void setIntegralLimits(float minimumIntegralOutput,
                           float maximumIntegralOutput);

    // Read-only access for tests and telemetry.
    float getProportionalGain() const;
    float getIntegralGain() const;
    float getIntegralOutput() const;

private:
    float proportionalGain_;
    float integralGain_;
    float integralOutput_;

    float minimumOutput_;
    float maximumOutput_;

    float minimumIntegralOutput_;
    float maximumIntegralOutput_;
};
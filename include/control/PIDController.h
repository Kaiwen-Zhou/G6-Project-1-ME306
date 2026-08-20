#pragma once

#include <stdint.h>

/*
 * PI position controller with velocity feedforward.
 *
 * Feedback:
 *     Kp * positionError + accumulated Ki * positionError * dt
 *
 * Feedforward:
 *     Kv * targetVelocity
 *
 * Position error is normally measured in encoder counts.
 * Target velocity is normally measured in counts/s.
 * The final output uses motor-command/PWM units.
 * u = Kp ​e + I + Kv ​Vreference​
 * e: reference count - actual count
 * targetVelocityCountsPerSecond: trajectory reference generated A/B reference velocity
 * Kp/Ki/Kv: PWM/count or count's or count/s
 */
class PIDController {
    public:
        PIDController(float proportionalGain, float integralGain, float minimumOutput, float maximumOutput,
                      float minimumIntegralOutput, float maximumIntegralOutput, float velocityFeedforwardGain = 0.0f);

        // PI position feedback plus velocity feedforward.
        float update(float error, float targetVelocityCountsPerSecond, float timeStepSeconds);

        // Endpoint-aware overload.
        //
        // blockedIntegralDirection:
        //   +1 blocks integral accumulation in the positive-output direction.
        //   -1 blocks integral accumulation in the negative-output direction.
        //    0 leaves integral accumulation unrestricted.
        //
        // integralBleedRatePerSecond gently removes stored integral that is
        // already in the blocked direction. Integral change in the opposite
        // direction remains allowed so braking and overshoot recovery can
        // still build integral normally.
        float update(float error,
                     float targetVelocityCountsPerSecond,
                     float timeStepSeconds,
                     int8_t blockedIntegralDirection,
                     float integralBleedRatePerSecond);

        // Compatibility overload for existing code.
        // Equivalent to target velocity = 0.
        float update(float error, float timeStepSeconds);

        // Clear the stored integral contribution.
        void reset();

        // Change feedback gains without changing the stored integral state.
        void setGains(float proportionalGain, float integralGain);

        void setVelocityFeedforwardGain(float velocityFeedforwardGain);

        void setOutputLimits(float minimumOutput, float maximumOutput);

        void setIntegralLimits(float minimumIntegralOutput, float maximumIntegralOutput);

        float getProportionalGain() const;
        float getIntegralGain() const;
        float getVelocityFeedforwardGain() const;
        float getIntegralOutput() const;

    private:
        float proportionalGain_;
        float integralGain_;
        float velocityFeedforwardGain_;

        // Integral contribution in motor-output units.
        float integralOutput_;

        float minimumOutput_;
        float maximumOutput_;

        float minimumIntegralOutput_;
        float maximumIntegralOutput_;
};

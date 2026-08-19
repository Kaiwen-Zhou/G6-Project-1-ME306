#pragma once

#include <stdint.h>

#include "control/Converter.h"
#include "hardware/Encoder.h"
#include "hardware/LimitSwitch.h"
#include "hardware/MotorDriver.h"

enum class HomingStage : uint8_t { IDLE, X_ORIGIN, Y_ORIGIN, COMPLETE, ABORTED };

enum class HomingPhase : uint8_t {
    IDLE,
    COARSE_APPROACH,
    CONTACT_PAUSE,
    BACKOFF,
    FINE_APPROACH,
    FINE_CONTACT_PAUSE,
    FINAL_RELEASE,
    RECORD_POSITION,
    COMPLETE,
    ABORTED
};

enum class ExpectedSwitch : uint8_t { X_MIN = 0, X_MAX = 1, Y_MIN = 2, Y_MAX = 3, NONE = 255 };

enum class HomingFault : uint8_t { NONE, TIMEOUT, WRONG_LIMIT, CONTRADICTORY_LIMITS, INVALID_CONFIGURATION };

struct HomingConfig {
        uint8_t coarseApproachPwm;
        uint8_t backoffPwm;
        uint8_t fineApproachPwm;
        uint8_t finalReleasePwm;

        float backoffDistanceMm;

        bool straightnessCorrectionEnabled;
        float straightnessKpPwmPerMm;
        uint8_t straightnessMaximumCorrectionPwm;
        float straightnessDeadbandMm;

        unsigned long contactPauseMs;
        unsigned long fineContactPauseMs;
        unsigned long searchTimeoutMs;
        unsigned long backoffTimeoutMs;
        unsigned long finalReleaseTimeoutMs;
        unsigned long overallTimeoutMs;
};

struct HomingResult {
        Encoder::CountPair xOrigin;
        Encoder::CountPair yOrigin;

        bool xValid;
        bool yValid;
};

class HomingController {
    public:
        HomingController(Encoder& encoderA, Encoder& encoderB, MotorDriver& motorA, MotorDriver& motorB,
                         const Converter& converter, LimitSwitch& xMinSwitch, LimitSwitch& xMaxSwitch,
                         LimitSwitch& yMinSwitch, LimitSwitch& yMaxSwitch, const HomingConfig& config,
                         bool debounceLimitInterrupts = true);

        // LimitSwitch::begin(...) and the ISR wiring remain hardware-startup
        // responsibilities. This method only resets homing state.
        void begin();

        // Home X directly to X_MIN, then Y directly to Y_MIN. Each target keeps
        // the coarse-contact, backoff, fine-contact, and final-release sequence.
        // X_MAX and Y_MAX remain monitored safety inputs but are not calibration
        // targets. Returns false only when homing cannot be started safely.
        bool start();

        // Execute at most one non-blocking state-machine step.
        void update();

        // Called from the main loop after a limit ISR is latched. Immediately
        // stops the open-loop primary homing motion and encoder straightness
        // trim. Configuration selects whether the
        // edge must then pass debounce verification before it is accepted.
        void notifyLimitInterrupt(uint8_t interruptMask);

        // Immediately stop all homing motor output.
        void stop();

        bool isActive() const;
        bool isComplete() const;
        bool hasFault() const;

        HomingStage stage() const;
        HomingPhase phase() const;
        ExpectedSwitch expectedSwitch() const;
        HomingFault fault() const;
        HomingResult result() const;

    private:
        static constexpr uint8_t LIMIT_SWITCH_COUNT = 4;

        void beginTarget(HomingStage nextStage);
        void setPhase(HomingPhase nextPhase);
        void advanceAfterOriginRecorded();

        void updateAllSwitches();
        void clearSwitchEvents();
        bool interruptVerificationPending();
        bool switchTriggered(ExpectedSwitch expected, uint8_t interruptMask) const;
        bool hasContradictoryLimits(uint8_t interruptMask = 0U) const;
        bool hasUnexpectedLimit(uint8_t interruptMask = 0U);

        LimitSwitch& switchFor(ExpectedSwitch expected);

        int8_t targetXDirection() const;
        int8_t targetYDirection() const;

        void driveTowardTarget(uint8_t pwm);
        void driveAwayFromTarget(uint8_t pwm);
        void driveCartesian(int8_t xDirection, int8_t yDirection, uint8_t pwm);
        float crossAxisErrorMm(int8_t xDirection, int8_t yDirection) const;
        uint8_t straightnessCorrectionPwm(float crossAxisErrorMm, uint8_t basePwm) const;
        void stopMotors();

        float distanceFromFirstContactMm() const;
        void recordCurrentTarget();

        bool phaseTimedOut(unsigned long timeoutMs) const;
        bool configurationIsValid() const;
        void fail(HomingFault fault);

        Encoder& encoderA_;
        Encoder& encoderB_;

        MotorDriver& motorA_;
        MotorDriver& motorB_;

        const Converter& converter_;

        LimitSwitch* limitSwitches_[LIMIT_SWITCH_COUNT];

        HomingConfig config_;
        HomingResult result_;

        Encoder::CountPair firstContactCounts_;
        Encoder::CountPair releaseCounts_;
        Encoder::CountPair targetStartCounts_;

        HomingStage stage_;
        HomingPhase phase_;
        ExpectedSwitch expectedSwitch_;
        HomingFault fault_;

        unsigned long overallStartMs_;
        unsigned long phaseStartMs_;

        // A non-expected switch that is already pressed at the start of a
        // target is allowed until it releases. If it presses again later,
        // it is treated as the wrong limit.
        uint8_t allowedPressedMask_;

        uint8_t interruptVerificationMask_;
        bool debounceLimitInterrupts_;

        bool active_;
};

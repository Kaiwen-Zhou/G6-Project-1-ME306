#pragma once

#include <stdint.h>

#include "control/Converter.h"
#include "hardware/Encoder.h"
#include "hardware/LimitSwitch.h"
#include "hardware/MotorDriver.h"

enum class HomingStage : uint8_t {
    IDLE,
    X_MAX,
    X_ORIGIN,
    Y_MAX,
    Y_ORIGIN,
    COMPLETE,
    ABORTED
};

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

enum class ExpectedSwitch : uint8_t {
    X_MIN = 0,
    X_MAX = 1,
    Y_MIN = 2,
    Y_MAX = 3,
    NONE = 255
};

enum class HomingFault : uint8_t {
    NONE,
    TIMEOUT,
    WRONG_LIMIT,
    CONTRADICTORY_LIMITS,
    INVALID_CONFIGURATION
};

struct HomingConfig {
    uint8_t coarseApproachPwm;
    uint8_t backoffPwm;
    uint8_t fineApproachPwm;
    uint8_t finalReleasePwm;

    float backoffDistanceMm;

    unsigned long contactPauseMs;
    unsigned long fineContactPauseMs;
    unsigned long searchTimeoutMs;
    unsigned long backoffTimeoutMs;
    unsigned long finalReleaseTimeoutMs;
    unsigned long overallTimeoutMs;
};

struct HomingResult {
    Encoder::CountPair xMaximum;
    Encoder::CountPair xOrigin;
    Encoder::CountPair yMaximum;
    Encoder::CountPair yOrigin;

    float xTravelMm;
    float yTravelMm;

    bool xValid;
    bool yValid;
};

class HomingController {
public:
    HomingController(
        Encoder& encoderA,
        Encoder& encoderB,
        MotorDriver& motorA,
        MotorDriver& motorB,
        const Converter& converter,
        LimitSwitch& xMinSwitch,
        LimitSwitch& xMaxSwitch,
        LimitSwitch& yMinSwitch,
        LimitSwitch& yMaxSwitch,
        const HomingConfig& config);

    // LimitSwitch::begin(...) and the ISR wiring remain hardware-startup
    // responsibilities. This method only resets homing state.
    void begin();

    // Returns false only when homing cannot be started safely.
    bool start();

    // Execute at most one non-blocking state-machine step.
    void update();

    // Immediately stop open-loop homing motion.
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
    void advanceAfterFineContact();

    void updateAllSwitches();
    void clearSwitchEvents();
    bool hasContradictoryLimits() const;
    bool hasUnexpectedLimit();

    LimitSwitch& switchFor(ExpectedSwitch expected);

    int8_t targetXDirection() const;
    int8_t targetYDirection() const;

    void driveTowardTarget(uint8_t pwm);
    void driveAwayFromTarget(uint8_t pwm);
    void driveCartesian(int8_t xDirection, int8_t yDirection, uint8_t pwm);
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

    bool active_;
};

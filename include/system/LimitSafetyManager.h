#pragma once

#include <stdint.h>

#include "communication/GCodeController.h"
#include "hardware/LimitSwitch.h"
#include "system/PlotterSystem.h"

/**
 * Normal-operation limit monitoring and recoverable-switch bookkeeping.
 *
 * Outside HOMING, call updateSwitches() every loop, pass latched ISR masks to
 * handleLimitInterrupts(), and call update() for the low-rate safety backup.
 * The manager classifies boundary-consistent faults and maintains the
 * expected-until-release policy used after M999 and successful homing.
 */
namespace plotter {

struct LimitSafetyUpdate {
        bool faultEntered;
        uint8_t pressedMask;
        uint8_t recoverableMask;
        uint8_t releasedExpectedMask;
        float positionXMm;
        float positionYMm;
};

class LimitSafetyManager {
    public:
        enum LimitMask : uint8_t {
            LEFT_LIMIT_MASK = 1U << 0,
            RIGHT_LIMIT_MASK = 1U << 1,
            BOTTOM_LIMIT_MASK = 1U << 2,
            TOP_LIMIT_MASK = 1U << 3
        };

        LimitSafetyManager(LimitSwitch& leftLimit, LimitSwitch& rightLimit, LimitSwitch& bottomLimit,
                           LimitSwitch& topLimit, GCodeController& gCodeController, PlotterSystem& plotterSystem);

        void begin();

        // Called every application loop outside HOMING so each switch can finish
        // its own debounce state machine.
        void updateSwitches();

        // Fast path for the switch mask latched by the limit ISRs. Unexpected
        // edges become candidates until their debounce verification completes.
        LimitSafetyUpdate handleLimitInterrupts(uint8_t interruptMask);

        // Low-rate debounced safety backup. It may report an UNEXPECTED_LIMIT
        // fault through PlotterSystem if an interrupt was missed.
        LimitSafetyUpdate update();

        // M999 policy: all currently pressed switches must already be expected or
        // have been attributed to a matching physical boundary at fault entry.
        bool faultResetAllowed();
        void armRecoverableLimitsAfterReset();

        // After a successful G28, currently pressed eligible origin switches
        // use the same expected-until-release policy as M999 recovery.
        uint8_t armPressedLimitsAfterHoming(uint8_t eligibleMask);

        // A new G28 starts a separate homing policy and clears recovery state.
        void clearForHoming();

        // Non-limit faults must not inherit a previous fault attribution.
        void clearRecoverableFaultAttribution();

        uint8_t pressedMask() const;
        uint8_t expectedMask() const;
        uint8_t recoverableMask() const;
        uint8_t resetBlockingMask() const;

    private:
        LimitSafetyUpdate enterUnexpectedLimitFault(uint8_t unexpectedMask);
        uint8_t collectConfirmedInterrupts();
        uint8_t unexpectedPressedMask() const;
        uint8_t classifyRecoverableBoundaryLimits(uint8_t candidateMask) const;
        uint8_t clearReleasedExpectedLimits();

        LimitSwitch& leftLimit_;
        LimitSwitch& rightLimit_;
        LimitSwitch& bottomLimit_;
        LimitSwitch& topLimit_;
        GCodeController& gCodeController_;
        PlotterSystem& plotterSystem_;

        uint8_t recoverableLimitMask_;
        uint8_t expectedLimitMask_;
        uint8_t pendingInterruptMask_;
        uint8_t lastResetBlockingLimitMask_;
        unsigned long lastSafetyCheckMs_;
};

} // namespace plotter

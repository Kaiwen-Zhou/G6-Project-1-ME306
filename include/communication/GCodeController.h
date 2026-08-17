#pragma once

#include <stdint.h>

#include "communication/GCodeParser.h"
#include "control/Converter.h"
#include "control/HomingController.h"
#include "hardware/Encoder.h"
#include "system/PlotterSystem.h"

/**
 * GCodeController.h
 *
 * Small application-layer bridge between GCodeParser and PlotterSystem.
 *
 * Responsibilities:
 *   - read one complete G-code line through GCodeParser;
 *   - pass G01, G28 and M999 to the correct PlotterSystem function;
 *   - commit parser inheritance only after the system accepts a command;
 *   - load configured X/Y soft limits after successful origin homing.
 *
 * This class does not read Serial, drive motors, update the control loop, or
 * decide whether a physical limit switch is released. Those remain main-loop
 * responsibilities so the class can also be tested on a computer.
 */

namespace plotter {

enum class GCodeControllerError : uint8_t {
    NONE,
    PARSE_ERROR,
    SYSTEM_NOT_STARTED,
    LIMIT_SWITCH_ACTIVE,
    MOVE_TOWARD_ACTIVE_LIMIT,
    FSM_REJECTED,
    INVALID_HOMING_RESULT
};

struct GCodeControllerResult {
        // False while processCharacter() is still waiting for CR or LF.
        bool lineComplete;

        // True only when the complete command was accepted.
        bool accepted;

        // True only when a non-zero G01 started physical motion.
        bool movementStarted;

        GCodeCommand command;
        GCodeParseError parseError;
        GCodeControllerError controllerError;
        RejectReason rejectReason;
};

class GCodeController {
    public:
        GCodeController(PlotterSystem& plotterSystem, 
                        HomingController& homingController, 
                        const Converter& converter,
                        Encoder& encoderA, 
                        Encoder& encoderB,
                        float xTravelMm, 
                        float yTravelMm,
                        float maximumFeedrateMmPerMinute, 
                        float maximumAccelerationMmPerSecondSquared,
                        GCodePositioningMode positioningMode);

        // Reset communication state. This deliberately does not initialise or
        // start PlotterSystem; the application starts it through G28.
        void begin();

        // Non-blocking serial-input path. Call once for every received character.
        // faultResetAllowed is decided by the application's switch-recovery
        // policy and is used only for M999.
        GCodeControllerResult processCharacter(char character, bool faultResetAllowed);

        // Complete-line path for tests and applications that already own a line
        // buffer. The line must be null terminated and must not include CR/LF.
        GCodeControllerResult processLine(const char* line, bool faultResetAllowed);

        // Call after PlotterSystem::update(). When origin homing has completed,
        // this loads the configured min-to-max travel into GCodeParser.
        // Returns true only on the update that successfully loads new limits.
        bool updateAfterSystem();

        // Stop an active HOMING or MOVING operation through the normal FAULT path.
        // Returns true when a new fault was reported.
        bool emergencyStop(FaultCode faultCode = FaultCode::INTERNAL_ERROR);

        // Explicitly invalidate the configured workspace when a caller knows the
        // coordinate reference is no longer trustworthy. Ordinary FAULT/M999
        // recovery deliberately preserves machine zero and the loaded soft limits.
        void requireNewHoming();

        // Supply the application-layer recovery exemption. Bit positions follow
        // ExpectedSwitch: X_MIN=0, X_MAX=1, Y_MIN=2, Y_MAX=3. This affects only
        // G01 direction validation; ordinary Cartesian soft limits still apply.
        void setExpectedLimitMask(uint8_t expectedLimitMask);

        bool systemStarted() const;
        bool safetyLimitsLoaded() const;

        GCodeSafetyLimits safetyLimits() const;
        Converter::CartesianDisplacement currentCartesianPosition() const;

        float maximumAccelerationMmPerSecondSquared() const;

        static const char* controllerErrorMessage(GCodeControllerError error);

    private:
        GCodeControllerResult executeParsedResult(const GCodeParseResult& parseResult, bool faultResetAllowed);

        bool moveWouldPressExpectedLimit(const GCodeCommand& command) const;

        void invalidateSafetyLimits();

        PlotterSystem& plotterSystem_;
        HomingController& homingController_;
        const Converter& converter_;
        Encoder& encoderA_;
        Encoder& encoderB_;

        GCodePositioningMode positioningMode_;
        GCodeParser parser_;
        GCodeSafetyLimits safetyLimits_;

        float xTravelMm_;
        float yTravelMm_;
        float maximumFeedrateMmPerMinute_;
        float maximumAccelerationMmPerSecondSquared_;

        bool systemStarted_;
        bool safetyLimitsLoaded_;
        uint8_t expectedLimitMask_;
};

} // namespace plotter

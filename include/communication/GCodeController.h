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
 *   - load measured X/Y soft limits after successful homing.
 *
 * This class does not read Serial, drive motors, update the control loop, or
 * decide whether a physical limit switch is released. Those remain main-loop
 * responsibilities so the class can also be tested on a computer.
 */

namespace plotter
{

enum class GCodeControllerError : uint8_t
{
    NONE,
    PARSE_ERROR,
    SYSTEM_NOT_STARTED,
    LIMIT_SWITCH_ACTIVE,
    FSM_REJECTED,
    INVALID_HOMING_RESULT
};

struct GCodeControllerResult
{
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

class GCodeController
{
public:
    GCodeController(
        PlotterSystem& plotterSystem,
        HomingController& homingController,
        const Converter& converter,
        Encoder& encoderA,
        Encoder& encoderB,
        float maximumFeedrateMmPerMinute,
        float maximumAccelerationMmPerSecondSquared);

    // Reset communication state. This deliberately does not initialise or
    // start PlotterSystem; the first accepted G28 does that.
    void begin();

    // Non-blocking serial-input path. Call once for every received character.
    // allLimitSwitchesReleased is used only by the external M999 reset policy.
    GCodeControllerResult processCharacter(
        char character,
        bool allLimitSwitchesReleased);

    // Complete-line path for tests and applications that already own a line
    // buffer. The line must be null terminated and must not include CR/LF.
    GCodeControllerResult processLine(
        const char* line,
        bool allLimitSwitchesReleased);

    // Call after PlotterSystem::update(). When homing has completed, this
    // loads the newly measured workspace into GCodeParser.
    // Returns true only on the update that successfully loads new limits.
    bool updateAfterSystem();

    // Stop an active HOMING or MOVING operation through the normal FAULT path.
    // Returns true when a new fault was reported.
    bool emergencyStop(FaultCode faultCode = FaultCode::INTERNAL_ERROR);

    // Invalidate the measured workspace after any fault that could make the
    // Cartesian position uncertain. M999 may clear the FSM fault, but G01
    // remains blocked until a complete G28 loads a new measurement.
    void requireNewHoming();

    bool systemStarted() const;
    bool safetyLimitsLoaded() const;

    GCodeSafetyLimits safetyLimits() const;
    Converter::CartesianDisplacement currentCartesianPosition() const;

    float maximumAccelerationMmPerSecondSquared() const;

    static const char* controllerErrorMessage(GCodeControllerError error);

private:
    GCodeControllerResult executeParsedResult(
        const GCodeParseResult& parseResult,
        bool allLimitSwitchesReleased);

    void invalidateSafetyLimits();

    PlotterSystem& plotterSystem_;
    HomingController& homingController_;
    const Converter& converter_;
    Encoder& encoderA_;
    Encoder& encoderB_;

    GCodeParser parser_;
    GCodeSafetyLimits safetyLimits_;

    float maximumFeedrateMmPerMinute_;
    float maximumAccelerationMmPerSecondSquared_;

    bool systemStarted_;
    bool safetyLimitsLoaded_;
};

}  // namespace plotter

#include "communication/GCodeController.h"

namespace plotter
{

namespace
{

GCodeSafetyLimits makeInvalidSafetyLimits(float maximumFeedrateMmPerMinute)
{
    // Minimum greater than maximum intentionally keeps G01 disabled until a
    // valid HomingResult is available.
    return {
        1.0f,
        0.0f,
        1.0f,
        0.0f,
        maximumFeedrateMmPerMinute
    };
}

GCodeCommand makeEmptyCommand()
{
    return {
        GCodeCommandType::NONE,
        0.0f,
        0.0f,
        0.0f,
        false,
        false,
        false
    };
}

GCodeControllerResult waitingResult()
{
    return {
        false,
        false,
        false,
        makeEmptyCommand(),
        GCodeParseError::NONE,
        GCodeControllerError::NONE,
        RejectReason::NONE
    };
}

GCodeControllerResult rejectedResult(
    const GCodeCommand& command,
    GCodeControllerError controllerError,
    RejectReason rejectReason = RejectReason::NONE)
{
    return {
        true,
        false,
        false,
        command,
        GCodeParseError::NONE,
        controllerError,
        rejectReason
    };
}

}  // namespace

GCodeController::GCodeController(
    PlotterSystem& plotterSystem,
    HomingController& homingController,
    const Converter& converter,
    Encoder& encoderA,
    Encoder& encoderB,
    float maximumFeedrateMmPerMinute,
    float maximumAccelerationMmPerSecondSquared)
    : plotterSystem_(plotterSystem),
      homingController_(homingController),
      converter_(converter),
      encoderA_(encoderA),
      encoderB_(encoderB),
      parser_(makeInvalidSafetyLimits(maximumFeedrateMmPerMinute)),
      safetyLimits_(makeInvalidSafetyLimits(maximumFeedrateMmPerMinute)),
      maximumFeedrateMmPerMinute_(maximumFeedrateMmPerMinute),
      maximumAccelerationMmPerSecondSquared_(
          maximumAccelerationMmPerSecondSquared),
      systemStarted_(false),
      safetyLimitsLoaded_(false)
{
}

void GCodeController::begin()
{
    systemStarted_ = false;
    safetyLimitsLoaded_ = false;
    parser_.resetState();
    invalidateSafetyLimits();
}

GCodeControllerResult GCodeController::processCharacter(
    char character,
    bool allLimitSwitchesReleased)
{
    const Converter::CartesianDisplacement current =
        currentCartesianPosition();

    const GCodeInputResult inputResult =
        parser_.processCharacter(
            character,
            current.xMm,
            current.yMm);

    if (!inputResult.lineComplete)
    {
        return waitingResult();
    }

    return executeParsedResult(
        inputResult.parseResult,
        allLimitSwitchesReleased);
}

GCodeControllerResult GCodeController::processLine(
    const char* line,
    bool allLimitSwitchesReleased)
{
    const Converter::CartesianDisplacement current =
        currentCartesianPosition();

    const GCodeParseResult parseResult =
        parser_.parseLine(
            line,
            current.xMm,
            current.yMm);

    return executeParsedResult(
        parseResult,
        allLimitSwitchesReleased);
}

bool GCodeController::updateAfterSystem()
{
    if (!systemStarted_ || safetyLimitsLoaded_)
    {
        return false;
    }

    if (plotterSystem_.state() != PlotterState::IDLE ||
        !plotterSystem_.machineZeroKnown())
    {
        return false;
    }

    const HomingResult result = homingController_.result();

    if (!result.xValid || !result.yValid ||
        !(result.xTravelMm > 0.0f) ||
        !(result.yTravelMm > 0.0f))
    {
        return false;
    }

    safetyLimits_ = {
        0.0f,
        result.xTravelMm,
        0.0f,
        result.yTravelMm,
        maximumFeedrateMmPerMinute_
    };

    // safetyLimitsLoaded_ = parser_.setSafetyLimits(safetyLimits_);
    // return safetyLimitsLoaded_;
    parser_ = GCodeParser(safetyLimits_);
    safetyLimitsLoaded_ = true;
    return true;
}

bool GCodeController::emergencyStop(FaultCode faultCode)
{
    if (!systemStarted_ || faultCode == FaultCode::NONE)
    {
        return false;
    }

    const PlotterState currentState = plotterSystem_.state();

    if (currentState != PlotterState::HOMING &&
        currentState != PlotterState::MOVING)
    {
        return false;
    }

    const bool accepted = plotterSystem_.reportFault(faultCode).accepted;

    if (accepted)
    {
        requireNewHoming();
    }

    return accepted;
}

void GCodeController::requireNewHoming()
{
    invalidateSafetyLimits();
}

bool GCodeController::systemStarted() const
{
    return systemStarted_;
}

bool GCodeController::safetyLimitsLoaded() const
{
    return safetyLimitsLoaded_;
}

GCodeSafetyLimits GCodeController::safetyLimits() const
{
    return safetyLimits_;
}

Converter::CartesianDisplacement
GCodeController::currentCartesianPosition() const
{
    const Encoder::CountPair counts =
        Encoder::getCountPair(encoderA_, encoderB_);

    return converter_.motorToCartesianDisplacement(
        static_cast<float>(counts.countA),
        static_cast<float>(counts.countB));
}

float GCodeController::maximumAccelerationMmPerSecondSquared() const
{
    return maximumAccelerationMmPerSecondSquared_;
}

const char* GCodeController::controllerErrorMessage(
    GCodeControllerError error)
{
    switch (error)
    {
        case GCodeControllerError::NONE:
            return "No controller error.";

        case GCodeControllerError::PARSE_ERROR:
            return "The G-code line was rejected by the parser.";

        case GCodeControllerError::SYSTEM_NOT_STARTED:
            return "Run G28 before motion or fault-reset commands.";

        case GCodeControllerError::LIMIT_SWITCH_ACTIVE:
            return "Release all limit switches before M999.";

        case GCodeControllerError::FSM_REJECTED:
            return "The finite state machine rejected the command.";

        case GCodeControllerError::INVALID_HOMING_RESULT:
            return "Homing did not produce a valid measured workspace.";
    }

    return "Unknown G-code controller error.";
}

GCodeControllerResult GCodeController::executeParsedResult(
    const GCodeParseResult& parseResult,
    bool allLimitSwitchesReleased)
{
    if (!parseResult.accepted)
    {
        return {
            true,
            false,
            false,
            parseResult.command,
            parseResult.error,
            GCodeControllerError::PARSE_ERROR,
            RejectReason::NONE
        };
    }

    const GCodeCommand& command = parseResult.command;

    if (command.type == GCodeCommandType::NONE)
    {
        return {
            true,
            true,
            false,
            command,
            GCodeParseError::NONE,
            GCodeControllerError::NONE,
            RejectReason::NONE
        };
    }

    if (command.type == GCodeCommandType::HOME)
    {
        bool accepted = false;
        RejectReason rejectReason = RejectReason::NONE;

        if (!systemStarted_)
        {
            plotterSystem_.begin();
            systemStarted_ = true;
            accepted = plotterSystem_.state() == PlotterState::HOMING;

            if (!accepted)
            {
                rejectReason = RejectReason::UNEXPECTED_EVENT;
            }
        }
        else
        {
            const FSMResult result = plotterSystem_.requestHoming();
            accepted = result.accepted;
            rejectReason = result.rejectReason;
        }

        if (!accepted)
        {
            return rejectedResult(
                command,
                GCodeControllerError::FSM_REJECTED,
                rejectReason);
        }

        // A new home measurement invalidates the previous soft limits until
        // the complete routine succeeds.
        invalidateSafetyLimits();
        parser_.commitCommand(command);

        return {
            true,
            true,
            false,
            command,
            GCodeParseError::NONE,
            GCodeControllerError::NONE,
            RejectReason::NONE
        };
    }

    if (!systemStarted_)
    {
        return rejectedResult(
            command,
            GCodeControllerError::SYSTEM_NOT_STARTED);
    }

    if (command.type == GCodeCommandType::RESET_FAULT)
    {
        if (!allLimitSwitchesReleased)
        {
            return rejectedResult(
                command,
                GCodeControllerError::LIMIT_SWITCH_ACTIVE);
        }

        const FSMResult result = plotterSystem_.resetFault();

        if (!result.accepted)
        {
            return rejectedResult(
                command,
                GCodeControllerError::FSM_REJECTED,
                result.rejectReason);
        }

        parser_.commitCommand(command);

        return {
            true,
            true,
            false,
            command,
            GCodeParseError::NONE,
            GCodeControllerError::NONE,
            RejectReason::NONE
        };
    }

    // A valid F-only line or G01 X0 Y0 changes modal state but does not need
    // to start the planner or motors.
    if (!command.hasMovement)
    {
        parser_.commitCommand(command);

        return {
            true,
            true,
            false,
            command,
            GCodeParseError::NONE,
            GCodeControllerError::NONE,
            RejectReason::NONE
        };
    }

    const FSMResult result =
        plotterSystem_.requestMove(
            // command.xDisplacementMm,
            // command.yDisplacementMm,
            command.xOffsetMm,
            command.yOffsetMm,
            command.feedrateMmPerMinute,
            maximumAccelerationMmPerSecondSquared_);

    if (!result.accepted)
    {
        return rejectedResult(
            command,
            GCodeControllerError::FSM_REJECTED,
            result.rejectReason);
    }

    parser_.commitCommand(command);

    return {
        true,
        true,
        true,
        command,
        GCodeParseError::NONE,
        GCodeControllerError::NONE,
        RejectReason::NONE
    };
}

void GCodeController::invalidateSafetyLimits()
{
    safetyLimits_ =
        makeInvalidSafetyLimits(maximumFeedrateMmPerMinute_);

    // parser_.setSafetyLimits(safetyLimits_);
    parser_ = GCodeParser(safetyLimits_);
    safetyLimitsLoaded_ = false;
}

}  // namespace plotter

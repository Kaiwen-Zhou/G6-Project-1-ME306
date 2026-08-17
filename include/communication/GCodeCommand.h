#pragma once

#include <stdint.h>

/**
 * GCodeCommand.h
 * Data shared between the G-code parser and the plotter system.
 *
 * G01 X and Y values are relative Cartesian offsets in millimetres.
 * The F value is a feedrate in millimetres per minute.
 *
 * This file contains data only. It does not read Serial, change the FSM,
 * plan a trajectory, convert X/Y to motor counts, or drive hardware.
 */

namespace plotter
{

enum class GCodeCommandType : uint8_t
{
    NONE,
    LINEAR_MOVE,
    HOME,
    RESET_FAULT
};

enum class GCodeParseError : uint8_t
{
    NONE,
    NULL_INPUT,
    LINE_TOO_LONG,
    INVALID_CHARACTER,
    MISSING_COMMAND,
    UNKNOWN_COMMAND,
    UNSUPPORTED_COMMAND,
    MISSING_LABEL,
    MISSING_VALUE,
    INVALID_NUMBER,
    NUMBER_OUT_OF_RANGE,
    UNKNOWN_PARAMETER,
    DUPLICATE_PARAMETER,
    MULTIPLE_COMMANDS,
    FEEDRATE_NOT_SET,
    FEEDRATE_NOT_POSITIVE,
    POSITION_OUT_OF_RANGE,
    UNEXPECTED_PARAMETER,
    INVALID_SAFETY_LIMITS
};

struct GCodeCommand
{
    GCodeCommandType type;

    // Relative Cartesian offsets used by G01.
    float xOffsetMm;
    float yOffsetMm;

    // G01 F value. TrajectoryPlanner accepts the same mm/min unit.
    float feedrateMmPerMinute;

    // False for a feedrate-only or zero-offset G01 command.
    bool hasMovement;

    // True when F came from the last committed G01 command.
    bool feedrateInherited;

    // Lecture 6 requires over-speed commands to be throttled, not rejected.
    // True means the requested F was reduced to the configured safe maximum.
    bool feedrateWasLimited;
};

struct GCodeParseResult
{
    // An empty line or comment-only line is accepted with command.type NONE.
    bool accepted;
    GCodeCommand command;
    GCodeParseError error;
};

}  // namespace plotter

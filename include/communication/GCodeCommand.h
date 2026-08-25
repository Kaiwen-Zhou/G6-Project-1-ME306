#pragma once

#include <stdint.h>

/**
 * Shared G-code command, positioning-mode, and parse-result data types.
 *
 * G01 X and Y input values may use absolute coordinates or relative offsets,
 * selected at compile time. GCodeParser always normalises an accepted move to
 * relative Cartesian offsets before it reaches the motion system.
 * The F value remains in millimetres per minute. This module contains data
 * only; parsing, state transitions, planning, and hardware control live in
 * their respective modules.
 */

namespace plotter {

enum class GCodeCommandType : uint8_t { NONE, LINEAR_MOVE, HOME, RESET_FAULT };

enum class GCodePositioningMode : uint8_t { ABSOLUTE, RELATIVE };

enum class GCodeParseError : uint8_t {
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

struct GCodeCommand {
        GCodeCommandType type;

        // Normalised relative Cartesian offsets used by the motion system.
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

struct GCodeParseResult {
        // An empty line or comment-only line is accepted with command.type NONE.
        bool accepted;
        GCodeCommand command;
        GCodeParseError error;
};

} // namespace plotter

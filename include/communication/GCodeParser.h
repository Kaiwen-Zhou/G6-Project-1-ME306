#pragma once

#include <stdint.h>

#include "communication/GCodeCommand.h"

/**
 * GCodeParser.h
 * Non-blocking, fixed-memory parser for the MECHENG 306 plotter.
 *
 * Supported commands:
 *   G01 or G1  - linear movement using X, Y and F
 *   G28        - request homing
 *   M999       - request recovery from the FAULT state
 *
 * G90 and G91 are not parsed. G01 positioning mode is selected at compile
 * time through SystemConfig::GCODE_POSITIONING_MODE and supplied to this
 * parser by GCodeController.
 *
 * A non-positive F is rejected. An F above the configured maximum is accepted
 * but reduced to that maximum, as required by the Lecture 6 demonstration.
 */

namespace plotter {

// The supported commands are short. This fixed limit prevents unbounded
// Serial input without using dynamic memory.
constexpr uint16_t GCODE_MAX_LINE_LENGTH = 96U;

struct GCodeSafetyLimits {
        float xMinimumMm;
        float xMaximumMm;
        float yMinimumMm;
        float yMaximumMm;
        float maximumFeedrateMmPerMinute;
};

struct GCodeInputResult {
        // False means that more Serial characters are still required.
        bool lineComplete;
        GCodeParseResult parseResult;
};

class GCodeParser {
    public:
        GCodeParser(const GCodeSafetyLimits& safetyLimits, GCodePositioningMode positioningMode);

        // Non-blocking Serial-facing interface. Call once for each received
        // character. Parsing occurs only after '\n' or '\r'. CRLF is treated as
        // one line ending. The function does not read Serial itself.
        GCodeInputResult processCharacter(char character, float currentXMm, float currentYMm);

        // Parse a complete null-terminated line. This is useful for unit tests or
        // for a caller that already performs line buffering.
        GCodeParseResult parseLine(const char* line, float currentXMm, float currentYMm) const;

        // Call only after the command has also been accepted by PlotterFSM/
        // PlotterSystem. A parser success alone is not enough to commit it.
        // This prevents a BUSY or FAULT rejection from changing inherited F or
        // the last-command G01 mode.
        void commitCommand(const GCodeCommand& command);

        // Clear the inherited feedrate, G01 mode, and any partial input line.
        void resetState();

        bool hasLastFeedrate() const;
        float lastFeedrateMmPerMinute() const;

        // Human-readable text for Serial error reporting.
        static const char* errorMessage(GCodeParseError error);

    private:
        void clearInputBuffer();

        GCodeSafetyLimits safetyLimits_;
        GCodePositioningMode positioningMode_;

        float lastFeedrateMmPerMinute_;
        bool hasLastFeedrate_;
        bool lastCommandWasLinear_;

        char lineBuffer_[GCODE_MAX_LINE_LENGTH + 1U];
        uint16_t lineLength_;
        bool lineTooLong_;
        bool lineContainsNull_;
        bool ignoreNextLineFeed_;
};

} // namespace plotter

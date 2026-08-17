#include "communication/GCodeParser.h"

#include <float.h>

namespace plotter {

namespace {

enum class NumberStatus : uint8_t { VALID, MISSING, INVALID, OUT_OF_RANGE };

struct NumberResult {
        NumberStatus status;
        float value;
        const char* nextCharacter;
};

struct CommandNumberResult {
        bool valid;
        uint16_t value;
        const char* nextCharacter;
};

bool isDigit(char character) {
    return character >= '0' && character <= '9';
}

bool isLetter(char character) {
    return (character >= 'A' && character <= 'Z') || (character >= 'a' && character <= 'z');
}

bool isWhitespace(char character) {
    return character == ' ' || character == '\t' || character == '\r' || character == '\n';
}

char toUpperCase(char character) {
    if (character >= 'a' && character <= 'z') {
        return static_cast<char>(character - 'a' + 'A');
    }

    return character;
}

bool isKnownLabel(char character) {
    const char upperCharacter = toUpperCase(character);

    return upperCharacter == 'G' || upperCharacter == 'M' || upperCharacter == 'X' || upperCharacter == 'Y' ||
           upperCharacter == 'F';
}

bool isFiniteFloat(float value) {
    // value == value rejects NaN. The range checks reject infinities.
    return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

GCodeCommand emptyCommand() {
    return {GCodeCommandType::NONE, 0.0f, 0.0f, 0.0f, false, false, false};
}

GCodeParseResult acceptedResult(const GCodeCommand& command) {
    return {true, command, GCodeParseError::NONE};
}

GCodeParseResult errorResult(GCodeParseError error) {
    return {false, emptyCommand(), error};
}

GCodeInputResult waitingForLine() {
    return {false, errorResult(GCodeParseError::NONE)};
}

NumberResult parseDecimalNumber(const char* text) {
    const char* cursor = text;
    bool negative = false;

    if (*cursor == '+' || *cursor == '-') {
        negative = (*cursor == '-');
        ++cursor;
    }

    bool hasAnyDigit = false;
    float value = 0.0f;

    while (isDigit(*cursor)) {
        hasAnyDigit = true;
        const float digit = static_cast<float>(*cursor - '0');

        // Check before multiplying so the calculation itself cannot overflow.
        if (value > (FLT_MAX - digit) / 10.0f) {
            return {NumberStatus::OUT_OF_RANGE, 0.0f, cursor};
        }

        value = value * 10.0f + digit;
        ++cursor;
    }

    if (*cursor == '.') {
        ++cursor;
        float decimalPlace = 0.1f;

        while (isDigit(*cursor)) {
            hasAnyDigit = true;
            const float digit = static_cast<float>(*cursor - '0');
            value += digit * decimalPlace;
            decimalPlace *= 0.1f;
            ++cursor;
        }
    }

    if (!hasAnyDigit) {
        if (*cursor == '\0' || *cursor == ';' || isWhitespace(*cursor) || isKnownLabel(*cursor)) {
            return {NumberStatus::MISSING, 0.0f, cursor};
        }

        return {NumberStatus::INVALID, 0.0f, cursor};
    }

    // A second decimal point, comma, or other punctuation makes the entire
    // number invalid. A letter may begin the next no-space token.
    if (*cursor != '\0' && *cursor != ';' && !isWhitespace(*cursor) && !isLetter(*cursor)) {
        return {NumberStatus::INVALID, 0.0f, cursor};
    }

    if (negative) {
        value = -value;
    }

    return {NumberStatus::VALID, value, cursor};
}

CommandNumberResult parseCommandNumber(const char* text) {
    const char* cursor = text;

    if (!isDigit(*cursor)) {
        return {false, 0U, cursor};
    }

    uint16_t value = 0U;

    while (isDigit(*cursor)) {
        const uint16_t digit = static_cast<uint16_t>(*cursor - '0');

        if (value > static_cast<uint16_t>((UINT16_MAX - digit) / 10U)) {
            return {false, 0U, cursor};
        }

        value = static_cast<uint16_t>(value * 10U + digit);
        ++cursor;
    }

    // G/M codes must be whole, unsigned numbers. A following letter may be
    // the next token in a no-space command such as G01X10.
    if (*cursor != '\0' && *cursor != ';' && !isWhitespace(*cursor) && !isLetter(*cursor)) {
        return {false, 0U, cursor};
    }

    return {true, value, cursor};
}

bool safetyLimitsAreValid(const GCodeSafetyLimits& limits) {
    return isFiniteFloat(limits.xMinimumMm) && isFiniteFloat(limits.xMaximumMm) && isFiniteFloat(limits.yMinimumMm) &&
           isFiniteFloat(limits.yMaximumMm) && isFiniteFloat(limits.maximumFeedrateMmPerMinute) &&
           limits.xMinimumMm <= limits.xMaximumMm && limits.yMinimumMm <= limits.yMaximumMm &&
           limits.maximumFeedrateMmPerMinute > 0.0f;
}

} // namespace

GCodeParser::GCodeParser(const GCodeSafetyLimits& safetyLimits, GCodePositioningMode positioningMode)
    : safetyLimits_(safetyLimits), positioningMode_(positioningMode), lastFeedrateMmPerMinute_(0.0f),
      hasLastFeedrate_(false), lastCommandWasLinear_(false), lineBuffer_{0}, lineLength_(0U), lineTooLong_(false),
      lineContainsNull_(false), ignoreNextLineFeed_(false) {
}

GCodeInputResult GCodeParser::processCharacter(char character, float currentXMm, float currentYMm) {
    if (character == '\n' && ignoreNextLineFeed_) {
        ignoreNextLineFeed_ = false;
        return waitingForLine();
    }

    const bool lineEnding = character == '\n' || character == '\r';

    if (!lineEnding) {
        ignoreNextLineFeed_ = false;

        if (character == '\0') {
            lineContainsNull_ = true;
            return waitingForLine();
        }

        if (lineLength_ < GCODE_MAX_LINE_LENGTH) {
            lineBuffer_[lineLength_] = character;
            ++lineLength_;
        } else {
            // Ignore the rest of this line, then report one error at newline.
            lineTooLong_ = true;
        }

        return waitingForLine();
    }

    if (character == '\r') {
        // Suppress the LF in a CRLF pair so it does not create an extra line.
        ignoreNextLineFeed_ = true;
    }

    GCodeParseResult result;

    if (lineTooLong_) {
        result = errorResult(GCodeParseError::LINE_TOO_LONG);
    } else if (lineContainsNull_) {
        result = errorResult(GCodeParseError::INVALID_CHARACTER);
    } else {
        lineBuffer_[lineLength_] = '\0';
        result = parseLine(lineBuffer_, currentXMm, currentYMm);
    }

    clearInputBuffer();
    return {true, result};
}

GCodeParseResult GCodeParser::parseLine(const char* line, float currentXMm, float currentYMm) const {
    if (line == nullptr) {
        return errorResult(GCodeParseError::NULL_INPUT);
    }

    uint16_t lineLength = 0U;

    while (line[lineLength] != '\0') {
        if (lineLength >= GCODE_MAX_LINE_LENGTH) {
            return errorResult(GCodeParseError::LINE_TOO_LONG);
        }

        ++lineLength;
    }

    const char* cursor = line;

    bool commandSeen = false;
    bool parameterSeen = false;
    bool xSeen = false;
    bool ySeen = false;
    bool feedrateSeen = false;

    GCodeCommandType commandType = GCodeCommandType::NONE;
    float xOffsetMm = 0.0f;
    float yOffsetMm = 0.0f;
    float feedrateMmPerMinute = 0.0f;

    while (true) {
        while (isWhitespace(*cursor)) {
            ++cursor;
        }

        if (*cursor == '\0' || *cursor == ';') {
            break;
        }

        if (isDigit(*cursor) || *cursor == '+' || *cursor == '-' || *cursor == '.') {
            return errorResult(GCodeParseError::MISSING_LABEL);
        }

        if (!isLetter(*cursor)) {
            return errorResult(GCodeParseError::INVALID_CHARACTER);
        }

        const char label = toUpperCase(*cursor);
        ++cursor;

        if (label == 'G' || label == 'M') {
            if (commandSeen) {
                return errorResult(GCodeParseError::MULTIPLE_COMMANDS);
            }

            const CommandNumberResult commandNumber = parseCommandNumber(cursor);

            if (!commandNumber.valid) {
                return errorResult(GCodeParseError::UNKNOWN_COMMAND);
            }

            cursor = commandNumber.nextCharacter;
            commandSeen = true;

            if (label == 'G' && commandNumber.value == 1U) {
                commandType = GCodeCommandType::LINEAR_MOVE;
            } else if (label == 'G' && commandNumber.value == 28U) {
                commandType = GCodeCommandType::HOME;
            } else if (label == 'M' && commandNumber.value == 999U) {
                commandType = GCodeCommandType::RESET_FAULT;
            } else {
                return errorResult(GCodeParseError::UNSUPPORTED_COMMAND);
            }

            continue;
        }

        if (label != 'X' && label != 'Y' && label != 'F') {
            return errorResult(GCodeParseError::UNKNOWN_PARAMETER);
        }

        const NumberResult number = parseDecimalNumber(cursor);

        if (number.status == NumberStatus::MISSING) {
            return errorResult(GCodeParseError::MISSING_VALUE);
        }

        if (number.status == NumberStatus::INVALID) {
            return errorResult(GCodeParseError::INVALID_NUMBER);
        }

        if (number.status == NumberStatus::OUT_OF_RANGE) {
            return errorResult(GCodeParseError::NUMBER_OUT_OF_RANGE);
        }

        cursor = number.nextCharacter;
        parameterSeen = true;

        if (label == 'X') {
            if (xSeen) {
                return errorResult(GCodeParseError::DUPLICATE_PARAMETER);
            }

            xSeen = true;
            xOffsetMm = number.value;
        } else if (label == 'Y') {
            if (ySeen) {
                return errorResult(GCodeParseError::DUPLICATE_PARAMETER);
            }

            ySeen = true;
            yOffsetMm = number.value;
        } else {
            if (feedrateSeen) {
                return errorResult(GCodeParseError::DUPLICATE_PARAMETER);
            }

            feedrateSeen = true;
            feedrateMmPerMinute = number.value;
        }
    }

    if (!commandSeen && !parameterSeen) {
        return acceptedResult(emptyCommand());
    }

    if (!commandSeen) {
        if (!lastCommandWasLinear_) {
            return errorResult(GCodeParseError::MISSING_COMMAND);
        }

        commandType = GCodeCommandType::LINEAR_MOVE;
    }

    if (commandType == GCodeCommandType::HOME || commandType == GCodeCommandType::RESET_FAULT) {
        if (parameterSeen) {
            return errorResult(GCodeParseError::UNEXPECTED_PARAMETER);
        }

        GCodeCommand command = emptyCommand();
        command.type = commandType;
        return acceptedResult(command);
    }

    if (!safetyLimitsAreValid(safetyLimits_)) {
        return errorResult(GCodeParseError::INVALID_SAFETY_LIMITS);
    }

    bool feedrateInherited = false;

    if (!feedrateSeen) {
        if (!hasLastFeedrate_) {
            return errorResult(GCodeParseError::FEEDRATE_NOT_SET);
        }

        feedrateMmPerMinute = lastFeedrateMmPerMinute_;
        feedrateInherited = true;
    }

    if (!isFiniteFloat(feedrateMmPerMinute) || feedrateMmPerMinute <= 0.0f) {
        return errorResult(GCodeParseError::FEEDRATE_NOT_POSITIVE);
    }

    bool feedrateWasLimited = false;

    if (feedrateMmPerMinute > safetyLimits_.maximumFeedrateMmPerMinute) {
        feedrateMmPerMinute = safetyLimits_.maximumFeedrateMmPerMinute;

        feedrateWasLimited = true;
    }

    const bool hasCoordinateParameter = xSeen || ySeen;

    if (hasCoordinateParameter) {
        if (!isFiniteFloat(currentXMm) || !isFiniteFloat(currentYMm)) {
            return errorResult(GCodeParseError::POSITION_OUT_OF_RANGE);
        }

        const float targetXMm = positioningMode_ == GCodePositioningMode::ABSOLUTE
                                    ? (xSeen ? xOffsetMm : currentXMm)
                                    : currentXMm + xOffsetMm;
        const float targetYMm = positioningMode_ == GCodePositioningMode::ABSOLUTE
                                    ? (ySeen ? yOffsetMm : currentYMm)
                                    : currentYMm + yOffsetMm;

        if (!isFiniteFloat(targetXMm) || !isFiniteFloat(targetYMm) || targetXMm < safetyLimits_.xMinimumMm ||
            targetXMm > safetyLimits_.xMaximumMm || targetYMm < safetyLimits_.yMinimumMm ||
            targetYMm > safetyLimits_.yMaximumMm) {
            return errorResult(GCodeParseError::POSITION_OUT_OF_RANGE);
        }

        xOffsetMm = targetXMm - currentXMm;
        yOffsetMm = targetYMm - currentYMm;
    }

    const bool hasMovement = xOffsetMm != 0.0f || yOffsetMm != 0.0f;

    const GCodeCommand command = {GCodeCommandType::LINEAR_MOVE,
                                  xOffsetMm,
                                  yOffsetMm,
                                  feedrateMmPerMinute,
                                  hasMovement,
                                  feedrateInherited,
                                  feedrateWasLimited};

    return acceptedResult(command);
}

void GCodeParser::commitCommand(const GCodeCommand& command) {
    switch (command.type) {
    case GCodeCommandType::NONE:
        break;

    case GCodeCommandType::LINEAR_MOVE:
        if (isFiniteFloat(command.feedrateMmPerMinute) && command.feedrateMmPerMinute > 0.0f) {
            lastFeedrateMmPerMinute_ = command.feedrateMmPerMinute;

            if (lastFeedrateMmPerMinute_ > safetyLimits_.maximumFeedrateMmPerMinute) {
                lastFeedrateMmPerMinute_ = safetyLimits_.maximumFeedrateMmPerMinute;
            }

            hasLastFeedrate_ = true;
            lastCommandWasLinear_ = true;
        }
        break;

    case GCodeCommandType::HOME:
        // A parameter-only line immediately after G28 is rejected rather
        // than guessed to be G01. The last valid F remains available for
        // a later explicit G01 command.
        lastCommandWasLinear_ = false;
        break;

    case GCodeCommandType::RESET_FAULT:
        // M999 is committed only after PlotterSystem accepts the reset.
        // Start again with no inherited motion state.
        resetState();
        break;
    }
}

void GCodeParser::resetState() {
    lastFeedrateMmPerMinute_ = 0.0f;
    hasLastFeedrate_ = false;
    lastCommandWasLinear_ = false;
    ignoreNextLineFeed_ = false;
    clearInputBuffer();
}

bool GCodeParser::hasLastFeedrate() const {
    return hasLastFeedrate_;
}

float GCodeParser::lastFeedrateMmPerMinute() const {
    return lastFeedrateMmPerMinute_;
}

const char* GCodeParser::errorMessage(GCodeParseError error) {
    switch (error) {
    case GCodeParseError::NONE:
        return "No error.";

    case GCodeParseError::NULL_INPUT:
        return "The G-code input pointer is null.";

    case GCodeParseError::LINE_TOO_LONG:
        return "The G-code line is too long.";

    case GCodeParseError::INVALID_CHARACTER:
        return "The G-code line contains an invalid character.";

    case GCodeParseError::MISSING_COMMAND:
        return "A G/M command is missing and G01 mode is not active.";

    case GCodeParseError::UNKNOWN_COMMAND:
        return "The G/M command is not a valid numbered command.";

    case GCodeParseError::UNSUPPORTED_COMMAND:
        return "Unsupported command; use G01, G28, or M999.";

    case GCodeParseError::MISSING_LABEL:
        return "A number is missing its G, M, X, Y, or F label.";

    case GCodeParseError::MISSING_VALUE:
        return "A parameter label is missing its numeric value.";

    case GCodeParseError::INVALID_NUMBER:
        return "A parameter contains an invalid decimal number.";

    case GCodeParseError::NUMBER_OUT_OF_RANGE:
        return "A numeric value is too large to represent safely.";

    case GCodeParseError::UNKNOWN_PARAMETER:
        return "Unknown parameter; G01 accepts only X, Y, and F.";

    case GCodeParseError::DUPLICATE_PARAMETER:
        return "X, Y, or F appears more than once.";

    case GCodeParseError::MULTIPLE_COMMANDS:
        return "Only one G/M command is allowed on each line.";

    case GCodeParseError::FEEDRATE_NOT_SET:
        return "No previous feedrate exists; provide a positive F value.";

    case GCodeParseError::FEEDRATE_NOT_POSITIVE:
        return "Feedrate must be greater than zero.";

    case GCodeParseError::POSITION_OUT_OF_RANGE:
        return "The G01 target would exceed the configured workspace.";

    case GCodeParseError::UNEXPECTED_PARAMETER:
        return "G28 and M999 do not accept X, Y, or F parameters.";

    case GCodeParseError::INVALID_SAFETY_LIMITS:
        return "The configured workspace or feedrate limits are invalid.";
    }

    return "Unknown parser error.";
}

void GCodeParser::clearInputBuffer() {
    lineLength_ = 0U;
    lineTooLong_ = false;
    lineContainsNull_ = false;
    lineBuffer_[0] = '\0';
}

} // namespace plotter

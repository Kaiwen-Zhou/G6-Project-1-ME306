#include "app/PlotterApplication.h"

#include <Arduino.h>

#include "config/PinConfig.h"
#include "config/SystemConfig.h"

namespace plotter {

namespace {
constexpr float INITIAL_A_KP = 0.8f;
constexpr float INITIAL_A_KI = 0.4f;
constexpr float INITIAL_A_KV = 0.01f;

constexpr float INITIAL_B_KP = 0.8f;
constexpr float INITIAL_B_KI = 0.4f;
constexpr float INITIAL_B_KV = 0.01f;

constexpr float CONTROLLER_MINIMUM_OUTPUT = -255.0f;
constexpr float CONTROLLER_MAXIMUM_OUTPUT = 255.0f;
constexpr float INTEGRAL_MINIMUM_OUTPUT = -20.0f;
constexpr float INTEGRAL_MAXIMUM_OUTPUT = 20.0f;
constexpr float POSITION_TOLERANCE_COUNTS = 20.0f;

constexpr uint8_t MOTOR_OUTPUT_LIMIT = SystemConfig::MOTOR_DEFAULT_OUTPUT_LIMIT;

constexpr float MAXIMUM_FEEDRATE_MM_PER_MINUTE = 1200.0f; // 20 mm/s
constexpr float MAXIMUM_ACCELERATION_MM_PER_SECOND_SQUARED = 35.0f;
constexpr unsigned long TELEMETRY_INTERVAL_MS = 20UL;
} // namespace

PlotterApplication* PlotterApplication::activeInstance_ = nullptr;

PlotterApplication::PlotterApplication()
    : encoderA_(true),
      encoderB_(false),
      leftLimit_(PinConfig::LIMIT_SWITCH_LEFT_PIN, SystemConfig::LIMIT_SWITCH_INPUT_MODE,
                 SystemConfig::LIMIT_SWITCH_DEBOUNCE_MS),
      rightLimit_(PinConfig::LIMIT_SWITCH_RIGHT_PIN, SystemConfig::LIMIT_SWITCH_INPUT_MODE,
                  SystemConfig::LIMIT_SWITCH_DEBOUNCE_MS),
      bottomLimit_(PinConfig::LIMIT_SWITCH_BOTTOM_PIN, SystemConfig::LIMIT_SWITCH_INPUT_MODE,
                   SystemConfig::LIMIT_SWITCH_DEBOUNCE_MS),
      topLimit_(PinConfig::LIMIT_SWITCH_TOP_PIN, SystemConfig::LIMIT_SWITCH_INPUT_MODE,
                SystemConfig::LIMIT_SWITCH_DEBOUNCE_MS),
      limitInterruptMask_(0U),
      motorA_(PinConfig::MOTOR_1_DIRECTION_PIN, PinConfig::MOTOR_1_PWM_PIN,
              SystemConfig::MOTOR_1_DIRECTION_INVERTED, MOTOR_OUTPUT_LIMIT),
      motorB_(PinConfig::MOTOR_2_DIRECTION_PIN, PinConfig::MOTOR_2_PWM_PIN,
              SystemConfig::MOTOR_2_DIRECTION_INVERTED, MOTOR_OUTPUT_LIMIT),
      pidA_(INITIAL_A_KP, INITIAL_A_KI, CONTROLLER_MINIMUM_OUTPUT, CONTROLLER_MAXIMUM_OUTPUT,
            INTEGRAL_MINIMUM_OUTPUT, INTEGRAL_MAXIMUM_OUTPUT, INITIAL_A_KV),
      pidB_(INITIAL_B_KP, INITIAL_B_KI, CONTROLLER_MINIMUM_OUTPUT, CONTROLLER_MAXIMUM_OUTPUT,
            INTEGRAL_MINIMUM_OUTPUT, INTEGRAL_MAXIMUM_OUTPUT, INITIAL_B_KV),
      axisA_(pidA_, motorA_, POSITION_TOLERANCE_COUNTS),
      axisB_(pidB_, motorB_, POSITION_TOLERANCE_COUNTS),
      converter_(SystemConfig::MOTOR_A_MM_PER_COUNT, SystemConfig::MOTOR_B_MM_PER_COUNT,
                 SystemConfig::MOTOR_A_COORDINATE_SIGN, SystemConfig::MOTOR_B_COORDINATE_SIGN),
      xyCoordinator_(encoderA_, encoderB_, axisA_, axisB_, converter_, SystemConfig::MOTION_CONTROL_PERIOD_MICROS),
      homingConfig_{SystemConfig::HOMING_COARSE_APPROACH_PWM,
                    SystemConfig::HOMING_BACKOFF_PWM,
                    SystemConfig::HOMING_FINE_APPROACH_PWM,
                    SystemConfig::HOMING_FINAL_RELEASE_PWM,
                    SystemConfig::HOMING_BACKOFF_DISTANCE_MM,
                    SystemConfig::HOMING_STRAIGHTNESS_CORRECTION_ENABLED,
                    SystemConfig::HOMING_STRAIGHTNESS_KP_PWM_PER_MM,
                    SystemConfig::HOMING_STRAIGHTNESS_MAXIMUM_CORRECTION_PWM,
                    SystemConfig::HOMING_STRAIGHTNESS_DEADBAND_MM,
                    SystemConfig::HOMING_IGNORE_X_MIN_DURING_Y,
                    SystemConfig::HOMING_CONTACT_PAUSE_MS,
                    SystemConfig::HOMING_FINE_CONTACT_PAUSE_MS,
                    SystemConfig::HOMING_SEARCH_TIMEOUT_MS,
                    SystemConfig::HOMING_BACKOFF_TIMEOUT_MS,
                    SystemConfig::HOMING_FINAL_RELEASE_TIMEOUT_MS,
                    SystemConfig::HOMING_OVERALL_TIMEOUT_MS},
      homingController_(encoderA_, encoderB_, motorA_, motorB_, converter_, leftLimit_, rightLimit_, bottomLimit_,
                        topLimit_, homingConfig_, SystemConfig::HOMING_LIMIT_DEBOUNCE_ENABLED),
      trajectoryPlanner_(),
      plotterSystem_(axisA_, axisB_, xyCoordinator_, trajectoryPlanner_, homingController_),
      gCodeController_(plotterSystem_, homingController_, converter_, encoderA_, encoderB_,
                       SystemConfig::MACHINE_X_TRAVEL_MM, SystemConfig::MACHINE_Y_TRAVEL_MM,
                       MAXIMUM_FEEDRATE_MM_PER_MINUTE, MAXIMUM_ACCELERATION_MM_PER_SECOND_SQUARED,
                       SystemConfig::GCODE_POSITIONING_MODE),
      limitSafety_(leftLimit_, rightLimit_, bottomLimit_, topLimit_, gCodeController_, plotterSystem_),
      serialLine_{},
      serialLineLength_(0U),
      serialLineTooLong_(false),
      ignoreNextLineFeed_(false),
      lastTelemetryMs_(0UL),
      lastState_(PlotterState::IDLE),
      lastHomingStage_(HomingStage::IDLE),
      lastHomingPhase_(HomingPhase::IDLE),
      homingCompletionReported_(false) {
}

void PlotterApplication::begin() {
    activeInstance_ = this;
    Serial.begin(SystemConfig::SERIAL_BAUD_RATE);

    motorA_.begin();
    motorB_.begin();
    Encoder::zeroCountPair(encoderA_, encoderB_);

    leftLimit_.begin(onLeftLimitInterrupt);
    rightLimit_.begin(onRightLimitInterrupt);
    bottomLimit_.begin(onBottomLimitInterrupt);
    topLimit_.begin(onTopLimitInterrupt);

    gCodeController_.begin();
    limitSafety_.begin();

    const GCodeControllerResult startupHoming = gCodeController_.processLine("G28", limitSafety_.faultResetAllowed());

    printStartupBanner();
    handleGCodeResult(startupHoming);
}

void PlotterApplication::update() {
    const uint8_t limitInterruptMask = consumeLimitInterruptMask();

    if (limitInterruptMask != 0U && gCodeController_.systemStarted()) {
        if (plotterSystem_.state() == PlotterState::HOMING) {
            homingController_.notifyLimitInterrupt(limitInterruptMask);
        } else {
            reportLimitSafetyUpdate(limitSafety_.handleLimitInterrupts(limitInterruptMask));
        }
    }

    // During HOMING, HomingController owns switch updates and event handling.
    if (!gCodeController_.systemStarted() || plotterSystem_.state() != PlotterState::HOMING) {
        limitSafety_.updateSwitches();
    }

    pollSerial();

    if (gCodeController_.systemStarted()) {
        reportLimitSafetyUpdate(limitSafety_.update());
        plotterSystem_.update();

        const bool limitsLoadedNow = gCodeController_.updateAfterSystem();
        reportHomingCompletion(limitsLoadedNow);
    }

    printTelemetry();
    reportStateChanges();
}

void PlotterApplication::onEncoderAInterrupt() {
    encoderA_.update();
}

void PlotterApplication::onEncoderBInterrupt() {
    encoderB_.update();
}

void PlotterApplication::onLeftLimitInterrupt() {
    if (activeInstance_ != nullptr) {
        activeInstance_->leftLimit_.notifyFromISR();
        activeInstance_->limitInterruptMask_ |= LimitSafetyManager::LEFT_LIMIT_MASK;
    }
}

void PlotterApplication::onRightLimitInterrupt() {
    if (activeInstance_ != nullptr) {
        activeInstance_->rightLimit_.notifyFromISR();
        activeInstance_->limitInterruptMask_ |= LimitSafetyManager::RIGHT_LIMIT_MASK;
    }
}

void PlotterApplication::onBottomLimitInterrupt() {
    if (activeInstance_ != nullptr) {
        activeInstance_->bottomLimit_.notifyFromISR();
        activeInstance_->limitInterruptMask_ |= LimitSafetyManager::BOTTOM_LIMIT_MASK;
    }
}

void PlotterApplication::onTopLimitInterrupt() {
    if (activeInstance_ != nullptr) {
        activeInstance_->topLimit_.notifyFromISR();
        activeInstance_->limitInterruptMask_ |= LimitSafetyManager::TOP_LIMIT_MASK;
    }
}

uint8_t PlotterApplication::consumeLimitInterruptMask() {
    noInterrupts();
    const uint8_t mask = limitInterruptMask_;
    limitInterruptMask_ = 0U;
    interrupts();
    return mask;
}

char PlotterApplication::upperAscii(char character) {
    if (character >= 'a' && character <= 'z') {
        return static_cast<char>(character - 'a' + 'A');
    }

    return character;
}

bool PlotterApplication::isSpace(char character) {
    return character == ' ' || character == '\t';
}

bool PlotterApplication::lineEqualsIgnoringSpaces(const char* line, const char* expected) {
    uint16_t lineIndex = 0U;
    uint16_t expectedIndex = 0U;

    while (true) {
        while (isSpace(line[lineIndex])) {
            ++lineIndex;
        }

        const char lineCharacter = upperAscii(line[lineIndex]);
        const char expectedCharacter = expected[expectedIndex];

        if (lineCharacter != expectedCharacter) {
            return false;
        }

        if (lineCharacter == '\0') {
            return true;
        }

        ++lineIndex;
        ++expectedIndex;
    }
}

void PlotterApplication::printLimitMaskNames(uint8_t mask) const {
    bool printedAny = false;

    if ((mask & LimitSafetyManager::LEFT_LIMIT_MASK) != 0U) {
        Serial.print(F("LEFT"));
        printedAny = true;
    }

    if ((mask & LimitSafetyManager::RIGHT_LIMIT_MASK) != 0U) {
        if (printedAny) {
            Serial.print(',');
        }
        Serial.print(F("RIGHT"));
        printedAny = true;
    }

    if ((mask & LimitSafetyManager::BOTTOM_LIMIT_MASK) != 0U) {
        if (printedAny) {
            Serial.print(',');
        }
        Serial.print(F("BOTTOM"));
        printedAny = true;
    }

    if ((mask & LimitSafetyManager::TOP_LIMIT_MASK) != 0U) {
        if (printedAny) {
            Serial.print(',');
        }
        Serial.print(F("TOP"));
        printedAny = true;
    }

    if (!printedAny) {
        Serial.print(F("NONE"));
    }
}

void PlotterApplication::printState(PlotterState state) const {
    switch (state) {
    case PlotterState::IDLE:
        Serial.print(F("IDLE"));
        break;
    case PlotterState::HOMING:
        Serial.print(F("HOMING"));
        break;
    case PlotterState::MOVING:
        Serial.print(F("MOVING"));
        break;
    case PlotterState::FAULT:
        Serial.print(F("FAULT"));
        break;
    }
}

void PlotterApplication::printFault(FaultCode fault) const {
    switch (fault) {
    case FaultCode::NONE:
        Serial.print(F("NONE"));
        break;
    case FaultCode::UNEXPECTED_LIMIT:
        Serial.print(F("UNEXPECTED_LIMIT"));
        break;
    case FaultCode::WRONG_HOMING_LIMIT:
        Serial.print(F("WRONG_HOMING_LIMIT"));
        break;
    case FaultCode::CONTRADICTORY_LIMITS:
        Serial.print(F("CONTRADICTORY_LIMITS"));
        break;
    case FaultCode::HOMING_TIMEOUT:
        Serial.print(F("HOMING_TIMEOUT"));
        break;
    case FaultCode::MOVE_TIMEOUT:
        Serial.print(F("MOVE_TIMEOUT"));
        break;
    case FaultCode::ENCODER_NO_MOTION:
        Serial.print(F("ENCODER_NO_MOTION"));
        break;
    case FaultCode::POSITION_OUT_OF_RANGE:
        Serial.print(F("POSITION_OUT_OF_RANGE"));
        break;
    case FaultCode::INTERNAL_ERROR:
        Serial.print(F("INTERNAL_ERROR"));
        break;
    }
}

void PlotterApplication::printRejectReason(RejectReason reason) const {
    switch (reason) {
    case RejectReason::NONE:
        Serial.print(F("NONE"));
        break;
    case RejectReason::BUSY:
        Serial.print(F("BUSY"));
        break;
    case RejectReason::MACHINE_ZERO_UNKNOWN:
        Serial.print(F("MACHINE_ZERO_UNKNOWN"));
        break;
    case RejectReason::FAULT_ACTIVE:
        Serial.print(F("FAULT_ACTIVE"));
        break;
    case RejectReason::UNEXPECTED_EVENT:
        Serial.print(F("UNEXPECTED_EVENT"));
        break;
    case RejectReason::INVALID_MOTION_PARAMETERS:
        Serial.print(F("INVALID_MOTION_PARAMETERS"));
        break;
    }
}

void PlotterApplication::printStatus() const {
    Serial.print(F("# STATUS state="));

    if (!gCodeController_.systemStarted()) {
        Serial.print(F("NOT_STARTED"));
    } else {
        printState(plotterSystem_.state());
    }

    const Encoder::CountPair counts = Encoder::getCountPair(encoderA_, encoderB_);
    const Converter::CartesianDisplacement position =
        converter_.motorToCartesianDisplacement(static_cast<float>(counts.countA), static_cast<float>(counts.countB));

    Serial.print(F(" limits="));
    Serial.print(leftLimit_.isPressed() ? 1 : 0);
    Serial.print(',');
    Serial.print(rightLimit_.isPressed() ? 1 : 0);
    Serial.print(',');
    Serial.print(bottomLimit_.isPressed() ? 1 : 0);
    Serial.print(',');
    Serial.print(topLimit_.isPressed() ? 1 : 0);
    Serial.print(F(" expected_mask=0x"));
    Serial.print(limitSafety_.expectedMask(), HEX);
    Serial.print(F(" recoverable_mask=0x"));
    Serial.print(limitSafety_.recoverableMask(), HEX);
    Serial.print(F(" counts="));
    Serial.print(counts.countA);
    Serial.print(',');
    Serial.print(counts.countB);
    Serial.print(F(" xy_mm="));
    Serial.print(position.xMm, 3);
    Serial.print(',');
    Serial.println(position.yMm, 3);
}

void PlotterApplication::printStartupBanner() const {
    Serial.println();
    Serial.println(F("# ME306 X-Y plotter ready"));
    Serial.println(F("# Active-high limits: HIGH=pressed"));
    Serial.print(F("# Motor output limit="));
    Serial.print(MOTOR_OUTPUT_LIMIT);
    Serial.println(F("/255"));
    Serial.print(F("# Fixed travel mm: X="));
    Serial.print(SystemConfig::MACHINE_X_TRAVEL_MM, 3);
    Serial.print(F(" Y="));
    Serial.println(SystemConfig::MACHINE_Y_TRAVEL_MM, 3);
    Serial.print(F("# Limit boundary tolerance mm="));
    Serial.println(SystemConfig::LIMIT_BOUNDARY_TOLERANCE_MM, 3);
    Serial.print(F("# Limit safety polling ms="));
    Serial.println(SystemConfig::LIMIT_SAFETY_CHECK_INTERVAL_MS);
    Serial.print(F("# G01 positioning mode="));
    Serial.println(SystemConfig::GCODE_POSITIONING_MODE == GCodePositioningMode::ABSOLUTE ? F("ABSOLUTE")
                                                                                         : F("RELATIVE"));

    if (!(SystemConfig::MACHINE_X_TRAVEL_MM > 0.0f) || !(SystemConfig::MACHINE_Y_TRAVEL_MM > 0.0f)) {
        Serial.println(F("# WARNING: set positive MACHINE_X/Y_TRAVEL_MM before G01"));
    }

    Serial.println(F("# Startup homing is automatic. Emergency stop: !"));
    printStatus();
    Serial.println(F("time_ms,reference_x_mm,actual_x_mm,error_x_mm,reference_y_mm,actual_y_mm,"
                     "error_y_mm,error_a_counts,pwm_a,integral_a,error_b_counts,pwm_b,integral_b"));
}

void PlotterApplication::emergencyStop() {
    const bool stopped = gCodeController_.emergencyStop();

    if (stopped) {
        limitSafety_.clearRecoverableFaultAttribution();
    }

    serialLineLength_ = 0U;
    serialLineTooLong_ = false;

    Serial.println(stopped ? F("# EMERGENCY STOP: FAULT latched") : F("# EMERGENCY STOP: system was already stopped"));
}

void PlotterApplication::handleGCodeResult(const GCodeControllerResult& result) {
    if (!result.lineComplete) {
        return;
    }

    if (!result.accepted) {
        Serial.print(F("# REJECTED: "));

        if (result.parseError != GCodeParseError::NONE) {
            Serial.println(GCodeParser::errorMessage(result.parseError));
        } else {
            Serial.print(GCodeController::controllerErrorMessage(result.controllerError));

            if (result.controllerError == GCodeControllerError::LIMIT_SWITCH_ACTIVE) {
                const Converter::CartesianDisplacement position = gCodeController_.currentCartesianPosition();

                Serial.print(F(" blocking="));
                printLimitMaskNames(limitSafety_.resetBlockingMask());
                Serial.print(F(" xy_mm="));
                Serial.print(position.xMm, 3);
                Serial.print(',');
                Serial.print(position.yMm, 3);
            }

            if (result.rejectReason != RejectReason::NONE) {
                Serial.print(F(" FSM="));
                printRejectReason(result.rejectReason);
            }

            Serial.println();
        }

        return;
    }

    if (result.command.type == GCodeCommandType::NONE) {
        return;
    }

    if (result.command.type == GCodeCommandType::HOME) {
        limitSafety_.clearForHoming();
        homingCompletionReported_ = false;
        Serial.println(F("# ACCEPTED G28: homing started"));
        return;
    }

    if (result.command.type == GCodeCommandType::RESET_FAULT) {
        limitSafety_.armRecoverableLimitsAfterReset();
        Serial.print(F("# ACCEPTED M999: fault cleared, expected_mask=0x"));
        Serial.println(limitSafety_.expectedMask(), HEX);
        return;
    }

    if (!result.movementStarted) {
        Serial.print(F("# ACCEPTED G01: feedrate stored F="));
        Serial.println(result.command.feedrateMmPerMinute, 2);
        return;
    }

    Serial.print(F("# ACCEPTED G01: dx="));
    Serial.print(result.command.xOffsetMm, 3);
    Serial.print(F(" dy="));
    Serial.print(result.command.yOffsetMm, 3);
    Serial.print(F(" F="));
    Serial.print(result.command.feedrateMmPerMinute, 2);

    if (result.command.feedrateWasLimited) {
        Serial.print(F(" (limited)"));
    }

    Serial.println();
    lastTelemetryMs_ = millis();
}

void PlotterApplication::processCompleteSerialLine() {
    serialLine_[serialLineLength_] = '\0';

    if (lineEqualsIgnoringSpaces(serialLine_, "X") || lineEqualsIgnoringSpaces(serialLine_, "STOP") ||
        lineEqualsIgnoringSpaces(serialLine_, "M112")) {
        emergencyStop();
    } else if (lineEqualsIgnoringSpaces(serialLine_, "STATUS")) {
        printStatus();
    } else if (serialLineTooLong_) {
        Serial.println(F("# REJECTED: serial line is too long"));
    } else {
        handleGCodeResult(gCodeController_.processLine(serialLine_, limitSafety_.faultResetAllowed()));
    }

    serialLineLength_ = 0U;
    serialLineTooLong_ = false;
    serialLine_[0] = '\0';
}

void PlotterApplication::pollSerial() {
    while (Serial.available() > 0) {
        const char character = static_cast<char>(Serial.read());

        if (character == '!') {
            emergencyStop();
            continue;
        }

        if (character == '\n' && ignoreNextLineFeed_) {
            ignoreNextLineFeed_ = false;
            continue;
        }

        if (character == '\r' || character == '\n') {
            ignoreNextLineFeed_ = (character == '\r');
            processCompleteSerialLine();
            continue;
        }

        ignoreNextLineFeed_ = false;

        if (serialLineLength_ < sizeof(serialLine_) - 1U) {
            serialLine_[serialLineLength_] = character;
            ++serialLineLength_;
        } else {
            serialLineTooLong_ = true;
        }
    }
}

void PlotterApplication::printTelemetry() {
    if (!gCodeController_.systemStarted() || plotterSystem_.state() != PlotterState::MOVING) {
        return;
    }

    const unsigned long currentTimeMs = millis();

    if ((currentTimeMs - lastTelemetryMs_) < TELEMETRY_INTERVAL_MS) {
        return;
    }

    lastTelemetryMs_ = currentTimeMs;
    const XYCoordinatorTelemetry data = xyCoordinator_.getTelemetry();
    const float errorXMm = data.referenceXDisplacementMm - data.actualXDisplacementMm;
    const float errorYMm = data.referenceYDisplacementMm - data.actualYDisplacementMm;

    Serial.print(currentTimeMs);
    Serial.print(',');
    Serial.print(data.referenceXDisplacementMm, 3);
    Serial.print(',');
    Serial.print(data.actualXDisplacementMm, 3);
    Serial.print(',');
    Serial.print(errorXMm, 3);
    Serial.print(',');
    Serial.print(data.referenceYDisplacementMm, 3);
    Serial.print(',');
    Serial.print(data.actualYDisplacementMm, 3);
    Serial.print(',');
    Serial.print(errorYMm, 3);
    Serial.print(',');
    Serial.print(data.motorA.trackingError, 2);
    Serial.print(',');
    Serial.print(data.motorA.motorOutput);
    Serial.print(',');
    Serial.print(data.motorA.integralOutput, 3);
    Serial.print(',');
    Serial.print(data.motorB.trackingError, 2);
    Serial.print(',');
    Serial.print(data.motorB.motorOutput);
    Serial.print(',');
    Serial.println(data.motorB.integralOutput, 3);
}

void PlotterApplication::reportStateChanges() {
    if (!gCodeController_.systemStarted()) {
        return;
    }

    const PlotterState state = plotterSystem_.state();

    if (state != lastState_) {
        lastState_ = state;
        Serial.print(F("# FSM -> "));
        printState(state);

        if (state == PlotterState::FAULT) {
            Serial.print(F(" fault="));
            printFault(plotterSystem_.activeFault());
        }

        Serial.println();
    }

    if (state == PlotterState::HOMING) {
        const HomingStage stage = homingController_.stage();
        const HomingPhase phase = homingController_.phase();

        if (stage != lastHomingStage_ || phase != lastHomingPhase_) {
            lastHomingStage_ = stage;
            lastHomingPhase_ = phase;
            Serial.print(F("# HOMING stage="));
            Serial.print(static_cast<uint8_t>(stage));
            Serial.print(F(" phase="));
            Serial.println(static_cast<uint8_t>(phase));
        }
    }
}

void PlotterApplication::reportLimitSafetyUpdate(const LimitSafetyUpdate& result) const {
    if (result.releasedExpectedMask != 0U) {
        Serial.print(F("# LIMIT RECOVERY: released expected mask=0x"));
        Serial.println(result.releasedExpectedMask, HEX);
    }

    if (!result.faultEntered) {
        return;
    }

    Serial.print(F("# LIMIT FAULT pressed="));
    printLimitMaskNames(result.pressedMask);
    Serial.print(F(" recoverable="));
    printLimitMaskNames(result.recoverableMask);
    Serial.print(F(" xy_mm="));
    Serial.print(result.positionXMm, 3);
    Serial.print(',');
    Serial.println(result.positionYMm, 3);
}

void PlotterApplication::reportHomingCompletion(bool limitsLoadedNow) {
    if (!homingController_.isComplete() || homingCompletionReported_) {
        return;
    }

    homingCompletionReported_ = true;
    const HomingResult result = homingController_.result();
    const uint8_t originLimitMask =
        static_cast<uint8_t>(LimitSafetyManager::LEFT_LIMIT_MASK | LimitSafetyManager::BOTTOM_LIMIT_MASK);
    const uint8_t expectedLimitMask = limitSafety_.armPressedLimitsAfterHoming(originLimitMask);

    Serial.print(F("# HOMING COMPLETE xOriginCounts="));
    Serial.print(result.xOrigin.countA);
    Serial.print(',');
    Serial.print(result.xOrigin.countB);
    Serial.print(F(" yOriginCounts="));
    Serial.print(result.yOrigin.countA);
    Serial.print(',');
    Serial.print(result.yOrigin.countB);
    Serial.print(F(" expected_mask=0x"));
    Serial.print(expectedLimitMask, HEX);

    if (limitsLoadedNow || gCodeController_.safetyLimitsLoaded()) {
        Serial.print(F(" soft_limits_mm=0.."));
        Serial.print(SystemConfig::MACHINE_X_TRAVEL_MM, 3);
        Serial.print(F(",0.."));
        Serial.println(SystemConfig::MACHINE_Y_TRAVEL_MM, 3);
    } else {
        Serial.println(F(" WARNING: set positive MACHINE_X/Y_TRAVEL_MM before G01"));
    }
}

} // namespace plotter

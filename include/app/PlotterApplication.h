#pragma once

#include <stdint.h>

#include "communication/GCodeController.h"
#include "control/AxisController.h"
#include "control/Converter.h"
#include "control/HomingController.h"
#include "control/PIDController.h"
#include "control/XYCoordinator.h"
#include "hardware/Encoder.h"
#include "hardware/LimitSwitch.h"
#include "hardware/MotorDriver.h"
#include "system/LimitSafetyManager.h"
#include "system/PlotterSystem.h"
#include "system/TrajectoryPlanner.h"

namespace plotter {

/**
 * Top-level firmware composition root.
 *
 * Arduino main.cpp owns only the framework entry points and interrupt vectors;
 * this class owns hardware objects, controllers, console processing and the
 * high-level update order.
 */
class PlotterApplication {
    public:
        PlotterApplication();

        void begin();
        void update();

        void onEncoderAInterrupt();
        void onEncoderBInterrupt();

    private:
        static void onLeftLimitInterrupt();
        static void onRightLimitInterrupt();
        static void onBottomLimitInterrupt();
        static void onTopLimitInterrupt();

        static char upperAscii(char character);
        static bool isSpace(char character);
        static bool lineEqualsIgnoringSpaces(const char* line, const char* expected);

        void printLimitMaskNames(uint8_t mask) const;
        void printState(PlotterState state) const;
        void printFault(FaultCode fault) const;
        void printRejectReason(RejectReason reason) const;
        void printStatus() const;
        void printStartupBanner() const;

        void emergencyStop();
        void handleGCodeResult(const GCodeControllerResult& result);
        void processCompleteSerialLine();
        void pollSerial();
        void printTelemetry();
        void reportStateChanges();
        void reportLimitSafetyUpdate(const LimitSafetyUpdate& result) const;
        void reportHomingCompletion(bool limitsLoadedNow);

        static PlotterApplication* activeInstance_;

        Encoder encoderA_;
        Encoder encoderB_;

        LimitSwitch leftLimit_;
        LimitSwitch rightLimit_;
        LimitSwitch bottomLimit_;
        LimitSwitch topLimit_;

        MotorDriver motorA_;
        MotorDriver motorB_;
        PIDController pidA_;
        PIDController pidB_;
        AxisController axisA_;
        AxisController axisB_;
        Converter converter_;
        XYCoordinator xyCoordinator_;
        HomingConfig homingConfig_;
        HomingController homingController_;
        TrajectoryPlanner trajectoryPlanner_;
        PlotterSystem plotterSystem_;
        GCodeController gCodeController_;
        LimitSafetyManager limitSafety_;

        char serialLine_[GCODE_MAX_LINE_LENGTH + 1U];
        uint16_t serialLineLength_;
        bool serialLineTooLong_;
        bool ignoreNextLineFeed_;

        unsigned long lastTelemetryMs_;
        PlotterState lastState_;
        HomingStage lastHomingStage_;
        HomingPhase lastHomingPhase_;
        bool homingCompletionReported_;
};

} // namespace plotter

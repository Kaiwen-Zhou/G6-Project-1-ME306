#include "control/XYCoordinator.h"

#include <Arduino.h>

namespace {
constexpr float MICROSECONDS_TO_SECONDS = 0.000001f;
}

XYCoordinator::XYCoordinator(Encoder& encoderA, Encoder& encoderB, 
                             AxisController& axisA, AxisController& axisB,
                             const Converter& converter, 
                             unsigned long controlIntervalMicros)
    : encoderA_(encoderA), encoderB_(encoderB), 
      axisA_(axisA), axisB_(axisB), 
      converter_(converter),
      controlIntervalMicros_(controlIntervalMicros), 
      lastUpdateMicros_(0), 
      moveStartCountA_(0), moveStartCountB_(0),
      latestCounts_{0, 0}, 
      referenceXDisplacementMm_(0.0f), referenceYDisplacementMm_(0.0f),
      referenceXVelocityMmPerSecond_(0.0f), referenceYVelocityMmPerSecond_(0.0f), 
      active_(false) {
}

void XYCoordinator::begin() {
    axisA_.begin();
    axisB_.begin();

    lastUpdateMicros_ = 0;

    moveStartCountA_ = 0;
    moveStartCountB_ = 0;

    latestCounts_ = Encoder::CountPair{0, 0};

    referenceXDisplacementMm_ = 0.0f;
    referenceYDisplacementMm_ = 0.0f;
    referenceXVelocityMmPerSecond_ = 0.0f;
    referenceYVelocityMmPerSecond_ = 0.0f;

    active_ = false;
}

void XYCoordinator::startMove() {
    // One atomic/synchronised snapshot establishes the move origin.
    latestCounts_ = Encoder::getCountPair(encoderA_, encoderB_);

    moveStartCountA_ = latestCounts_.countA;
    moveStartCountB_ = latestCounts_.countB;

    axisA_.startTracking(latestCounts_.countA);
    axisB_.startTracking(latestCounts_.countB);

    referenceXDisplacementMm_ = 0.0f;
    referenceYDisplacementMm_ = 0.0f;
    referenceXVelocityMmPerSecond_ = 0.0f;
    referenceYVelocityMmPerSecond_ = 0.0f;

    lastUpdateMicros_ = micros();

    active_ = true;
}

void XYCoordinator::setCartesianReference(float xDisplacementMm, float yDisplacementMm,
                                          float xVelocityMmPerSecond, float yVelocityMmPerSecond) {
    setCartesianReference(xDisplacementMm, yDisplacementMm, xVelocityMmPerSecond,
                          yVelocityMmPerSecond, 0.0f);
}

void XYCoordinator::setCartesianReference(float xDisplacementMm, float yDisplacementMm, float xVelocityMmPerSecond,
                                          float yVelocityMmPerSecond, float remainingDistanceFraction) {
    if (!active_) {
        return;
    }

    const Converter::MotorReference motorReference = converter_.cartesianToMotorReference(
        xDisplacementMm, yDisplacementMm, xVelocityMmPerSecond, yVelocityMmPerSecond);

    // Converter returns displacement relative to the move origin.
    // AxisController requires an absolute encoder-count reference.
    const float absoluteAReference = static_cast<float>(moveStartCountA_) + motorReference.aDisplacementCounts;

    const float absoluteBReference = static_cast<float>(moveStartCountB_) + motorReference.bDisplacementCounts;

    axisA_.setReference(absoluteAReference, motorReference.aVelocityCountsPerSecond,
                        remainingDistanceFraction);

    axisB_.setReference(absoluteBReference, motorReference.bVelocityCountsPerSecond,
                        remainingDistanceFraction);

    referenceXDisplacementMm_ = xDisplacementMm;
    referenceYDisplacementMm_ = yDisplacementMm;

    referenceXVelocityMmPerSecond_ = xVelocityMmPerSecond;

    referenceYVelocityMmPerSecond_ = yVelocityMmPerSecond;
}

void XYCoordinator::update() {
    if (!active_) {
        return;
    }

    const unsigned long currentTimeMicros = micros();

    // Unsigned subtraction remains valid across micros() overflow.
    const unsigned long elapsedMicros = currentTimeMicros - lastUpdateMicros_;

    if (elapsedMicros == 0 || elapsedMicros < controlIntervalMicros_) {
        return;
    }

    const float timeStepSeconds = static_cast<float>(elapsedMicros) * MICROSECONDS_TO_SECONDS;

    // Read both encoder counts exactly once for this control cycle.
    latestCounts_ = Encoder::getCountPair(encoderA_, encoderB_);

    lastUpdateMicros_ = currentTimeMicros;

    // Both controllers receive positions from the same snapshot
    // and exactly the same measured dt.
    axisA_.update(latestCounts_.countA, timeStepSeconds);

    axisB_.update(latestCounts_.countB, timeStepSeconds);
}

void XYCoordinator::stop() {
    axisA_.stop();
    axisB_.stop();

    referenceXVelocityMmPerSecond_ = 0.0f;
    referenceYVelocityMmPerSecond_ = 0.0f;

    lastUpdateMicros_ = 0;
    active_ = false;
}

void XYCoordinator::reset() {
    latestCounts_ = Encoder::getCountPair(encoderA_, encoderB_);

    axisA_.reset(latestCounts_.countA);
    axisB_.reset(latestCounts_.countB);

    moveStartCountA_ = latestCounts_.countA;
    moveStartCountB_ = latestCounts_.countB;

    referenceXDisplacementMm_ = 0.0f;
    referenceYDisplacementMm_ = 0.0f;
    referenceXVelocityMmPerSecond_ = 0.0f;
    referenceYVelocityMmPerSecond_ = 0.0f;

    lastUpdateMicros_ = 0;
    active_ = false;
}

bool XYCoordinator::isActive() const {
    return active_;
}

bool XYCoordinator::areAxesWithinTolerance() const {
    return active_ && axisA_.isWithinTolerance() && axisB_.isWithinTolerance();
}

XYCoordinatorTelemetry XYCoordinator::getTelemetry() const {
    const float aDisplacementCounts = static_cast<float>(latestCounts_.countA) - static_cast<float>(moveStartCountA_);

    const float bDisplacementCounts = static_cast<float>(latestCounts_.countB) - static_cast<float>(moveStartCountB_);

    const Converter::CartesianDisplacement actualDisplacement =
        converter_.motorToCartesianDisplacement(aDisplacementCounts, bDisplacementCounts);

    return XYCoordinatorTelemetry{
        referenceXDisplacementMm_,      referenceYDisplacementMm_, referenceXVelocityMmPerSecond_,
        referenceYVelocityMmPerSecond_, actualDisplacement.xMm,    actualDisplacement.yMm,
        axisA_.getTelemetry(),          axisB_.getTelemetry(),     active_};
}

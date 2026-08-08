#include "control/Converter.h"
#include "config/SystemConfig.h"

Converter::Converter(Encoder& encoderA, Encoder& encoderB)
    : encoderA_(encoderA),
      encoderB_(encoderB),
      previousCountA_(0),
      previousCountB_(0),
      deltaA_(0),
      deltaB_(0),
      deltaX_(0.0f),
      deltaY_(0.0f)
{}

void Converter::begin() {
    // Establish the initial count reference without producing
    // an artificial movement on the first update.
    Encoder::CountPair counts = Encoder::getCountPair(encoderA_, encoderB_);

    previousCountA_ = counts.countA;
    previousCountB_ = counts.countB;

    deltaA_ = 0;
    deltaB_ = 0;
    deltaX_ = 0.0f;
    deltaY_ = 0.0f;
}

void Converter::update() {
    // A and B are captured during the same atomic block.
    Encoder::CountPair counts = Encoder::getCountPair(encoderA_, encoderB_);

    // Movement since the previous update
    deltaA_ = counts.countA - previousCountA_;
    deltaB_ = counts.countB - previousCountB_;

    previousCountA_ = counts.countA;
    previousCountB_ = counts.countB;

    CartesianDelta cartesianDelta = convertToXY(deltaA_, deltaB_);

    deltaX_ = cartesianDelta.deltaX;
    deltaY_ = cartesianDelta.deltaY;
}

Converter::CartesianDelta Converter::convertToXY(int32_t deltaA,int32_t deltaB) const {
    const float displacementA =
        static_cast<float>(deltaA) * SystemConfig::MOTOR_OUTPUT_DISTANCE_PER_COUNT;

    const float displacementB =
        static_cast<float>(deltaB) * SystemConfig::MOTOR_OUTPUT_DISTANCE_PER_COUNT;

    CartesianDelta result;

    result.deltaX = (displacementA + displacementB) * 0.5f;
    result.deltaY = (displacementA - displacementB) * 0.5f;

    return result;
}

float Converter::getDeltaX() const {
    return deltaX_;
}

float Converter::getDeltaY() const {
    return deltaY_;
}

int32_t Converter::getDeltaA() const {
    return deltaA_;
}

int32_t Converter::getDeltaB() const {
    return deltaB_;
}
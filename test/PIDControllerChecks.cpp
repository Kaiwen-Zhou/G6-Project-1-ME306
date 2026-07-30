#include <cassert>
#include <cmath>
#include <iostream>

#include "control/PIDController.h"

namespace
{
constexpr float FLOAT_TOLERANCE = 0.0001f;

void expectNear(float actual, float expected)
{
    assert(std::fabs(actual - expected) < FLOAT_TOLERANCE);
}

void checkProportionalOutput()
{
    PIDController controller(2.0f, 0.0f,
                             -100.0f, 100.0f,
                             -100.0f, 100.0f);

    expectNear(controller.update(3.0f, 0.1f), 6.0f);
}

void checkIntegralAccumulation()
{
    PIDController controller(0.0f, 2.0f,
                             -100.0f, 100.0f,
                             -100.0f, 100.0f);

    expectNear(controller.update(1.0f, 0.5f), 1.0f);
    expectNear(controller.update(1.0f, 0.5f), 2.0f);
    expectNear(controller.getIntegralOutput(), 2.0f);
}

void checkOutputAndIntegralLimits()
{
    PIDController outputLimitedController(10.0f, 0.0f,
                                          -50.0f, 50.0f,
                                          -50.0f, 50.0f);

    expectNear(outputLimitedController.update(10.0f, 0.1f), 50.0f);
    expectNear(outputLimitedController.update(-10.0f, 0.1f), -50.0f);

    PIDController integralLimitedController(0.0f, 10.0f,
                                            -100.0f, 100.0f,
                                            -3.0f, 3.0f);

    expectNear(integralLimitedController.update(1.0f, 1.0f), 3.0f);
    expectNear(integralLimitedController.getIntegralOutput(), 3.0f);
}

void checkAntiWindup()
{
    PIDController sameDirectionController(1.0f, 1.0f,
                                          -5.0f, 5.0f,
                                          -100.0f, 100.0f);

    // Positive integration would push an already high output farther above
    // the maximum, so it must be blocked.
    expectNear(sameDirectionController.update(10.0f, 1.0f), 5.0f);
    expectNear(sameDirectionController.getIntegralOutput(), 0.0f);

    PIDController unwindController(0.0f, 1.0f,
                                   -20.0f, 20.0f,
                                   -100.0f, 100.0f);

    // First build a positive integral while the wider output range allows it.
    expectNear(unwindController.update(10.0f, 1.0f), 10.0f);

    // Narrowing the output range represents an already-saturated controller.
    unwindController.setOutputLimits(-5.0f, 5.0f);

    // The reversed error reduces the integral from 10 to 9 even though the
    // returned output is still saturated at +5.
    expectNear(unwindController.update(-1.0f, 1.0f), 5.0f);
    expectNear(unwindController.getIntegralOutput(), 9.0f);

    float output = 0.0f;
    for (int updateNumber = 0; updateNumber < 5; ++updateNumber)
    {
        output = unwindController.update(-1.0f, 1.0f);
    }

    expectNear(output, 4.0f);
}

void checkReset()
{
    PIDController controller(0.0f, 1.0f,
                             -100.0f, 100.0f,
                             -100.0f, 100.0f);

    controller.update(4.0f, 1.0f);
    controller.reset();

    expectNear(controller.getIntegralOutput(), 0.0f);
    expectNear(controller.update(0.0f, 0.1f), 0.0f);
}
}  // namespace

int main()
{
    checkProportionalOutput();
    checkIntegralAccumulation();
    checkOutputAndIntegralLimits();
    checkAntiWindup();
    checkReset();

    std::cout << "PASS: all PIDController checks passed\n";
    return 0;
}

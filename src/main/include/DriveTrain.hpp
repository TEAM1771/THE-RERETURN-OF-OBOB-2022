#pragma once

#include <units/angle.h>

namespace DriveTrain
{
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/

    void init();

    void drive(float l, float r);
    void stop();

    bool rotate(units::degree_t desired_CCW_rot);
    void driveStraight(float velocity_percent, units::degree_t desired_CCW_rot);

    void shift(bool up);
    void shiftToggle();
    void autoShift();

    [[nodiscard]] double getFLPos();
    [[nodiscard]] double getFRPos();
}

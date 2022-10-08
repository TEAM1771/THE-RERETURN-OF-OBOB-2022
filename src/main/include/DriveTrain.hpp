#pragma once

#include <units/angle.h>

namespace DriveTrain
{
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/

    void init();

    void drive(float l, float r);

    bool rotate(units::degree_t desired_CCW_rot);
    void driveStraight(float percent_speed, units::degree_t desired_CCW_rot);

    void shift(bool up);
    void shiftToggle();
    void autoShift();
}

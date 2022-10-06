#pragma once

#include <units/angle.h>

namespace DriveTrain {
    void drive(float l, float r);
    bool rotate(units::degree_t desired_CCW_rot);
    void driveStraight(float percent_speed, units::degree_t desired_CCW_rot);
}

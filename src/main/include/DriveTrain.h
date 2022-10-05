#pragma once

#include <rev/CanSparkMAX.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>

namespace DriveTrain {
    void drive(float l, float r);
    bool rotate(units::degree_t desired_CCW_rot);
    void driveStraight(float percent_speed, units::degree_t desired_CCW_rot);
}

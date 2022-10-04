#pragma once

#include <rev/CanSparkMAX.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>

namespace DriveTrain {
    void drive(float l, float r);
    void rotate(units::degree_t theta);
}

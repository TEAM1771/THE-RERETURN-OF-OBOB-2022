#pragma once

#include <units/angle.h>

namespace DriveTrain
{
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/

    void init();

    void drive(double l, double r);
    void stop();

    bool rotate(units::degree_t desired_CCW_rot, units::degree_t tolerance = 1_deg);
    void driveStraight(double velocity_percent, units::degree_t desired_CCW_rot);

    void shift(bool up);
    void shiftToggle();
    void autoShift();

    [[nodiscard]] double getFLPos();
    [[nodiscard]] double getFRPos();

    void brakeMode();
    void coastMode();
}

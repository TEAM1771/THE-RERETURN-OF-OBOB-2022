#pragma once

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>

namespace Navx
{
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/

    void init();

    // Returns values with 0 being front and positive angles going CW
    [[nodiscard]] units::degree_t getYaw();

    void zeroYaw();

    // [[nodiscard]] units::degree_t getPitch();

    // [[nodiscard]] units::degree_t getRoll();

    [[nodiscard]] frc::Rotation2d getCCWHeading();

    [[nodiscard]] frc::Rotation2d getCWHeading();
}
#pragma once

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>

namespace Navx
{
    void init();

    // Returns values with 0 being front and positive angles going CW
    [[nodiscard]] units::degree_t getAngle();

    void zeroYaw();

    [[nodiscard]] double getPitch();

    [[nodiscard]] frc::Rotation2d getCCWHeading();

    [[nodiscard]] frc::Rotation2d getCWHeading();

    void setNavxOffset(units::degree_t new_offset);
}
#include "Navx.hpp"

#include <AHRS.h>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>

#include <chrono>

#include <wpi/numbers>

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

static std::unique_ptr<AHRS> navx;

static bool first_time_getting_angle = true;

/******************************************************************/
/*                   Public Function Defintions                   */
/******************************************************************/

void Navx::init()
{
    navx = std::make_unique<AHRS>(frc::SPI::Port::kMXP);
}

// Returns values with 0 being front and positive angles going CW
units::degree_t Navx::getYaw()
{

    if (first_time_getting_angle)
    {
        navx->ZeroYaw(); // This can't be called in init() since the gyro will still be calibrating
                         // Plus this is a great way of accounting for bot movement since it has been powered on

        first_time_getting_angle = false;
    }

    return units::degree_t{navx->GetYaw()};
}

void Navx::zeroYaw()
{
    navx->ZeroYaw();
}

// units::degree_t Navx::getPitch()
// {
//     return getCorrectedAngles().pitch;
// }

// units::degree_t Navx::getRoll()
// {
//     return getCorrectedAngles().roll;
// }

frc::Rotation2d Navx::getCCWHeading() { return {-getYaw()}; }

frc::Rotation2d Navx::getCWHeading() { return {getYaw()}; }

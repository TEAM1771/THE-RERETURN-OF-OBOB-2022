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

/* LINEAR ALGEBRA :)

static double PITCH_OFFSET;
static double ROLL_OFFSET;


struct Angles
{
    units::degree_t pitch;
    units::degree_t roll;
    units::degree_t yaw;
};

Angles getCorrectedAngles()
{
    if (first_run)
    {

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(10s);
        first_run = !first_run;

        PITCH_OFFSET = navx->GetPitch();
        ROLL_OFFSET = navx->GetRoll();
        YAW_OFFSET = navx->GetYaw();
    };

    auto const pitch = navx->GetPitch() - PITCH_OFFSET;
    auto const roll = navx->GetRoll() - ROLL_OFFSET;
    auto const yaw = navx->GetYaw() - YAW_OFFSET;

    frc::SmartDashboard::PutNumber("Raw Pitch", navx->GetPitch());
    frc::SmartDashboard::PutNumber("Raw Roll", navx->GetRoll());
    frc::SmartDashboard::PutNumber("Raw Yaw", navx->GetYaw());

    frc::SmartDashboard::PutNumber("Offset Pitch", pitch);
    frc::SmartDashboard::PutNumber("Offset Roll", roll);
    frc::SmartDashboard::PutNumber("Offset Yaw", yaw);

    frc::SmartDashboard::PutNumber("Pitch Offset", PITCH_OFFSET);
    frc::SmartDashboard::PutNumber("Roll Offset", ROLL_OFFSET);
    frc::SmartDashboard::PutNumber("Yaw Offset", YAW_OFFSET);

    // [[maybe_unused]] constexpr auto PITCH_OFFSET =
    //     (-90 + -15.6) * (wpi::numbers::pi / 180);

    constexpr auto COS = -0.26891982051;
    constexpr auto SIN = -0.96316256682;

    return {
        units::degree_t{COS * pitch + SIN * yaw},
        units::degree_t{roll},
        units::degree_t{-SIN * pitch + COS * yaw}};
}
*/

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

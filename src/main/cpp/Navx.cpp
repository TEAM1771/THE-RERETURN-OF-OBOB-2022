#include "Navx.hpp"

#include <AHRS.h>
#include <cmath>

#include <wpi/numbers>

struct Angles
{
    units::degree_t pitch;
    units::degree_t roll;
    units::degree_t yaw;
};

static std::unique_ptr<AHRS> navx;

Angles getCorrectedAngles()
{

    [[maybe_unused]] constexpr auto PITCH_OFFSET =
        (-90 + -15.6) * (wpi::numbers::pi / 180);
    constexpr auto COS = -0.26891982051;
    constexpr auto SIN = -0.96316256682;

    auto const pitch = navx->GetPitch();
    auto const roll = navx->GetRoll();
    auto const yaw = navx->GetYaw();

    return {
        units::degree_t{pitch},
        units::degree_t{COS * roll + -SIN * yaw},
        units::degree_t{SIN * roll + COS * yaw}};
}

void Navx::init()
{
    navx = std::make_unique<AHRS>(frc::SPI::Port::kMXP);
}

// Returns values with 0 being front and positive angles going CW
units::degree_t Navx::getYaw()
{
    static bool first_time_getting_angle = true;

    if (first_time_getting_angle)
    {
        navx->ZeroYaw(); // This can't be called in init() since the gyro will still be calibrating
        first_time_getting_angle = false;
    }
    return getCorrectedAngles().yaw;
}

void Navx::zeroYaw()
{
    navx->ZeroYaw();
}

units::degree_t Navx::getPitch()
{
    return getCorrectedAngles().pitch;
}

units::degree_t Navx::getRoll()
{
    return getCorrectedAngles().roll;
}

frc::Rotation2d Navx::getCCWHeading() { return {-getYaw()}; }
// or navx->GetRotation()

frc::Rotation2d Navx::getCWHeading() { return {getYaw()}; }

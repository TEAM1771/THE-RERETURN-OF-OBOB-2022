#include <Navx.h>

#include <AHRS.h>

static std::unique_ptr<AHRS> navx;
static auto navx_offset = 0_deg;

void Navx::init() {
    navx = std::make_unique<AHRS>(frc::SPI::Port::kMXP);
}

// Returns values with 0 being front and positive angles going CW
units::degree_t Navx::getAngle()
{
    static bool first_time_getting_angle = true;

    if (first_time_getting_angle)
    {
        navx->ZeroYaw(); // This can't be called in init() since the gyro will still be calibrating
        first_time_getting_angle = false;
    }
    return units::degree_t{navx->GetAngle()} + navx_offset;
}

void Navx::zeroYaw()
{
    navx->ZeroYaw();
}

double Navx::getPitch()
{
    // Roll instead of pitch because navx is installed sideways
    return navx->GetRoll();
}

frc::Rotation2d Navx::getCCWHeading() { return {-getAngle()}; }
// or navx->GetRotation()

frc::Rotation2d Navx::getCWHeading() { return {getAngle()}; }

void Navx::setNavxOffset(units::degree_t new_offset)
{
    navx_offset = new_offset;
}

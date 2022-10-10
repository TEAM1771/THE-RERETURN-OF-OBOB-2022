#include "DriveTrain.hpp"
#include "Navx.hpp"

#include <frc/Solenoid.h>
#include <frc/PneumaticHub.h>
#include <frc/Compressor.h>
#include <wpi/numbers>
#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/CanSparkMAX.h>

#include <cmath>

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

// For rotate and driveStraight functions
constexpr double ROT_P = 1 / 30.0;
constexpr double MAX_ROT = .3;

// For drive function
static rev::CANSparkMax f_l{1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
static rev::CANSparkMax b_l{2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
static rev::CANSparkMax f_r{3, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
static rev::CANSparkMax b_r{4, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

// For autoShift()
static auto const f_l_encoder = f_l.GetEncoder();
static auto const f_r_encoder = f_r.GetEncoder();
constexpr auto SHIFT_UP_THRESHOLD = 3000;
constexpr auto SHIFT_DOWN_THRESHOLD = 2000;

// For shift function
constexpr int P_HUB_ID = 15;
constexpr auto P_HUB_TYPE = frc::PneumaticsModuleType::REVPH;
static frc::Solenoid shifter{P_HUB_ID, P_HUB_TYPE, 15};

// For enabling the compressor in init()
static frc::Compressor compressor{P_HUB_ID, P_HUB_TYPE};

/******************************************************************/
/*                  Private Function Definitions                  */
/******************************************************************/

units::degree_t diffFromCurrentRot(units::degree_t new_CCW_rot)
{
    units::degree_t current_CCW_rot = Navx::getCCWHeading().Degrees();

    units::degree_t diff{new_CCW_rot - current_CCW_rot}; // Difference between current & desired

    diff = units::degree_t{std::fmod(diff.value(), 360)}; // Rounds any angle above 360 or below -360
                                                          // to be between -360, 360

    if (diff < -180_deg)
        diff += 360_deg; // Constrains to [-180, 360] by optimizing angles < -180
    if (diff > 180_deg)
        diff -= 360_deg; // Constrains to [-180, 180] by optimizing angles > 180

    return diff;
}

/******************************************************************/
/*                   Public Function Defintions                   */
/******************************************************************/

void DriveTrain::init()
{
    compressor.EnableDigital();

    coastMode();

    f_l.SetInverted(true);
    b_l.SetInverted(true);

    f_r.SetInverted(false);
    b_r.SetInverted(false);
}

void DriveTrain::drive(double l, double r)
{
    if (l > 1)
        l = 1;
    else if (l < -1)
        l = -1;

    if (r > 1)
        r = 1;
    else if (r < -1)
        r = -1;

    f_l.Set(l);
    b_l.Set(l);
    f_r.Set(r);
    b_r.Set(r);
}

void DriveTrain::stop()
{
    drive(0, 0);
}

bool DriveTrain::rotate(units::degree_t desired_CCW_rot, units::degree_t tolerance)
{
    // How far off is the bot?
    units::degree_t to_go_CCW = diffFromCurrentRot(desired_CCW_rot);



    // Determine a speed percentage to spin at (positive is CCW, negative is CW)
    double rot_percent = to_go_CCW.value() * ROT_P;

    if (std::abs(rot_percent) > MAX_ROT)
        rot_percent = (rot_percent > 0) ? MAX_ROT : -MAX_ROT;

    // For debugging / tuning
    // frc::SmartDashboard::PutNumber("to_go_CCW", to_go_CCW.value());
    // frc::SmartDashboard::PutNumber("Current rot (from NavX)", Navx::getCCWHeading().Degrees().value());
    // frc::SmartDashboard::PutNumber("rot_percent", rot_percent);

    // Spin the bot at that percent speed
    // -, + to create CCW spin
    drive(-rot_percent, rot_percent);

    // Return true if within tolerance
    return to_go_CCW < tolerance && to_go_CCW > -tolerance;
}

void DriveTrain::driveStraight(double velocity_percent, units::degree_t desired_CCW_rot)
{
    // How far off is the bot?
    units::degree_t to_go_CCW = diffFromCurrentRot(desired_CCW_rot);

    // Make variables to manipulate
    double l = velocity_percent;
    double r = velocity_percent;

    // Determine a percent to correct (positive is CCW, negative is CW)
    double correction = to_go_CCW.value() * ROT_P;

    if (std::abs(correction) > MAX_ROT)
        correction = MAX_ROT;

    if (correction > 0) // If correction is CCW
    {
        r += correction; // Speed up right
        l -= correction; // Slow down left
    }
    else // If correction is CW
    {
        l += -correction; // Speed up left
        r -= -correction; // Slow down right
    }

    drive(l, r);
}

void DriveTrain::shift(bool up)
{
    if (up != shifter.Get())
        shifter.Set(up);
}

void DriveTrain::shiftToggle()
{
    shifter.Toggle();
}

void DriveTrain::autoShift()
{
    auto const left_vel = f_l_encoder.GetVelocity();
    auto const right_vel = f_r_encoder.GetVelocity();

    auto const speed = std::abs((left_vel + right_vel) / 2);

    if (speed > SHIFT_UP_THRESHOLD && !shifter.Get())
        DriveTrain::shift(true);
    else if (speed < SHIFT_DOWN_THRESHOLD && shifter.Get())
        DriveTrain::shift(false);
}

double DriveTrain::getFLPos()
{
    return f_l_encoder.GetPosition();
}

double DriveTrain::getFRPos()
{
    return f_r_encoder.GetPosition();
}

void DriveTrain::brakeMode()
{
    f_l.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    f_r.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    b_l.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    b_r.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void DriveTrain::coastMode()
{
    f_l.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    f_r.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    b_l.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    b_r.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}
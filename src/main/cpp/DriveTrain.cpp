#include <DriveTrain.h>
#include <cmath>
#include <Navx.h>

static constexpr double P_VAL = 1 / 180.0;
static constexpr units::degree_t DEADBAND = 3_deg;

static constexpr double SPEED_UP_SLOW_DOWN_THRESHOLD = .9;

static rev::CANSparkMax u_left{1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
static rev::CANSparkMax l_left{2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
static rev::CANSparkMax u_right{3, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
static rev::CANSparkMax l_right{4, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

units::degree_t diffFromCurrentRot(units::degree_t new_CCW_rot)
{
    units::degree_t current_rot = Navx::getCCWHeading().Degrees();

    units::degree_t diff{new_CCW_rot - current_rot}; // Difference between current & desired

    diff = units::degree_t{std::fmod(diff.value(), 360)}; // Rounds any angle above 360 or below -360
                                                          // to be between -360, 360

    if (diff < -180_deg)
        diff += 360_deg; // Constrains to [-180, 360] by optimizing angles < -180
    if (diff > 180_deg)
        diff -= 360_deg; // Constrains to [-180, 180] by optimizing angles > 180

    return diff;
}

void DriveTrain::drive(float l, float r)
{
    if (l > 1)
        l = 1;
    else if (l < -1)
        l = -1;

    if (r > 1)
        r = 1;
    else if (r < -1)
        r = -1;

    u_left.Set(l);
    u_right.Set(-r);
    l_left.Set(l);
    l_right.Set(-r);
}

bool DriveTrain::rotate(units::degree_t desired_CCW_rot)
{
    // How far off is the bot?
    units::degree_t to_go_CCW = diffFromCurrentRot(desired_CCW_rot);

    // If less than deadband, don't bother correcting.
    if (to_go_CCW < DEADBAND && to_go_CCW > -DEADBAND)
        return true; // Return true because bot reached desired rot

    // Determine a speed percentage to spin at (positive is CCW, negative is CW)
    int rot_percent = to_go_CCW.value() * P_VAL;

    // Spin the bot at that percent speed
    // -, + to create CCW spin
    drive(-rot_percent, rot_percent);

    // Return false because bot did not yet reach desired rot
    return false;
}

void DriveTrain::driveStraight(float percent_speed, units::degree_t desired_CCW_rot)
{
    // How far off is the bot?
    units::degree_t to_go_CCW = diffFromCurrentRot(desired_CCW_rot);

    // If less than deadband, don't bother correcting.
    if (to_go_CCW < DEADBAND && to_go_CCW > -DEADBAND)
    {
        drive(percent_speed, percent_speed);
        return; // Return to exit function
    }

    // Make variables to manipulate
    int l = percent_speed;
    int r = percent_speed;

    // Determine a percent to correct (positive is CCW, negative is CW)
    double correction = to_go_CCW.value() * P_VAL;

    // If the motor percent power you have to work with is
    // less than the percent you need to correct
    if (1 - std::abs(percent_speed) < std::abs(correction))
    {
        if (correction > 0)   // If correction is CCW
            l -= correction;  // Slow down left
        else                  // If correction is CW
            r -= -correction; // Slow down right
    }
    else
    {
        if (correction > 0)   // If correction is CCW
            r += correction;  // Speed up right
        else                  // If correction is CW
            l += -correction; // Speed up left
    }
}
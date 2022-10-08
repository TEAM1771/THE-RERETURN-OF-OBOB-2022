#include "Auton.hpp"
#include "DriveTrain.hpp"
#include "Intake.hpp"

#include <units/angle.h>
#include <units/time.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <thread>
#include <functional>

using namespace std::chrono_literals;

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

static frc::SendableChooser<std::function<void()>> auton_selector;

/******************************************************************/
/*               Private Helper Function Definitions              */
/******************************************************************/
void sleep()
{
    std::this_thread::sleep_for(10ms);
}

void rotateUntil(units::degree_t desired_CCW_rot)
{
    while (!DriveTrain::rotate(desired_CCW_rot))
        sleep();
}

void wait(units::second_t time)
{
    frc::Timer timer;
    timer.Start();

    while (!timer.HasElapsed(time))
        sleep();
}

// void driveFor(double l, double r, units::second_t time);

void driveStraightFor(double velocity_percent, units::degree_t desired_CCW_rot, units::second_t time)
{
    frc::Timer timer;
    timer.Start();

    while (!timer.HasElapsed(time))
    {
        DriveTrain::driveStraight(velocity_percent, desired_CCW_rot);
        sleep();
    }

    DriveTrain::stop();
}

// void driveUntil(double l, double r, double motor_position);

void driveStraightFor(double velocity_percent, units::degree_t desired_CCW_rot, double desired_motor_rotations)
{
    double start_l_pos = DriveTrain::getFLPos();
    double start_r_pos = DriveTrain::getFRPos();

    double delta_l_pos = 0;
    double delta_r_pos = 0;

    double distance_traveled = 0;

    while (desired_motor_rotations > distance_traveled)
    {
        delta_l_pos = std::abs(DriveTrain::getFLPos() - start_l_pos);
        delta_r_pos = std::abs(DriveTrain::getFRPos() - start_r_pos);

        distance_traveled = (delta_l_pos + delta_r_pos) / 2;

        DriveTrain::driveStraight(velocity_percent, desired_CCW_rot);

        sleep();
    }

    DriveTrain::stop();
}

/******************************************************************/
/*               Private Auton Function Definitions               */
/******************************************************************/

void roboBuddies()
{
    driveStraightFor(.3, 0_deg, 10000);
    rotateUntil(110_deg);
    wait(5_s);
    Intake::run(true);
}

void prepareToBully()
{
    Intake::run(true);
    driveStraightFor(.3, 0_deg, 8000);
    rotateUntil(180_deg);
    driveStraightFor(.3, 0_deg, 8000);
    rotateUntil(-100_deg);
    driveStraightFor(.3, 0_deg, 14000);
    rotateUntil(180_deg);
}

/******************************************************************/
/*                   Public Function Defintions                   */
/******************************************************************/

void Auton::pushSelector()
{
    auton_selector.SetDefaultOption("Default - Prepare to Bully (pickup 2 and face)", prepareToBully);
    auton_selector.AddOption("Buddy up with Robosaurus", roboBuddies);

    frc::SmartDashboard::PutData("Auton Selector", &auton_selector);
}

void Auton::run()
{
    // Runs selected Auton
    auton_selector.GetSelected()();
}
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

void rotateUntil(units::degree_t desired_CCW_rot, units::degree_t tolerance = 1_deg)
{
    while (!DriveTrain::rotate(desired_CCW_rot, tolerance))
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

    while (distance_traveled < desired_motor_rotations)
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

void robo6()
{
    frc::Timer timer;
    timer.Start();

    driveStraightFor(.2, 0_deg, 60);
    rotateUntil(-45_deg);
    while (!timer.HasElapsed(13_s))
        sleep();
    Intake::run(true);
}

void robo5()
{
    DriveTrain::brakeMode();

    frc::Timer timer;
    timer.Start();

    while (!timer.HasElapsed(6.5_s))
        sleep();

    driveStraightFor(-.4, 0_deg, 60);
    rotateUntil(-90_deg);
    driveStraightFor(.4, -90_deg, 50);
    rotateUntil(-85_deg);
    DriveTrain::stop();

    while (!timer.HasElapsed(11.75_s))
        sleep();


    Intake::run(true);

    DriveTrain::coastMode();
}

void prepareToBully()
{
    Intake::run(false);
    rotateUntil(3_deg, 1.5_deg);
    driveStraightFor(-.25, 3_deg, 55);
    driveStraightFor(.25, 3_deg, 55);
    rotateUntil(-107_deg);
    driveStraightFor(-.3, -104_deg, 95);
    rotateUntil(-15_deg);

    DriveTrain::stop();
}

/******************************************************************/
/*                   Public Function Defintions                   */
/******************************************************************/

void Auton::pushSelector()
{
    auton_selector.SetDefaultOption("Default - Prepare to Bully (pickup 2 and face)", prepareToBully);
    auton_selector.AddOption("Robosaurus 6 + 1 Ball", robo6);
    auton_selector.AddOption("Robosaurus 5 + 1 Ball", robo5);

    frc::SmartDashboard::PutData("Auton Selector", &auton_selector);
}

void Auton::run()
{
    // Runs selected Auton
    auton_selector.GetSelected()();
}
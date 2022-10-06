// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"
#include "Buttons.hpp"
#include "DriveTrain.hpp"
#include "Intake.hpp"
#include "Navx.hpp"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <fmt/core.h>
#include <rev/CANSparkMax.h>

void Robot::RobotInit()
{
  DriveTrain::init();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  frc::SmartDashboard::PutNumber("Pitch", Navx::getPitch().value());
  frc::SmartDashboard::PutNumber("Roll", Navx::getRoll().value());
  frc::SmartDashboard::PutNumber("Yaw", Navx::getYaw().value());
}

void Robot::AutonomousInit()
{
  using namespace std::literals::chrono_literals;
  DriveTrain::drive(0, 1);
  std::this_thread::sleep_for(15s);
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
  double const l = BUTTON::JOY_L.GetY();
  double const r = BUTTON::JOY_R.GetY();

  DriveTrain::autoShift();

  DriveTrain::drive(l, r);

  if (BUTTON::INTAKE)
    Intake::run(false);
  else if (BUTTON::OUTTAKE)
    Intake::run(true);
  else
    Intake::stop();

  // if (BUTTON::SHIFT.getRawButtonPressed())
  //   DriveTrain::shiftToggle();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif

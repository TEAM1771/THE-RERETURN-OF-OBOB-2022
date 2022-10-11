// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"
#include "Buttons.hpp"
#include "DriveTrain.hpp"
#include "Intake.hpp"
#include "Navx.hpp"
#include "Auton.hpp"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <fmt/core.h>
#include <rev/CANSparkMax.h>

/******************************************************************/
/*                   Public Function Defintions                   */
/******************************************************************/

void Robot::RobotInit()
{
  DriveTrain::init();
  Navx::init();
  Auton::pushSelector();
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
  // Tuning/debugging purposes

  // frc::SmartDashboard::PutNumber("Pitch", Navx::getPitch().value());
  // frc::SmartDashboard::PutNumber("Roll", Navx::getRoll().value());
  // frc::SmartDashboard::PutNumber("Yaw", Navx::getYaw().value());
}

void Robot::AutonomousInit()
{
  Auton::run();
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
  double const l = -BUTTON::JOY_L.GetY();
  double const r = -BUTTON::JOY_R.GetY();

  // DriveTrain::shift(DriveTrain::GEAR::LOW);

  DriveTrain::drive(l, r);

  if (BUTTON::INTAKE)
    Intake::run(false);
  else if (BUTTON::OUTTAKE)
    Intake::run(true);
  else
    Intake::stop();

  // Manual shifting
  // if (BUTTON::SHIFT.getRawButtonPressed())
  //   DriveTrain::shiftToggle();


  if(BUTTON::JOY_R.GetZ() <= 0.5)
    DriveTrain::autoShift();
  else
    DriveTrain::shift(DriveTrain::GEAR::LOW);
  // Tuning/debugging purposes
  // frc::SmartDashboard::PutNumber("Yaw", Navx::getYaw().value());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic()
{
  // For testing rotate and driveStraight functions
  double const l = -BUTTON::JOY_L.GetY();

  if (BUTTON::INTAKE)
    DriveTrain::rotate(0_deg);
  else if (BUTTON::OUTTAKE)
    DriveTrain::rotate(90_deg);
  else
    DriveTrain::driveStraight(l, 0_deg);

  // Tuning/debugging purposes
  // frc::SmartDashboard::PutNumber("Yaw", Navx::getYaw().value());
}

// This is just to run the WPILIB magic
#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif

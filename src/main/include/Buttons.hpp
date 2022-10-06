#pragma once

#include <frc/Joystick.h>

class JoystickButton
{
    /******************************************************************/
    /*                  Private Variable Declarations                 */
    /******************************************************************/
    frc::GenericHID &stick;
    int const button;

public:
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/
    JoystickButton(frc::GenericHID &, int const button);
    [[nodiscard]] operator bool() const { return stick.GetRawButton(button); }
    [[nodiscard]] bool getRawButton() const;
    [[nodiscard]] bool getRawButtonPressed();
    [[nodiscard]] bool getRawButtonReleased();
};

/******************************************************************/
/*                        Public Constants                        */
/******************************************************************/
namespace BUTTON
{
    inline frc::Joystick JOY_L{0};
    inline frc::Joystick JOY_R{1};
    inline frc::Joystick JOY_EXTRA{2};

    inline JoystickButton SHIFT{JOY_R, 11};
    inline JoystickButton INTAKE{JOY_L, 7};
    inline JoystickButton OUTTAKE{JOY_L, 6};

} // namespace BUTTON
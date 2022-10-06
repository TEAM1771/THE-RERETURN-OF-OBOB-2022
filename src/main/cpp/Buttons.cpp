#include "Buttons.hpp"

#include <fmt/format.h>

/******************************************************************/
/*                   Public Function Defintions                   */
/******************************************************************/

JoystickButton::JoystickButton(frc::GenericHID &stick, int button)
    : stick{stick}, button{button}
{
    if (button > stick.GetButtonCount() || button < 1)
        fmt::print("Invalid Button Assignment: {}\n", button);
}

bool JoystickButton::getRawButton() const
{
    return stick.GetRawButton(button);
}

bool JoystickButton::getRawButtonPressed()
{
    return stick.GetRawButtonPressed(button);
}

bool JoystickButton::getRawButtonReleased()
{
    return stick.GetRawButtonReleased(button);
}
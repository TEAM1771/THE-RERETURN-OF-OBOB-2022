#include "Intake.hpp"

#include <rev/CanSparkMAX.h>

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

static rev::CANSparkMax intake{10, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

/******************************************************************/
/*                   Public Function Defintions                   */
/******************************************************************/
void Intake::run(bool invert)
{
    intake.Set(invert ? -1 : 1);
}

void Intake::stop()
{
    intake.Set(0);
}
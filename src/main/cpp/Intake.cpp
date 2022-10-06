#include "Intake.hpp"

#include <rev/CanSparkMAX.h>

static rev::CANSparkMax intake{10, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

void Intake::run(bool invert)
{
    intake.Set(invert ? -1 : 1);
}

void Intake::stop()
{
    intake.Set(0);
}
#include <DriveTrain.h>

static rev::CANSparkMax u_left{1, rev::CANSparkMaxLowLevel::MotorType::kBrushless}; 
static rev::CANSparkMax l_left{2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
static rev::CANSparkMax u_right{3, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
static rev::CANSparkMax l_right{4, rev::CANSparkMaxLowLevel::MotorType::kBrushless};


void DriveTrain::drive(float l, float r) {
    u_left.Set(l);
    u_right.Set(-r);
    l_left.Set(l);
    l_right.Set(-r);
}

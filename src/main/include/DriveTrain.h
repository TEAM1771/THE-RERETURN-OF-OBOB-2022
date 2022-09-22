#pragma once

#include <rev/CanSparkMAX.h>

class DriveTrain {
    private:
        rev::CANSparkMax u_left{1, rev::CANSparkMaxLowLevel::MotorType::kBrushless}; 
        rev::CANSparkMax l_left{2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax u_right{3, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        rev::CANSparkMax l_right{4, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    public:
        void drive(float l, float r);
};

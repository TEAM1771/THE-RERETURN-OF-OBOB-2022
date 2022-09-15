#pragma once

#include <rev/CanSparkMAX.h>

class DriveTrain {
    private:
        using enum rev::CANSparkMaxLowLevel::MotorType;
        rev::CANSparkMax u_left{1, kBrushless}; 
        rev::CANSparkMax l_left{2, kBrushless};
        rev::CANSparkMax u_right{3, kBrushless};
        rev::CANSparkMax l_right{4, kBrushless};
    public:
        void drive(float l, float r);
};

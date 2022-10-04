#include <DriveTrain.h>
#include <cmath>
#include <Navx.h>
static const float p_value = 1/180;
static const units::degrees_per_second_t m_speed{1};
static rev::CANSparkMax u_left{1, rev::CANSparkMaxLowLevel::MotorType::kBrushless}; 
static rev::CANSparkMax l_left{2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
static rev::CANSparkMax u_right{3, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
static rev::CANSparkMax l_right{4, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
static const bool field_relative = true;
static const double rot_p;

void DriveTrain::drive(float l, float r) {
    u_left.Set(l);
    u_right.Set(-r);
    l_left.Set(l);
    l_right.Set(-r);
}

void DriveTrain::rotate(units::degree_t theta) {
    float togo = std::fmod((theta - Navx::getCCWHeading().Degrees()).value(), 360);

    if (togo < -180)
        togo += 360; // Constrains to [-180, 360] & optimizes angles < -180
    if (togo > 180)
        togo -= 360;

    if (std::abs(togo) < 3)
        togo = 0;

    togo *= p_value;
    if (togo > 1) {
        togo = std::floor(togo);
    } else {
        togo = std::ceil(togo);
    }
    drive(togo, -togo);   
}
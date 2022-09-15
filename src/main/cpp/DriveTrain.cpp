#include <DriveTrain.h>

void DriveTrain::drive(float l, float r) {
    u_left.Set(l);
    u_right.Set(-r);
    l_left.Set(l);
    l_right.Set(-r);
}
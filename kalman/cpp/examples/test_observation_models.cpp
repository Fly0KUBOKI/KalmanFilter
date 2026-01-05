#include <cstdio>
#include "../Lib/MEUKF/inc/meukf_observation_models.hpp"
#include "../Lib/Quaternion/quaternion_functions.hpp"

using namespace meukf;
using namespace cmath_fx;

int main() {
    // Build a 15D state with identity quaternion
    Vector<15,float> x15 = Vector<15,float>::Zero();
    // position/velocity/ba/bg left zero
    // quaternion identity [w,x,y,z] = [1,0,0,0]
    x15(6,0) = 1.0f;
    x15(7,0) = 0.0f;
    x15(8,0) = 0.0f;
    x15(9,0) = 0.0f;

    Vector<3,float> z = AccelObservationModel::h_accel(x15);
    std::printf("Accel prediction (identity quat): %.6f, %.6f, %.6f\n", z(0,0), z(1,0), z(2,0));

    // Expected: -g_ned = [0,0,-9.81]
    return 0;
}

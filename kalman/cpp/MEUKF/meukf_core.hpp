#pragma once
#include "meukf_types.hpp"
#include "../Common/Math/fixed_matrix.hpp"
#include "../Common/Math/quaternion.hpp"

namespace meukf {

using Vector3 = cmath_fx::Matrix<3, 1, float>;
using Vector2 = cmath_fx::Matrix<2, 1, float>;
using Vector4 = cmath_fx::Matrix<4, 1, float>;
using Matrix3x3 = cmath_fx::Matrix<3, 3, float>;
using Matrix2x2 = cmath_fx::Matrix<2, 2, float>;
using Matrix3x2 = cmath_fx::Matrix<3, 2, float>;
using Matrix15x15 = cmath_fx::Matrix<15, 15, float>;
using Matrix15x2 = cmath_fx::Matrix<15, 2, float>;
using Matrix15x3 = cmath_fx::Matrix<15, 3, float>;
using Vector15 = cmath_fx::Matrix<15, 1, float>;

class MEUKFCore {
public:
    // Main step function
    static void step(const MEUKFInput& input, MEUKFOutput& output);

private:
    // Prediction step
    static void predict(State& state, const SensorData& sensor, const Params& params);
    
    // MEUKF Attitude Update (Accel)
    static void update_accel_meukf(State& state, const Vector3& accel_meas, const Params& params, MEUKFOutput& output);
    
    // MEUKF Attitude Update (Mag)
    static void update_mag_meukf(State& state, const Vector3& mag_meas, const Params& params, MEUKFOutput& output);
    
    // GPS Update
    static void update_gps(State& state, const Vector3& gps_meas, const Params& params, MEUKFOutput& output);
    
    // Baro Update
    static void update_baro(State& state, float alt_baro, const Params& params, MEUKFOutput& output);

    // Helpers
    static void state_to_vars(const State& s, Vector3& p, Vector3& v, Vector4& q, Vector3& ba, Vector3& bg, Matrix15x15& P);
    static void vars_to_state(const Vector3& p, const Vector3& v, const Vector4& q, const Vector3& ba, const Vector3& bg, const Matrix15x15& P, State& s);
};

} // namespace meukf

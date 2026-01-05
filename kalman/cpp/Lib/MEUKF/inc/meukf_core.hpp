#pragma once

// Copied from Inc/MEUKF/meukf_core.hpp (paths adjusted for Lib/ location)

#include "unified_types.hpp"
#include "meukf_types.hpp"  // MEUKFInput, MEUKFOutput用
#include "../../Matrix/fixed_matrix.hpp"
#include "../../Quaternion/quaternion_functions.hpp"

namespace meukf {

// float型を使用（ユーザー要求により）
using Vector3 = cmath_fx::Matrix<3, 1, float>;
using Vector2 = cmath_fx::Matrix<2, 1, float>;
using Vector4 = cmath_fx::Matrix<4, 1, float>;
using Matrix3x3 = cmath_fx::Matrix<3, 3, float>;
using Matrix2x2 = cmath_fx::Matrix<2, 2, float>;
using Matrix3x2 = cmath_fx::Matrix<3, 2, float>;
using Matrix2x3 = cmath_fx::Matrix<2, 3, float>;
using Matrix15x15 = cmath_fx::Matrix<15, 15, float>;
using Matrix15x2 = cmath_fx::Matrix<15, 2, float>;
using Matrix15x3 = cmath_fx::Matrix<15, 3, float>;
using Vector15 = cmath_fx::Matrix<15, 1, float>;

class MEUKFCore {
public:
    // Main step function
    static void step(const MEUKFInput& input, MEUKFOutput& output);

    // Comparison helper (test): run both original and UKF-backed accel updates
    // Returns true if both updates executed; outputs are written to out_orig and out_ukf
    static bool compare_accel_updates(const State& init_state, const SensorData& sensor, const Params& params, MEUKFOutput& out_orig, MEUKFOutput& out_ukf);

private:
    // Prediction step
    static void predict(State& state, const SensorData& sensor, const Params& params);
    
    // MEUKF Attitude Update (Accel)
    static void update_accel_meukf(State& state, const Vector3& accel_meas, const Params& params, MEUKFOutput& output);
    // UKF-backed accel update (delegates to ukf_generic.hpp)
    static void update_accel_meukf_ukf_version(State& state, const Vector3& accel_meas, const Params& params, MEUKFOutput& output);
    
    // MEUKF Attitude Update (Mag)
    static void update_mag_meukf(State& state, const Vector3& mag_meas, const Params& params, MEUKFOutput& output);
    // UKF-backed mag update
    static void update_mag_meukf_ukf_version(State& state, const Vector3& mag_meas, const Params& params, MEUKFOutput& output);
    
    // GPS Update
    static void update_gps(State& state, const Vector3& gps_meas, const Params& params, MEUKFOutput& output);
    
    // Baro Update
    static void update_baro(State& state, float alt_baro, const Params& params, MEUKFOutput& output);

    // ZUPT Update
    static void update_zupt(State& state, const Params& params, MEUKFOutput& output);

    // Helpers
    static void state_to_vars(const State& s, Vector3& p, Vector3& v, Vector4& q, Vector3& ba, Vector3& bg, Matrix15x15& P);
    static void vars_to_state(const Vector3& p, const Vector3& v, const Vector4& q, const Vector3& ba, const Vector3& bg, const Matrix15x15& P, State& s);
};

} // namespace meukf

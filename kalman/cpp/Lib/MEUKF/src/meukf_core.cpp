#include "../inc/meukf_core.hpp"
#include "../../Common/inc/Math/math_utils.hpp"
#include "../../Quaternion/quaternion_functions.hpp"
#include "../../UKF/inc/ukf_update.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <cstdlib>

namespace meukf {


// -----------------------------------------------------------------
// GPS/Baro/ZUPT UKF-version wrappers
// These currently delegate to the original MEUKF implementations
// so we can switch call sites incrementally. Replace with actual
// UKF-backed implementations later as needed.
// -----------------------------------------------------------------
void MEUKFCore::update_gps_meukf_ukf_version(State& state, const Vector3& gps_meas, const Params& params, MEUKFOutput& output)
{
    // UKF-backed GPS update using full 15-dim error-state (p,v,theta,ba,bg)
    using UKF = ukf::UKFUpdate<15, 3, float>;
    using Vector15 = UKF::VectorN;
    using Matrix15x15 = UKF::MatrixNN;
    using Matrix15x3 = UKF::MatrixNM;
    using Matrix3x3 = UKF::MatrixMM;
    using Vector3f = UKF::VectorM;

    // Convert State -> UKF state vector x (nominal state composition)
    Vector15 x; Matrix15x15 P;
    // p (0..2)
    for (int i = 0; i < 3; ++i) x(i,0) = state.p[i];
    // v (3..5)
    for (int i = 0; i < 3; ++i) x(3+i,0) = state.v[i];
    // small-angle attitude (6..8): initially zero (error state representation)
    for (int i = 0; i < 3; ++i) x(6+i,0) = 0.0f;
    // ba (9..11)
    for (int i = 0; i < 3; ++i) x(9+i,0) = state.ba[i];
    // bg (12..14)
    for (int i = 0; i < 3; ++i) x(12+i,0) = state.bg[i];

    // Fill P from state.P (row-major stored)
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) P(i,j) = state.P[i*15 + j];

    // Measurement vector z (GPS position)
    Vector3f z;
    z(0,0) = gps_meas(0,0);
    z(1,0) = gps_meas(1,0);
    z(2,0) = gps_meas(2,0);

    // Observation function: extract position from state vector
    auto h_func = [](const Vector15& xv) {
        Vector3f zv;
        zv(0,0) = xv(0,0);
        zv(1,0) = xv(1,0);
        zv(2,0) = xv(2,0);
        return zv;
    };

    // Measurement noise R (3x3)
    Matrix3x3 R = Matrix3x3::Zero();
    for (int i = 0; i < 3; ++i) R(i,i) = params.noise_gps[i];

    // UKF parameters
    ukf::UKFParams up; up.alpha = params.alpha; up.beta = params.beta; up.kappa = params.kappa;

    // Optional outputs
    Matrix15x3 K_out;
    Matrix3x3 S_out;
    Vector3f y_out;

    bool ok = UKF::update(x, P, z, h_func, R, up, &K_out, &S_out, &y_out);
    if (!ok) {
        output.status = 1;
        return;
    }

    // Write back updated nominal state: p,v,apply attitude small-angle to quaternion, ba,bg
    // p (0..2)
    for (int i = 0; i < 3; ++i) state.p[i] = x(i,0);
    // v (3..5)
    for (int i = 0; i < 3; ++i) state.v[i] = x(3+i,0);
    // attitude: small-angle theta = x(6..8)
    Vector4 q_in; q_in(0,0)=state.q[0]; q_in(1,0)=state.q[1]; q_in(2,0)=state.q[2]; q_in(3,0)=state.q[3];
    Vector4 dq; dq(0,0) = 1.0f; dq(1,0) = 0.5f * x(6,0); dq(2,0) = 0.5f * x(7,0); dq(3,0) = 0.5f * x(8,0);
    cquat::normalize_quat(dq);
    Vector4 q_new; cquat::multiply_quat(q_in, dq, q_new);
    cquat::normalize_quat(q_new);
    state.q[0] = q_new(0,0); state.q[1] = q_new(1,0); state.q[2] = q_new(2,0); state.q[3] = q_new(3,0);
    // ba/bg
    for (int i = 0; i < 3; ++i) state.ba[i] = x(9+i,0);
    for (int i = 0; i < 3; ++i) state.bg[i] = x(12+i,0);

    // Write back covariance P (row-major)
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) state.P[i*15 + j] = P(i,j);

    // Diagnostics
    // last_K: 15 x 3 row-major
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 3; ++j) output.last_K[i*3 + j] = K_out(i,j);
    // last_S
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) output.last_S[i*3 + j] = S_out(i,j);
    // last_y
    for (int i = 0; i < 3; ++i) output.last_y[i] = y_out(i,0);
    output.last_y_len = 3;
    output.last_sensor_type = 3; // GPS
    // pred_P already set earlier in step()
}

void MEUKFCore::update_baro_meukf_ukf_version(State& state, float alt_baro, const Params& params, MEUKFOutput& output)
{
    // UKF-backed Baro update: measurement is altitude -> p_z
    using UKF1 = ukf::UKFUpdate<15, 1, float>;
    using Vector15 = UKF1::VectorN;
    using Matrix15x15 = UKF1::MatrixNN;
    using Matrix15x1 = UKF1::MatrixNM;
    using Matrix1x1 = UKF1::MatrixMM;
    using Vector1f = UKF1::VectorM;

    Vector15 x; Matrix15x15 P;
    for (int i = 0; i < 3; ++i) x(i,0) = state.p[i];
    for (int i = 0; i < 3; ++i) x(3+i,0) = state.v[i];
    for (int i = 0; i < 3; ++i) x(6+i,0) = 0.0f;
    for (int i = 0; i < 3; ++i) x(9+i,0) = state.ba[i];
    for (int i = 0; i < 3; ++i) x(12+i,0) = state.bg[i];
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) P(i,j) = state.P[i*15 + j];

    Vector1f z; z(0,0) = alt_baro;

    auto h_func_bar = [](const Vector15& xv) {
        Vector1f zv;
        zv(0,0) = xv(2,0); // p_z
        return zv;
    };

    Matrix1x1 R = Matrix1x1::Zero(); R(0,0) = params.noise_baro;
    ukf::UKFParams up; up.alpha = params.alpha; up.beta = params.beta; up.kappa = params.kappa;

    Matrix15x1 K_out; Matrix1x1 S_out; Vector1f y_out;
    bool ok = UKF1::update(x, P, z, h_func_bar, R, up, &K_out, &S_out, &y_out);
    if (!ok) { output.status = 1; return; }

    // Apply updates back to state
    for (int i = 0; i < 3; ++i) state.p[i] = x(i,0);
    for (int i = 0; i < 3; ++i) state.v[i] = x(3+i,0);
    Vector4 q_in; q_in(0,0)=state.q[0]; q_in(1,0)=state.q[1]; q_in(2,0)=state.q[2]; q_in(3,0)=state.q[3];
    Vector4 dq; dq(0,0)=1.0f; dq(1,0)=0.5f * x(6,0); dq(2,0)=0.5f * x(7,0); dq(3,0)=0.5f * x(8,0);
    cquat::normalize_quat(dq); Vector4 q_new; cquat::multiply_quat(q_in, dq, q_new); cquat::normalize_quat(q_new);
    state.q[0]=q_new(0,0); state.q[1]=q_new(1,0); state.q[2]=q_new(2,0); state.q[3]=q_new(3,0);
    for (int i = 0; i < 3; ++i) state.ba[i] = x(9+i,0);
    for (int i = 0; i < 3; ++i) state.bg[i] = x(12+i,0);
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) state.P[i*15 + j] = P(i,j);

    // Diagnostics
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 3; ++j) output.last_K[i*3 + j] = 0.0f;
    for (int i = 0; i < 15; ++i) output.last_K[i*3 + 0] = K_out(i,0);
    output.last_S[0] = S_out(0,0);
    output.last_y[0] = y_out(0,0); output.last_y_len = 1; output.last_sensor_type = 4; // baro
}

void MEUKFCore::update_zupt_meukf_ukf_version(State& state, const Params& params, MEUKFOutput& output)
{
    // UKF-backed ZUPT: measurement is zero velocity (v_x,v_y,v_z)
    using UKF3 = ukf::UKFUpdate<15, 3, float>;
    using Vector15 = UKF3::VectorN;
    using Matrix15x15 = UKF3::MatrixNN;
    using Matrix15x3 = UKF3::MatrixNM;
    using Matrix3x3 = UKF3::MatrixMM;
    using Vector3f = UKF3::VectorM;

    Vector15 x; Matrix15x15 P;
    for (int i = 0; i < 3; ++i) x(i,0) = state.p[i];
    for (int i = 0; i < 3; ++i) x(3+i,0) = state.v[i];
    for (int i = 0; i < 3; ++i) x(6+i,0) = 0.0f;
    for (int i = 0; i < 3; ++i) x(9+i,0) = state.ba[i];
    for (int i = 0; i < 3; ++i) x(12+i,0) = state.bg[i];
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) P(i,j) = state.P[i*15 + j];

    Vector3f z; z(0,0)=0.0f; z(1,0)=0.0f; z(2,0)=0.0f;

    auto h_func_zupt = [](const Vector15& xv) {
        Vector3f zv;
        zv(0,0) = xv(3,0);
        zv(1,0) = xv(4,0);
        zv(2,0) = xv(5,0);
        return zv;
    };

    Matrix3x3 R = Matrix3x3::Zero();
    for (int i = 0; i < 3; ++i) R(i,i) = params.noise_zupt[i];
    ukf::UKFParams up; up.alpha = params.alpha; up.beta = params.beta; up.kappa = params.kappa;

    Matrix15x3 K_out; Matrix3x3 S_out; Vector3f y_out;
    bool ok = UKF3::update(x, P, z, h_func_zupt, R, up, &K_out, &S_out, &y_out);
    if (!ok) { output.status = 1; return; }

    for (int i = 0; i < 3; ++i) state.p[i] = x(i,0);
    for (int i = 0; i < 3; ++i) state.v[i] = x(3+i,0);
    Vector4 q_in; q_in(0,0)=state.q[0]; q_in(1,0)=state.q[1]; q_in(2,0)=state.q[2]; q_in(3,0)=state.q[3];
    Vector4 dq; dq(0,0)=1.0f; dq(1,0)=0.5f * x(6,0); dq(2,0)=0.5f * x(7,0); dq(3,0)=0.5f * x(8,0);
    cquat::normalize_quat(dq); Vector4 q_new; cquat::multiply_quat(q_in, dq, q_new); cquat::normalize_quat(q_new);
    state.q[0]=q_new(0,0); state.q[1]=q_new(1,0); state.q[2]=q_new(2,0); state.q[3]=q_new(3,0);
    for (int i = 0; i < 3; ++i) state.ba[i] = x(9+i,0);
    for (int i = 0; i < 3; ++i) state.bg[i] = x(12+i,0);
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) state.P[i*15 + j] = P(i,j);

    // Diagnostics
    for (int i = 0; i < 15*3; ++i) output.last_K[i] = 0.0f;
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 3; ++j) output.last_K[i*3 + j] = K_out(i,j);
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) output.last_S[i*3 + j] = S_out(i,j);
    for (int i = 0; i < 3; ++i) output.last_y[i] = y_out(i,0);
    output.last_y_len = 3; output.last_sensor_type = 5; // zupt
}
// Debug helper: control logging via env var MEUKF_DEBUG_LEVEL (0=off, 1=minimal, 2=verbose)
static int get_debug_level() {
    const char* s = std::getenv("MEUKF_DEBUG_LEVEL");
    if (!s) return 0;
    int v = 0;
    try { v = std::atoi(s); } catch(...) { v = 0; }
    return v;
}

// Helper to create Vector3
static Vector3 make_vector3(double x, double y, double z) {
    Vector3 v;
    v(0,0) = x; v(1,0) = y; v(2,0) = z;
    return v;
}

// Helper to create Vector2
static Vector2 make_vector2(float x, float y) {
    Vector2 v;
    v(0,0) = x; v(1,0) = y;
    return v;
}

// Helper to create Vector4
static Vector4 make_vector4(double w, double x, double y, double z) {
    Vector4 v;
    v(0,0) = w; v(1,0) = x; v(2,0) = y; v(3,0) = z;
    return v;
}

// Helper to calculate norm of Vector3
static double vector3_norm(const Vector3& v) {
    return std::sqrt(v(0,0)*v(0,0) + v(1,0)*v(1,0) + v(2,0)*v(2,0));
}

// Helper for Cholesky Decomposition (3x3)
// Returns true if successful, false if not positive definite
static bool cholesky3x3(const Matrix3x3& A, Matrix3x3& L) {
    float l[3][3] = {0};
    float a[3][3];
    
    for(int i=0; i<3; ++i)
        for(int j=0; j<3; ++j)
            a[i][j] = A(i,j);

    // Initialize L to zero
    for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) L(i,j) = 0.0f;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j <= i; j++) {
            float sum = 0;
            for (int k = 0; k < j; k++) {
                sum += l[i][k] * l[j][k];
            }

            if (i == j) {
                float val = a[i][i] - sum;
                if (val <= 1e-9f) return false; // floatに適した閾値
                l[i][j] = std::sqrt(val);
            } else {
                l[i][j] = (1.0f / l[j][j] * (a[i][j] - sum));
            }
        }
    }

    for(int i=0; i<3; ++i)
        for(int j=0; j<3; ++j)
            L(i,j) = l[i][j];
            
    return true;
}

// 堅牢なCholesky分解（MATLAB実装に合わせた多段フォールバック）
static bool cholesky3x3_robust(Matrix3x3& A, Matrix3x3& L) {
    // 1. 対称化
    for(int i=0; i<3; ++i) {
        for(int j=i+1; j<3; ++j) {
            float avg = (A(i,j) + A(j,i)) / 2.0f;
            A(i,j) = avg;
            A(j,i) = avg;
        }
    }
    
    // 2. 最小固有値チェック（簡易版: 対角要素の最小値で近似）
    float min_diag = A(0,0);
    for(int i=1; i<3; ++i) {
        if (A(i,i) < min_diag) min_diag = A(i,i);
    }
    
    // 3. 正則化（正定値でない場合）
    if (min_diag <= 0.0f) {
        float reg = std::abs(min_diag) + 1e-6f;
        for(int i=0; i<3; ++i) A(i,i) += reg;
    }
    
    // 4. Cholesky分解
    if (cholesky3x3(A, L)) {
        return true;
    }
    
    // 5. より強い正則化で再試行
    for(int i=0; i<3; ++i) A(i,i) += 1e-4f;
    if (cholesky3x3(A, L)) {
        return true;
    }
    
    // 6. 最終手段: 対角近似
    L = Matrix3x3::Zero();
    for(int i=0; i<3; ++i) {
        L(i,i) = std::sqrt(std::max(0.0f, A(i,i)));
    }
    return true;
}

// 共分散行列の正定値化
static void ensure_positive_definite(Matrix3x3& P) {
    // 1. 対称化
    for(int i=0; i<3; ++i) {
        for(int j=i+1; j<3; ++j) {
            float avg = (P(i,j) + P(j,i)) / 2.0f;
            P(i,j) = avg;
            P(j,i) = avg;
        }
    }
    
    // 2. 固有値チェック（簡易版: 対角要素の最小値）
    float min_diag = P(0,0);
    for(int i=1; i<3; ++i) {
        if (P(i,i) < min_diag) min_diag = P(i,i);
    }
    
    // 3. 正則化
    if (min_diag <= 0.0f) {
        float reg = std::abs(min_diag) + 1e-8f;
        for(int i=0; i<3; ++i) P(i,i) += reg;
        
        // 再度対称化
        for(int i=0; i<3; ++i) {
            for(int j=i+1; j<3; ++j) {
                float avg = (P(i,j) + P(j,i)) / 2.0f;
                P(i,j) = avg;
                P(j,i) = avg;
            }
        }
    }
}

void MEUKFCore::step(const MEUKFInput& input, MEUKFOutput& output) {
    // Initialize output
    output.new_state = input.prev_state;
    output.status = 0;
    for(int i=0; i<10; ++i) output.debug_info[i] = 0.0;
    // Initialize optional debug outputs
    for(int i=0;i<15*3;++i) output.last_K[i] = 0.0f;
    for(int i=0;i<3;++i) output.last_y[i] = 0.0f;
    output.last_y_len = 0;
    output.last_sensor_type = 0;
    for(int i=0;i<15*15;++i) output.pred_P[i] = 0.0f;

    // 1. Prediction Step
    if (input.sensor.dt > 0.0) {
        predict(output.new_state, input.sensor, input.params);
    }

    // Capture predicted covariance (P) immediately after predict() and before any updates
    for(int i=0;i<15*15;++i) {
        output.pred_P[i] = output.new_state.P[i];
    }

    // 2. Update Step
    
    // Accel Update (MEUKF)
    if (input.sensor.update_accel) {
        Vector3 a_meas = make_vector3(input.sensor.accel[0], input.sensor.accel[1], input.sensor.accel[2]);
        // Use UKF-backed accel update unconditionally (MEUKF replaced by UKF library)
        update_accel_meukf_ukf_version(output.new_state, a_meas, input.params, output);
    }

    // Mag Update (MEUKF)
    if (input.sensor.update_mag) {
        Vector3 m_meas = make_vector3(input.sensor.mag[0], input.sensor.mag[1], input.sensor.mag[2]);
        // Replaced: use UKF-backed mag update
        update_mag_meukf_ukf_version(output.new_state, m_meas, input.params, output);
    }

    // GPS Update
    if (input.sensor.update_gps) {
        Vector3 gps_meas = make_vector3(input.sensor.gps_pos[0], input.sensor.gps_pos[1], input.sensor.gps_pos[2]);
        update_gps_meukf_ukf_version(output.new_state, gps_meas, input.params, output);
    }

    // Baro Update
    if (input.sensor.update_baro) {
        float alt_baro = input.sensor.alt_baro;
        update_baro_meukf_ukf_version(output.new_state, alt_baro, input.params, output);
    }

    // ZUPT Update (常に実行)
    if (input.sensor.update_zupt) {
        update_zupt_meukf_ukf_version(output.new_state, input.params, output);
    }
}

void MEUKFCore::state_to_vars(const State& s, Vector3& p, Vector3& v, Vector4& q, Vector3& ba, Vector3& bg, Matrix15x15& P) {
    p(0,0) = s.p[0]; p(1,0) = s.p[1]; p(2,0) = s.p[2];
    v(0,0) = s.v[0]; v(1,0) = s.v[1]; v(2,0) = s.v[2];
    q(0,0) = s.q[0]; q(1,0) = s.q[1]; q(2,0) = s.q[2]; q(3,0) = s.q[3];
    ba(0,0) = s.ba[0]; ba(1,0) = s.ba[1]; ba(2,0) = s.ba[2];
    bg(0,0) = s.bg[0]; bg(1,0) = s.bg[1]; bg(2,0) = s.bg[2];
    
    for(int i=0; i<15; ++i) {
        for(int j=0; j<15; ++j) {
            P(i, j) = s.P[i*15 + j];
        }
    }
}

bool MEUKFCore::compare_accel_updates(const State& init_state, const SensorData& sensor, const Params& params, MEUKFOutput& out_orig, MEUKFOutput& out_ukf) {
    // Comparison helper disabled: original legacy accel update is deprecated.
    // Provide a minimal fallback: mark original as not-run and run UKF-backed update.
    out_orig.new_state = init_state;
    out_orig.status = 1; // deprecated

    MEUKFOutput tmp_ukf; tmp_ukf.new_state = init_state; tmp_ukf.status = 0;
    Vector3 a_meas = make_vector3(sensor.accel[0], sensor.accel[1], sensor.accel[2]);
    try {
        update_accel_meukf_ukf_version(tmp_ukf.new_state, a_meas, params, tmp_ukf);
    } catch(...) { tmp_ukf.status = 1; }
    out_ukf = tmp_ukf;
    return true;
}

void MEUKFCore::vars_to_state(const Vector3& p, const Vector3& v, const Vector4& q, const Vector3& ba, const Vector3& bg, const Matrix15x15& P, State& s) {
    s.p[0] = p(0,0); s.p[1] = p(1,0); s.p[2] = p(2,0);
    s.v[0] = v(0,0); s.v[1] = v(1,0); s.v[2] = v(2,0);
    s.q[0] = q(0,0); s.q[1] = q(1,0); s.q[2] = q(2,0); s.q[3] = q(3,0);
    s.ba[0] = ba(0,0); s.ba[1] = ba(1,0); s.ba[2] = ba(2,0);
    s.bg[0] = bg(0,0); s.bg[1] = bg(1,0); s.bg[2] = bg(2,0);
    
    for(int i=0; i<15; ++i) {
        for(int j=0; j<15; ++j) {
            s.P[i*15 + j] = P(i, j);
        }
    }
}

void MEUKFCore::predict(State& state, const SensorData& sensor, const Params& params) {
    Vector3 p, v, ba, bg;
    Vector4 q;
    Matrix15x15 P;
    state_to_vars(state, p, v, q, ba, bg, P);

    float dt = sensor.dt;
    Vector3 a_meas = make_vector3(sensor.accel[0], sensor.accel[1], sensor.accel[2]);
    Vector3 w_meas = make_vector3(sensor.gyro[0], sensor.gyro[1], sensor.gyro[2]);
    Vector3 g = make_vector3(params.g[0], params.g[1], params.g[2]);

    // 1. Nominal State Update
    // Attitude Update
    Vector3 w_corrected = w_meas - bg;
    
    // Quaternion integration
    float w_norm = vector3_norm(w_corrected);
    
    Vector4 dq;
    if (w_norm * dt < 1e-9f) {
        dq = make_vector4(1.0f, 0.0f, 0.0f, 0.0f);
    } else {
        float half_angle = w_norm * dt * 0.5f;
        float s = std::sin(half_angle);
        dq(0,0) = std::cos(half_angle);
        dq(1,0) = (w_corrected(0,0)/w_norm) * s;
        dq(2,0) = (w_corrected(1,0)/w_norm) * s;
        dq(3,0) = (w_corrected(2,0)/w_norm) * s;
    }
    
    Vector4 q_new;
    cquat::multiply_quat(q, dq, q_new);
    cquat::normalize_quat(q_new);

    // Velocity and Position Update
    // 重要: 更新後のクォータニオンq_newを使用して回転行列を計算
    Matrix3x3 R;
    cquat::quat_to_rotm(q_new, R);  // q -> q_new に修正
    Vector3 a_corrected = a_meas - ba;
    // a_meas is proper acceleration (includes reaction to gravity).
    // a_kinematic = R * a_meas + g (where g is [0,0,-9.8])
    Vector3 a_world = R * a_corrected + g;
    
    Vector3 p_new = p + v * dt + a_world * (0.5 * dt * dt);
    Vector3 v_new = v + a_world * dt;

    // 2. Error Covariance Prediction
    Matrix15x15 F = Matrix15x15::Identity();
    
    // Position derivatives
    for(int i=0; i<3; ++i) F(i, 3+i) = dt; // dp/dv
    
    // Velocity derivatives
    // dv/dtheta = -R * [a_corrected]_x * dt
    Matrix3x3 a_skew;
    a_skew(0, 0) = 0; a_skew(0, 1) = -a_corrected(2,0); a_skew(0, 2) = a_corrected(1,0);
    a_skew(1, 0) = a_corrected(2,0); a_skew(1, 1) = 0; a_skew(1, 2) = -a_corrected(0,0);
    a_skew(2, 0) = -a_corrected(1,0); a_skew(2, 1) = a_corrected(0,0); a_skew(2, 2) = 0;
    
    // Note: The derivation of dv/dtheta depends on the definition of error state.
    // If error is defined as global perturbation: p_true = p + dp, v_true = v + dv, R_true = R * (I + [dtheta]x)
    // Then a_world_true = R_true * a_corrected + g
    //                   = R * (I + [dtheta]x) * a_corrected + g
    //                   = R * a_corrected + R * [dtheta]x * a_corrected + g
    //                   = a_world + R * (-[a_corrected]x * dtheta)
    // So dv = R * (-[a_corrected]x) * dtheta * dt
    // This matches the code below.
    
    Matrix3x3 dv_dtheta = R * a_skew * (-dt);
    for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) F(3+i, 6+j) = dv_dtheta(i, j);
    
    // dv/dba = -R * dt
    Matrix3x3 dv_dba = R * (-dt);
    for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) F(3+i, 9+j) = dv_dba(i, j);

    // Attitude derivatives
    // dtheta/dtheta = I - [w_corrected]_x * dt
    Matrix3x3 w_skew;
    w_skew(0, 0) = 0; w_skew(0, 1) = -w_corrected(2,0); w_skew(0, 2) = w_corrected(1,0);
    w_skew(1, 0) = w_corrected(2,0); w_skew(1, 1) = 0; w_skew(1, 2) = -w_corrected(0,0);
    w_skew(2, 0) = -w_corrected(1,0); w_skew(2, 1) = w_corrected(0,0); w_skew(2, 2) = 0;
    
    Matrix3x3 dtheta_dtheta = Matrix3x3::Identity() - w_skew * dt;
    for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) F(6+i, 6+j) = dtheta_dtheta(i, j);
    
    // dtheta/dbg = -I * dt
    for(int i=0; i<3; ++i) F(6+i, 12+i) = -dt;

    // Process Noise Q
    // MATLAB側で既にQから逆算した分散が渡されるため、dtでスケーリング
    // MATLAB: params_struct.noise_accel = diag(Q_adapted(4:6, 4:6)) / (obj.dt^2);
    // C++: Q(4:6, 4:6) = noise_accel * dt^2
    Matrix15x15 Q = Matrix15x15::Zero();
    float dt2 = dt * dt;
    for(int i=0; i<3; ++i) {
        Q(3+i, 3+i) = params.noise_accel[i] * dt2;  // 速度: σ^2 * dt^2
        Q(6+i, 6+i) = params.noise_gyro[i] * dt2;   // 姿勢: σ^2 * dt^2
        Q(9+i, 9+i) = params.noise_ba[i] * dt;      // 加速度バイアス: σ^2 * dt
        Q(12+i, 12+i) = params.noise_bg[i] * dt;    // ジャイロバイアス: σ^2 * dt
    }

    // P = F * P * F^T + Q
    Matrix15x15 P_new = F * P * F.transpose() + Q;

    // Debug: check for NaN or extremely large values in P_new and dump context if found (only if debug enabled)
    try {
        int dbg = get_debug_level();
        if (dbg >= 1) {
            bool any_nan = false;
            double max_abs = 0.0;
            for(int i=0;i<15;++i) {
                for(int j=0;j<15;++j) {
                    double val = static_cast<double>(P_new(i,j));
                    if (!std::isfinite(val)) any_nan = true;
                    double av = std::abs(val);
                    if (av > max_abs) max_abs = av;
                }
            }
            // DEBUG DISABLED: predict_debug output - not needed for current analysis
            // if (any_nan || max_abs > 1e6) {
            //     std::ofstream dbgfile("Results/predict_debug.txt", std::ios::app);
            //     ... (commented out)
            // }
        }
    } catch(...) {}

    vars_to_state(p_new, v_new, q_new, ba, bg, P_new, state);
}

void MEUKFCore::update_accel_meukf_ukf_version(State& state, const Vector3& a_meas, const Params& params, MEUKFOutput& output) {
    // Extract state parts
    Vector3 p, v, ba, bg;
    Vector4 q_nom;
    Matrix15x15 P_full;
    state_to_vars(state, p, v, q_nom, ba, bg, P_full);

    // Preserve pre-update full covariance for diagnostics
    Matrix15x15 P_full_pre = P_full;

    // Attitude covariance block (3x3) — indices 6..8 in the 15x15 state ordering
    Matrix3x3 P_att;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) P_att(i, j) = P_full(6 + i, 6 + j);

    // UKF parameters: use MEUKF params to keep behavior consistent
    ukf::UKFParams uparams;
    uparams.alpha = params.alpha;
    uparams.beta = params.beta;
    uparams.kappa = params.kappa;

    // Use 2D observation (x,y) to match original MEUKF accel update behavior
    using UKF = ukf::UKFUpdate<3, 2, float>;

    // Initial small-angle state (3x1) — zero mean
    cmath_fx::Vector<3,float> x_err = cmath_fx::Vector<3,float>::Zero();

    // Observation function: maps small-angle error vector -> predicted accel 2D (x,y)
    auto h_func_2d = [q_nom, params](const cmath_fx::Vector<3,float>& dtheta) -> cmath_fx::Vector<2,float> {
        float dx = dtheta(0,0); float dy = dtheta(1,0); float dz = dtheta(2,0);
        float angle = std::sqrt(dx*dx + dy*dy + dz*dz);
        cmath_fx::Vector<4,float> dq;
        if (angle < 1e-9f) {
            dq(0,0) = 1.0f; dq(1,0) = 0.0f; dq(2,0) = 0.0f; dq(3,0) = 0.0f;
        } else {
            float s = std::sin(angle * 0.5f);
            dq(0,0) = std::cos(angle * 0.5f);
            dq(1,0) = (dx/angle) * s;
            dq(2,0) = (dy/angle) * s;
            dq(3,0) = (dz/angle) * s;
        }

        cmath_fx::Vector<4,float> q_i;
        cquat::multiply_quat(q_nom, dq, q_i);
        cquat::normalize_quat(q_i);

        cmath_fx::Matrix<3,3,float> Rm;
        cquat::quat_to_rotm(q_i, Rm);
        cmath_fx::Vector<3,float> g_vec;
        g_vec(0,0) = params.g[0]; g_vec(1,0) = params.g[1]; g_vec(2,0) = params.g[2];
        cmath_fx::Vector<3,float> a_pred = (Rm.transpose() * g_vec) * -1.0f;

        cmath_fx::Vector<2,float> z2;
        z2(0,0) = a_pred(0,0);
        z2(1,0) = a_pred(1,0);
        return z2;
    };

    // Dynamic R scaling per original MEUKF: scale based on gravity deviation and floor
    float a_norm = std::sqrt(a_meas(0,0)*a_meas(0,0) + a_meas(1,0)*a_meas(1,0) + a_meas(2,0)*a_meas(2,0));
    float gravity_deviation = std::abs(a_norm - std::sqrt(params.g[0]*params.g[0] + params.g[1]*params.g[1] + params.g[2]*params.g[2]));
    float R_scale = 1.0f + (gravity_deviation / 0.7f);
    float R_floor = 0.25f;

    cmath_fx::Matrix<2,2,float> R2 = cmath_fx::Matrix<2,2,float>::Zero();
    for (int i=0;i<2;i++) {
        float R_est = params.noise_accel[i];
        R2(i,i) = std::max(R_est, R_floor) * R_scale;
    }

    // Perform UKF update (2D observation)
    cmath_fx::Matrix<2,2,float> S_out;
    cmath_fx::Matrix<3,2,float> K_out;
    cmath_fx::Vector<2,float> y_out;

    // Build 2D measurement vector from a_meas (x,y)
    cmath_fx::Vector<2,float> z_meas2;
    z_meas2(0,0) = a_meas(0,0);
    z_meas2(1,0) = a_meas(1,0);

    // Call UKF on copies so we keep original P_att/x for manual post-processing
    cmath_fx::Vector<3,float> x_tmp = x_err;
    cmath_fx::Matrix<3,3,float> P_att_tmp;
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) P_att_tmp(i,j) = P_att(i,j);

    bool ok = UKF::update(x_tmp, P_att_tmp, z_meas2, h_func_2d, R2, uparams, &K_out, &S_out, &y_out);
    if (!ok) {
        output.status = 1;
        return;
    }

    // Compute S_inv
    cmath_fx::Matrix<2,2,float> S2_inv;
    if (!S_out.inverse(S2_inv)) {
        output.status = 1;
        return;
    }

    // Reconstruct H_sub and P_cross (as in original path)
    Matrix3x3 Rmat;
    cquat::quat_to_rotm(q_nom, Rmat);
    Vector3 g_body = Rmat.transpose() * make_vector3(params.g[0], params.g[1], params.g[2]);
    Matrix3x3 g_skew;
    g_skew(0, 0) = 0; g_skew(0, 1) = -g_body(2,0); g_skew(0, 2) = g_body(1,0);
    g_skew(1, 0) = g_body(2,0); g_skew(1, 1) = 0; g_skew(1, 2) = -g_body(0,0);
    g_skew(2, 0) = -g_body(1,0); g_skew(2, 1) = g_body(0,0); g_skew(2, 2) = 0;
    Matrix3x3 H_att = g_skew * -1.0f;
    Matrix2x3 H_sub;
    for(int r=0; r<2; ++r) for(int c=0; c<3; ++c) H_sub(r, c) = H_att(r, c);

    Matrix15x3 P_cross;
    for(int r=0; r<15; ++r) for(int c=0; c<3; ++c) P_cross(r, c) = P_full(r, 6 + c);

    // Build full K using P_cross and S2_inv (matches original MEUKF approach)
    Matrix15x2 tmp = P_cross * H_sub.transpose();
    Matrix15x2 K_full = tmp * S2_inv;

    // Innovation vector from UKF output
    cmath_fx::Vector<2,float> y2 = y_out;

    // Innovation limit (0.05 rad) and mahalanobis checks (5.0 then 2.5 attenuation)
    float max_innovation = 0.05f;
    float innov_norm = std::sqrt(y2(0,0)*y2(0,0) + y2(1,0)*y2(1,0));
    if (innov_norm > max_innovation) {
        float scale = max_innovation / innov_norm;
        y2(0,0) *= scale; y2(1,0) *= scale;
    }
    // Mahalanobis
    cmath_fx::Vector<2,float> S2_inv_y = S2_inv * y2;
    float mahal_sq = y2(0,0)*S2_inv_y(0,0) + y2(1,0)*S2_inv_y(1,0);
    float mahal = std::sqrt(mahal_sq);
    if (mahal > 5.0f) { output.status = 1; return; }
    if (mahal > 2.5f) {
        float att = 2.5f / mahal; y2(0,0) *= att; y2(1,0) *= att;
    }

    // Small-angle update (use K_out returned by UKF)
    cmath_fx::Matrix<3,2,float> K_small = K_out;
    cmath_fx::Vector<3,float> dx_small = K_small * y2;

    // dtheta magnitude limit: 0.6 deg for roll/pitch, and force yaw=0
    float dtheta_norm = std::sqrt(dx_small(0,0)*dx_small(0,0) + dx_small(1,0)*dx_small(1,0));
    float max_dtheta = 0.6f * 3.14159265f / 180.0f;
    if (dtheta_norm > max_dtheta) {
        float sc = max_dtheta / dtheta_norm; dx_small(0,0) *= sc; dx_small(1,0) *= sc;
    }
    dx_small(2,0) = 0.0f; // force yaw zero

    // Update attitude covariance block using K_small and S_out
    cmath_fx::Matrix<3,3,float> P_att_upd = P_att;
    // Compute P_att_upd = P_att - K_small * S_out * K_small'
    cmath_fx::Matrix<3,2,float> KS = K_small * S_out;
    cmath_fx::Matrix<3,3,float> KSKt = KS * K_small.transpose();
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) P_att_upd(i,j) = P_att(i,j) - KSKt(i,j);
    // Symmetrize and ensure PD
    for (int i=0;i<3;++i) for (int j=i+1;j<3;++j) {
        float avg = (P_att_upd(i,j) + P_att_upd(j,i)) * 0.5f; P_att_upd(i,j)=avg; P_att_upd(j,i)=avg;
    }
    ensure_positive_definite(P_att_upd);

    // Apply full-state update dx_full = K_full * y2
    Vector15 dx_full;
    for(int r=0; r<15; ++r) dx_full(r,0) = K_full(r,0) * y2(0,0) + K_full(r,1) * y2(1,0);

    p = p + make_vector3(dx_full(0,0), dx_full(1,0), dx_full(2,0));
    v = v + make_vector3(dx_full(3,0), dx_full(4,0), dx_full(5,0));
    ba = ba + make_vector3(dx_full(9,0), dx_full(10,0), dx_full(11,0));
    bg = bg + make_vector3(dx_full(12,0), dx_full(13,0), dx_full(14,0));

    // Joseph-form full covariance update using K_full and R2
    Matrix15x15 KH_full = Matrix15x15::Zero();
    for(int i=0;i<15;++i) for(int j=0;j<15;++j) {
        float sum=0.0f; for(int k=0;k<3;++k) { if (j>=6 && j<9) sum += K_full(i,k) * H_sub(k, j-6); } KH_full(i,j)=sum;
    }
    Matrix15x15 I = Matrix15x15::Identity();
    Matrix15x15 I_KH = I - KH_full;
    Matrix15x15 P_tmp = I_KH * P_full * I_KH.transpose();
    Matrix15x15 KRKt = Matrix15x15::Zero();
    for(int i=0;i<15;++i) for(int j=0;j<15;++j) {
        float sum2=0.0f; for(int k_idx=0;k_idx<2;++k_idx) for(int l_idx=0;l_idx<2;++l_idx) sum2 += K_full(i,k_idx) * R2(k_idx,l_idx) * K_full(j,l_idx);
        KRKt(i,j)=sum2;
    }
    P_full = P_tmp + KRKt;
    P_full = (P_full + P_full.transpose()) * 0.5f;

    // Quaternion injection from dx_small
    float dxs = dx_small(0,0), dys = dx_small(1,0), dzs = dx_small(2,0);
    float ang = std::sqrt(dxs*dxs + dys*dys + dzs*dzs);
    cmath_fx::Vector<4,float> dqv;
    if (ang < 1e-9f) { dqv(0,0)=1.0f; dqv(1,0)=dqv(2,0)=dqv(3,0)=0.0f; }
    else { float s=std::sin(ang*0.5f); dqv(0,0)=std::cos(ang*0.5f); dqv(1,0)=(dxs/ang)*s; dqv(2,0)=(dys/ang)*s; dqv(3,0)=(dzs/ang)*s; }
    cmath_fx::Vector<4,float> q_updated_v; cquat::multiply_quat(q_nom, dqv, q_updated_v); cquat::normalize_quat(q_updated_v);

    // Write back updated P_att block into P_full
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) P_full(6+i,6+j) = P_att_upd(i,j);

    // Diagnostics: save pre-update P, K_full, S_out, innovation
    for (int r=0;r<15;++r) for (int c=0;c<15;++c) output.pred_P[r*15 + c] = P_full_pre(r,c);
    for (int r=0;r<15;++r) {
        output.last_K[r*3 + 0] = K_full(r,0);
        output.last_K[r*3 + 1] = K_full(r,1);
        output.last_K[r*3 + 2] = 0.0f;
    }
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) output.last_S[i*3 + j] = 0.0f;
    output.last_S[0*3 + 0] = S_out(0,0);
    output.last_S[0*3 + 1] = S_out(0,1);
    output.last_S[1*3 + 0] = S_out(1,0);
    output.last_S[1*3 + 1] = S_out(1,1);
    for (int i=0;i<3;i++) output.last_y[i]=0.0f;
    output.last_y[0]=y2(0,0); output.last_y[1]=y2(1,0); output.last_y_len=2; output.last_sensor_type=1;

    vars_to_state(p, v, q_updated_v, ba, bg, P_full, state);
}

void MEUKFCore::update_mag_meukf_ukf_version(State& state, const Vector3& m_meas, const Params& params, MEUKFOutput& output) {
    // Extract state parts
    Vector3 p, v, ba, bg;
    Vector4 q_nom;
    Matrix15x15 P_full;
    state_to_vars(state, p, v, q_nom, ba, bg, P_full);

    // Preserve pre-update full covariance for diagnostics
    Matrix15x15 P_full_pre = P_full;

    // Attitude covariance block
    Matrix3x3 P_att;
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) P_att(i,j) = P_full(6+i, 6+j);

    // UKF params
    ukf::UKFParams uparams;
    uparams.alpha = params.alpha;
    uparams.beta = params.beta;
    uparams.kappa = params.kappa;

    using UKF3 = ukf::UKFUpdate<3,3,float>;

    // small-angle zero mean
    cmath_fx::Vector<3,float> x_err = cmath_fx::Vector<3,float>::Zero();

    // observation function: small-angle -> predicted mag (3D)
    auto h_func_3d = [q_nom, params](const cmath_fx::Vector<3,float>& dtheta) -> cmath_fx::Vector<3,float> {
        float dx = dtheta(0,0), dy = dtheta(1,0), dz = dtheta(2,0);
        float angle = std::sqrt(dx*dx + dy*dy + dz*dz);
        cmath_fx::Vector<4,float> dq;
        if (angle < 1e-9f) { dq(0,0)=1; dq(1,0)=dq(2,0)=dq(3,0)=0; }
        else { float s = std::sin(angle*0.5f); dq(0,0)=std::cos(angle*0.5f); dq(1,0)=(dx/angle)*s; dq(2,0)=(dy/angle)*s; dq(3,0)=(dz/angle)*s; }
        cmath_fx::Vector<4,float> q_i; cquat::multiply_quat(q_nom, dq, q_i); cquat::normalize_quat(q_i);
        cmath_fx::Matrix<3,3,float> Rm; cquat::quat_to_rotm(q_i, Rm);
        cmath_fx::Vector<3,float> mag_ref;
        mag_ref(0,0) = params.mag_ref[0]; mag_ref(1,0) = params.mag_ref[1]; mag_ref(2,0) = params.mag_ref[2];
        // predicted magnetometer reading in body frame
        cmath_fx::Vector<3,float> m_pred = Rm.transpose() * mag_ref;
        return m_pred;
    };

    // Measurement noise R (3x3)
    cmath_fx::Matrix<3,3,float> R3 = cmath_fx::Matrix<3,3,float>::Zero();
    for (int i=0;i<3;++i) R3(i,i) = std::max(params.noise_mag[i], 1e-6f);

    cmath_fx::Matrix<3,3,float> S_out;
    cmath_fx::Matrix<3,3,float> K_out;
    cmath_fx::Vector<3,float> y_out;

    cmath_fx::Vector<3,float> z_meas;
    z_meas(0,0) = m_meas(0,0); z_meas(1,0) = m_meas(1,0); z_meas(2,0) = m_meas(2,0);

    cmath_fx::Vector<3,float> x_tmp = x_err;
    cmath_fx::Matrix<3,3,float> P_att_tmp;
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) P_att_tmp(i,j) = P_att(i,j);

    bool ok = UKF3::update(x_tmp, P_att_tmp, z_meas, h_func_3d, R3, uparams, &K_out, &S_out, &y_out);
    if (!ok) { output.status = 1; return; }

    // compute S_inv
    cmath_fx::Matrix<3,3,float> S_inv;
    if (!S_out.inverse(S_inv)) { output.status = 1; return; }

    // Build H_sub (3x3) as -skew(R^T*mag_ref) used in original path
    Matrix3x3 Rmat; cquat::quat_to_rotm(q_nom, Rmat);
    Vector3 mag_ref_vec = make_vector3(params.mag_ref[0], params.mag_ref[1], params.mag_ref[2]);
    Vector3 m_body = Rmat.transpose() * mag_ref_vec;
    Matrix3x3 m_skew;
    m_skew(0, 0) = 0; m_skew(0, 1) = -m_body(2,0); m_skew(0, 2) = m_body(1,0);
    m_skew(1, 0) = m_body(2,0); m_skew(1, 1) = 0; m_skew(1, 2) = -m_body(0,0);
    m_skew(2, 0) = -m_body(1,0); m_skew(2, 1) = m_body(0,0); m_skew(2, 2) = 0;
    Matrix3x3 H_sub = m_skew;

    // P_cross and K_full
    Matrix15x3 P_cross; for(int r=0;r<15;++r) for(int c=0;c<3;++c) P_cross(r,c) = P_full(r,6+c);
    Matrix15x3 tmp = P_cross * H_sub.transpose();
    Matrix15x3 K_full = tmp * S_inv;

    // Innovation vector
    cmath_fx::Vector<3,float> y3 = y_out;
    // Mahalanobis checks similar to original
    cmath_fx::Vector<3,float> S_inv_y = S_inv * y3;
    float mahal_sq = y3(0,0)*S_inv_y(0,0) + y3(1,0)*S_inv_y(1,0) + y3(2,0)*S_inv_y(2,0);
    float mahal = std::sqrt(mahal_sq);
    if (mahal > 5.0f) { output.status = 1; return; }
    if (mahal > 2.5f) { float att = 2.5f/mahal; y3(0,0)*=att; y3(1,0)*=att; y3(2,0)*=att; }

    // Small-angle update (K_out is 3x3 here)
    cmath_fx::Matrix<3,3,float> K_small = K_out;
    cmath_fx::Vector<3,float> dx_small = K_small * y3;

    // Attitude covariance update: P_att_upd = P_att - K_small * S_out * K_small'
    cmath_fx::Matrix<3,3,float> KS = K_small * S_out;
    cmath_fx::Matrix<3,3,float> KSKt = KS * K_small.transpose();
    cmath_fx::Matrix<3,3,float> P_att_upd = P_att;
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) P_att_upd(i,j) = P_att(i,j) - KSKt(i,j);
    for(int i=0;i<3;++i) for(int j=i+1;j<3;++j) { float avg = 0.5f*(P_att_upd(i,j)+P_att_upd(j,i)); P_att_upd(i,j)=avg; P_att_upd(j,i)=avg; }
    ensure_positive_definite(P_att_upd);

    // dx_full and apply to state
    Vector15 dx_full; for(int r=0;r<15;++r) dx_full(r,0) = K_full(r,0)*y3(0,0) + K_full(r,1)*y3(1,0) + K_full(r,2)*y3(2,0);
    p = p + make_vector3(dx_full(0,0), dx_full(1,0), dx_full(2,0));
    v = v + make_vector3(dx_full(3,0), dx_full(4,0), dx_full(5,0));
    ba = ba + make_vector3(dx_full(9,0), dx_full(10,0), dx_full(11,0));
    bg = bg + make_vector3(dx_full(12,0), dx_full(13,0), dx_full(14,0));

    // Joseph-form full covariance update using K_full and R3
    Matrix15x15 KH_full = Matrix15x15::Zero();
    for(int i=0;i<15;++i) for(int j=0;j<15;++j) { float sum=0.0f; if (j>=6 && j<9) { for(int k=0;k<3;++k) sum += K_full(i,k) * H_sub(k, j-6); } KH_full(i,j)=sum; }
    Matrix15x15 I = Matrix15x15::Identity();
    Matrix15x15 I_KH = I - KH_full;
    Matrix15x15 P_tmp = I_KH * P_full * I_KH.transpose();
    Matrix15x15 KRKt = Matrix15x15::Zero();
    for(int i=0;i<15;++i) for(int j=0;j<15;++j) { float sum2=0.0f; for(int k_idx=0;k_idx<3;++k_idx) for(int l_idx=0;l_idx<3;++l_idx) sum2 += K_full(i,k_idx) * R3(k_idx,l_idx) * K_full(j,l_idx); KRKt(i,j)=sum2; }
    P_full = P_tmp + KRKt;
    P_full = (P_full + P_full.transpose()) * 0.5f;

    // Quaternion injection from dx_small
    float dxs=dx_small(0,0), dys=dx_small(1,0), dzs=dx_small(2,0);
    float ang = std::sqrt(dxs*dxs + dys*dys + dzs*dzs);
    Vector4 dqv; if (ang < 1e-9f) { dqv = make_vector4(1,0,0,0);} else { float s = std::sin(ang*0.5f); dqv = make_vector4(std::cos(ang*0.5f),(dxs/ang)*s,(dys/ang)*s,(dzs/ang)*s); }
    Vector4 q_updated; cquat::multiply_quat(q_nom, dqv, q_updated); cquat::normalize_quat(q_updated);

    // write back updated P_att into P_full
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) P_full(6+i,6+j) = P_att_upd(i,j);

    // Diagnostics
    for(int r=0;r<15;++r) for(int c=0;c<15;++c) output.pred_P[r*15 + c] = P_full_pre(r,c);
    for(int r=0;r<15;++r) { output.last_K[r*3 + 0] = K_full(r,0); output.last_K[r*3 + 1] = K_full(r,1); output.last_K[r*3 + 2] = K_full(r,2); }
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) output.last_S[i*3 + j] = 0.0f;
    output.last_S[0*3 + 0] = S_out(0,0); output.last_S[0*3 + 1] = S_out(0,1); output.last_S[0*3 + 2] = S_out(0,2);
    output.last_S[1*3 + 0] = S_out(1,0); output.last_S[1*3 + 1] = S_out(1,1); output.last_S[1*3 + 2] = S_out(1,2);
    output.last_S[2*3 + 0] = S_out(2,0); output.last_S[2*3 + 1] = S_out(2,1); output.last_S[2*3 + 2] = S_out(2,2);
    output.last_y[0]=y3(0,0); output.last_y[1]=y3(1,0); output.last_y[2]=y3(2,0); output.last_y_len=3; output.last_sensor_type=2;

    vars_to_state(p, v, q_updated, ba, bg, P_full, state);
}

} // namespace meukf

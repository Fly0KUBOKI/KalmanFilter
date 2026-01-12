#include "../inc/meukf_core.hpp"
#include "../../Matrix/fixed_matrix.hpp"
#include "../../Core/statistics.hpp"
#include "../../Core/geometry.hpp"
#include "../../Core/numerical.hpp"
#include "../../Quaternion/quaternion_functions.hpp"
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <algorithm>

#include "../inc/meukf_helpers.hpp"

namespace meukf {

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
    
    Matrix3x3 dv_dtheta = R * a_skew * (-dt);
    for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) F(3+i, 6+j) = dv_dtheta(i, j);
    
    // dv/dba = -R * dt
    Matrix3x3 dv_dba = R * (-dt);
    for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) F(3+i, 9+j) = dv_dba(i, j);

    // Attitude derivatives
    Matrix3x3 w_skew;
    w_skew(0, 0) = 0; w_skew(0, 1) = -w_corrected(2,0); w_skew(0, 2) = w_corrected(1,0);
    w_skew(1, 0) = w_corrected(2,0); w_skew(1, 1) = 0; w_skew(1, 2) = -w_corrected(0,0);
    w_skew(2, 0) = -w_corrected(1,0); w_skew(2, 1) = w_corrected(0,0); w_skew(2, 2) = 0;
    
    Matrix3x3 dtheta_dtheta = Matrix3x3::Identity() - w_skew * dt;
    for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) F(6+i, 6+j) = dtheta_dtheta(i, j);
    
    // dtheta/dbg = -I * dt
    for(int i=0; i<3; ++i) F(6+i, 12+i) = -dt;

    // Process Noise Q
    Matrix15x15 Q = Matrix15x15::Zero();
    float dt2 = dt * dt;
    for(int i=0; i<3; ++i) {
        Q(3+i, 3+i) = params.noise_accel[i] * dt2;  // 速度: σ^2 * dt^2
        Q(6+i, 6+i) = params.noise_gyro[i] * dt2;   // 姿勢: σ^2 * dt^2
        Q(9+i, 9+i) = params.noise_ba[i] * dt;      // 加速度バイアス: σ^2 * dt
        Q(12+i, 12+i) = params.noise_bg[i] * dt;    // ジャイロバイアス: σ^2 * dt
    }

    Matrix15x15 P_new = F * P * F.transpose() + Q;

    // Minimal debug NaN/Inf check
    try {
        int dbg = get_debug_level();
        if (dbg >= 1) {
            bool any_nan = false;
            double max_abs = 0.0;
            for(int i=0;i<15;++i) {
                for(int j=0;j<15;++j) {
                    double val = P_new(i,j);
                    if (!std::isfinite(val)) any_nan = true;
                    double av = std::abs(val);
                    if (av > max_abs) max_abs = av;
                }
            }
            (void)any_nan; (void)max_abs;
        }
    } catch(...) {}

    vars_to_state(p_new, v_new, q_new, ba, bg, P_new, state);
}

} // namespace meukf

#include "../../Inc/ESKF/eskf_runner.hpp"
#include "../../Inc/ESKF/eskf_core.hpp"
#include "../../Inc/ESKF/eskf_postprocess.hpp"
#include "../../Inc/Common/Math/quaternion_lib.hpp"
#include "../../Inc/Common/Sensor/sensor_filter.hpp"
#include <cmath>
#include <cstring>
#include <vector>

namespace eskf {

ESKFRunner::ESKFRunner() {
    // Constructor - no initialization needed for static methods
}

void ESKFRunner::predict(ESKFState* s, const double* a_meas, const double* w_meas) {
    using namespace cmath_fx;
    using namespace common::sensor;
    using PredictParams = PredictPostprocessParams;
    using QuatF = quat_lib::Quaternion<float>;
    
    // Convert double to float for ESKFCore (uses float internally)
    Vector<3, float> p_f, v_f, ba_f, bg_f, a_meas_f, w_meas_f, g_f;
    Vector<4, float> q_f;
    Matrix<15, 15, float> P_f, Qnom_f, Qadapt_f, Pnew_f;
    Vector<3, float> gyro_thr_f, accel_thr_f;
    
    // Convert inputs
    for (int i = 0; i < 3; ++i) {
        p_f(i, 0) = static_cast<float>(s->p[i]);
        v_f(i, 0) = static_cast<float>(s->v[i]);
        ba_f(i, 0) = static_cast<float>(s->ba[i]);
        bg_f(i, 0) = static_cast<float>(s->bg[i]);
        a_meas_f(i, 0) = static_cast<float>(a_meas[i]);
        w_meas_f(i, 0) = static_cast<float>(w_meas[i]);
        g_f(i, 0) = static_cast<float>(s->g[i]);
        gyro_thr_f(i, 0) = 0.0f;  // Default thresholds
        accel_thr_f(i, 0) = 0.0f;
    }
    for (int i = 0; i < 4; ++i) {
        q_f(i, 0) = static_cast<float>(s->q[i]);
    }
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            P_f(i, j) = static_cast<float>(s->P[i + j*15]);
            Qnom_f(i, j) = static_cast<float>(s->Q_nominal[i + j*15]);
        }
    }
    
    float dt_f = static_cast<float>(s->dt);
    
    // Adaptive Q scaling
    Qadapt_f = Qnom_f;
    if (s->adaptive_q_enabled) {
        ESKFCore::compute_adaptive_Q(Qnom_f, a_meas_f, w_meas_f, Qadapt_f);
    }
    
    // Integrate nominal state
    ESKFCore::integrate_nominal(p_f, v_f, q_f, ba_f, bg_f, a_meas_f, w_meas_f, dt_f, g_f, gyro_thr_f, accel_thr_f);
    
    // Predict covariance
    ESKFCore::predict_covariance(P_f, q_f, a_meas_f, ba_f, w_meas_f, bg_f, Qadapt_f, dt_f, Pnew_f);
    
    // Ensure symmetry
    for (int i = 0; i < 15; ++i) {
        for (int j = i + 1; j < 15; ++j) {
            float v = 0.5f * (Pnew_f(i, j) + Pnew_f(j, i));
            Pnew_f(i, j) = v;
            Pnew_f(j, i) = v;
        }
    }
    
    // Copy Pnew_f to P_f for postprocessing
    P_f = Pnew_f;
    
    // Copy w_body
    memcpy(s->w_body, w_meas, 3 * sizeof(double));
    
    // Predict postprocess
    Vector<3, float> a_for_vel_f;
    for (int i = 0; i < 3; ++i) {
        a_for_vel_f(i, 0) = static_cast<float>(a_meas[i]);
    }
    bool enable_accel_z = s->enable_accel_z_integration;
    float accel_z_threshold = static_cast<float>(s->accel_z_threshold);
    float accel_z_damping = static_cast<float>(s->accel_z_damping);
    float velocity_damping = static_cast<float>(s->velocity_damping);
    
    // 1. accel_z_integration - Direct C++ implementation using quaternion_lib.hpp
    if (enable_accel_z) {
        apply_accel_z_integration(v_f, q_f, a_for_vel_f, dt_f, g_f, accel_z_threshold, accel_z_damping);
    }
    
    // 2-6. 後処理（velocity_damping, P normalization, velocity clipping）
    PredictParams params;
    params.enable_accel_z_integration = false;  // 既に上で処理済み
    params.accel_z_threshold = accel_z_threshold;
    params.accel_z_damping = accel_z_damping;
    params.velocity_damping = velocity_damping;
    predict_postprocess(v_f, q_f, P_f, a_for_vel_f, dt_f, g_f, params);
    
    // 3. divergence_guard.regularize_covariance (mex_sensor_filter を統合)
    static SensorFilterLib filter_lib;  // Static instance for reuse
    // Convert Matrix<15,15,float> to FixedMatrix (cm)
    using cm = cmath_fx::FixedMatrix;
    cm P_fixed(15, 15);
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            P_fixed(i, j) = P_f(i, j);
        }
    }
    filter_lib.divergence_guard.regularize_covariance(P_fixed);
    // Convert back to Matrix<15,15,float>
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            P_f(i, j) = P_fixed(i, j);
        }
    }
    
    // 4. Velocity covariance clipping
    regularize_covariance(P_f);
    
    // 5. Velocity magnitude clipping
    apply_velocity_clipping(v_f, P_f, 3.0f);
    
    // Ensure symmetry of P
    for (int i = 0; i < 15; ++i) {
        for (int j = i + 1; j < 15; ++j) {
            float v = 0.5f * (P_f(i, j) + P_f(j, i));
            P_f(i, j) = v;
            P_f(j, i) = v;
        }
    }
    
    // Convert back to double (all state variables)
    for (int i = 0; i < 3; ++i) {
        s->p[i] = static_cast<double>(p_f(i, 0));
        s->v[i] = static_cast<double>(v_f(i, 0));
        s->ba[i] = static_cast<double>(ba_f(i, 0));
        s->bg[i] = static_cast<double>(bg_f(i, 0));
    }
    for (int i = 0; i < 4; ++i) {
        s->q[i] = static_cast<double>(q_f(i, 0));
    }
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            s->P[i + j*15] = static_cast<double>(P_f(i, j));
        }
    }
}

void ESKFRunner::apply_accel_z_integration(
    cmath_fx::Vector<3, float>& v,
    const cmath_fx::Vector<4, float>& q,
    const cmath_fx::Vector<3, float>& a_for_vel,
    float dt,
    const cmath_fx::Vector<3, float>& g,
    float accel_z_threshold,
    float accel_z_damping
) {
    using QuatF = quat_lib::Quaternion<float>;
    using namespace cmath_fx;
    
    // Convert Vector<4, float> to Quaternion<float>
    QuatF quat(q(0, 0), q(1, 0), q(2, 0), q(3, 0));
    quat.normalize();
    
    // Get rotation matrix (row-major order from quaternion_lib)
    float R_row[9];
    quat.to_rotation_matrix(R_row);
    
    // Convert row-major to column-major with transpose
    // mex_quaternion_lib returns transposed matrix for MATLAB (column-major)
    // So we need to transpose: R(i,j) = R_row[j*3 + i]
    Matrix<3, 3, float> R;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            // Transpose: row-major R_row to column-major R
            R(i, j) = R_row[j * 3 + i];
        }
    }
    
    // Calculate R * a_for_vel (matrix-vector multiplication)
    Vector<3, float> Ra;
    for (int i = 0; i < 3; ++i) {
        Ra(i, 0) = 0.0f;
        for (int j = 0; j < 3; ++j) {
            Ra(i, 0) += R(i, j) * a_for_vel(j, 0);
        }
    }
    
    // Calculate (R * a_for_vel) - [0; 0; g(3)]
    Vector<3, float> a_ned;
    a_ned(0, 0) = Ra(0, 0);
    a_ned(1, 0) = Ra(1, 0);
    a_ned(2, 0) = Ra(2, 0) - g(2, 0);
    
    float az_excess = a_ned(2, 0);
    if (std::abs(az_excess) > accel_z_threshold) {
        v(2, 0) = v(2, 0) * (1.0f - accel_z_damping) + az_excess * dt;
    }
}

void ESKFRunner::apply_velocity_clipping(
    cmath_fx::Vector<3, float>& v,
    cmath_fx::Matrix<15, 15, float>& P,
    float max_vel
) {
    using namespace cmath_fx;
    
    // Clip velocity magnitude to max_vel
    float vnorm = 0.0f;
    for (int i = 0; i < 3; ++i) {
        vnorm += v(i, 0) * v(i, 0);
    }
    vnorm = std::sqrt(vnorm);
    
    if (vnorm > max_vel) {
        float scale = max_vel / vnorm;
        for (int i = 0; i < 3; ++i) {
            v(i, 0) *= scale;
        }
    }
}

void ESKFRunner::regularize_covariance(
    cmath_fx::Matrix<15, 15, float>& P
) {
    using namespace cmath_fx;
    
    // vel_indices = [4, 5, 6] (1-based in MATLAB, but we use 0-based internally: 3, 4, 5)
    std::vector<int> vel_indices = {3, 4, 5};
    
    // Define max variances consistent with MATLAB limits
    std::vector<float> max_var(15);
    for (int i = 0; i < 15; ++i) {
        max_var[i] = 1e6f; // default large
    }
    // position 0:2 -> 100^2
    for (int i = 0; i < 3; ++i) {
        max_var[i] = 100.0f * 100.0f;
    }
    // velocity 3:5 -> 20^2
    for (int i = 3; i < 6; ++i) {
        max_var[i] = 20.0f * 20.0f;
    }
    // attitude 6:8 -> (deg2rad(45))^2
    float d45 = 45.0f * 3.14159265f / 180.0f;
    for (int i = 6; i < 9; ++i) {
        max_var[i] = d45 * d45;
    }
    // accel bias 9:11
    for (int i = 9; i < 12; ++i) {
        max_var[i] = 0.1f;
    }
    // gyro bias 12:14
    for (int i = 12; i < 15; ++i) {
        max_var[i] = 0.01f;
    }
    
    // Clip covariance per-velocity-index
    for (size_t kk = 0; kk < vel_indices.size(); ++kk) {
        int idx = vel_indices[kk];
        if (idx < 0 || idx >= 15) continue;
        float Pii = P(idx, idx);
        if (Pii > max_var[idx]) {
            float factor = std::sqrt(max_var[idx] / Pii);
            for (int j = 0; j < 15; ++j) {
                P(idx, j) *= factor;
            }
            for (int i = 0; i < 15; ++i) {
                P(i, idx) *= factor;
            }
            P(idx, idx) = max_var[idx];
        }
    }
}

} // namespace eskf



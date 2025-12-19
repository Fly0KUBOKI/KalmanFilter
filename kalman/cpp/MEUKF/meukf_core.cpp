#include "meukf_core.hpp"
#include "mex.h" // Added for debugging
#include "../Common/Math/math_utils.hpp"
#include "../Common/Math/quaternion.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <cstdlib>

namespace meukf {

// Debug helper: control logging via env var MEX_DEBUG (0=off, 1=minimal, 2=verbose)
static int get_debug_level() {
    const char* s = std::getenv("MEX_DEBUG");
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

    // 1. Prediction Step
    if (input.sensor.dt > 0.0) {
        predict(output.new_state, input.sensor, input.params);
    }

    // 2. Update Step
    
    // Accel Update (MEUKF)
    if (input.sensor.update_accel) {
        Vector3 a_meas = make_vector3(input.sensor.accel[0], input.sensor.accel[1], input.sensor.accel[2]);
        update_accel_meukf(output.new_state, a_meas, input.params, output);
    }

    // Mag Update (MEUKF)
    if (input.sensor.update_mag) {
        Vector3 m_meas = make_vector3(input.sensor.mag[0], input.sensor.mag[1], input.sensor.mag[2]);
        update_mag_meukf(output.new_state, m_meas, input.params, output);
    }

    // GPS Update
    if (input.sensor.update_gps) {
        Vector3 gps_meas = make_vector3(input.sensor.gps_pos[0], input.sensor.gps_pos[1], input.sensor.gps_pos[2]);
        update_gps(output.new_state, gps_meas, input.params, output);
    }
    
    // Baro Update
    if (input.sensor.update_baro) {
        update_baro(output.new_state, input.sensor.alt_baro, input.params, output);
    }

    // ZUPT Update (常に実行)
    if (input.sensor.update_zupt) {
        update_zupt(output.new_state, input.params, output);
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
            if (any_nan || max_abs > 1e6) {
                std::ofstream dbgfile("Results/predict_debug.txt", std::ios::app);
                if (dbgfile.is_open()) {
                    dbgfile << std::fixed << std::setprecision(9);
                    dbgfile << "MARKER=PREDICT_DEBUG";
                    dbgfile << ", any_nan=" << (any_nan?1:0);
                    dbgfile << ", max_abs_P_new=" << max_abs;
                    dbgfile << ", dt=" << dt;
                    dbgfile << std::endl;

                    dbgfile << "-- state before (p v q ba bg) --\n";
                    dbgfile << p(0,0) << "," << p(1,0) << "," << p(2,0) << ",";
                    dbgfile << v(0,0) << "," << v(1,0) << "," << v(2,0) << ",";
                    dbgfile << q(0,0) << "," << q(1,0) << "," << q(2,0) << "," << q(3,0) << ",";
                    dbgfile << ba(0,0) << "," << ba(1,0) << "," << ba(2,0) << ",";
                    dbgfile << bg(0,0) << "," << bg(1,0) << "," << bg(2,0) << std::endl;

                    dbgfile << "-- P_new_diag --\n";
                    for(int ii=0; ii<15; ++ii) dbgfile << P_new(ii,ii) << (ii==14?"":"\,");
                    dbgfile << std::endl;

                    if (dbg >= 2) {
                        dbgfile << "-- P_new_flat (first 5x5) --\n";
                        for(int ii=0; ii<5; ++ii) {
                            for(int jj=0; jj<5; ++jj) dbgfile << P_new(ii,jj) << (jj==4?"":"\,");
                            dbgfile << std::endl;
                        }
                        dbgfile << std::endl;
                    }
                    dbgfile.close();
                }
            }
        }
    } catch(...) {}

    vars_to_state(p_new, v_new, q_new, ba, bg, P_new, state);
}

void MEUKFCore::update_accel_meukf(State& state, const Vector3& a_meas, const Params& params, MEUKFOutput& output) {
    // 高速回転中は加速度更新をスキップ (MATLABコミット 7eb70e29: 1.5 rad/s)
    // Note: 角速度情報はSensorDataに含まれているが、この関数では直接アクセスできない
    // 代替案: a_measのノルムが重力から大きく外れている場合はスキップ
    float a_norm_check = vector3_norm(a_meas);
    if (a_norm_check < 0.1f || std::abs(a_norm_check - 9.81f) > 3.0f) {
        output.status = 1;
        return;
    }

    Vector3 p, v, ba, bg;
    Vector4 q_nom;
    Matrix15x15 P_full;
    state_to_vars(state, p, v, q_nom, ba, bg, P_full);

    Matrix3x3 P_att;
    for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) P_att(i, j) = P_full(6+i, 6+j);

    // 正定値化 (MATLABコミット 7eb70e29)
    // 対称化
    for(int i=0; i<3; ++i) {
        for(int j=i+1; j<3; ++j) {
            float avg = (P_att(i,j) + P_att(j,i)) / 2.0f;
            P_att(i,j) = avg;
            P_att(j,i) = avg;
        }
    }
    
    // 最小固有値チェック（簡易版: 対角要素の最小値で近似）
    float min_diag_accel = P_att(0,0);
    for(int i=1; i<3; ++i) {
        if (P_att(i,i) < min_diag_accel) min_diag_accel = P_att(i,i);
    }
    
    // 正定値でない場合は正則化
    if (min_diag_accel <= 0.0f) {
        float reg = std::abs(min_diag_accel) + 1e-6f;
        for(int i=0; i<3; ++i) P_att(i,i) += reg;
    }

    float alpha = params.alpha;
    float beta = params.beta;
    float kappa = params.kappa;
    int n = 3;
    float lambda = alpha * alpha * (n + kappa) - n;
    
    float Wm0 = lambda / (n + lambda);
    float Wc0 = Wm0 + (1.0f - alpha * alpha + beta);
    float Wi = 1.0f / (2.0f * (n + lambda));

    // Cholesky Decomposition
    Matrix3x3 L;
    if (!cholesky3x3(P_att, L)) {
        // Fallback: Diagonal approximation
        L = Matrix3x3::Zero();
        for(int i=0; i<3; ++i) L(i,i) = std::sqrt(std::max(0.0f, (float)P_att(i,i)));
    }
    
    Vector3 sigma_points[7];
    sigma_points[0] = make_vector3(0,0,0); // Zero vector
    float gamma = std::sqrt(n + lambda);
    
    for(int i=0; i<3; ++i) {
        Vector3 col = make_vector3(L(0,i), L(1,i), L(2,i)); // i-th column of L
        col = col * gamma;
        sigma_points[1+i] = col;
        sigma_points[1+3+i] = col * -1.0;
    }

    Vector3 g_vec = make_vector3(params.g[0], params.g[1], params.g[2]);
    Vector3 z_pred_sigma[7];
    Vector3 z_mean = make_vector3(0,0,0);

    for(int i=0; i<7; ++i) {
        Vector3 dtheta = sigma_points[i];
        float angle = vector3_norm(dtheta);

        Vector4 dq;
        if (angle < 1e-9f) {
            dq = make_vector4(1.0f, 0.0f, 0.0f, 0.0f);
        } else {
            float s = std::sin(angle * 0.5f);
            dq(0,0) = std::cos(angle * 0.5f);
            dq(1,0) = (dtheta(0,0)/angle) * s;
            dq(2,0) = (dtheta(1,0)/angle) * s;
            dq(3,0) = (dtheta(2,0)/angle) * s;
        }
        
        Vector4 q_i;
        cquat::multiply_quat(q_nom, dq, q_i);
        
        Matrix3x3 R_i;
        cquat::quat_to_rotm(q_i, R_i);
        // Accelerometer measures upward force (-g in body frame)
        // g_vec is [0,0,-9.8] (downward)
        // Expected measurement = - (R' * g)
        Vector3 a_pred = (R_i.transpose() * g_vec) * -1.0;
        
        z_pred_sigma[i] = a_pred;
        
        float w = (i==0) ? Wm0 : Wi;
        z_mean = z_mean + a_pred * w;
    }

    // 2D観測: x,y成分のみを使用 (MATLABコミット 7eb70e29 に合わせる)
    Vector2 z_mean_2d = make_vector2(z_mean(0,0), z_mean(1,0));

    Vector2 z_pred_sigma_2d[7];
    for(int i=0; i<7; ++i) {
        z_pred_sigma_2d[i] = make_vector2(z_pred_sigma[i](0,0), z_pred_sigma[i](1,0));
    }

    Matrix2x2 S_2d = Matrix2x2::Zero();
    Matrix3x2 P_xz_2d = Matrix3x2::Zero();

    for(int i=0; i<7; ++i) {
        Vector2 z_diff = z_pred_sigma_2d[i] - z_mean_2d;
        Vector3 x_diff = sigma_points[i];
        
        float w = (i==0) ? Wc0 : Wi;
        
        for(int r=0; r<2; ++r) {
            for(int c=0; c<2; ++c) {
                S_2d(r, c) += w * z_diff(r,0) * z_diff(c,0);
            }
        }
        for(int r=0; r<3; ++r) {
            for(int c=0; c<2; ++c) {
                P_xz_2d(r, c) += w * x_diff(r,0) * z_diff(c,0);
            }
        }
    }

    // 動的R調整 (MATLABコミット 7eb70e29)
    float a_norm = vector3_norm(a_meas);
    float gravity_deviation = std::abs(a_norm - 9.81f);
    float R_scale = 1.0f + (gravity_deviation / 0.7f);
    float R_floor = 0.25f;

    for(int i=0; i<2; ++i) {
        float R_est = params.noise_accel[i];
        S_2d(i, i) += std::max(R_est, R_floor) * R_scale;
    }

    Matrix2x2 S_2d_inv;
    if(!S_2d.inverse(S_2d_inv)) {
        // Inversion failed
        output.status = 1;
        return;
    }
    Matrix3x2 K = P_xz_2d * S_2d_inv;

    // 2D観測値
    Vector2 a_meas_2d = make_vector2(a_meas(0,0), a_meas(1,0));

    Vector2 y = a_meas_2d - z_mean_2d;

    // イノベーション制限 (MATLABコミット 7eb70e29: 0.05rad)
    float max_innovation = 0.05f;
    float innov_norm = std::sqrt(y(0,0)*y(0,0) + y(1,0)*y(1,0));
    if (innov_norm > max_innovation) {
        float scale = max_innovation / innov_norm;
        y(0,0) *= scale;
        y(1,0) *= scale;
    }

    // マハラノビス距離チェック (MATLABコミット 7eb70e29: 3.5-sigma棄却)
    float mahal_dist_sq = 0.0f;
    for(int i=0; i<2; ++i) {
        for(int j=0; j<2; ++j) {
            mahal_dist_sq += y(i,0) * S_2d_inv(i,j) * y(j,0);
        }
    }
    float mahal_dist = std::sqrt(mahal_dist_sq);
    
    if (mahal_dist > 5.0f) { // Changed from 3.5 to 5.0 to match MATLAB
        // 外れ値として棄却
        output.status = 1;
        return;
    }
    
    // 2.5-sigma以上は減衰
    if (mahal_dist > 2.5f) {
        float attenuation = 2.5f / mahal_dist;
        y(0,0) *= attenuation;
        y(1,0) *= attenuation;
    }

    Vector3 dx = K * y;

    // dtheta の大きさ制限 (MATLABコミット 7eb70e29: 0.6度)
    float dtheta_norm = std::sqrt(dx(0,0)*dx(0,0) + dx(1,0)*dx(1,0)); // Roll/Pitchのみ
    float max_dtheta = 0.6f * 3.14159265f / 180.0f; // 0.6 degrees in radians
    if (dtheta_norm > max_dtheta) {
        float scale = max_dtheta / dtheta_norm;
        dx(0,0) *= scale;
        dx(1,0) *= scale;
    }
    
    // Yaw成分を強制ゼロ (MATLABコミット 7eb70e29)
    dx(2,0) = 0.0f;

    // --- Cross-Covariance Update (2D observation) ---
    // Calculate P_att_upd (UKF update)
    Matrix3x3 P_att_upd = P_att - K * S_2d * K.transpose();

    // Calculate H_sub = -[R^T * g]_x (first 2 rows)
    Matrix3x3 R;
    cquat::quat_to_rotm(q_nom, R);
    Vector3 g_body = R.transpose() * g_vec;
    
    Matrix3x3 g_skew;
    g_skew(0, 0) = 0; g_skew(0, 1) = -g_body(2,0); g_skew(0, 2) = g_body(1,0);
    g_skew(1, 0) = g_body(2,0); g_skew(1, 1) = 0; g_skew(1, 2) = -g_body(0,0);
    g_skew(2, 0) = -g_body(1,0); g_skew(2, 1) = g_body(0,0); g_skew(2, 2) = 0;
    
    Matrix3x3 H_att = g_skew * -1.0f; // MATLAB: -skew(g_body)
    
    Matrix2x3 H_sub;
    for(int r=0; r<2; ++r) for(int c=0; c<3; ++c) H_sub(r, c) = H_att(r, c);
    
    // P_cross = P_full(:, 6:8) (15x3)
    Matrix15x3 P_cross;
    for(int r=0; r<15; ++r) for(int c=0; c<3; ++c) P_cross(r, c) = P_full(r, 6+c);
    
    // K_full = P_cross * H_sub' * S_2d_inv
    Matrix15x2 tmp = P_cross * H_sub.transpose();
    Matrix15x2 K_full = tmp * S_2d_inv;
    
    // Update Full State (2D innovation)
    // Uncommented to enable bias estimation
    
    Vector15 dx_full;
    for(int r=0; r<15; ++r) {
        dx_full(r, 0) = K_full(r, 0) * y(0,0) + K_full(r, 1) * y(1,0);
    }
    
    // Apply dx_full to p, v, ba, bg
    p = p + make_vector3(dx_full(0,0), dx_full(1,0), dx_full(2,0));
    v = v + make_vector3(dx_full(3,0), dx_full(4,0), dx_full(5,0));
    ba = ba + make_vector3(dx_full(9,0), dx_full(10,0), dx_full(11,0));
    bg = bg + make_vector3(dx_full(12,0), dx_full(13,0), dx_full(14,0));
    
    // --- Full Covariance Update ---
    // P = P - K * H * P
    // Calculate P_dec = K_full * (H_sub * P(6:8, :))
    Matrix15x15 P_dec = Matrix15x15::Zero();
    
    for(int i=0; i<15; ++i) {
        for(int j=0; j<15; ++j) {
            float sum = 0.0f;
            for(int k=0; k<2; ++k) {
                float hp_kj = 0.0f;
                for(int l=0; l<3; ++l) {
                    hp_kj += H_sub(k, l) * P_full(6+l, j);
                }
                sum += K_full(i, k) * hp_kj;
            }
            P_dec(i, j) = sum;
        }
    }
    // Replace simple subtraction with Joseph form for numerical parity with MATLAB:
    // Build KH_full = K_full * H_full (where H_full has H_sub at cols 6..8)
    Matrix15x15 KH_full = Matrix15x15::Zero();
    for(int i=0; i<15; ++i) {
        for(int j=0; j<15; ++j) {
            float sum = 0.0f;
            for(int k=0; k<3; ++k) {
                // H_full(k, j) is non-zero only when j in [6..8]
                if (j >= 6 && j < 9) {
                    sum += K_full(i, k) * H_sub(k, j - 6);
                }
            }
            KH_full(i,j) = sum;
        }
    }

    Matrix15x15 I = Matrix15x15::Identity();
    Matrix15x15 I_KH = I - KH_full;
    Matrix15x15 P_tmp = I_KH * P_full * I_KH.transpose();
    // Build R_mag (3x3 diag)
    Matrix3x3 R_mag = Matrix3x3::Zero();
    for(int ii=0; ii<3; ++ii) R_mag(ii,ii) = params.noise_mag[ii];
    Matrix15x15 KRKt = Matrix15x15::Zero();
    for(int i=0; i<15; ++i) {
        for(int j=0; j<15; ++j) {
            float sum2 = 0.0f;
            for(int k_idx=0; k_idx<3; ++k_idx) {
                for(int l_idx=0; l_idx<3; ++l_idx) {
                    sum2 += K_full(i, k_idx) * R_mag(k_idx, l_idx) * K_full(j, l_idx);
                }
            }
            KRKt(i,j) = sum2;
        }
    }
    P_full = P_tmp + KRKt;
    P_full = (P_full + P_full.transpose()) * 0.5f;

    float angle = vector3_norm(dx);

    Vector4 dq_update;
    if (angle < 1e-9f) {
        dq_update = make_vector4(1, 0, 0, 0);
    } else {
        float s = std::sin(angle * 0.5f);
        dq_update(0,0) = std::cos(angle * 0.5f);
        dq_update(1,0) = (dx(0,0)/angle) * s;
        dq_update(2,0) = (dx(1,0)/angle) * s;
        dq_update(3,0) = (dx(2,0)/angle) * s;
    }
    Vector4 q_updated;
    cquat::multiply_quat(q_nom, dq_update, q_updated);
    cquat::normalize_quat(q_updated);

    // Store 2D innovation norm for debug
    output.debug_info[0] = innov_norm;

    vars_to_state(p, v, q_updated, ba, bg, P_full, state);
}

void MEUKFCore::update_mag_meukf(State& state, const Vector3& m_meas, const Params& params, MEUKFOutput& output) {
    Vector3 p, v, ba, bg;
    Vector4 q_nom;
    Matrix15x15 P_full;
    state_to_vars(state, p, v, q_nom, ba, bg, P_full);

    Matrix3x3 P_att;
    for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) P_att(i, j) = P_full(6+i, 6+j);

    // 正定値化 (MATLABコミット 7eb70e29)
    // 対称化
    for(int i=0; i<3; ++i) {
        for(int j=i+1; j<3; ++j) {
            float avg = (P_att(i,j) + P_att(j,i)) / 2.0f;
            P_att(i,j) = avg;
            P_att(j,i) = avg;
        }
    }
    
    // 最小固有値チェック（簡易版: 対角要素の最小値で近似）
    float min_diag_mag = P_att(0,0);
    for(int i=1; i<3; ++i) {
        if (P_att(i,i) < min_diag_mag) min_diag_mag = P_att(i,i);
    }
    
    // 正定値でない場合は正則化
    if (min_diag_mag <= 0.0f) {
        float reg = std::abs(min_diag_mag) + 1e-6f;
        for(int i=0; i<3; ++i) P_att(i,i) += reg;
    }

    float alpha = params.alpha;
    float beta = params.beta;
    float kappa = params.kappa;
    int n = 3;
    float lambda = alpha * alpha * (n + kappa) - n;
    
    float Wm0 = lambda / (n + lambda);
    float Wc0 = Wm0 + (1.0f - alpha * alpha + beta);
    float Wi = 1.0f / (2.0f * (n + lambda));

    // Cholesky Decomposition
    Matrix3x3 L;
    if (!cholesky3x3(P_att, L)) {
        // Fallback: Diagonal approximation
        L = Matrix3x3::Zero();
        for(int i=0; i<3; ++i) L(i,i) = std::sqrt(std::max(0.0f, (float)P_att(i,i)));
    }
    
    Vector3 sigma_points[7];
    sigma_points[0] = make_vector3(0,0,0); // Zero vector
    float gamma = std::sqrt(n + lambda);
    
    for(int i=0; i<3; ++i) {
        Vector3 col = make_vector3(L(0,i), L(1,i), L(2,i)); // i-th column of L
        col = col * gamma;
        sigma_points[1+i] = col;
        sigma_points[1+3+i] = col * -1.0;
    }

    // Vector3 mag_ref_vec = make_vector3(params.mag_ref[0], params.mag_ref[1], params.mag_ref[2]);
    // Normalize mag_ref_vec to match MATLAB's normalized measurement
    Vector3 mag_ref_vec = make_vector3(params.mag_ref[0], params.mag_ref[1], params.mag_ref[2]);
    float mag_ref_norm = vector3_norm(mag_ref_vec);
    
    float noise_scale = 1.0f;
    if (mag_ref_norm > 1e-9f) {
        mag_ref_vec = mag_ref_vec * (1.0f / mag_ref_norm);
        noise_scale = 1.0f / (mag_ref_norm * mag_ref_norm);
    } else {
        mag_ref_vec = make_vector3(0.0f, 1.0f, 0.0f); // Default to Y-axis (North) if zero
    }

    // Normalize m_meas to match mag_ref_vec (unit vector)
    Vector3 m_meas_norm = m_meas;
    float m_meas_len = vector3_norm(m_meas);
    if (m_meas_len > 1e-9f) {
        m_meas_norm = m_meas * (1.0f / m_meas_len);
    }

    Vector3 z_pred_sigma[7];
    Vector3 z_mean = make_vector3(0,0,0);

    for(int i=0; i<7; ++i) {
        Vector3 dtheta = sigma_points[i];
        float angle = vector3_norm(dtheta);

        Vector4 dq;
        if (angle < 1e-9f) {
            dq = make_vector4(1.0f, 0.0f, 0.0f, 0.0f);
        } else {
            float s = std::sin(angle * 0.5f);
            dq(0,0) = std::cos(angle * 0.5f);
            dq(1,0) = (dtheta(0,0)/angle) * s;
            dq(2,0) = (dtheta(1,0)/angle) * s;
            dq(3,0) = (dtheta(2,0)/angle) * s;
        }
        
        Vector4 q_i;
        cquat::multiply_quat(q_nom, dq, q_i);
        
        Matrix3x3 R_i;
        cquat::quat_to_rotm(q_i, R_i);
        Vector3 m_pred = R_i.transpose() * mag_ref_vec;
        
        z_pred_sigma[i] = m_pred;
        
        float w = (i==0) ? Wm0 : Wi;
        z_mean = z_mean + m_pred * w;
    }

    Matrix3x3 S = Matrix3x3::Zero();
    Matrix3x3 P_xz = Matrix3x3::Zero();

    for(int i=0; i<7; ++i) {
        Vector3 z_diff = z_pred_sigma[i] - z_mean;
        Vector3 x_diff = sigma_points[i];
        
        float w = (i==0) ? Wc0 : Wi;
        
        for(int r=0; r<3; ++r) {
            for(int c=0; c<3; ++c) {
                S(r, c) += w * z_diff(r,0) * z_diff(c,0);
                P_xz(r, c) += w * x_diff(r,0) * z_diff(c,0);
            }
        }
    }

    // mexPrintf("DEBUG: mag_ref_norm=%f, noise_scale=%f\n", mag_ref_norm, noise_scale);
    // mexPrintf("DEBUG: params.noise_mag=[%f, %f, %f]\n", params.noise_mag[0], params.noise_mag[1], params.noise_mag[2]);
    for(int i=0; i<3; ++i) S(i, i) += std::max(params.noise_mag[i] * noise_scale, 1e-6f); // Scale noise and use small floor

    Matrix3x3 S_inv;
    if(!S.inverse(S_inv)) {
        // Inversion failed
        output.status = 1;
        return;
    }
    Matrix3x3 K = P_xz * S_inv;

    Vector3 y = m_meas_norm - z_mean;

    // イノベーション制限 (MATLABコミット 7eb70e29: 0.1rad)
    float max_innovation = 0.1f;
    float innov_norm = vector3_norm(y);
    if (innov_norm > max_innovation) {
        float scale = max_innovation / innov_norm;
        y = y * scale;
    }

    // マハラノビス距離チェック (MATLABコミット 7eb70e29: 4.0-sigma棄却)
    Vector3 S_inv_y = S_inv * y;
    float mahal_dist_sq = y(0,0)*S_inv_y(0,0) + y(1,0)*S_inv_y(1,0) + y(2,0)*S_inv_y(2,0);
    float mahal_dist = std::sqrt(mahal_dist_sq);
    
    // DEBUG PRINT
    // mexPrintf("Mag Update: y=[%f, %f, %f], S_diag=[%f, %f, %f], mahal=%f\n", 
    //     y(0,0), y(1,0), y(2,0), S(0,0), S(1,1), S(2,2), mahal_dist);

    if (mahal_dist > 4.0f) { // Reverted to 4.0
        // 外れ値として棄却
        output.status = 1;
        return;
    }
    
    // 2.5-sigma以上は減衰
    if (mahal_dist > 2.5f) {
        float attenuation = 2.5f / mahal_dist;
        y = y * attenuation;
    }

    Vector3 dx = K * y;
    
    // dtheta の大きさ制限 (MATLABコミット 7eb70e29: 1.0度)
    float dtheta_norm = vector3_norm(dx);
    float max_dtheta = 1.0f * 3.14159265f / 180.0f; // 1.0 degrees in radians
    if (dtheta_norm > max_dtheta) {
        float scale = max_dtheta / dtheta_norm;
        dx = dx * scale;
    }

    // --- Cross-Covariance Update (3D observation for magnetometer) ---
    // Calculate P_att_upd (UKF update)
    Matrix3x3 P_att_upd = P_att - K * S * K.transpose();

    // Calculate H_sub = [R^T * m_ref]_x
    Matrix3x3 R;
    cquat::quat_to_rotm(q_nom, R);
    Vector3 m_body = R.transpose() * mag_ref_vec;
    
    Matrix3x3 m_skew;
    m_skew(0, 0) = 0; m_skew(0, 1) = -m_body(2,0); m_skew(0, 2) = m_body(1,0);
    m_skew(1, 0) = m_body(2,0); m_skew(1, 1) = 0; m_skew(1, 2) = -m_body(0,0);
    m_skew(2, 0) = -m_body(1,0); m_skew(2, 1) = m_body(0,0); m_skew(2, 2) = 0;
    
    Matrix3x3 H_sub = m_skew; // MATLAB: skew(h_mag)
    
    // P_cross = P_full(:, 6:8) (15x3)
    Matrix15x3 P_cross;
    for(int r=0; r<15; ++r) for(int c=0; c<3; ++c) P_cross(r, c) = P_full(r, 6+c);
    
    // K_full = P_cross * H_sub' * S_inv
    Matrix15x3 tmp = P_cross * H_sub.transpose();
    Matrix15x3 K_full = tmp * S_inv;
    
    // Update Full State
    // Uncommented to enable bias estimation
    
    Vector15 dx_full;
    for(int r=0; r<15; ++r) {
        dx_full(r, 0) = K_full(r, 0) * y(0,0) + K_full(r, 1) * y(1,0) + K_full(r, 2) * y(2,0);
    }
    
    // Apply dx_full to p, v, ba, bg
    p = p + make_vector3(dx_full(0,0), dx_full(1,0), dx_full(2,0));
    v = v + make_vector3(dx_full(3,0), dx_full(4,0), dx_full(5,0));
    ba = ba + make_vector3(dx_full(9,0), dx_full(10,0), dx_full(11,0));
    bg = bg + make_vector3(dx_full(12,0), dx_full(13,0), dx_full(14,0));
    
    // --- Full Covariance Update ---
    // P = P - K * H * P
    // Calculate P_dec = K_full * (H_sub * P(6:8, :))
    Matrix15x15 P_dec = Matrix15x15::Zero();
    
    for(int i=0; i<15; ++i) {
        for(int j=0; j<15; ++j) {
            float sum = 0.0f;
            for(int k=0; k<3; ++k) {
                float hp_kj = 0.0f;
                for(int l=0; l<3; ++l) {
                    hp_kj += H_sub(k, l) * P_full(6+l, j);
                }
                sum += K_full(i, k) * hp_kj;
            }
            P_dec(i, j) = sum;
        }
    }
    P_full = P_full - P_dec;

    float angle = vector3_norm(dx);

    Vector4 dq_update;
    if (angle < 1e-9f) {
        dq_update = make_vector4(1, 0, 0, 0);
    } else {
        float s = std::sin(angle * 0.5f);
        dq_update(0,0) = std::cos(angle * 0.5f);
        dq_update(1,0) = (dx(0,0)/angle) * s;
        dq_update(2,0) = (dx(1,0)/angle) * s;
        dq_update(3,0) = (dx(2,0)/angle) * s;
    }
    Vector4 q_updated;
    cquat::multiply_quat(q_nom, dq_update, q_updated);
    cquat::normalize_quat(q_updated);

    output.debug_info[1] = innov_norm;

    vars_to_state(p, v, q_updated, ba, bg, P_full, state);
}

void MEUKFCore::update_gps(State& state, const Vector3& gps_meas, const Params& params, MEUKFOutput& output) {
    Vector3 p, v, ba, bg;
    Vector4 q;
    Matrix15x15 P;
    state_to_vars(state, p, v, q, ba, bg, P);

    Vector3 z_pred = p;
    Vector3 y = gps_meas - z_pred;
    
    Matrix3x3 P_pos;
    for(int i=0; i<3; ++i) for(int j=0; j<3; ++j) P_pos(i, j) = P(i, j);
    
    Matrix3x3 S = P_pos;
    for(int i=0; i<3; ++i) S(i, i) += params.noise_gps[i];
    
    Matrix3x3 S_inv;
    if(!S.inverse(S_inv)) {
        output.status = 1;
        return;
    }
    
    Matrix15x3 PHt;
    for(int i=0; i<15; ++i) for(int j=0; j<3; ++j) PHt(i, j) = P(i, j);
    
    Matrix15x3 K = PHt * S_inv;
    Vector15 dx = K * y;

    // Debug: append GPS update diagnostics to Results/gps_debug.txt (only if debug enabled)
    try {
        int dbg = get_debug_level();
        if (dbg >= 2) {
            std::ofstream gdbg("Results/gps_debug.txt", std::ios::app);
            if (gdbg.is_open()) {
                gdbg << std::fixed << std::setprecision(6);
                // z_pred, y (3), R (diag), S (3x3 flattened), K_pos (3x3 flattened), dx_pos (3)
                gdbg << "z_pred=" << p(0,0) << "," << p(1,0) << "," << p(2,0);
                gdbg << ", y=" << y(0,0) << "," << y(1,0) << "," << y(2,0);
                gdbg << ", R=" << params.noise_gps[0] << "," << params.noise_gps[1] << "," << params.noise_gps[2];
                gdbg << ", S=";
                for(int ii=0; ii<3; ++ii) for(int jj=0; jj<3; ++jj) gdbg << S(ii,jj) << (ii==2 && jj==2?"":",");
                gdbg << ", K_pos=";
                for(int ii=0; ii<3; ++ii) for(int jj=0; jj<3; ++jj) gdbg << K(ii,jj) << (ii==2 && jj==2?"":",");
                gdbg << ", dx_pos=" << dx(0,0) << "," << dx(1,0) << "," << dx(2,0);

                // Add an explicit marker to detect whether this logging comes from the rebuilt binary
                gdbg << ", MARKER=GPS_LOG_EXTENDED_v2";

                // Log P (position block) before update (flattened row-major)
                gdbg << ", P_pos_before=";
                for(int ii=0; ii<3; ++ii) for(int jj=0; jj<3; ++jj) gdbg << P_pos(ii,jj) << (ii==2 && jj==2?"":",");

                // Also log a small fingerprint of full P (diag elements)
                gdbg << ", P_diag_before=";
                for(int ii=0; ii<15; ++ii) gdbg << P(ii,ii) << (ii==14?"":",");

                gdbg << std::endl;
                gdbg.close();
            }
        }
    } catch(...) {}

    p = p + make_vector3(dx(0,0), dx(1,0), dx(2,0));
    v = v + make_vector3(dx(3,0), dx(4,0), dx(5,0));
    
    Vector3 dtheta = make_vector3(dx(6,0), dx(7,0), dx(8,0));
    float angle = vector3_norm(dtheta);

    Vector4 dq;
    if (angle < 1e-9f) {
        dq = make_vector4(1.0f, 0.0f, 0.0f, 0.0f);
    } else {
        float s = std::sin(angle * 0.5f);
        dq(0,0) = std::cos(angle * 0.5f);
        dq(1,0) = (dtheta(0,0)/angle) * s;
        dq(2,0) = (dtheta(1,0)/angle) * s;
        dq(3,0) = (dtheta(2,0)/angle) * s;
    }
    Vector4 q_new;
    cquat::multiply_quat(q, dq, q_new);
    cquat::normalize_quat(q_new);
    q = q_new;
    
    ba = ba + make_vector3(dx(9,0), dx(10,0), dx(11,0));
    bg = bg + make_vector3(dx(12,0), dx(13,0), dx(14,0));
    
    Matrix15x15 KH = Matrix15x15::Zero();
    for(int i=0; i<15; ++i) {
        for(int j=0; j<3; ++j) {
            KH(i, j) = K(i, j);
        }
    }
    
    Matrix15x15 I = Matrix15x15::Identity();
    // Use Joseph form for covariance update for numerical equivalence with MATLAB implementation:
    // P = (I - K*H) * P * (I - K*H)' + K * R * K'
    Matrix15x15 KH_full = Matrix15x15::Zero();
    for(int i=0; i<15; ++i) {
        for(int j=0; j<3; ++j) {
            KH_full(i, j) = KH(i, j); // columns 0..2 correspond to pos block
        }
    }
    // Build R (3x3) from params.noise_gps
    Matrix3x3 R_gps = Matrix3x3::Zero();
    for(int ii=0; ii<3; ++ii) R_gps(ii,ii) = params.noise_gps[ii];

    // Expand KH into full 15x15 representation: KH_full (15x15) already has first 3 cols used above
    // Construct I_KH and apply Joseph form
    Matrix15x15 I_KH = I - KH_full;
    Matrix15x15 P_tmp = I_KH * P * I_KH.transpose();
    // K is 15x3 (PHt * S_inv)
    Matrix15x15 K_R_Kt = Matrix15x15::Zero();
    for(int i=0; i<15; ++i) {
        for(int j=0; j<15; ++j) {
            float sum = 0.0f;
            for(int k_idx=0; k_idx<3; ++k_idx) {
                for(int l_idx=0; l_idx<3; ++l_idx) {
                    sum += K(i, k_idx) * R_gps(k_idx, l_idx) * K(j, l_idx);
                }
            }
            K_R_Kt(i,j) = sum;
        }
    }
    P = P_tmp + K_R_Kt;
    // Ensure symmetry
    P = (P + P.transpose()) * 0.5f;

    // Debug: append P after GPS update (only if debug level >=2)
    try {
        int dbg = get_debug_level();
        if (dbg >= 2) {
            std::ofstream gdbg2("Results/gps_debug.txt", std::ios::app);
            if (gdbg2.is_open()) {
                gdbg2 << std::fixed << std::setprecision(6);
                // P_pos_after and diag of full P
                Matrix3x3 P_pos_after;
                for(int ii=0; ii<3; ++ii) for(int jj=0; jj<3; ++jj) P_pos_after(ii,jj) = P(ii,jj);
                // Add marker to the after-update block as well
                gdbg2 << ", MARKER=GPS_LOG_EXTENDED_v2";
                gdbg2 << "P_pos_after=";
                for(int ii=0; ii<3; ++ii) for(int jj=0; jj<3; ++jj) gdbg2 << P_pos_after(ii,jj) << (ii==2 && jj==2?"":",");
                gdbg2 << ", P_diag_after=";
                for(int ii=0; ii<15; ++ii) gdbg2 << P(ii,ii) << (ii==14?"":",");
                gdbg2 << std::endl;
                gdbg2.close();
            }
        }
    } catch(...) {}

    vars_to_state(p, v, q, ba, bg, P, state);
}

void MEUKFCore::update_baro(State& state, float alt_baro, const Params& params, MEUKFOutput& output) {
    Vector3 p, v, ba, bg;
    Vector4 q;
    Matrix15x15 P;
    state_to_vars(state, p, v, q, ba, bg, P);

    // H = [0,0,1, zeros(1,12)]
    // z = alt_baro, h = p(3)
    float z_pred = p(2,0);  // p(3) in MATLAB = p(2) in C++ (0-indexed)
    float y = alt_baro - z_pred;

    // Debug: append baro update diagnostics to Results/baro_debug.txt (partial: alt_baro, z_pred, y)
    try {
        std::ofstream dbg("Results/baro_debug.txt", std::ios::app);
        if(dbg.is_open()) {
            dbg << std::fixed << std::setprecision(6);
            dbg << "alt_baro=" << alt_baro << ", z_pred=" << z_pred << ", y=" << y << ", R=" << params.noise_baro;
            // small fingerprint: diag of full P before update
            dbg << ", P_diag_before=";
            for(int ii=0; ii<15; ++ii) dbg << P(ii,ii) << (ii==14?"":",");
            dbg << std::endl;
            dbg.close();
        }
    } catch(...) {
        // ignore logging failures
    }
    
    // S = H*P*H' + R
    float P_z = P(2, 2);
    float S = P_z + params.noise_baro;
    
    if (S < 1e-9f) {
        output.status = 1;
        return;
    }
    
    float S_inv = 1.0f / S;
    
    // K = P*H' / S
    Vector15 K;
    for(int i=0; i<15; ++i) {
        K(i,0) = P(i, 2) * S_inv;
    }
    
    // dx = K * y
    Vector15 dx;
    for(int i=0; i<15; ++i) {
        dx(i,0) = K(i,0) * y;
    }

    // Debug: append K(2) and dx(2) and whether applied
    try {
        std::ofstream dbg("Results/baro_debug.txt", std::ios::app);
        if(dbg.is_open()) {
            dbg << "S=" << S << ", K_z=" << K(2,0) << ", dx_z=" << dx(2,0);
            bool applied = (std::abs(dx(2,0)) >= 0.1f);
            dbg << ", applied=" << (applied?1:0) << std::endl;
            dbg.close();
        }
    } catch(...) {}
    
    // 高度更新のみ（閾値チェック付き）
    if (std::abs(dx(2,0)) >= 0.1f) {
        p(2,0) = p(2,0) + dx(2,0);
    }
    
    // P = (I - K*H) * P
    Matrix15x15 KH = Matrix15x15::Zero();
    for(int i=0; i<15; ++i) {
        KH(i, 2) = K(i,0);
    }
    
    Matrix15x15 I = Matrix15x15::Identity();
    // Joseph form for baro (scalar R)
    Matrix15x15 KH_full = Matrix15x15::Zero();
    for(int i=0; i<15; ++i) {
        KH_full(i, 2) = KH(i, 2);
    }
    Matrix15x15 I_KH = I - KH_full;
    Matrix15x15 P_tmp = I_KH * P * I_KH.transpose();
    // Build K*R*K' (R is scalar params.noise_baro)
    Matrix15x15 K_R_Kt = Matrix15x15::Zero();
    for(int i=0; i<15; ++i) for(int j=0; j<15; ++j) K_R_Kt(i,j) = K(i,0) * params.noise_baro * K(j,0);
    P = P_tmp + K_R_Kt;
    P = (P + P.transpose()) * 0.5f;
    
    // Debug: append P diag after baro update
    try {
        std::ofstream dbg2("Results/baro_debug.txt", std::ios::app);
        if(dbg2.is_open()) {
            dbg2 << std::fixed << std::setprecision(6);
            dbg2 << "P_diag_after=";
            for(int ii=0; ii<15; ++ii) dbg2 << P(ii,ii) << (ii==14?"":",");
            dbg2 << std::endl;
            dbg2.close();
        }
    } catch(...) {}

    vars_to_state(p, v, q, ba, bg, P, state);
}

void MEUKFCore::update_zupt(State& state, const Params& params, MEUKFOutput& output) {
    Vector3 p, v, ba, bg;
    Vector4 q;
    Matrix15x15 P;
    state_to_vars(state, p, v, q, ba, bg, P);

    // ZUPT: Observe velocity = 0
    // z = [0;0;0], h = v
    // y = z - h = -v
    Vector3 y;
    y(0,0) = -v(0,0);
    y(1,0) = -v(1,0);
    y(2,0) = -v(2,0);

    // H = [0, I, 0, 0, 0]
    // S = H*P*H' + R
    // H*P*H' is simply the velocity block of P (indices 3,4,5)
    Matrix3x3 P_vv;
    for(int i=0; i<3; ++i) {
        for(int j=0; j<3; ++j) {
            P_vv(i,j) = P(3+i, 3+j);
        }
    }

    Matrix3x3 R;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R(i,j) = (i == j) ? params.noise_zupt[i] : 0.0f;
        }
    }

    Matrix3x3 S = P_vv + R;

    // Invert S
    Matrix3x3 S_inv;
    if (!S.inverse(S_inv)) {
        output.status = 2; // Singular matrix
        return;
    }

    // K = P * H' * S_inv
    // P * H' is the block of columns 3,4,5 of P
    Matrix15x3 PHt;
    for(int i=0; i<15; ++i) {
        for(int j=0; j<3; ++j) {
            PHt(i,j) = P(i, 3+j);
        }
    }

    Matrix15x3 K = PHt * S_inv;

    // dx = K * y
    Vector15 dx = K * y;

    // Update State
    // Position
    p(0,0) += dx(0,0); p(1,0) += dx(1,0); p(2,0) += dx(2,0);
    // Velocity
    v(0,0) += dx(3,0); v(1,0) += dx(4,0); v(2,0) += dx(5,0);
    // Attitude (Error quaternion)
    Vector3 dtheta;
    dtheta(0,0) = dx(6,0); dtheta(1,0) = dx(7,0); dtheta(2,0) = dx(8,0);
    
    // Apply attitude correction (q_new = q * dq)
    // Approximation: dq = [1, dtheta/2]
    Vector4 dq;
    dq(0,0) = 1.0f;
    dq(1,0) = 0.5f * dtheta(0,0);
    dq(2,0) = 0.5f * dtheta(1,0);
    dq(3,0) = 0.5f * dtheta(2,0);
    cquat::normalize_quat(dq);
    
    Vector4 q_new;
    cquat::multiply_quat(q, dq, q_new);
    cquat::normalize_quat(q_new);
    q = q_new;

    // Bias
    ba(0,0) += dx(9,0); ba(1,0) += dx(10,0); ba(2,0) += dx(11,0);
    bg(0,0) += dx(12,0); bg(1,0) += dx(13,0); bg(2,0) += dx(14,0);

    // Update Covariance
    // P = (I - K*H) * P
    // K*H is 15x15, but only columns 3,4,5 are non-zero (equal to K)
    Matrix15x15 KH = Matrix15x15::Zero();
    for(int i=0; i<15; ++i) {
        for(int j=0; j<3; ++j) {
            KH(i, 3+j) = K(i,j);
        }
    }

    Matrix15x15 I_mat = Matrix15x15::Identity();
    P = (I_mat - KH) * P;

    // Symmetrize P
    for(int i=0; i<15; ++i) {
        for(int j=i+1; j<15; ++j) {
            float val = 0.5f * (P(i,j) + P(j,i));
            P(i,j) = val;
            P(j,i) = val;
        }
    }

    vars_to_state(p, v, q, ba, bg, P, state);
}

} // namespace meukf

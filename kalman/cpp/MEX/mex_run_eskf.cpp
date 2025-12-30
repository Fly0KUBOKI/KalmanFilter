/* mex_run_eskf.cpp
 * Complete ESKF implementation in a single MEX file.
 * Replaces ESKF.m entirely.
 * Uses mexCallMATLAB to call existing MEX functions for accuracy.
 *
 * API:
 *   handle = mex_run_eskf('init', obs, static_time, dt)
 *   mex_run_eskf('step', handle, obs, k)
 *   state = mex_run_eskf('get_state', handle)
 *   mex_run_eskf('free', handle)
 */

#include <mex.h>
#include <cmath>
#include <cstring>
#include <string>
#include <map>
#include <vector>

// レイヤー1: 基本型（最初に配置）
#include "../Inc/Common/Math/fixed_matrix.hpp"

// レイヤー2: ユーティリティ
#include "../Inc/Common/Math/vector_utils.hpp"
#include "../Inc/Common/Math/quaternion_lib.hpp"
#include "../Inc/Common/Math/statistics.hpp"

// レイヤー3: ESKF コア
#include "../Inc/ESKF/eskf_core.hpp"
#include "../Inc/ESKF/eskf_postprocess.hpp"
#include "../Inc/ESKF/eskf_state.hpp"

// レイヤー4: 統合層
#include "../Inc/Common/filter_management.hpp"
#include "../Inc/Common/Sensor/sensor_filter.hpp"
#include "../Inc/Common/Sensor/sensor_preprocessor.hpp"
#include "../Inc/ESKF/eskf_sensor_updates.hpp"
#include "../Inc/MEX/mex_type_conversion.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace common::math;
using namespace common::filter;
using namespace common::sensor;
using Quat = quat_lib::Quaternion<double>;
using QuatF = quat_lib::Quaternion<float>;
using namespace cmath_fx;
using namespace eskf;
using namespace mex_conv;
using cm = cmath_fx::FixedMatrix;  // Alias for sensor filter

static std::map<uint64_t, ESKFState*> g_states;
static uint64_t g_next_handle = 1;
static SensorFilterLib g_filter_lib;  // Global sensor filter library instance

// Utility functions
static void copy_vec(double* dst, const double* src, int n) {
    memcpy(dst, src, n * sizeof(double));
}

static std::string getCmd(const mxArray* a) {
    char buf[256] = {0};
    if (!mxIsChar(a)) return "";
    mxGetString(a, buf, sizeof(buf));
    return std::string(buf);
}

// quat_to_euler: use quaternion_lib.hpp directly
static void quat_to_euler(const double* q_in, double* euler) {
    Quat quat(q_in[0], q_in[1], q_in[2], q_in[3]);
    quat.normalize();
    
    double roll_deg, pitch_deg, yaw_deg;
    quat.to_euler(roll_deg, pitch_deg, yaw_deg);
    
    // Convert degrees to radians
    euler[0] = roll_deg * M_PI / 180.0;
    euler[1] = pitch_deg * M_PI / 180.0;
    euler[2] = yaw_deg * M_PI / 180.0;
}

static void getVec3(const mxArray* s, const char* xname, const char* yname, const char* zname, mwIndex idx, double* out) {
    mxArray* fx = mxGetField(s, 0, xname);
    mxArray* fy = mxGetField(s, 0, yname);
    mxArray* fz = mxGetField(s, 0, zname);
    out[0] = fx ? mxGetPr(fx)[idx] : 0.0;
    out[1] = fy ? mxGetPr(fy)[idx] : 0.0;
    out[2] = fz ? mxGetPr(fz)[idx] : 0.0;
}

#define getAccel(obs, idx, out) getVec3(obs, "ax", "ay", "az", idx, out)
#define getGyro(obs, idx, out)  getVec3(obs, "wx", "wy", "wz", idx, out)
#define getMag(obs, idx, out)   getVec3(obs, "mx", "my", "mz", idx, out)

// Predict using ESKFCore directly (mex_adaptive_predict を統合)
static void call_predict(ESKFState* s, const double* a_meas, const double* w_meas) {
    using namespace eskf;  // Ensure namespace is available in this function
    using PredictParams = eskf::PredictPostprocessParams;  // Type alias for clarity
    
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
    
    copy_vec(s->w_body, w_meas, 3);
    
    // Predict postprocess (mex_eskf_predict_postprocess を統合)
    // Note: v_f, q_f, P_f, g_f, dt_f are already declared above
    // Only need a_for_vel_f for postprocessing
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
        // Convert Vector<4, float> to Quaternion<float>
        QuatF quat(q_f(0, 0), q_f(1, 0), q_f(2, 0), q_f(3, 0));
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
                Ra(i, 0) += R(i, j) * a_for_vel_f(j, 0);
            }
        }
        
        // Calculate (R * a_for_vel) - [0; 0; g(3)]
        Vector<3, float> a_ned;
        a_ned(0, 0) = Ra(0, 0);
        a_ned(1, 0) = Ra(1, 0);
        a_ned(2, 0) = Ra(2, 0) - g_f(2, 0);
        
        float az_excess = a_ned(2, 0);
        if (std::abs(az_excess) > accel_z_threshold) {
            v_f(2, 0) = v_f(2, 0) * (1.0f - accel_z_damping) + az_excess * dt_f;
        }
    }
    
    // 2-6. 後処理（velocity_damping, P normalization, velocity clipping）
    PredictParams params;
    params.enable_accel_z_integration = false;  // 既に上で処理済み
    params.accel_z_threshold = accel_z_threshold;
    params.accel_z_damping = accel_z_damping;
    params.velocity_damping = velocity_damping;
    eskf::predict_postprocess(v_f, q_f, P_f, a_for_vel_f, dt_f, g_f, params);
    
    // 3. divergence_guard.regularize_covariance (mex_sensor_filter を統合)
    using namespace common::sensor;
    static SensorFilterLib filter_lib;  // Static instance for reuse
    // Convert Matrix<15,15,float> to FixedMatrix (cm)
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
    
    // 5. divergence_guard.check_and_clip_velocity (mex_sensor_filter を統合)
    // vel_indices = [4, 5, 6] (1-based in MATLAB, but we use 0-based internally: 3, 4, 5)
    std::vector<int> vel_indices = {3, 4, 5};
    bool was_clipped = false;
    
    // Define max variances consistent with MATLAB limits
    std::vector<float> max_var(15);
    for(int i=0; i<15; ++i) max_var[i] = 1e6f; // default large
    // position 0:2 -> 100^2
    for(int i=0; i<3; ++i) max_var[i] = 100.0f*100.0f;
    // velocity 3:5 -> 20^2
    for(int i=3; i<6; ++i) max_var[i] = 20.0f*20.0f;
    // attitude 6:8 -> (deg2rad(45))^2
    float d45 = 45.0f * 3.14159265f / 180.0f;
    for(int i=6; i<9; ++i) max_var[i] = d45*d45;
    // accel bias 9:11
    for(int i=9; i<12; ++i) max_var[i] = 0.1f;
    // gyro bias 12:14
    for(int i=12; i<15; ++i) max_var[i] = 0.01f;
    
    // Clip covariance per-velocity-index
    for(size_t kk=0; kk<vel_indices.size(); ++kk) {
        int idx = vel_indices[kk];
        if (idx < 0 || idx >= 15) continue;
        float Pii = P_f(idx, idx);
        if (Pii > max_var[idx]) {
            float factor = sqrtf(max_var[idx] / Pii);
            for(int j=0; j<15; ++j) P_f(idx, j) *= factor;
            for(int i=0; i<15; ++i) P_f(i, idx) *= factor;
            P_f(idx, idx) = max_var[idx];
            was_clipped = true;
        }
    }
    
    // Clip velocity magnitude to max_vel (3.0 m/s)
    float max_vel = 3.0f;
    float vnorm = 0.0f;
    for(int i=0; i<3; ++i) vnorm += v_f(i, 0) * v_f(i, 0);
    vnorm = sqrtf(vnorm);
    if (vnorm > max_vel) {
        float scale = max_vel / vnorm;
        for(int i=0; i<3; ++i) v_f(i, 0) *= scale;
        was_clipped = true;
    }
    
    // Ensure symmetry of P
    for(int i=0; i<15; ++i) {
        for(int j=i+1; j<15; ++j) {
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

// Helper: Check if any value is NaN
static bool is_nan_any(const double* v, int n) {
    for (int i = 0; i < n; ++i) {
        if (mxIsNaN(v[i])) return true;
    }
    return false;
}

// Helper: Compute 3D vector norm
static double norm3(const double* v) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// Call sensor update (revert to original mexCallMATLAB implementation with preprocessing)
static void call_sensor_update(ESKFState* s, const char* type, const double* meas, int meas_len, double sample) {
    // Revert to original implementation from mex_eskf_sensor_updates_full.cpp
    double p[3], v[3], q[4], ba[3], bg[3], P[15*15], g[3];
    memcpy(p, s->p, 3 * sizeof(double));
    memcpy(v, s->v, 3 * sizeof(double));
    memcpy(q, s->q, 4 * sizeof(double));
    memcpy(ba, s->ba, 3 * sizeof(double));
    memcpy(bg, s->bg, 3 * sizeof(double));
    memcpy(P, s->P, 15*15*sizeof(double));
    memcpy(g, s->g, 3 * sizeof(double));
    double dt = s->dt;
    
    double out_p[3], out_v[3], out_q[4], out_ba[3], out_bg[3], out_P[15*15];
    memcpy(out_p, p, 3 * sizeof(double));
    memcpy(out_v, v, 3 * sizeof(double));
    memcpy(out_q, q, 4 * sizeof(double));
    memcpy(out_ba, ba, 3 * sizeof(double));
    memcpy(out_bg, bg, 3 * sizeof(double));
    memcpy(out_P, P, 15*15*sizeof(double));
    
    bool should_skip = true;
    
    if (strcmp(type, "accel") == 0 && meas_len == 3) {
        // Preprocess accel
        mxArray* prhs_pre[3];
        mxArray* plhs_pre[3];
        prhs_pre[0] = mxCreateString("preprocess_accel");
        mxArray* a_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
        memcpy(mxGetPr(a_arr), meas, 3 * sizeof(double));
        prhs_pre[1] = a_arr;
        mxArray* prev_a = mxCreateDoubleMatrix(3, 1, mxREAL);
        memcpy(mxGetPr(prev_a), s->prev_accel, 3 * sizeof(double));
        prhs_pre[2] = prev_a;
        
        double a_corrected[3];
        if (mexCallMATLAB(3, plhs_pre, 3, prhs_pre, "mex_sensor_preprocessor") == 0) {
            memcpy(a_corrected, mxGetPr(plhs_pre[0]), 3 * sizeof(double));
            bool is_outlier = mxIsLogicalScalarTrue(plhs_pre[1]);
            bool no_change = mxIsLogicalScalarTrue(plhs_pre[2]);
            
            // Check w_body norm
            double w_norm = 0.0;
            for (int i = 0; i < 3; ++i) {
                double w = s->w_body[i];
                w_norm += w * w;
            }
            w_norm = sqrt(w_norm);
            
            if (!no_change && !is_nan_any(a_corrected, 3) && !is_outlier && (w_norm <= 1.5)) {
                should_skip = false;
                memcpy(s->prev_accel, meas, 3 * sizeof(double));
            }
            for (int i = 0; i < 3; ++i) mxDestroyArray(plhs_pre[i]);
        }
        for (int i = 0; i < 3; ++i) mxDestroyArray(prhs_pre[i]);
        
        if (!should_skip) {
            // Call mex_eskf_do_update
            mxArray* prhs_u[11];
            mxArray* plhs_u[7];
            prhs_u[0] = mxCreateString("accel");
            mxArray* ac = mxCreateDoubleMatrix(3, 1, mxREAL);
            memcpy(mxGetPr(ac), a_corrected, 3 * sizeof(double));
            prhs_u[1] = ac;
            mxArray* pp = mxCreateDoubleMatrix(3, 1, mxREAL);
            memcpy(mxGetPr(pp), out_p, 3 * sizeof(double));
            prhs_u[2] = pp;
            mxArray* vv = mxCreateDoubleMatrix(3, 1, mxREAL);
            memcpy(mxGetPr(vv), out_v, 3 * sizeof(double));
            prhs_u[3] = vv;
            mxArray* qq = mxCreateDoubleMatrix(4, 1, mxREAL);
            memcpy(mxGetPr(qq), out_q, 4 * sizeof(double));
            prhs_u[4] = qq;
            mxArray* bba = mxCreateDoubleMatrix(3, 1, mxREAL);
            memcpy(mxGetPr(bba), out_ba, 3 * sizeof(double));
            prhs_u[5] = bba;
            mxArray* bbg = mxCreateDoubleMatrix(3, 1, mxREAL);
            memcpy(mxGetPr(bbg), out_bg, 3 * sizeof(double));
            prhs_u[6] = bbg;
            mxArray* PP = mxCreateDoubleMatrix(15, 15, mxREAL);
            memcpy(mxGetPr(PP), out_P, 15*15*sizeof(double));
            prhs_u[7] = PP;
            mxArray* gg = mxCreateDoubleMatrix(3, 1, mxREAL);
            memcpy(mxGetPr(gg), g, 3 * sizeof(double));
            prhs_u[8] = gg;
            prhs_u[9] = mxCreateDoubleScalar(dt);
            prhs_u[10] = mxCreateDoubleScalar(sample);
            
            if (mexCallMATLAB(7, plhs_u, 11, prhs_u, "mex_eskf_do_update") == 0) {
                if (!mxIsLogicalScalarTrue(plhs_u[6])) {
                    memcpy(out_p, mxGetPr(plhs_u[0]), 3 * sizeof(double));
                    memcpy(out_v, mxGetPr(plhs_u[1]), 3 * sizeof(double));
                    memcpy(out_q, mxGetPr(plhs_u[2]), 4 * sizeof(double));
                    memcpy(out_ba, mxGetPr(plhs_u[3]), 3 * sizeof(double));
                    memcpy(out_bg, mxGetPr(plhs_u[4]), 3 * sizeof(double));
                    memcpy(out_P, mxGetPr(plhs_u[5]), 15*15*sizeof(double));
                }
                for (int i = 0; i < 7; ++i) mxDestroyArray(plhs_u[i]);
            }
            for (int i = 0; i < 11; ++i) mxDestroyArray(prhs_u[i]);
        }
    }
    else if (strcmp(type, "mag") == 0 && meas_len == 3) {
        // Preprocess mag
        mxArray* prhs_pre[3];
        mxArray* plhs_pre[3];
        prhs_pre[0] = mxCreateString("preprocess_mag");
        mxArray* m_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
        memcpy(mxGetPr(m_arr), meas, 3 * sizeof(double));
        prhs_pre[1] = m_arr;
        mxArray* prev_m = mxCreateDoubleMatrix(3, 1, mxREAL);
        memcpy(mxGetPr(prev_m), s->prev_mag, 3 * sizeof(double));
        prhs_pre[2] = prev_m;
        
        double m_filtered[3];
        if (mexCallMATLAB(3, plhs_pre, 3, prhs_pre, "mex_sensor_preprocessor") == 0) {
            memcpy(m_filtered, mxGetPr(plhs_pre[0]), 3 * sizeof(double));
            bool is_outlier = mxIsLogicalScalarTrue(plhs_pre[1]);
            bool no_change = mxIsLogicalScalarTrue(plhs_pre[2]);
            
            if (!no_change && !is_nan_any(m_filtered, 3) && !is_outlier) {
                should_skip = false;
                memcpy(s->prev_mag, meas, 3 * sizeof(double));
            }
            for (int i = 0; i < 3; ++i) mxDestroyArray(plhs_pre[i]);
        }
        for (int i = 0; i < 3; ++i) mxDestroyArray(prhs_pre[i]);
        
        if (!should_skip) {
            // Call mex_eskf_do_update
            mxArray* prhs_u[11];
            mxArray* plhs_u[7];
            prhs_u[0] = mxCreateString("mag");
            mxArray* mf = mxCreateDoubleMatrix(3, 1, mxREAL);
            memcpy(mxGetPr(mf), m_filtered, 3 * sizeof(double));
            prhs_u[1] = mf;
            mxArray* pp = mxCreateDoubleMatrix(3, 1, mxREAL);
            memcpy(mxGetPr(pp), out_p, 3 * sizeof(double));
            prhs_u[2] = pp;
            mxArray* vv = mxCreateDoubleMatrix(3, 1, mxREAL);
            memcpy(mxGetPr(vv), out_v, 3 * sizeof(double));
            prhs_u[3] = vv;
            mxArray* qq = mxCreateDoubleMatrix(4, 1, mxREAL);
            memcpy(mxGetPr(qq), out_q, 4 * sizeof(double));
            prhs_u[4] = qq;
            mxArray* bba = mxCreateDoubleMatrix(3, 1, mxREAL);
            memcpy(mxGetPr(bba), out_ba, 3 * sizeof(double));
            prhs_u[5] = bba;
            mxArray* bbg = mxCreateDoubleMatrix(3, 1, mxREAL);
            memcpy(mxGetPr(bbg), out_bg, 3 * sizeof(double));
            prhs_u[6] = bbg;
            mxArray* PP = mxCreateDoubleMatrix(15, 15, mxREAL);
            memcpy(mxGetPr(PP), out_P, 15*15*sizeof(double));
            prhs_u[7] = PP;
            mxArray* gg = mxCreateDoubleMatrix(3, 1, mxREAL);
            memcpy(mxGetPr(gg), g, 3 * sizeof(double));
            prhs_u[8] = gg;
            prhs_u[9] = mxCreateDoubleScalar(dt);
            prhs_u[10] = mxCreateDoubleScalar(sample);
            
            if (mexCallMATLAB(7, plhs_u, 11, prhs_u, "mex_eskf_do_update") == 0) {
                if (!mxIsLogicalScalarTrue(plhs_u[6])) {
                    memcpy(out_p, mxGetPr(plhs_u[0]), 3 * sizeof(double));
                    memcpy(out_v, mxGetPr(plhs_u[1]), 3 * sizeof(double));
                    memcpy(out_q, mxGetPr(plhs_u[2]), 4 * sizeof(double));
                    memcpy(out_ba, mxGetPr(plhs_u[3]), 3 * sizeof(double));
                    memcpy(out_bg, mxGetPr(plhs_u[4]), 3 * sizeof(double));
                    memcpy(out_P, mxGetPr(plhs_u[5]), 15*15*sizeof(double));
                }
                for (int i = 0; i < 7; ++i) mxDestroyArray(plhs_u[i]);
            }
            for (int i = 0; i < 11; ++i) mxDestroyArray(prhs_u[i]);
        }
    }
    else if (strcmp(type, "baro") == 0 && meas_len == 1) {
        double pressure = meas[0];
        double prev_baro = s->prev_baro;
        
        if (fabs(pressure - prev_baro) > s->buffer_tolerance) {
            should_skip = false;
            s->prev_baro = pressure;
            
            // Preprocess baro
            mxArray* prhs_pre[2];
            mxArray* plhs_pre[1];
            prhs_pre[0] = mxCreateString("preprocess_baro");
            prhs_pre[1] = mxCreateDoubleScalar(pressure);
            
            double alt_baro = 0.0;
            if (mexCallMATLAB(1, plhs_pre, 2, prhs_pre, "mex_sensor_preprocessor") == 0) {
                alt_baro = mxGetScalar(plhs_pre[0]);
                mxDestroyArray(plhs_pre[0]);
            }
            for (int i = 0; i < 2; ++i) mxDestroyArray(prhs_pre[i]);
            
            if (!should_skip) {
                // Call mex_eskf_do_update
                mxArray* prhs_u[11];
                mxArray* plhs_u[7];
                prhs_u[0] = mxCreateString("baro");
                prhs_u[1] = mxCreateDoubleScalar(alt_baro);
                mxArray* pp = mxCreateDoubleMatrix(3, 1, mxREAL);
                memcpy(mxGetPr(pp), out_p, 3 * sizeof(double));
                prhs_u[2] = pp;
                mxArray* vv = mxCreateDoubleMatrix(3, 1, mxREAL);
                memcpy(mxGetPr(vv), out_v, 3 * sizeof(double));
                prhs_u[3] = vv;
                mxArray* qq = mxCreateDoubleMatrix(4, 1, mxREAL);
                memcpy(mxGetPr(qq), out_q, 4 * sizeof(double));
                prhs_u[4] = qq;
                mxArray* bba = mxCreateDoubleMatrix(3, 1, mxREAL);
                memcpy(mxGetPr(bba), out_ba, 3 * sizeof(double));
                prhs_u[5] = bba;
                mxArray* bbg = mxCreateDoubleMatrix(3, 1, mxREAL);
                memcpy(mxGetPr(bbg), out_bg, 3 * sizeof(double));
                prhs_u[6] = bbg;
                mxArray* PP = mxCreateDoubleMatrix(15, 15, mxREAL);
                memcpy(mxGetPr(PP), out_P, 15*15*sizeof(double));
                prhs_u[7] = PP;
                mxArray* gg = mxCreateDoubleMatrix(3, 1, mxREAL);
                memcpy(mxGetPr(gg), g, 3 * sizeof(double));
                prhs_u[8] = gg;
                prhs_u[9] = mxCreateDoubleScalar(dt);
                prhs_u[10] = mxCreateDoubleScalar(sample);
                
                if (mexCallMATLAB(7, plhs_u, 11, prhs_u, "mex_eskf_do_update") == 0) {
                    if (!mxIsLogicalScalarTrue(plhs_u[6])) {
                        memcpy(out_p, mxGetPr(plhs_u[0]), 3 * sizeof(double));
                        memcpy(out_v, mxGetPr(plhs_u[1]), 3 * sizeof(double));
                        memcpy(out_q, mxGetPr(plhs_u[2]), 4 * sizeof(double));
                        memcpy(out_ba, mxGetPr(plhs_u[3]), 3 * sizeof(double));
                        memcpy(out_bg, mxGetPr(plhs_u[4]), 3 * sizeof(double));
                        memcpy(out_P, mxGetPr(plhs_u[5]), 15*15*sizeof(double));
                    }
                    for (int i = 0; i < 7; ++i) mxDestroyArray(plhs_u[i]);
                }
                for (int i = 0; i < 11; ++i) mxDestroyArray(prhs_u[i]);
            }
        }
    }
    
    // Update state if not skipped
    if (!should_skip) {
        memcpy(s->p, out_p, 3 * sizeof(double));
        memcpy(s->v, out_v, 3 * sizeof(double));
        memcpy(s->q, out_q, 4 * sizeof(double));
        memcpy(s->ba, out_ba, 3 * sizeof(double));
        memcpy(s->bg, out_bg, 3 * sizeof(double));
        memcpy(s->P, out_P, 15*15*sizeof(double));
    }
}

// Call GPS sensor update (revert to original mexCallMATLAB implementation with preprocessing)
static void call_gps_update(ESKFState* s, double lat, double lon, double alt, double sample) {
    // Revert to original implementation from mex_eskf_sensor_updates_full.cpp
    double p[3], v[3], q[4], ba[3], bg[3], P[15*15], g[3];
    memcpy(p, s->p, 3 * sizeof(double));
    memcpy(v, s->v, 3 * sizeof(double));
    memcpy(q, s->q, 4 * sizeof(double));
    memcpy(ba, s->ba, 3 * sizeof(double));
    memcpy(bg, s->bg, 3 * sizeof(double));
    memcpy(P, s->P, 15*15*sizeof(double));
    memcpy(g, s->g, 3 * sizeof(double));
    double dt = s->dt;
    
    double out_p[3], out_v[3], out_q[4], out_ba[3], out_bg[3], out_P[15*15];
    memcpy(out_p, p, 3 * sizeof(double));
    memcpy(out_v, v, 3 * sizeof(double));
    memcpy(out_q, q, 4 * sizeof(double));
    memcpy(out_ba, ba, 3 * sizeof(double));
    memcpy(out_bg, bg, 3 * sizeof(double));
    memcpy(out_P, P, 15*15*sizeof(double));
    
    // Preprocess GPS
    mxArray* prhs_pre[5];
    mxArray* plhs_pre[3];
    prhs_pre[0] = mxCreateString("preprocess_gps");
    prhs_pre[1] = mxCreateDoubleScalar(lat);
    prhs_pre[2] = mxCreateDoubleScalar(lon);
    prhs_pre[3] = mxCreateDoubleScalar(alt);
    mxArray* go = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(go), s->gps_origin, 3 * sizeof(double));
    prhs_pre[4] = go;
    
    bool should_skip = true;
    double z_gps[3];
    
    if (mexCallMATLAB(3, plhs_pre, 5, prhs_pre, "mex_sensor_preprocessor") == 0) {
        memcpy(z_gps, mxGetPr(plhs_pre[0]), 3 * sizeof(double));
        bool is_outlier = mxIsLogicalScalarTrue(plhs_pre[1]);
        bool no_change = mxIsLogicalScalarTrue(plhs_pre[2]);
        
        if (!no_change && !is_outlier) {
            should_skip = false;
        }
        for (int i = 0; i < 3; ++i) mxDestroyArray(plhs_pre[i]);
    }
    for (int i = 0; i < 5; ++i) mxDestroyArray(prhs_pre[i]);
    
    if (!should_skip) {
        // Call mex_eskf_do_update
        mxArray* prhs_u[11];
        mxArray* plhs_u[7];
        prhs_u[0] = mxCreateString("gps");
        mxArray* zg = mxCreateDoubleMatrix(3, 1, mxREAL);
        memcpy(mxGetPr(zg), z_gps, 3 * sizeof(double));
        prhs_u[1] = zg;
        mxArray* pp = mxCreateDoubleMatrix(3, 1, mxREAL);
        memcpy(mxGetPr(pp), out_p, 3 * sizeof(double));
        prhs_u[2] = pp;
        mxArray* vv = mxCreateDoubleMatrix(3, 1, mxREAL);
        memcpy(mxGetPr(vv), out_v, 3 * sizeof(double));
        prhs_u[3] = vv;
        mxArray* qq = mxCreateDoubleMatrix(4, 1, mxREAL);
        memcpy(mxGetPr(qq), out_q, 4 * sizeof(double));
        prhs_u[4] = qq;
        mxArray* bba = mxCreateDoubleMatrix(3, 1, mxREAL);
        memcpy(mxGetPr(bba), out_ba, 3 * sizeof(double));
        prhs_u[5] = bba;
        mxArray* bbg = mxCreateDoubleMatrix(3, 1, mxREAL);
        memcpy(mxGetPr(bbg), out_bg, 3 * sizeof(double));
        prhs_u[6] = bbg;
        mxArray* PP = mxCreateDoubleMatrix(15, 15, mxREAL);
        memcpy(mxGetPr(PP), out_P, 15*15*sizeof(double));
        prhs_u[7] = PP;
        mxArray* gg = mxCreateDoubleMatrix(3, 1, mxREAL);
        memcpy(mxGetPr(gg), g, 3 * sizeof(double));
        prhs_u[8] = gg;
        prhs_u[9] = mxCreateDoubleScalar(dt);
        prhs_u[10] = mxCreateDoubleScalar(sample);
        
        if (mexCallMATLAB(7, plhs_u, 11, prhs_u, "mex_eskf_do_update") == 0) {
            if (!mxIsLogicalScalarTrue(plhs_u[6])) {
                memcpy(out_p, mxGetPr(plhs_u[0]), 3 * sizeof(double));
                memcpy(out_v, mxGetPr(plhs_u[1]), 3 * sizeof(double));
                memcpy(out_q, mxGetPr(plhs_u[2]), 4 * sizeof(double));
                memcpy(out_ba, mxGetPr(plhs_u[3]), 3 * sizeof(double));
                memcpy(out_bg, mxGetPr(plhs_u[4]), 3 * sizeof(double));
                memcpy(out_P, mxGetPr(plhs_u[5]), 15*15*sizeof(double));
            }
            for (int i = 0; i < 7; ++i) mxDestroyArray(plhs_u[i]);
        }
        for (int i = 0; i < 11; ++i) mxDestroyArray(prhs_u[i]);
        
        // Update prev_gps
        s->prev_gps_lat = lat;
        s->prev_gps_lon = lon;
        s->prev_gps_alt = alt;
    }
    
    // Update state if not skipped
    if (!should_skip) {
        memcpy(s->p, out_p, 3 * sizeof(double));
        memcpy(s->v, out_v, 3 * sizeof(double));
        memcpy(s->q, out_q, 4 * sizeof(double));
        memcpy(s->ba, out_ba, 3 * sizeof(double));
        memcpy(s->bg, out_bg, 3 * sizeof(double));
        memcpy(s->P, out_P, 15*15*sizeof(double));
    }
}

// Reset check (実装はSrc/Common/filter_management.cppに移動)
static void check_and_reset(ESKFState* s, int k) {
    // Check for divergence (implementation moved to Src/Common/filter_management.cpp)
    // Convert P matrix to Matrix type
    Matrix<15, 15, float> P_float;
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            P_float(i, j) = static_cast<float>(s->P[i + j*15]);
        }
    }
    
    Vector<3, float> p_float, v_float, ba_float, bg_float;
    Vector<4, float> q_float;
    for (int i = 0; i < 3; ++i) {
        p_float(i, 0) = static_cast<float>(s->p[i]);
        v_float(i, 0) = static_cast<float>(s->v[i]);
        ba_float(i, 0) = static_cast<float>(s->ba[i]);
        bg_float(i, 0) = static_cast<float>(s->bg[i]);
    }
    for (int i = 0; i < 4; ++i) {
        q_float(i, 0) = static_cast<float>(s->q[i]);
    }
    
    bool need_reset = check_state_divergence(p_float, v_float, q_float, ba_float, bg_float, P_float);
    
    if (need_reset) {
        s->last_reset_step = k;
        
        // Reset state using filter_management directly (mex_filter_management を統合)
        using namespace common::filter;
        Vector<3, float> v_float, ba_float, bg_float;
        Vector<4, float> q_float;
        Matrix<15, 15, float> P_float;
        
        // Reset P matrix using setIdentityScaled (reset_scale = 0.01)
        float reset_scale = 0.01f;
        setIdentityScaled(P_float, reset_scale);
        
        // Convert current state to float type
        for (int i = 0; i < 3; ++i) {
            v_float(i, 0) = static_cast<float>(s->v[i]);
            ba_float(i, 0) = static_cast<float>(s->ba[i]);
            bg_float(i, 0) = static_cast<float>(s->bg[i]);
        }
        for (int i = 0; i < 4; ++i) {
            q_float(i, 0) = static_cast<float>(s->q[i]);
        }
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                P_float(i, j) = static_cast<float>(s->P[i + j*15]);
            }
        }
        
        // Reset processing
        reset_state_on_divergence(v_float, ba_float, bg_float, q_float, P_float);
        
        // Convert results back to double type
        for (int i = 0; i < 3; ++i) {
            s->v[i] = static_cast<double>(v_float(i, 0));
            s->ba[i] = static_cast<double>(ba_float(i, 0));
            s->bg[i] = static_cast<double>(bg_float(i, 0));
        }
        for (int i = 0; i < 4; ++i) {
            s->q[i] = static_cast<double>(q_float(i, 0));
        }
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                s->P[i + j*15] = static_cast<double>(P_float(i, j));
            }
        }
    }
}

// ZUPT check and update (implementation moved to Src/Common/filter_management.cpp)
static void zupt_check_and_update(ESKFState* s, const double* a_meas, const double* w_meas) {
    // ZUPT check (implementation moved to Src/Common/filter_management.cpp)
    Vector<3, float> a_float, w_float;
    for (int i = 0; i < 3; ++i) {
        a_float(i, 0) = static_cast<float>(a_meas[i]);
        w_float(i, 0) = static_cast<float>(w_meas[i]);
    }
    
    bool stationary = check_zupt_condition(a_float, w_float, 
                                           static_cast<float>(s->zupt_threshold_accel),
                                           static_cast<float>(s->zupt_threshold_gyro));
    
    if (stationary) {
        s->zupt_counter++;
    } else {
        s->zupt_counter = 0;
    }
    
    s->is_stationary = (s->zupt_counter >= s->zupt_min_duration);
    
    if (s->is_stationary) {
        // ZUPT update using ESKFCore directly (mex_eskf_zupt を統合)
        using namespace eskf;
        Vector<3, float> v_in, v_out;
        Matrix<15, 15, float> P_in, P_out;
        
        // Convert double to float
        for (int i = 0; i < 3; ++i) {
            v_in(i, 0) = static_cast<float>(s->v[i]);
        }
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                P_in(i, j) = static_cast<float>(s->P[i + j*15]);
            }
        }
        
        // Call ESKFCore::update_zupt
        ESKFCore::update_zupt(v_in, P_in, v_out, P_out);
        
        // Convert back to double
        for (int i = 0; i < 3; ++i) {
            s->v[i] = static_cast<double>(v_out(i, 0));
        }
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                s->P[i + j*15] = static_cast<double>(P_out(i, j));
            }
        }
    }
}

// Helper: Get field from MATLAB struct
static const mxArray* get_field(const mxArray* s, const char* name) {
    if (!mxIsStruct(s)) return nullptr;
    return mxGetField(s, 0, name);
}

static const mxArray* get_field_any(const mxArray* s, const char* name1, const char* name2) {
    const mxArray* f = get_field(s, name1);
    if (f) return f;
    return get_field(s, name2);
}

static double* get_data(const mxArray* arr) {
    if (!arr) return nullptr;
    return mxGetPr(arr);
}

static int get_length(const mxArray* arr) {
    if (!arr) return 0;
    return static_cast<int>(mxGetNumberOfElements(arr));
}

// Initialize ESKF state (integrated from mex_eskf_constructor)
static uint64_t do_init(const mxArray* obs, double static_time, double dt) {
    ESKFState* s = new ESKFState();
    memset(s, 0, sizeof(ESKFState));
    s->valid = true;
    s->dt = dt;
    
    const double GRAVITY = 9.80665;
    const double DEG2RAD = 0.017453292519943295;
    
    // 静止サンプル数の計算
    int N_static = static_cast<int>(floor(static_time / dt));
    
    // 基本状態の初期化
    double p[3] = {0, 0, 0};
    double v[3] = {0, 0, 0};
    double g[3] = {0, 0, GRAVITY};
    double q[4] = {1, 0, 0, 0};
    double ba[3] = {0, 0, 0};
    double bg[3] = {0, 0, 0};
    
    // ノイズパラメータのデフォルト値
    double sigma_a = 0.1;
    double sigma_g = DEG2RAD * 0.1;
    double sigma_mag = 10.0;
    double sigma_press = 1.0;
    double sigma_gps = 1.0;
    double gyro_noise_threshold = DEG2RAD * 0.1;
    
    // GPS原点
    double gps_origin[3] = {0, 0, 0};
    
    // 静止データがある場合の処理
    const mxArray* ax_arr = get_field_any(obs, "ax", "accel_x");
    const mxArray* ay_arr = get_field_any(obs, "ay", "accel_y");
    const mxArray* az_arr = get_field_any(obs, "az", "accel_z");
    
    int n_samples = ax_arr ? get_length(ax_arr) : 0;
    if (N_static > n_samples) N_static = n_samples;
    
    if (ax_arr && ay_arr && az_arr && N_static > 10) {
        double* ax = get_data(ax_arr);
        double* ay = get_data(ay_arr);
        double* az = get_data(az_arr);
        
        // 加速度平均と標準偏差
        double accel_mean_x, accel_mean_y, accel_mean_z;
        compute_mean_3d(ax, ay, az, N_static, &accel_mean_x, &accel_mean_y, &accel_mean_z);
        sigma_a = compute_std_3d(ax, ay, az, N_static, accel_mean_x, accel_mean_y, accel_mean_z);
        if (sigma_a < 0.01) sigma_a = 0.01;
        
        // 初期姿勢計算（Roll/Pitch）
        double phi = atan2(-accel_mean_y, -accel_mean_z);
        double theta = atan2(accel_mean_x, sqrt(accel_mean_y*accel_mean_y + accel_mean_z*accel_mean_z));
        
        // ジャイロデータ
        const mxArray* wx_arr = get_field_any(obs, "wx", "gyro_x");
        const mxArray* wy_arr = get_field_any(obs, "wy", "gyro_y");
        const mxArray* wz_arr = get_field_any(obs, "wz", "gyro_z");
        
        if (wx_arr && wy_arr && wz_arr) {
            double* wx = get_data(wx_arr);
            double* wy = get_data(wy_arr);
            double* wz = get_data(wz_arr);
            
            double gyro_mean_x, gyro_mean_y, gyro_mean_z;
            compute_mean_3d(wx, wy, wz, N_static, &gyro_mean_x, &gyro_mean_y, &gyro_mean_z);
            double sigma_g_deg = compute_std_3d(wx, wy, wz, N_static, gyro_mean_x, gyro_mean_y, gyro_mean_z);
            sigma_g = DEG2RAD * sigma_g_deg;
            if (sigma_g < 0.001) sigma_g = 0.001;
            
            // gyro_noise_threshold の計算
            double std_wx = compute_std(wx, N_static, gyro_mean_x);
            double std_wy = compute_std(wy, N_static, gyro_mean_y);
            double std_wz = compute_std(wz, N_static, gyro_mean_z);
            double max_std = std_wx;
            if (std_wy > max_std) max_std = std_wy;
            if (std_wz > max_std) max_std = std_wz;
            gyro_noise_threshold = 2.0 * DEG2RAD * max_std;
        }
        
        // 磁気データからYaw計算
        const mxArray* mx_arr = get_field_any(obs, "mx", "mag_x");
        const mxArray* my_arr = get_field_any(obs, "my", "mag_y");
        const mxArray* mz_arr = get_field_any(obs, "mz", "mag_z");
        
        double psi = 0.0;
        if (mx_arr && my_arr && mz_arr) {
            double* mx = get_data(mx_arr);
            double* my = get_data(my_arr);
            double* mz = get_data(mz_arr);
            
            double mag_mean_x, mag_mean_y, mag_mean_z;
            compute_mean_3d(mx, my, mz, N_static, &mag_mean_x, &mag_mean_y, &mag_mean_z);
            sigma_mag = compute_std_3d(mx, my, mz, N_static, mag_mean_x, mag_mean_y, mag_mean_z);
            if (sigma_mag < 0.1) sigma_mag = 0.1;
            
            // Roll/Pitchのみのクォータニオン
            Quat quat_rp = Quat::from_euler(phi * 180.0 / M_PI, theta * 180.0 / M_PI, 0.0);
            quat_rp.normalize();
            
            // 回転行列
            double R_rp[9];
            quat_rp.to_rotation_matrix(R_rp);
            
            // 磁気を水平面に射影
            double m_level_x = R_rp[0]*mag_mean_x + R_rp[3]*mag_mean_y + R_rp[6]*mag_mean_z;
            double m_level_y = R_rp[1]*mag_mean_x + R_rp[4]*mag_mean_y + R_rp[7]*mag_mean_z;
            
            psi = -atan2(m_level_y, m_level_x);
        }
        
        // 最終クォータニオン
        Quat quat_final = Quat::from_euler(phi * 180.0 / M_PI, theta * 180.0 / M_PI, psi * 180.0 / M_PI);
        quat_final.normalize();
        q[0] = quat_final.w;
        q[1] = quat_final.x;
        q[2] = quat_final.y;
        q[3] = quat_final.z;
        
        // 気圧データ
        const mxArray* pressure_arr = get_field_any(obs, "pressure", "baro");
        if (pressure_arr) {
            double* pressure = get_data(pressure_arr);
            
            // 気圧高度計算
            std::vector<double> alt_baro(N_static);
            double alt_mean = 0.0;
            for (int i = 0; i < N_static; ++i) {
                alt_baro[i] = 44330.0 * (1.0 - pow(pressure[i] / 101325.0, 0.1903));
                alt_mean += alt_baro[i];
            }
            alt_mean /= N_static;
            
            double sum_sq = 0.0;
            for (int i = 0; i < N_static; ++i) {
                double diff = alt_baro[i] - alt_mean;
                sum_sq += diff * diff;
            }
            sigma_press = sqrt(sum_sq / (N_static - 1));
            if (sigma_press < 0.1) sigma_press = 0.1;
        }
        
        // GPSデータ
        const mxArray* lat_arr = get_field_any(obs, "lat", "gps_lat");
        const mxArray* lon_arr = get_field_any(obs, "lon", "gps_lon");
        const mxArray* alt_arr = get_field_any(obs, "alt", "gps_alt");
        
        if (lat_arr && lon_arr && alt_arr) {
            double* lat = get_data(lat_arr);
            double* lon = get_data(lon_arr);
            double* alt = get_data(alt_arr);
            
            // GPS原点計算（NaNを除外）
            double lat_sum = 0.0, lon_sum = 0.0, alt_sum = 0.0;
            int valid_count = 0;
            for (int i = 0; i < N_static; ++i) {
                if (!mxIsNaN(lat[i])) {
                    lat_sum += lat[i];
                    lon_sum += lon[i];
                    alt_sum += alt[i];
                    valid_count++;
                }
            }
            
            if (valid_count > 0) {
                gps_origin[0] = lat_sum / valid_count;
                gps_origin[1] = lon_sum / valid_count;
                gps_origin[2] = alt_sum / valid_count;
                
                // GPS標準偏差計算
                double cos_lat0 = cos(gps_origin[0] * DEG2RAD);
                std::vector<double> x_m(valid_count), y_m(valid_count), z_m(valid_count);
                int idx = 0;
                for (int i = 0; i < N_static; ++i) {
                    if (!mxIsNaN(lat[i])) {
                        y_m[idx] = (lat[i] - gps_origin[0]) / 9.0e-6;
                        x_m[idx] = (lon[i] - gps_origin[1]) / (9.0e-6 / cos_lat0);
                        z_m[idx] = alt[i] - gps_origin[2];
                        idx++;
                    }
                }
                
                double mean_x = 0, mean_y = 0, mean_z = 0;
                for (int i = 0; i < valid_count; ++i) {
                    mean_x += x_m[i];
                    mean_y += y_m[i];
                    mean_z += z_m[i];
                }
                mean_x /= valid_count;
                mean_y /= valid_count;
                mean_z /= valid_count;
                
                double std_x = compute_std(x_m.data(), valid_count, mean_x);
                double std_y = compute_std(y_m.data(), valid_count, mean_y);
                double std_z = compute_std(z_m.data(), valid_count, mean_z);
                sigma_gps = (std_x + std_y + std_z) / 3.0;
                if (sigma_gps < 0.1) sigma_gps = 0.1;
            }
        }
    }
    
    // Q行列の初期化
    double Q[15*15] = {0};
    for (int i = 3; i < 6; ++i) Q[i*15 + i] = 0.003 * 0.003;  // 速度
    for (int i = 6; i < 9; ++i) Q[i*15 + i] = 0.003 * 0.003;  // 姿勢
    for (int i = 9; i < 12; ++i) Q[i*15 + i] = sigma_a * sigma_a * 1e-3;  // 加速度バイアス
    for (int i = 12; i < 15; ++i) Q[i*15 + i] = sigma_g * sigma_g * 1e-3;  // ジャイロバイアス
    
    // P行列の初期化
    double P[15*15] = {0};
    for (int i = 0; i < 15; ++i) P[i*15 + i] = 0.01;
    for (int i = 0; i < 3; ++i) P[i*15 + i] = 5.0;  // 位置
    for (int i = 3; i < 6; ++i) P[i*15 + i] = 0.5;  // 速度
    for (int i = 9; i < 12; ++i) P[i*15 + i] = 0.5;  // 加速度バイアス
    for (int i = 12; i < 15; ++i) P[i*15 + i] = 0.1;  // ジャイロバイアス
    
    // 状態をコピー
    copy_vec(s->p, p, 3);
    copy_vec(s->v, v, 3);
    copy_vec(s->q, q, 4);
    copy_vec(s->ba, ba, 3);
    copy_vec(s->bg, bg, 3);
    memcpy(s->P, P, 15*15*sizeof(double));
    memcpy(s->Q_nominal, Q, 15*15*sizeof(double));
    copy_vec(s->g, g, 3);
    copy_vec(s->gps_origin, gps_origin, 3);
    s->gyro_noise_threshold = gyro_noise_threshold;
    
    // 前回値の初期化
    double zeros3[3] = {0, 0, 0};
    copy_vec(s->prev_accel, zeros3, 3);
    copy_vec(s->prev_gyro, zeros3, 3);
    copy_vec(s->prev_mag, zeros3, 3);
    s->prev_gps_lat = 0;
    s->prev_gps_lon = 0;
    s->prev_gps_alt = 0;
    s->prev_baro = 0;
    s->buffer_tolerance = 1e-9;
    
    // その他のパラメータ
    s->zupt_threshold_accel = 1.0;
    s->zupt_threshold_gyro = DEG2RAD * 3.0;
    s->zupt_min_duration = 10;
    s->zupt_counter = 0;
    s->is_stationary = false;
    s->adaptive_q_enabled = true;
    s->velocity_damping = 0.0;
    s->enable_accel_z_integration = true;
    s->accel_z_threshold = 0.5;
    s->accel_z_damping = 0.1;
    s->baro_weight = 0.2;
    copy_vec(s->w_body, zeros3, 3);
    s->last_reset_step = 0;
    
    uint64_t handle = g_next_handle++;
    g_states[handle] = s;
    return handle;
}

// Step
static void do_step(ESKFState* s, const mxArray* obs, int k) {
    int idx = k - 1;
    double a[3], w[3], m[3];
    getAccel(obs, idx, a);
    getGyro(obs, idx, w);
    getMag(obs, idx, m);
    
    // Convert gyro to rad/s
    double deg2rad = M_PI / 180.0;
    w[0] *= deg2rad; w[1] *= deg2rad; w[2] *= deg2rad;
    
    // Predict
    call_predict(s, a, w);
    
    // ZUPT check
    zupt_check_and_update(s, a, w);
    
    // Sensor updates
    call_sensor_update(s, "accel", a, 3, (double)k);
    call_sensor_update(s, "mag", m, 3, (double)k);
    
    // Baro
    mxArray* baro_field = mxGetField(obs, 0, "pressure");
    if (baro_field) {
        double baro = mxGetPr(baro_field)[idx];
        double baro_arr[1] = {baro};
        call_sensor_update(s, "baro", baro_arr, 1, (double)k);
    }
    
    // GPS
    mxArray* gps_lat = mxGetField(obs, 0, "lat");
    mxArray* gps_lon = mxGetField(obs, 0, "lon");
    mxArray* gps_alt = mxGetField(obs, 0, "alt");
    if (gps_lat && gps_lon && gps_alt) {
        double lat = mxGetPr(gps_lat)[idx];
        double lon = mxGetPr(gps_lon)[idx];
        double alt = mxGetPr(gps_alt)[idx];
        if (!std::isnan(lat) && !std::isnan(lon)) {
            call_gps_update(s, lat, lon, alt, (double)k);
        }
    }
    
    // Reset check
    check_and_reset(s, k);
}

// Get state
static mxArray* do_get_state(ESKFState* s) {
    const char* fields[] = {"p", "v", "q", "euler", "ba", "bg", "P"};
    mxArray* out = mxCreateStructMatrix(1, 1, 7, fields);
    
    mxArray* p = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(p), s->p, 3*sizeof(double));
    mxSetField(out, 0, "p", p);
    
    mxArray* v = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(v), s->v, 3*sizeof(double));
    mxSetField(out, 0, "v", v);
    
    mxArray* q = mxCreateDoubleMatrix(4, 1, mxREAL);
    memcpy(mxGetPr(q), s->q, 4*sizeof(double));
    mxSetField(out, 0, "q", q);
    
    double euler[3];
    quat_to_euler(s->q, euler);
    mxArray* eu = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* eu_ptr = mxGetPr(eu);
    eu_ptr[0] = euler[0] * 180.0 / M_PI;
    eu_ptr[1] = euler[1] * 180.0 / M_PI;
    eu_ptr[2] = euler[2] * 180.0 / M_PI;
    mxSetField(out, 0, "euler", eu);
    
    mxArray* ba = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(ba), s->ba, 3*sizeof(double));
    mxSetField(out, 0, "ba", ba);
    
    mxArray* bg = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(bg), s->bg, 3*sizeof(double));
    mxSetField(out, 0, "bg", bg);
    
    mxArray* P = mxCreateDoubleMatrix(15, 15, mxREAL);
    memcpy(mxGetPr(P), s->P, 15*15*sizeof(double));
    mxSetField(out, 0, "P", P);
    
    return out;
}

// Free
static void do_free(uint64_t handle) {
    auto it = g_states.find(handle);
    if (it != g_states.end()) {
        delete it->second;
        g_states.erase(it);
    }
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) mexErrMsgIdAndTxt("mex_run_eskf:usage", "Command required");
    std::string cmd = getCmd(prhs[0]);
    
    if (cmd == "init") {
        if (nrhs < 4) mexErrMsgIdAndTxt("mex_run_eskf:usage", "init requires (obs, static_time, dt)");
        const mxArray* obs = prhs[1];
        double static_time = mxGetScalar(prhs[2]);
        double dt = mxGetScalar(prhs[3]);
        uint64_t handle = do_init(obs, static_time, dt);
        plhs[0] = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
        *((uint64_t*)mxGetData(plhs[0])) = handle;
    }
    else if (cmd == "step") {
        if (nrhs < 4) mexErrMsgIdAndTxt("mex_run_eskf:usage", "step requires (handle, obs, k)");
        uint64_t handle = *((uint64_t*)mxGetData(prhs[1]));
        const mxArray* obs = prhs[2];
        int k = (int)mxGetScalar(prhs[3]);
        auto it = g_states.find(handle);
        if (it == g_states.end()) mexErrMsgIdAndTxt("mex_run_eskf:invalid", "Invalid handle");
        do_step(it->second, obs, k);
    }
    else if (cmd == "get_state") {
        if (nrhs < 2) mexErrMsgIdAndTxt("mex_run_eskf:usage", "get_state requires (handle)");
        uint64_t handle = *((uint64_t*)mxGetData(prhs[1]));
        auto it = g_states.find(handle);
        if (it == g_states.end()) mexErrMsgIdAndTxt("mex_run_eskf:invalid", "Invalid handle");
        plhs[0] = do_get_state(it->second);
    }
    else if (cmd == "free") {
        if (nrhs < 2) mexErrMsgIdAndTxt("mex_run_eskf:usage", "free requires (handle)");
        uint64_t handle = *((uint64_t*)mxGetData(prhs[1]));
        do_free(handle);
    }
    else {
        mexErrMsgIdAndTxt("mex_run_eskf:unknown", "Unknown command: %s", cmd.c_str());
    }
}

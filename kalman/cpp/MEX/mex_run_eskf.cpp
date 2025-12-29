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
#include "../Inc/Common/Math/quaternion_lib.hpp"
#include "../Inc/Common/Math/vector_utils.hpp"
#include "../Inc/Common/filter_management.hpp"
#include "../Inc/Common/Math/fixed_matrix.hpp"
#include "../Inc/Common/Sensor/sensor_filter.hpp"
#include "../Inc/ESKF/eskf_postprocess.hpp"
#include "../Inc/ESKF/eskf_core.hpp"
#include "../Inc/MEX/mex_type_conversion.hpp"
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace common::math;
using namespace common::filter;
using Quat = quat_lib::Quaternion<double>;
using QuatF = quat_lib::Quaternion<float>;
using namespace cmath_fx;
using namespace eskf;
using namespace mex_conv;

// ESKF State Structure
struct ESKFState {
    double p[3], v[3], q[4], ba[3], bg[3];
    double P[15*15];
    double Q_nominal[15*15];
    double g[3];
    double dt;
    double gps_origin[3];
    double prev_accel[3], prev_gyro[3], prev_mag[3];
    double prev_gps_lat, prev_gps_lon, prev_gps_alt;
    double prev_baro;
    double buffer_tolerance;
    double w_body[3];
    double velocity_damping;
    double baro_weight;
    double zupt_threshold_accel, zupt_threshold_gyro;
    int zupt_min_duration;
    int zupt_counter;
    bool is_stationary;
    bool adaptive_q_enabled;
    bool enable_accel_z_integration;
    double accel_z_threshold, accel_z_damping;
    double gyro_noise_threshold;
    int last_reset_step;
    bool valid;
};

static std::map<uint64_t, ESKFState*> g_states;
static uint64_t g_next_handle = 1;

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

// Call sensor update
static void call_sensor_update(ESKFState* s, const char* type, const double* meas, int meas_len, double sample) {
    // Create state struct
    const char* fields[] = {"p","v","q","ba","bg","P","g","dt","w_body","prev_accel","prev_mag","prev_baro","buffer_tolerance","baro_weight","gps_origin"};
    mxArray* state = mxCreateStructMatrix(1,1,15,fields);
    
    mxArray* f_p = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_p), s->p, 3);
    mxSetField(state, 0, "p", f_p);
    mxArray* f_v = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_v), s->v, 3);
    mxSetField(state, 0, "v", f_v);
    mxArray* f_q = mxCreateDoubleMatrix(4,1,mxREAL); copy_vec(mxGetPr(f_q), s->q, 4);
    mxSetField(state, 0, "q", f_q);
    mxArray* f_ba = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_ba), s->ba, 3);
    mxSetField(state, 0, "ba", f_ba);
    mxArray* f_bg = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_bg), s->bg, 3);
    mxSetField(state, 0, "bg", f_bg);
    mxArray* f_P = mxCreateDoubleMatrix(15,15,mxREAL); memcpy(mxGetPr(f_P), s->P, 15*15*8);
    mxSetField(state, 0, "P", f_P);
    mxArray* f_g = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_g), s->g, 3);
    mxSetField(state, 0, "g", f_g);
    mxSetField(state, 0, "dt", mxCreateDoubleScalar(s->dt));
    mxArray* f_wb = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_wb), s->w_body, 3);
    mxSetField(state, 0, "w_body", f_wb);
    mxArray* f_pa = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_pa), s->prev_accel, 3);
    mxSetField(state, 0, "prev_accel", f_pa);
    mxArray* f_pm = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_pm), s->prev_mag, 3);
    mxSetField(state, 0, "prev_mag", f_pm);
    mxSetField(state, 0, "prev_baro", mxCreateDoubleScalar(s->prev_baro));
    mxSetField(state, 0, "buffer_tolerance", mxCreateDoubleScalar(s->buffer_tolerance));
    mxSetField(state, 0, "baro_weight", mxCreateDoubleScalar(s->baro_weight));
    mxArray* f_go = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_go), s->gps_origin, 3);
    mxSetField(state, 0, "gps_origin", f_go);
    
    mxArray* prhs[4];
    mxArray* plhs[9]; // max outputs (gps has 9)
    
    prhs[0] = mxCreateString(type);
    prhs[1] = mxCreateDoubleMatrix(meas_len, 1, mxREAL); copy_vec(mxGetPr(prhs[1]), meas, meas_len);
    prhs[2] = state;
    prhs[3] = mxIsNaN(sample) ? mxCreateDoubleMatrix(0,0,mxREAL) : mxCreateDoubleScalar(sample);
    
    int nlhs_out = 7;
    if (strcmp(type, "gps") == 0) nlhs_out = 9;
    
    if (mexCallMATLAB(nlhs_out, plhs, 4, prhs, "mex_eskf_sensor_updates_full") == 0) {
        copy_vec(s->p, mxGetPr(plhs[0]), 3);
        copy_vec(s->v, mxGetPr(plhs[1]), 3);
        copy_vec(s->q, mxGetPr(plhs[2]), 4);
        copy_vec(s->ba, mxGetPr(plhs[3]), 3);
        copy_vec(s->bg, mxGetPr(plhs[4]), 3);
        memcpy(s->P, mxGetPr(plhs[5]), 15*15*8);
        
        if (strcmp(type, "accel") == 0) {
            copy_vec(s->prev_accel, mxGetPr(plhs[6]), 3);
        } else if (strcmp(type, "mag") == 0) {
            copy_vec(s->prev_mag, mxGetPr(plhs[6]), 3);
        } else if (strcmp(type, "baro") == 0) {
            s->prev_baro = mxGetScalar(plhs[6]);
        } else if (strcmp(type, "gps") == 0) {
            s->prev_gps_lat = mxGetScalar(plhs[6]);
            s->prev_gps_lon = mxGetScalar(plhs[7]);
            s->prev_gps_alt = mxGetScalar(plhs[8]);
        }
        for (int i=0; i<nlhs_out; i++) mxDestroyArray(plhs[i]);
    }
    
    mxDestroyArray(prhs[0]);
    mxDestroyArray(prhs[1]);
    mxDestroyArray(state);
    mxDestroyArray(prhs[3]);
}

// Call GPS sensor update (special case with multiple meas)
static void call_gps_update(ESKFState* s, double lat, double lon, double alt, double sample) {
    const char* fields[] = {"p","v","q","ba","bg","P","g","dt","gps_origin"};
    mxArray* state = mxCreateStructMatrix(1,1,9,fields);
    
    mxArray* f_p = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_p), s->p, 3);
    mxSetField(state, 0, "p", f_p);
    mxArray* f_v = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_v), s->v, 3);
    mxSetField(state, 0, "v", f_v);
    mxArray* f_q = mxCreateDoubleMatrix(4,1,mxREAL); copy_vec(mxGetPr(f_q), s->q, 4);
    mxSetField(state, 0, "q", f_q);
    mxArray* f_ba = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_ba), s->ba, 3);
    mxSetField(state, 0, "ba", f_ba);
    mxArray* f_bg = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_bg), s->bg, 3);
    mxSetField(state, 0, "bg", f_bg);
    mxArray* f_P = mxCreateDoubleMatrix(15,15,mxREAL); memcpy(mxGetPr(f_P), s->P, 15*15*8);
    mxSetField(state, 0, "P", f_P);
    mxArray* f_g = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_g), s->g, 3);
    mxSetField(state, 0, "g", f_g);
    mxSetField(state, 0, "dt", mxCreateDoubleScalar(s->dt));
    mxArray* f_go = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_go), s->gps_origin, 3);
    mxSetField(state, 0, "gps_origin", f_go);
    
    mxArray* prhs[6];
    mxArray* plhs[9];
    
    prhs[0] = mxCreateString("gps");
    prhs[1] = mxCreateDoubleScalar(lat);
    prhs[2] = mxCreateDoubleScalar(lon);
    prhs[3] = mxCreateDoubleScalar(alt);
    prhs[4] = state;
    prhs[5] = mxIsNaN(sample) ? mxCreateDoubleMatrix(0,0,mxREAL) : mxCreateDoubleScalar(sample);
    
    if (mexCallMATLAB(9, plhs, 6, prhs, "mex_eskf_sensor_updates_full") == 0) {
        copy_vec(s->p, mxGetPr(plhs[0]), 3);
        copy_vec(s->v, mxGetPr(plhs[1]), 3);
        copy_vec(s->q, mxGetPr(plhs[2]), 4);
        copy_vec(s->ba, mxGetPr(plhs[3]), 3);
        copy_vec(s->bg, mxGetPr(plhs[4]), 5);
        memcpy(s->P, mxGetPr(plhs[5]), 15*15*8);
        s->prev_gps_lat = mxGetScalar(plhs[6]);
        s->prev_gps_lon = mxGetScalar(plhs[7]);
        s->prev_gps_alt = mxGetScalar(plhs[8]);
        for (int i=0; i<9; i++) mxDestroyArray(plhs[i]);
    }
    
    for (int i=0; i<6; i++) mxDestroyArray(prhs[i]);
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

// Initialize using mex_eskf_constructor
static uint64_t do_init(const mxArray* obs, double static_time, double dt) {
    ESKFState* s = new ESKFState();
    memset(s, 0, sizeof(ESKFState));
    s->valid = true;
    s->dt = dt;
    
    // Call mex_eskf_constructor
    mxArray* prhs[4];
    mxArray* plhs[1];
    
    prhs[0] = mxCreateString("init");
    prhs[1] = const_cast<mxArray*>(obs);
    prhs[2] = mxCreateDoubleScalar(static_time);
    prhs[3] = mxCreateDoubleScalar(dt);
    
    if (mexCallMATLAB(1, plhs, 4, prhs, "mex_eskf_constructor") == 0) {
        mxArray* init_data = plhs[0];
        
        // Extract state
        copy_vec(s->p, mxGetPr(mxGetField(init_data, 0, "p")), 3);
        copy_vec(s->v, mxGetPr(mxGetField(init_data, 0, "v")), 3);
        copy_vec(s->q, mxGetPr(mxGetField(init_data, 0, "q")), 4);
        copy_vec(s->ba, mxGetPr(mxGetField(init_data, 0, "ba")), 3);
        copy_vec(s->bg, mxGetPr(mxGetField(init_data, 0, "bg")), 3);
        memcpy(s->P, mxGetPr(mxGetField(init_data, 0, "P")), 15*15*8);
        memcpy(s->Q_nominal, mxGetPr(mxGetField(init_data, 0, "Q_nominal")), 15*15*8);
        copy_vec(s->g, mxGetPr(mxGetField(init_data, 0, "g")), 3);
        s->dt = mxGetScalar(mxGetField(init_data, 0, "dt"));
        copy_vec(s->gps_origin, mxGetPr(mxGetField(init_data, 0, "gps_origin")), 3);
        s->gyro_noise_threshold = mxGetScalar(mxGetField(init_data, 0, "gyro_noise_threshold"));
        
        copy_vec(s->prev_accel, mxGetPr(mxGetField(init_data, 0, "prev_accel")), 3);
        copy_vec(s->prev_gyro, mxGetPr(mxGetField(init_data, 0, "prev_gyro")), 3);
        copy_vec(s->prev_mag, mxGetPr(mxGetField(init_data, 0, "prev_mag")), 3);
        s->prev_gps_lat = mxGetScalar(mxGetField(init_data, 0, "prev_gps_lat"));
        s->prev_gps_lon = mxGetScalar(mxGetField(init_data, 0, "prev_gps_lon"));
        s->prev_gps_alt = mxGetScalar(mxGetField(init_data, 0, "prev_gps_alt"));
        s->prev_baro = mxGetScalar(mxGetField(init_data, 0, "prev_baro"));
        s->buffer_tolerance = mxGetScalar(mxGetField(init_data, 0, "buffer_tolerance"));
        
        s->zupt_threshold_accel = mxGetScalar(mxGetField(init_data, 0, "zupt_threshold_accel"));
        s->zupt_threshold_gyro = mxGetScalar(mxGetField(init_data, 0, "zupt_threshold_gyro"));
        s->zupt_min_duration = (int)mxGetScalar(mxGetField(init_data, 0, "zupt_min_duration"));
        s->zupt_counter = (int)mxGetScalar(mxGetField(init_data, 0, "zupt_counter"));
        s->is_stationary = mxIsLogicalScalarTrue(mxGetField(init_data, 0, "is_stationary"));
        
        s->adaptive_q_enabled = mxIsLogicalScalarTrue(mxGetField(init_data, 0, "adaptive_q_enabled"));
        s->velocity_damping = mxGetScalar(mxGetField(init_data, 0, "velocity_damping"));
        
        s->enable_accel_z_integration = mxIsLogicalScalarTrue(mxGetField(init_data, 0, "enable_accel_z_integration"));
        s->accel_z_threshold = mxGetScalar(mxGetField(init_data, 0, "accel_z_threshold"));
        s->accel_z_damping = mxGetScalar(mxGetField(init_data, 0, "accel_z_damping"));
        s->baro_weight = mxGetScalar(mxGetField(init_data, 0, "baro_weight"));
        
        copy_vec(s->w_body, mxGetPr(mxGetField(init_data, 0, "w_body")), 3);
        s->last_reset_step = (int)mxGetScalar(mxGetField(init_data, 0, "last_reset_step"));
        
        mxDestroyArray(init_data);
    } else {
        delete s;
        mexErrMsgIdAndTxt("mex_run_eskf:init", "mex_eskf_constructor failed");
    }
    
    mxDestroyArray(prhs[0]);
    mxDestroyArray(prhs[2]);
    mxDestroyArray(prhs[3]);
    
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

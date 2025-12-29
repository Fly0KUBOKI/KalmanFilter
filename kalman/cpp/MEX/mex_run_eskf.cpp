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
#include "../Inc/ESKF/eskf_state.hpp"
#include "../Inc/ESKF/eskf_runner.hpp"
#include "../Inc/Common/Math/fixed_matrix.hpp"
#include "../Inc/Common/Math/quaternion_lib.hpp"
#include "../Inc/Common/Math/vector_utils.hpp"
#include "../Inc/Common/Math/statistics.hpp"
#include "../Inc/Common/filter_management.hpp"
#include "../Inc/Common/Sensor/sensor_filter.hpp"
#include "../Inc/Common/Sensor/sensor_preprocessor.hpp"
#include "../Inc/ESKF/eskf_postprocess.hpp"
#include "../Inc/ESKF/eskf_core.hpp"
#include "../Inc/MEX/mex_type_conversion.hpp"
#include <vector>
#include <algorithm>

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

static std::map<uint64_t, ESKFState*> g_states;
static uint64_t g_next_handle = 1;

// Static instance of SensorFilterLib for reuse (mex_eskf_do_update統合用)
// Note: ESKFRunner also has its own filter_lib_, but we keep this for sensor_update/gps_update
static SensorFilterLib g_filter_lib;

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

// Helper functions for field access (mex_eskf_constructor統合用)
static const mxArray* get_field(const mxArray* s, const char* name) {
    if (!mxIsStruct(s)) return nullptr;
    return mxGetField(s, 0, name);
}

static bool has_field(const mxArray* s, const char* name) {
    return get_field(s, name) != nullptr;
}

static bool has_field_any(const mxArray* s, const char* name1, const char* name2) {
    return has_field(s, name1) || has_field(s, name2);
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

// Constants
static const double GRAVITY = 9.80665;
static const double DEG2RAD = 0.017453292519943295;
static const double RAD2DEG = 57.29577951308232;

// Quaternion helper functions (mex_eskf_constructor統合用)
static void quaternion_from_euler(double roll, double pitch, double yaw, double* q) {
    // quaternion_lib.hppのfrom_eulerは度数法を期待するが、ここではラジアンを使用
    // ラジアンを度数に変換
    double roll_deg = roll * RAD2DEG;
    double pitch_deg = pitch * RAD2DEG;
    double yaw_deg = yaw * RAD2DEG;
    
    Quat quat = Quat::from_euler(roll_deg, pitch_deg, yaw_deg);
    q[0] = quat.w;
    q[1] = quat.x;
    q[2] = quat.y;
    q[3] = quat.z;
}

static void quaternion_to_rotation_matrix(const double* q, double* R) {
    Quat quat(q[0], q[1], q[2], q[3]);
    quat.normalize();
    
    // quaternion_lib.hppのto_rotation_matrixはrow-majorで返す
    // MATLABはcolumn-majorなので変換が必要
    double R_row[9];
    quat.to_rotation_matrix(R_row);
    
    // row-major -> column-major変換
    R[0] = R_row[0]; R[3] = R_row[1]; R[6] = R_row[2];
    R[1] = R_row[3]; R[4] = R_row[4]; R[7] = R_row[5];
    R[2] = R_row[6]; R[5] = R_row[7]; R[8] = R_row[8];
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

// Predict using ESKFRunner (moved to Src/ESKF/eskf_runner.cpp)
static void call_predict(ESKFState* s, const double* a_meas, const double* w_meas) {
    ESKFRunner::predict(s, a_meas, w_meas);
}

// Helper function: check if any value is NaN
static bool is_nan_any(const double* v, int n) {
    for (int i = 0; i < n; ++i) {
        if (std::isnan(v[i])) return true;
    }
    return false;
}

// Helper function: compute 3D vector norm
static double norm3(const double* v) {
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// Call sensor update (mex_eskf_sensor_updates_full を統合、前処理はsensor_preprocessor.hppを使用)
static void call_sensor_update(ESKFState* s, const char* type, const double* meas, int meas_len, double sample) {
    // 出力バッファ
    double out_p[3], out_v[3], out_q[4], out_ba[3], out_bg[3], out_P[15*15];
    copy_vec(out_p, s->p, 3);
    copy_vec(out_v, s->v, 3);
    copy_vec(out_q, s->q, 4);
    copy_vec(out_ba, s->ba, 3);
    copy_vec(out_bg, s->bg, 3);
    memcpy(out_P, s->P, 15*15*sizeof(double));
    
    bool should_skip = true;
    
    if (strcmp(type, "accel") == 0) {
        // Accel前処理 (sensor_preprocessor.hppを使用)
        Vector<3, float> a_meas_f, prev_accel_f;
        for (int i = 0; i < 3; ++i) {
            a_meas_f(i, 0) = static_cast<float>(meas[i]);
            prev_accel_f(i, 0) = static_cast<float>(s->prev_accel[i]);
        }
        
        PreprocessResult pre_result = preprocess_accel(a_meas_f, prev_accel_f, s->buffer_tolerance);
        
        double a_corrected[3];
        for (int i = 0; i < 3; ++i) {
            a_corrected[i] = static_cast<double>(pre_result.output(i, 0));
        }
        
        if (!pre_result.no_change && !is_nan_any(a_corrected, 3) && !pre_result.is_outlier && (norm3(s->w_body) <= 1.5)) {
            should_skip = false;
            copy_vec(s->prev_accel, meas, 3);
        }
        
        if (!should_skip) {
            // mex_eskf_do_updateロジックを統合（mex_sensor_filter呼び出しを直接C++関数に置き換え）
            const char* sensor_type = "accel";
            const double* meas = a_corrected;
            int meas_len = 3;
            
            // R取得（mex_sensor_filter → g_filter_lib.noise_estimator.get_R_matrix()に置き換え）
            double R_noise[3] = {0.01, 0.01, 0.01};
            {
                cm R = g_filter_lib.noise_estimator.get_R_matrix(sensor_type);
                int n = R.rows * R.cols;
                if (n == 9) {
                    R_noise[0] = static_cast<double>(R(0, 0));
                    R_noise[1] = static_cast<double>(R(1, 1));
                    R_noise[2] = static_cast<double>(R(2, 2));
                } else if (n >= 3) {
                    R_noise[0] = static_cast<double>(R(0, 0));
                    R_noise[1] = static_cast<double>(R(1, 0));
                    R_noise[2] = static_cast<double>(R(2, 0));
                } else if (n == 1) {
                    R_noise[0] = R_noise[1] = R_noise[2] = static_cast<double>(R(0, 0));
                }
            }
            
            // sensor_data構造体を構築
            const char* sd_fields[] = {"accel", "gyro", "mag", "gps_pos", "alt_baro", "dt",
                "update_accel", "update_gyro", "update_mag", "update_gps", "update_baro", "update_zupt"};
            mxArray* sensor_data = mxCreateStructMatrix(1, 1, 12, sd_fields);
            
            double zeros3[3] = {0, 0, 0};
            mxArray* accel_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(accel_arr), meas, 3);
            mxArray* gyro_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(gyro_arr), zeros3, 3);
            mxArray* mag_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(mag_arr), zeros3, 3);
            mxArray* gps_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(gps_arr), zeros3, 3);
            
            mxSetField(sensor_data, 0, "accel", accel_arr);
            mxSetField(sensor_data, 0, "gyro", gyro_arr);
            mxSetField(sensor_data, 0, "mag", mag_arr);
            mxSetField(sensor_data, 0, "gps_pos", gps_arr);
            mxSetField(sensor_data, 0, "alt_baro", mxCreateDoubleScalar(0));
            mxSetField(sensor_data, 0, "dt", mxCreateDoubleScalar(s->dt));
            mxSetField(sensor_data, 0, "update_accel", mxCreateLogicalScalar(true));
            mxSetField(sensor_data, 0, "update_gyro", mxCreateLogicalScalar(false));
            mxSetField(sensor_data, 0, "update_mag", mxCreateLogicalScalar(false));
            mxSetField(sensor_data, 0, "update_gps", mxCreateLogicalScalar(false));
            mxSetField(sensor_data, 0, "update_baro", mxCreateLogicalScalar(false));
            mxSetField(sensor_data, 0, "update_zupt", mxCreateLogicalScalar(false));
            
            // mex_params構造体
            const char* mp_fields[] = {"g", "mag_ref", "noise_accel", "noise_gyro", "noise_ba", "noise_bg",
                "noise_mag", "noise_gps", "noise_baro", "noise_zupt", "alpha", "beta", "kappa", "trace_sample"};
            mxArray* mex_params = mxCreateStructMatrix(1, 1, 14, mp_fields);
            
            mxArray* g_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(g_arr), s->g, 3);
            mxSetField(mex_params, 0, "g", g_arr);
            double mag_ref[3] = {50, 0, 0};
            mxArray* mag_ref_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(mag_ref_arr), mag_ref, 3);
            mxSetField(mex_params, 0, "mag_ref", mag_ref_arr);
            
            mxArray* na = mxCreateDoubleMatrix(3, 1, mxREAL);
            for (int i = 0; i < 3; ++i) R_noise[i] *= 1.5;
            copy_vec(mxGetPr(na), R_noise, 3);
            mxArray* ng = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(ng), zeros3, 3);
            mxArray* nba = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(nba), zeros3, 3);
            mxArray* nbg = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(nbg), zeros3, 3);
            mxArray* nm = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(nm), zeros3, 3);
            mxArray* ngps = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(ngps), zeros3, 3);
            mxArray* nz = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(nz), zeros3, 3);
            
            mxSetField(mex_params, 0, "noise_accel", na);
            mxSetField(mex_params, 0, "noise_gyro", ng);
            mxSetField(mex_params, 0, "noise_ba", nba);
            mxSetField(mex_params, 0, "noise_bg", nbg);
            mxSetField(mex_params, 0, "noise_mag", nm);
            mxSetField(mex_params, 0, "noise_gps", ngps);
            mxSetField(mex_params, 0, "noise_baro", mxCreateDoubleScalar(0));
            mxSetField(mex_params, 0, "noise_zupt", nz);
            mxSetField(mex_params, 0, "alpha", mxCreateDoubleScalar(1e-3));
            mxSetField(mex_params, 0, "beta", mxCreateDoubleScalar(2));
            mxSetField(mex_params, 0, "kappa", mxCreateDoubleScalar(0));
            mxSetField(mex_params, 0, "trace_sample", mxCreateDoubleScalar(sample));
            
            // state構造体
            const char* st_fields[] = {"p", "v", "q", "ba", "bg", "P"};
            mxArray* state_s = mxCreateStructMatrix(1, 1, 6, st_fields);
            mxArray* p_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(p_arr), out_p, 3);
            mxArray* v_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(v_arr), out_v, 3);
            mxArray* q_arr = mxCreateDoubleMatrix(4, 1, mxREAL); copy_vec(mxGetPr(q_arr), out_q, 4);
            mxArray* ba_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(ba_arr), out_ba, 3);
            mxArray* bg_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(bg_arr), out_bg, 3);
            mxArray* P_arr = mxCreateDoubleMatrix(15, 15, mxREAL); memcpy(mxGetPr(P_arr), out_P, 15*15*8);
            mxSetField(state_s, 0, "p", p_arr);
            mxSetField(state_s, 0, "v", v_arr);
            mxSetField(state_s, 0, "q", q_arr);
            mxSetField(state_s, 0, "ba", ba_arr);
            mxSetField(state_s, 0, "bg", bg_arr);
            mxSetField(state_s, 0, "P", P_arr);
            
            // mex_meukf_step_v2 呼び出し（維持）
            mxArray* prhs_m[3] = {state_s, sensor_data, mex_params};
            mxArray* plhs_m[3];
            if (mexCallMATLAB(3, plhs_m, 3, prhs_m, "mex_meukf_step_v2") == 0) {
                mxArray* new_state = plhs_m[0];
                mxArray* dbg_out = plhs_m[1];
                
                // noise estimate更新（mex_sensor_filter → g_filter_lib.noise_estimator.estimate()に置き換え）
                if (mxIsStruct(dbg_out) && mxGetField(dbg_out, 0, "innov") && mxGetField(dbg_out, 0, "H")) {
                    // Convert innov to cm
                    Vector<3, float> innov_vec;
                    if (matToVector<3>(mxGetField(dbg_out, 0, "innov"), innov_vec)) {
                        cm innov_cm(3, 1);
                        for (int i = 0; i < 3; ++i) innov_cm(i, 0) = innov_vec(i, 0);
                        
                        // Convert H to cm
                        int H_rows = mxGetM(mxGetField(dbg_out, 0, "H"));
                        int H_cols = mxGetN(mxGetField(dbg_out, 0, "H"));
                        cm H_cm(H_rows, H_cols);
                        std::vector<float> H_tmp(static_cast<size_t>(H_rows) * static_cast<size_t>(H_cols));
                        mex_conv::mxArrayToFloatArray(mxGetField(dbg_out, 0, "H"), H_tmp.data(), static_cast<size_t>(H_rows) * static_cast<size_t>(H_cols));
                        for (int j = 0; j < H_cols; ++j) {
                            for (int i = 0; i < H_rows; ++i) {
                                H_cm(i, j) = H_tmp[static_cast<size_t>(j) * static_cast<size_t>(H_rows) + static_cast<size_t>(i)];
                            }
                        }
                        
                        // Convert P to cm
                        cm P_cm(15, 15);
                        std::vector<float> P_tmp(15 * 15);
                        mex_conv::mxArrayToFloatArray(P_arr, P_tmp.data(), 15 * 15);
                        for (int j = 0; j < 15; ++j) {
                            for (int i = 0; i < 15; ++i) {
                                P_cm(i, j) = P_tmp[static_cast<size_t>(j) * 15 + static_cast<size_t>(i)];
                            }
                        }
                        
                        // Call noise_estimator.estimate directly
                        g_filter_lib.noise_estimator.estimate(sensor_type, innov_cm, H_cm, P_cm);
                    }
                }
                
                // postprocess (mex_eskf_update_postprocess を統合)
                if (mxIsStruct(dbg_out) && mxGetField(dbg_out, 0, "dx")) {
                    // Get dx (15x1)
                    Vector<15, float> dx;
                    if (!matToVector<15>(mxGetField(dbg_out, 0, "dx"), dx)) {
                        // Fallback: use new_state directly
                        copy_vec(out_p, mxGetPr(mxGetField(new_state, 0, "p")), 3);
                        copy_vec(out_v, mxGetPr(mxGetField(new_state, 0, "v")), 3);
                        copy_vec(out_q, mxGetPr(mxGetField(new_state, 0, "q")), 4);
                        copy_vec(out_ba, mxGetPr(mxGetField(new_state, 0, "ba")), 3);
                        copy_vec(out_bg, mxGetPr(mxGetField(new_state, 0, "bg")), 3);
                        if (mxGetField(new_state, 0, "P")) {
                            double* new_P = mxGetPr(mxGetField(new_state, 0, "P"));
                            for (int i = 0; i < 15; ++i) {
                                for (int j = 0; j < 15; ++j) {
                                    out_P[i + j*15] = 0.5 * (new_P[i + j*15] + new_P[j + i*15]);
                                }
                            }
                        }
                        should_skip = false;
                    } else {
                        // Get innov
                        const mxArray* innov = mxGetField(dbg_out, 0, "innov");
                        
                        // Get state
                        Vector<3, float> state_p, state_v, state_ba, state_bg;
                        Vector<4, float> state_q;
                        Matrix<15, 15, float> state_P, new_state_P;
                        
                        if (!matToVector<3>(p_arr, state_p)) state_p = Vector<3, float>::Zero();
                        if (!matToVector<3>(v_arr, state_v)) state_v = Vector<3, float>::Zero();
                        if (!matToVector<4>(q_arr, state_q)) state_q = Vector<4, float>::Zero();
                        if (!matToVector<3>(ba_arr, state_ba)) state_ba = Vector<3, float>::Zero();
                        if (!matToVector<3>(bg_arr, state_bg)) state_bg = Vector<3, float>::Zero();
                        if (!matToMatrix<15, 15>(P_arr, state_P)) state_P = Matrix<15, 15, float>::Zero();
                        if (mxGetField(new_state, 0, "P")) {
                            if (!matToMatrix<15, 15>(mxGetField(new_state, 0, "P"), new_state_P)) {
                                new_state_P = state_P;
                            }
                        } else {
                            new_state_P = state_P;
                        }
                        
                        // Call divergence_guard.check_and_attenuate directly (mex_sensor_filter → g_filter_lib.divergence_guard.check_and_attenuate()に置き換え)
                        cm innov_cm;
                        if (mxGetField(dbg_out, 0, "innov")) {
                            int innov_len = mxGetM(mxGetField(dbg_out, 0, "innov"));
                            innov_cm.resize(innov_len, 1);
                            std::vector<float> innov_tmp(static_cast<size_t>(innov_len));
                            mex_conv::mxArrayToFloatArray(mxGetField(dbg_out, 0, "innov"), innov_tmp.data(), static_cast<size_t>(innov_len));
                            for (int i = 0; i < innov_len; ++i) innov_cm(i, 0) = innov_tmp[i];
                        }
                        
                        cm dx_cm(15, 1);
                        for (int i = 0; i < 15; ++i) dx_cm(i, 0) = dx(i, 0);
                        
                        bool was_attenuated = false;
                        bool should_skip_div = g_filter_lib.divergence_guard.check_and_attenuate(sensor_type, innov_cm, dx_cm, was_attenuated);
                        
                        // Get dx_out from dx_cm
                        Vector<15, float> dx_out;
                        for (int i = 0; i < 15; ++i) dx_out(i, 0) = dx_cm(i, 0);
                        
                        should_skip = should_skip_div;
                        
                        // Output: new_state (p, v, q, ba, bg, P), should_skip
                        Vector<3, float> new_p, new_v, new_ba, new_bg;
                        Vector<4, float> new_q;
                        Matrix<15, 15, float> out_P_mat = new_state_P;
                        
                        if (should_skip) {
                            // Return original state
                            new_p = state_p;
                            new_v = state_v;
                            new_q = state_q;
                            new_ba = state_ba;
                            new_bg = state_bg;
                            out_P_mat = state_P;
                        } else if (was_attenuated) {
                            // Apply dx_out
                            UpdatePostprocessResult updated = update_state_from_dx(dx_out, state_p, state_v, state_q, state_ba, state_bg, new_state_P);
                            new_p = updated.p;
                            new_v = updated.v;
                            new_q = updated.q;
                            new_ba = updated.ba;
                            new_bg = updated.bg;
                            out_P_mat = updated.P;
                        } else {
                            // Use new_state directly (no attenuation)
                            UpdatePostprocessResult updated = update_state_from_dx(dx, state_p, state_v, state_q, state_ba, state_bg, new_state_P);
                            new_p = updated.p;
                            new_v = updated.v;
                            new_q = updated.q;
                            new_ba = updated.ba;
                            new_bg = updated.bg;
                            out_P_mat = updated.P;
                        }
                        
                        // Convert back to double arrays
                        for (int i = 0; i < 3; ++i) {
                            out_p[i] = static_cast<double>(new_p(i, 0));
                            out_v[i] = static_cast<double>(new_v(i, 0));
                            out_ba[i] = static_cast<double>(new_ba(i, 0));
                            out_bg[i] = static_cast<double>(new_bg(i, 0));
                        }
                        for (int i = 0; i < 4; ++i) {
                            out_q[i] = static_cast<double>(new_q(i, 0));
                        }
                        for (int i = 0; i < 15; ++i) {
                            for (int j = 0; j < 15; ++j) {
                                out_P[i + j*15] = static_cast<double>(out_P_mat(i, j));
                            }
                        }
                    }
                } else {
                    copy_vec(out_p, mxGetPr(mxGetField(new_state, 0, "p")), 3);
                    copy_vec(out_v, mxGetPr(mxGetField(new_state, 0, "v")), 3);
                    copy_vec(out_q, mxGetPr(mxGetField(new_state, 0, "q")), 4);
                    copy_vec(out_ba, mxGetPr(mxGetField(new_state, 0, "ba")), 3);
                    copy_vec(out_bg, mxGetPr(mxGetField(new_state, 0, "bg")), 3);
                    if (mxGetField(new_state, 0, "P")) {
                        double* new_P = mxGetPr(mxGetField(new_state, 0, "P"));
                        for (int i = 0; i < 15; ++i) {
                            for (int j = 0; j < 15; ++j) {
                                out_P[i + j*15] = 0.5 * (new_P[i + j*15] + new_P[j + i*15]);
                            }
                        }
                    }
                }
                
                for (int i = 0; i < 3; ++i) mxDestroyArray(plhs_m[i]);
            }
            
            mxDestroyArray(sensor_data);
            mxDestroyArray(mex_params);
            mxDestroyArray(state_s);
        }
        
        // 出力を状態にコピー
        copy_vec(s->p, out_p, 3);
        copy_vec(s->v, out_v, 3);
        copy_vec(s->q, out_q, 4);
        copy_vec(s->ba, out_ba, 3);
        copy_vec(s->bg, out_bg, 3);
        memcpy(s->P, out_P, 15*15*sizeof(double));
        
    } else if (strcmp(type, "mag") == 0) {
        // Mag前処理 (sensor_preprocessor.hppを使用)
        Vector<3, float> m_meas_f, prev_mag_f;
        for (int i = 0; i < 3; ++i) {
            m_meas_f(i, 0) = static_cast<float>(meas[i]);
            prev_mag_f(i, 0) = static_cast<float>(s->prev_mag[i]);
        }
        
        PreprocessResult pre_result = preprocess_mag(m_meas_f, prev_mag_f, s->buffer_tolerance);
        
        double m_filtered[3];
        for (int i = 0; i < 3; ++i) {
            m_filtered[i] = static_cast<double>(pre_result.output(i, 0));
        }
        
        if (!pre_result.no_change && !is_nan_any(m_filtered, 3) && !pre_result.is_outlier) {
            should_skip = false;
            copy_vec(s->prev_mag, meas, 3);
        }
        
        if (!should_skip) {
            // mex_eskf_do_updateロジックを統合（mex_sensor_filter呼び出しを直接C++関数に置き換え）
            const char* sensor_type = "mag";
            const double* meas = m_filtered;
            int meas_len = 3;
            
            // R取得（mex_sensor_filter → g_filter_lib.noise_estimator.get_R_matrix()に置き換え）
            double R_noise[3] = {0.01, 0.01, 0.01};
            {
                cm R = g_filter_lib.noise_estimator.get_R_matrix(sensor_type);
                int n = R.rows * R.cols;
                if (n == 9) {
                    R_noise[0] = static_cast<double>(R(0, 0));
                    R_noise[1] = static_cast<double>(R(1, 1));
                    R_noise[2] = static_cast<double>(R(2, 2));
                } else if (n >= 3) {
                    R_noise[0] = static_cast<double>(R(0, 0));
                    R_noise[1] = static_cast<double>(R(1, 0));
                    R_noise[2] = static_cast<double>(R(2, 0));
                } else if (n == 1) {
                    R_noise[0] = R_noise[1] = R_noise[2] = static_cast<double>(R(0, 0));
                }
            }
            
            // sensor_data構造体を構築
            const char* sd_fields[] = {"accel", "gyro", "mag", "gps_pos", "alt_baro", "dt",
                "update_accel", "update_gyro", "update_mag", "update_gps", "update_baro", "update_zupt"};
            mxArray* sensor_data = mxCreateStructMatrix(1, 1, 12, sd_fields);
            
            double zeros3[3] = {0, 0, 0};
            mxArray* accel_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(accel_arr), zeros3, 3);
            mxArray* gyro_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(gyro_arr), zeros3, 3);
            mxArray* mag_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(mag_arr), meas, 3);
            mxArray* gps_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(gps_arr), zeros3, 3);
            
            mxSetField(sensor_data, 0, "accel", accel_arr);
            mxSetField(sensor_data, 0, "gyro", gyro_arr);
            mxSetField(sensor_data, 0, "mag", mag_arr);
            mxSetField(sensor_data, 0, "gps_pos", gps_arr);
            mxSetField(sensor_data, 0, "alt_baro", mxCreateDoubleScalar(0));
            mxSetField(sensor_data, 0, "dt", mxCreateDoubleScalar(s->dt));
            mxSetField(sensor_data, 0, "update_accel", mxCreateLogicalScalar(false));
            mxSetField(sensor_data, 0, "update_gyro", mxCreateLogicalScalar(false));
            mxSetField(sensor_data, 0, "update_mag", mxCreateLogicalScalar(true));
            mxSetField(sensor_data, 0, "update_gps", mxCreateLogicalScalar(false));
            mxSetField(sensor_data, 0, "update_baro", mxCreateLogicalScalar(false));
            mxSetField(sensor_data, 0, "update_zupt", mxCreateLogicalScalar(false));
            
            // mex_params構造体
            const char* mp_fields[] = {"g", "mag_ref", "noise_accel", "noise_gyro", "noise_ba", "noise_bg",
                "noise_mag", "noise_gps", "noise_baro", "noise_zupt", "alpha", "beta", "kappa", "trace_sample"};
            mxArray* mex_params = mxCreateStructMatrix(1, 1, 14, mp_fields);
            
            mxArray* g_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(g_arr), s->g, 3);
            mxSetField(mex_params, 0, "g", g_arr);
            double mag_ref[3] = {50, 0, 0};
            mxArray* mag_ref_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(mag_ref_arr), mag_ref, 3);
            mxSetField(mex_params, 0, "mag_ref", mag_ref_arr);
            
            mxArray* na = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(na), zeros3, 3);
            mxArray* ng = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(ng), zeros3, 3);
            mxArray* nba = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(nba), zeros3, 3);
            mxArray* nbg = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(nbg), zeros3, 3);
            mxArray* nm = mxCreateDoubleMatrix(3, 1, mxREAL);
            for (int i = 0; i < 3; ++i) R_noise[i] *= 1.5;
            copy_vec(mxGetPr(nm), R_noise, 3);
            mxArray* ngps = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(ngps), zeros3, 3);
            mxArray* nz = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(nz), zeros3, 3);
            
            mxSetField(mex_params, 0, "noise_accel", na);
            mxSetField(mex_params, 0, "noise_gyro", ng);
            mxSetField(mex_params, 0, "noise_ba", nba);
            mxSetField(mex_params, 0, "noise_bg", nbg);
            mxSetField(mex_params, 0, "noise_mag", nm);
            mxSetField(mex_params, 0, "noise_gps", ngps);
            mxSetField(mex_params, 0, "noise_baro", mxCreateDoubleScalar(0));
            mxSetField(mex_params, 0, "noise_zupt", nz);
            mxSetField(mex_params, 0, "alpha", mxCreateDoubleScalar(1e-3));
            mxSetField(mex_params, 0, "beta", mxCreateDoubleScalar(2));
            mxSetField(mex_params, 0, "kappa", mxCreateDoubleScalar(0));
            mxSetField(mex_params, 0, "trace_sample", mxCreateDoubleScalar(sample));
            
            // state構造体
            const char* st_fields[] = {"p", "v", "q", "ba", "bg", "P"};
            mxArray* state_s = mxCreateStructMatrix(1, 1, 6, st_fields);
            mxArray* p_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(p_arr), out_p, 3);
            mxArray* v_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(v_arr), out_v, 3);
            mxArray* q_arr = mxCreateDoubleMatrix(4, 1, mxREAL); copy_vec(mxGetPr(q_arr), out_q, 4);
            mxArray* ba_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(ba_arr), out_ba, 3);
            mxArray* bg_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(bg_arr), out_bg, 3);
            mxArray* P_arr = mxCreateDoubleMatrix(15, 15, mxREAL); memcpy(mxGetPr(P_arr), out_P, 15*15*8);
            mxSetField(state_s, 0, "p", p_arr);
            mxSetField(state_s, 0, "v", v_arr);
            mxSetField(state_s, 0, "q", q_arr);
            mxSetField(state_s, 0, "ba", ba_arr);
            mxSetField(state_s, 0, "bg", bg_arr);
            mxSetField(state_s, 0, "P", P_arr);
            
            // mex_meukf_step_v2 呼び出し（維持）
            mxArray* prhs_m[3] = {state_s, sensor_data, mex_params};
            mxArray* plhs_m[3];
            if (mexCallMATLAB(3, plhs_m, 3, prhs_m, "mex_meukf_step_v2") == 0) {
                mxArray* new_state = plhs_m[0];
                mxArray* dbg_out = plhs_m[1];
                
                // noise estimate更新（mex_sensor_filter → g_filter_lib.noise_estimator.estimate()に置き換え）
                if (mxIsStruct(dbg_out) && mxGetField(dbg_out, 0, "innov") && mxGetField(dbg_out, 0, "H")) {
                    // Convert innov to cm
                    Vector<3, float> innov_vec;
                    if (matToVector<3>(mxGetField(dbg_out, 0, "innov"), innov_vec)) {
                        cm innov_cm(3, 1);
                        for (int i = 0; i < 3; ++i) innov_cm(i, 0) = innov_vec(i, 0);
                        
                        // Convert H to cm
                        int H_rows = mxGetM(mxGetField(dbg_out, 0, "H"));
                        int H_cols = mxGetN(mxGetField(dbg_out, 0, "H"));
                        cm H_cm(H_rows, H_cols);
                        std::vector<float> H_tmp(static_cast<size_t>(H_rows) * static_cast<size_t>(H_cols));
                        mex_conv::mxArrayToFloatArray(mxGetField(dbg_out, 0, "H"), H_tmp.data(), static_cast<size_t>(H_rows) * static_cast<size_t>(H_cols));
                        for (int j = 0; j < H_cols; ++j) {
                            for (int i = 0; i < H_rows; ++i) {
                                H_cm(i, j) = H_tmp[static_cast<size_t>(j) * static_cast<size_t>(H_rows) + static_cast<size_t>(i)];
                            }
                        }
                        
                        // Convert P to cm
                        cm P_cm(15, 15);
                        std::vector<float> P_tmp(15 * 15);
                        mex_conv::mxArrayToFloatArray(P_arr, P_tmp.data(), 15 * 15);
                        for (int j = 0; j < 15; ++j) {
                            for (int i = 0; i < 15; ++i) {
                                P_cm(i, j) = P_tmp[static_cast<size_t>(j) * 15 + static_cast<size_t>(i)];
                            }
                        }
                        
                        // Call noise_estimator.estimate directly
                        g_filter_lib.noise_estimator.estimate(sensor_type, innov_cm, H_cm, P_cm);
                    }
                }
                
                // postprocess (mex_eskf_update_postprocess を統合)
                if (mxIsStruct(dbg_out) && mxGetField(dbg_out, 0, "dx")) {
                    // Get dx (15x1)
                    Vector<15, float> dx;
                    if (!matToVector<15>(mxGetField(dbg_out, 0, "dx"), dx)) {
                        // Fallback: use new_state directly
                        copy_vec(out_p, mxGetPr(mxGetField(new_state, 0, "p")), 3);
                        copy_vec(out_v, mxGetPr(mxGetField(new_state, 0, "v")), 3);
                        copy_vec(out_q, mxGetPr(mxGetField(new_state, 0, "q")), 4);
                        copy_vec(out_ba, mxGetPr(mxGetField(new_state, 0, "ba")), 3);
                        copy_vec(out_bg, mxGetPr(mxGetField(new_state, 0, "bg")), 3);
                        if (mxGetField(new_state, 0, "P")) {
                            double* new_P = mxGetPr(mxGetField(new_state, 0, "P"));
                            for (int i = 0; i < 15; ++i) {
                                for (int j = 0; j < 15; ++j) {
                                    out_P[i + j*15] = 0.5 * (new_P[i + j*15] + new_P[j + i*15]);
                                }
                            }
                        }
                        should_skip = false;
                    } else {
                        // Get innov
                        const mxArray* innov = mxGetField(dbg_out, 0, "innov");
                        
                        // Get state
                        Vector<3, float> state_p, state_v, state_ba, state_bg;
                        Vector<4, float> state_q;
                        Matrix<15, 15, float> state_P, new_state_P;
                        
                        if (!matToVector<3>(p_arr, state_p)) state_p = Vector<3, float>::Zero();
                        if (!matToVector<3>(v_arr, state_v)) state_v = Vector<3, float>::Zero();
                        if (!matToVector<4>(q_arr, state_q)) state_q = Vector<4, float>::Zero();
                        if (!matToVector<3>(ba_arr, state_ba)) state_ba = Vector<3, float>::Zero();
                        if (!matToVector<3>(bg_arr, state_bg)) state_bg = Vector<3, float>::Zero();
                        if (!matToMatrix<15, 15>(P_arr, state_P)) state_P = Matrix<15, 15, float>::Zero();
                        if (mxGetField(new_state, 0, "P")) {
                            if (!matToMatrix<15, 15>(mxGetField(new_state, 0, "P"), new_state_P)) {
                                new_state_P = state_P;
                            }
                        } else {
                            new_state_P = state_P;
                        }
                        
                        // Call divergence_guard.check_and_attenuate directly (mex_sensor_filter → g_filter_lib.divergence_guard.check_and_attenuate()に置き換え)
                        cm innov_cm;
                        if (mxGetField(dbg_out, 0, "innov")) {
                            int innov_len = mxGetM(mxGetField(dbg_out, 0, "innov"));
                            innov_cm.resize(innov_len, 1);
                            std::vector<float> innov_tmp(static_cast<size_t>(innov_len));
                            mex_conv::mxArrayToFloatArray(mxGetField(dbg_out, 0, "innov"), innov_tmp.data(), static_cast<size_t>(innov_len));
                            for (int i = 0; i < innov_len; ++i) innov_cm(i, 0) = innov_tmp[i];
                        }
                        
                        cm dx_cm(15, 1);
                        for (int i = 0; i < 15; ++i) dx_cm(i, 0) = dx(i, 0);
                        
                        bool was_attenuated = false;
                        bool should_skip_div = g_filter_lib.divergence_guard.check_and_attenuate(sensor_type, innov_cm, dx_cm, was_attenuated);
                        
                        // Get dx_out from dx_cm
                        Vector<15, float> dx_out;
                        for (int i = 0; i < 15; ++i) dx_out(i, 0) = dx_cm(i, 0);
                        
                        should_skip = should_skip_div;
                        
                        // Output: new_state (p, v, q, ba, bg, P), should_skip
                        Vector<3, float> new_p, new_v, new_ba, new_bg;
                        Vector<4, float> new_q;
                        Matrix<15, 15, float> out_P_mat = new_state_P;
                        
                        if (should_skip) {
                            // Return original state
                            new_p = state_p;
                            new_v = state_v;
                            new_q = state_q;
                            new_ba = state_ba;
                            new_bg = state_bg;
                            out_P_mat = state_P;
                        } else if (was_attenuated) {
                            // Apply dx_out
                            UpdatePostprocessResult updated = update_state_from_dx(dx_out, state_p, state_v, state_q, state_ba, state_bg, new_state_P);
                            new_p = updated.p;
                            new_v = updated.v;
                            new_q = updated.q;
                            new_ba = updated.ba;
                            new_bg = updated.bg;
                            out_P_mat = updated.P;
                        } else {
                            // Use new_state directly (no attenuation)
                            UpdatePostprocessResult updated = update_state_from_dx(dx, state_p, state_v, state_q, state_ba, state_bg, new_state_P);
                            new_p = updated.p;
                            new_v = updated.v;
                            new_q = updated.q;
                            new_ba = updated.ba;
                            new_bg = updated.bg;
                            out_P_mat = updated.P;
                        }
                        
                        // Convert back to double arrays
                        for (int i = 0; i < 3; ++i) {
                            out_p[i] = static_cast<double>(new_p(i, 0));
                            out_v[i] = static_cast<double>(new_v(i, 0));
                            out_ba[i] = static_cast<double>(new_ba(i, 0));
                            out_bg[i] = static_cast<double>(new_bg(i, 0));
                        }
                        for (int i = 0; i < 4; ++i) {
                            out_q[i] = static_cast<double>(new_q(i, 0));
                        }
                        for (int i = 0; i < 15; ++i) {
                            for (int j = 0; j < 15; ++j) {
                                out_P[i + j*15] = static_cast<double>(out_P_mat(i, j));
                            }
                        }
                    }
                } else {
                    copy_vec(out_p, mxGetPr(mxGetField(new_state, 0, "p")), 3);
                    copy_vec(out_v, mxGetPr(mxGetField(new_state, 0, "v")), 3);
                    copy_vec(out_q, mxGetPr(mxGetField(new_state, 0, "q")), 4);
                    copy_vec(out_ba, mxGetPr(mxGetField(new_state, 0, "ba")), 3);
                    copy_vec(out_bg, mxGetPr(mxGetField(new_state, 0, "bg")), 3);
                    if (mxGetField(new_state, 0, "P")) {
                        double* new_P = mxGetPr(mxGetField(new_state, 0, "P"));
                        for (int i = 0; i < 15; ++i) {
                            for (int j = 0; j < 15; ++j) {
                                out_P[i + j*15] = 0.5 * (new_P[i + j*15] + new_P[j + i*15]);
                            }
                        }
                    }
                }
                
                for (int i = 0; i < 3; ++i) mxDestroyArray(plhs_m[i]);
            }
            
            mxDestroyArray(sensor_data);
            mxDestroyArray(mex_params);
            mxDestroyArray(state_s);
        }
        
        // 出力を状態にコピー
        copy_vec(s->p, out_p, 3);
        copy_vec(s->v, out_v, 3);
        copy_vec(s->q, out_q, 4);
        copy_vec(s->ba, out_ba, 3);
        copy_vec(s->bg, out_bg, 3);
        memcpy(s->P, out_P, 15*15*sizeof(double));
        
    } else if (strcmp(type, "baro") == 0) {
        // Baro前処理 (sensor_preprocessor.hppを使用)
        double pressure = meas[0];
        double new_prev_baro = s->prev_baro;
        
        if (std::fabs(pressure - s->prev_baro) > s->buffer_tolerance) {
            new_prev_baro = pressure;
            
            double alt_baro = preprocess_baro(pressure);
            
            if (!std::isnan(alt_baro)) {
                should_skip = false;
                
                // baro_weight適用
                double weight_factor = 1.0 / s->baro_weight;
                out_P[2 + 2*15] *= weight_factor;  // P(3,3)
                
                // mex_eskf_do_updateロジックを統合（mex_sensor_filter呼び出しを直接C++関数に置き換え）
                const char* sensor_type = "baro";
                double meas_arr[1] = {alt_baro};
                const double* meas_ptr = meas_arr;
                int meas_len = 1;
                
                // R取得（mex_sensor_filter → g_filter_lib.noise_estimator.get_R_matrix()に置き換え）
                double R_noise[3] = {0.01, 0.01, 0.01};
                {
                    cm R = g_filter_lib.noise_estimator.get_R_matrix(sensor_type);
                    int n = R.rows * R.cols;
                    if (n == 9) {
                        R_noise[0] = static_cast<double>(R(0, 0));
                        R_noise[1] = static_cast<double>(R(1, 1));
                        R_noise[2] = static_cast<double>(R(2, 2));
                    } else if (n >= 3) {
                        R_noise[0] = static_cast<double>(R(0, 0));
                        R_noise[1] = static_cast<double>(R(1, 0));
                        R_noise[2] = static_cast<double>(R(2, 0));
                    } else if (n == 1) {
                        R_noise[0] = R_noise[1] = R_noise[2] = static_cast<double>(R(0, 0));
                    }
                }
                
                // sensor_data構造体を構築
                const char* sd_fields[] = {"accel", "gyro", "mag", "gps_pos", "alt_baro", "dt",
                    "update_accel", "update_gyro", "update_mag", "update_gps", "update_baro", "update_zupt"};
                mxArray* sensor_data = mxCreateStructMatrix(1, 1, 12, sd_fields);
                
                double zeros3[3] = {0, 0, 0};
                mxArray* accel_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(accel_arr), zeros3, 3);
                mxArray* gyro_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(gyro_arr), zeros3, 3);
                mxArray* mag_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(mag_arr), zeros3, 3);
                mxArray* gps_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(gps_arr), zeros3, 3);
                
                mxSetField(sensor_data, 0, "accel", accel_arr);
                mxSetField(sensor_data, 0, "gyro", gyro_arr);
                mxSetField(sensor_data, 0, "mag", mag_arr);
                mxSetField(sensor_data, 0, "gps_pos", gps_arr);
                mxDestroyArray(mxGetField(sensor_data, 0, "alt_baro"));
                mxSetField(sensor_data, 0, "alt_baro", mxCreateDoubleScalar(alt_baro));
                mxSetField(sensor_data, 0, "dt", mxCreateDoubleScalar(s->dt));
                mxSetField(sensor_data, 0, "update_accel", mxCreateLogicalScalar(false));
                mxSetField(sensor_data, 0, "update_gyro", mxCreateLogicalScalar(false));
                mxSetField(sensor_data, 0, "update_mag", mxCreateLogicalScalar(false));
                mxSetField(sensor_data, 0, "update_gps", mxCreateLogicalScalar(false));
                mxSetField(sensor_data, 0, "update_baro", mxCreateLogicalScalar(true));
                mxSetField(sensor_data, 0, "update_zupt", mxCreateLogicalScalar(false));
                
                // mex_params構造体
                const char* mp_fields[] = {"g", "mag_ref", "noise_accel", "noise_gyro", "noise_ba", "noise_bg",
                    "noise_mag", "noise_gps", "noise_baro", "noise_zupt", "alpha", "beta", "kappa", "trace_sample"};
                mxArray* mex_params = mxCreateStructMatrix(1, 1, 14, mp_fields);
                
                mxArray* g_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(g_arr), s->g, 3);
                mxSetField(mex_params, 0, "g", g_arr);
                double mag_ref[3] = {50, 0, 0};
                mxArray* mag_ref_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(mag_ref_arr), mag_ref, 3);
                mxSetField(mex_params, 0, "mag_ref", mag_ref_arr);
                
                mxArray* na = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(na), zeros3, 3);
                mxArray* ng = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(ng), zeros3, 3);
                mxArray* nba = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(nba), zeros3, 3);
                mxArray* nbg = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(nbg), zeros3, 3);
                mxArray* nm = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(nm), zeros3, 3);
                mxArray* ngps = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(ngps), zeros3, 3);
                mxDestroyArray(mxGetField(mex_params, 0, "noise_baro"));
                mxSetField(mex_params, 0, "noise_baro", mxCreateDoubleScalar(R_noise[0]));
                mxArray* nz = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(nz), zeros3, 3);
                
                mxSetField(mex_params, 0, "noise_accel", na);
                mxSetField(mex_params, 0, "noise_gyro", ng);
                mxSetField(mex_params, 0, "noise_ba", nba);
                mxSetField(mex_params, 0, "noise_bg", nbg);
                mxSetField(mex_params, 0, "noise_mag", nm);
                mxSetField(mex_params, 0, "noise_gps", ngps);
                mxSetField(mex_params, 0, "noise_zupt", nz);
                mxSetField(mex_params, 0, "alpha", mxCreateDoubleScalar(1e-3));
                mxSetField(mex_params, 0, "beta", mxCreateDoubleScalar(2));
                mxSetField(mex_params, 0, "kappa", mxCreateDoubleScalar(0));
                mxSetField(mex_params, 0, "trace_sample", mxCreateDoubleScalar(sample));
                
                // state構造体
                const char* st_fields[] = {"p", "v", "q", "ba", "bg", "P"};
                mxArray* state_s = mxCreateStructMatrix(1, 1, 6, st_fields);
                mxArray* p_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(p_arr), out_p, 3);
                mxArray* v_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(v_arr), out_v, 3);
                mxArray* q_arr = mxCreateDoubleMatrix(4, 1, mxREAL); copy_vec(mxGetPr(q_arr), out_q, 4);
                mxArray* ba_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(ba_arr), out_ba, 3);
                mxArray* bg_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(bg_arr), out_bg, 3);
                mxArray* P_arr = mxCreateDoubleMatrix(15, 15, mxREAL); memcpy(mxGetPr(P_arr), out_P, 15*15*8);
                mxSetField(state_s, 0, "p", p_arr);
                mxSetField(state_s, 0, "v", v_arr);
                mxSetField(state_s, 0, "q", q_arr);
                mxSetField(state_s, 0, "ba", ba_arr);
                mxSetField(state_s, 0, "bg", bg_arr);
                mxSetField(state_s, 0, "P", P_arr);
                
                // mex_meukf_step_v2 呼び出し（維持）
                mxArray* prhs_m[3] = {state_s, sensor_data, mex_params};
                mxArray* plhs_m[3];
                if (mexCallMATLAB(3, plhs_m, 3, prhs_m, "mex_meukf_step_v2") == 0) {
                    mxArray* new_state = plhs_m[0];
                    mxArray* dbg_out = plhs_m[1];
                    
                    // noise estimate更新（mex_sensor_filter → g_filter_lib.noise_estimator.estimate()に置き換え）
                    if (mxIsStruct(dbg_out) && mxGetField(dbg_out, 0, "innov") && mxGetField(dbg_out, 0, "H")) {
                        // Convert innov to cm
                        Vector<1, float> innov_vec;
                        if (matToVector<1>(mxGetField(dbg_out, 0, "innov"), innov_vec)) {
                            cm innov_cm(1, 1);
                            for (int i = 0; i < 1; ++i) innov_cm(i, 0) = innov_vec(i, 0);
                            
                            // Convert H to cm
                            int H_rows = mxGetM(mxGetField(dbg_out, 0, "H"));
                            int H_cols = mxGetN(mxGetField(dbg_out, 0, "H"));
                            cm H_cm(H_rows, H_cols);
                            std::vector<float> H_tmp(static_cast<size_t>(H_rows) * static_cast<size_t>(H_cols));
                            mex_conv::mxArrayToFloatArray(mxGetField(dbg_out, 0, "H"), H_tmp.data(), static_cast<size_t>(H_rows) * static_cast<size_t>(H_cols));
                            for (int j = 0; j < H_cols; ++j) {
                                for (int i = 0; i < H_rows; ++i) {
                                    H_cm(i, j) = H_tmp[static_cast<size_t>(j) * static_cast<size_t>(H_rows) + static_cast<size_t>(i)];
                                }
                            }
                            
                            // Convert P to cm
                            cm P_cm(15, 15);
                            std::vector<float> P_tmp(15 * 15);
                            mex_conv::mxArrayToFloatArray(P_arr, P_tmp.data(), 15 * 15);
                            for (int j = 0; j < 15; ++j) {
                                for (int i = 0; i < 15; ++i) {
                                    P_cm(i, j) = P_tmp[static_cast<size_t>(j) * 15 + static_cast<size_t>(i)];
                                }
                            }
                            
                            // Call noise_estimator.estimate directly
                            g_filter_lib.noise_estimator.estimate(sensor_type, innov_cm, H_cm, P_cm);
                        }
                    }
                    
                    // postprocess (mex_eskf_update_postprocess を統合)
                    if (mxIsStruct(dbg_out) && mxGetField(dbg_out, 0, "dx")) {
                        // Get dx (15x1)
                        Vector<15, float> dx;
                        if (!matToVector(mxGetField(dbg_out, 0, "dx"), dx)) {
                            // Fallback: use new_state directly
                            copy_vec(out_p, mxGetPr(mxGetField(new_state, 0, "p")), 3);
                            copy_vec(out_v, mxGetPr(mxGetField(new_state, 0, "v")), 3);
                            copy_vec(out_q, mxGetPr(mxGetField(new_state, 0, "q")), 4);
                            copy_vec(out_ba, mxGetPr(mxGetField(new_state, 0, "ba")), 3);
                            copy_vec(out_bg, mxGetPr(mxGetField(new_state, 0, "bg")), 3);
                            if (mxGetField(new_state, 0, "P")) {
                                double* new_P = mxGetPr(mxGetField(new_state, 0, "P"));
                                for (int i = 0; i < 15; ++i) {
                                    for (int j = 0; j < 15; ++j) {
                                        out_P[i + j*15] = 0.5 * (new_P[i + j*15] + new_P[j + i*15]);
                                    }
                                }
                            }
                            should_skip = false;
                        } else {
                            // Get innov
                            const mxArray* innov = mxGetField(dbg_out, 0, "innov");
                            
                            // Get state
                            Vector<3, float> state_p, state_v, state_ba, state_bg;
                            Vector<4, float> state_q;
                            Matrix<15, 15, float> state_P, new_state_P;
                            
                            if (!matToVector(p_arr, state_p)) state_p = Vector<3, float>::Zero();
                            if (!matToVector(v_arr, state_v)) state_v = Vector<3, float>::Zero();
                            if (!matToVector(q_arr, state_q)) state_q = Vector<4, float>::Zero();
                            if (!matToVector(ba_arr, state_ba)) state_ba = Vector<3, float>::Zero();
                            if (!matToVector(bg_arr, state_bg)) state_bg = Vector<3, float>::Zero();
                            if (!matToMatrix(P_arr, state_P)) state_P = Matrix<15, 15, float>::Zero();
                            if (mxGetField(new_state, 0, "P")) {
                                if (!matToMatrix<15, 15>(mxGetField(new_state, 0, "P"), new_state_P)) {
                                    new_state_P = state_P;
                                }
                            } else {
                                new_state_P = state_P;
                            }
                            
                            // Call divergence_guard.check_and_attenuate directly (mex_sensor_filter → g_filter_lib.divergence_guard.check_and_attenuate()に置き換え)
                            cm innov_cm;
                            if (mxGetField(dbg_out, 0, "innov")) {
                                int innov_len = mxGetM(mxGetField(dbg_out, 0, "innov"));
                                innov_cm.resize(innov_len, 1);
                                std::vector<float> innov_tmp(static_cast<size_t>(innov_len));
                                mex_conv::mxArrayToFloatArray(mxGetField(dbg_out, 0, "innov"), innov_tmp.data(), static_cast<size_t>(innov_len));
                                for (int i = 0; i < innov_len; ++i) innov_cm(i, 0) = innov_tmp[i];
                            }
                            
                            cm dx_cm(15, 1);
                            for (int i = 0; i < 15; ++i) dx_cm(i, 0) = dx(i, 0);
                            
                            bool was_attenuated = false;
                            bool should_skip_div = g_filter_lib.divergence_guard.check_and_attenuate(sensor_type, innov_cm, dx_cm, was_attenuated);
                            
                            // Get dx_out from dx_cm
                            Vector<15, float> dx_out;
                            for (int i = 0; i < 15; ++i) dx_out(i, 0) = dx_cm(i, 0);
                            
                            should_skip = should_skip_div;
                            
                            // Output: new_state (p, v, q, ba, bg, P), should_skip
                            Vector<3, float> new_p, new_v, new_ba, new_bg;
                            Vector<4, float> new_q;
                            Matrix<15, 15, float> out_P_mat = new_state_P;
                            
                            if (should_skip) {
                                // Return original state
                                new_p = state_p;
                                new_v = state_v;
                                new_q = state_q;
                                new_ba = state_ba;
                                new_bg = state_bg;
                                out_P_mat = state_P;
                            } else if (was_attenuated) {
                                // Apply dx_out
                                UpdatePostprocessResult updated = update_state_from_dx(dx_out, state_p, state_v, state_q, state_ba, state_bg, new_state_P);
                                new_p = updated.p;
                                new_v = updated.v;
                                new_q = updated.q;
                                new_ba = updated.ba;
                                new_bg = updated.bg;
                                out_P_mat = updated.P;
                            } else {
                                // Use new_state directly (no attenuation)
                                UpdatePostprocessResult updated = update_state_from_dx(dx, state_p, state_v, state_q, state_ba, state_bg, new_state_P);
                                new_p = updated.p;
                                new_v = updated.v;
                                new_q = updated.q;
                                new_ba = updated.ba;
                                new_bg = updated.bg;
                                out_P_mat = updated.P;
                            }
                            
                            // Convert back to double arrays
                            for (int i = 0; i < 3; ++i) {
                                out_p[i] = static_cast<double>(new_p(i, 0));
                                out_v[i] = static_cast<double>(new_v(i, 0));
                                out_ba[i] = static_cast<double>(new_ba(i, 0));
                                out_bg[i] = static_cast<double>(new_bg(i, 0));
                            }
                            for (int i = 0; i < 4; ++i) {
                                out_q[i] = static_cast<double>(new_q(i, 0));
                            }
                            for (int i = 0; i < 15; ++i) {
                                for (int j = 0; j < 15; ++j) {
                                    out_P[i + j*15] = static_cast<double>(out_P_mat(i, j));
                                }
                            }
                        }
                    } else {
                        copy_vec(out_p, mxGetPr(mxGetField(new_state, 0, "p")), 3);
                        copy_vec(out_v, mxGetPr(mxGetField(new_state, 0, "v")), 3);
                        copy_vec(out_q, mxGetPr(mxGetField(new_state, 0, "q")), 4);
                        copy_vec(out_ba, mxGetPr(mxGetField(new_state, 0, "ba")), 3);
                        copy_vec(out_bg, mxGetPr(mxGetField(new_state, 0, "bg")), 3);
                        if (mxGetField(new_state, 0, "P")) {
                            double* new_P = mxGetPr(mxGetField(new_state, 0, "P"));
                            for (int i = 0; i < 15; ++i) {
                                for (int j = 0; j < 15; ++j) {
                                    out_P[i + j*15] = 0.5 * (new_P[i + j*15] + new_P[j + i*15]);
                                }
                            }
                        }
                    }
                    
                    for (int i = 0; i < 3; ++i) mxDestroyArray(plhs_m[i]);
                }
                
                mxDestroyArray(sensor_data);
                mxDestroyArray(mex_params);
                mxDestroyArray(state_s);
                
                // baro_weight復元
                out_P[2 + 2*15] /= weight_factor;
            }
        }
        
        // 出力を状態にコピー
        copy_vec(s->p, out_p, 3);
        copy_vec(s->v, out_v, 3);
        copy_vec(s->q, out_q, 4);
        copy_vec(s->ba, out_ba, 3);
        copy_vec(s->bg, out_bg, 3);
        memcpy(s->P, out_P, 15*15*sizeof(double));
        s->prev_baro = new_prev_baro;
    }
}

// Call GPS sensor update (special case with multiple meas) - mex_eskf_sensor_updates_full を統合
static void call_gps_update(ESKFState* s, double lat, double lon, double alt, double sample) {
    // 出力バッファ
    double out_p[3], out_v[3], out_q[4], out_ba[3], out_bg[3], out_P[15*15];
    copy_vec(out_p, s->p, 3);
    copy_vec(out_v, s->v, 3);
    copy_vec(out_q, s->q, 4);
    copy_vec(out_ba, s->ba, 3);
    copy_vec(out_bg, s->bg, 3);
    memcpy(out_P, s->P, 15*15*sizeof(double));
    
    bool should_skip = true;
    
    // GPS前処理 (sensor_preprocessor.hppを使用)
    Vector<3, float> gps_origin_f;
    for (int i = 0; i < 3; ++i) {
        gps_origin_f(i, 0) = static_cast<float>(s->gps_origin[i]);
    }
    
    PreprocessResult pre_result = preprocess_gps(lat, lon, alt, gps_origin_f, s->buffer_tolerance);
    
    double z_gps[3];
    for (int i = 0; i < 3; ++i) {
        z_gps[i] = static_cast<double>(pre_result.output(i, 0));
    }
    
    if (!pre_result.no_change && !pre_result.is_outlier) {
        should_skip = false;
    }
    
    if (!should_skip) {
        // mex_eskf_do_updateロジックを統合（mex_sensor_filter呼び出しを直接C++関数に置き換え）
        const char* sensor_type = "gps";
        const double* meas = z_gps;
        int meas_len = 3;
        
        // R取得（mex_sensor_filter → g_filter_lib.noise_estimator.get_R_matrix()に置き換え）
        double R_noise[3] = {0.01, 0.01, 0.01};
        {
            cm R = g_filter_lib.noise_estimator.get_R_matrix(sensor_type);
            int n = R.rows * R.cols;
            if (n == 9) {
                R_noise[0] = static_cast<double>(R(0, 0));
                R_noise[1] = static_cast<double>(R(1, 1));
                R_noise[2] = static_cast<double>(R(2, 2));
            } else if (n >= 3) {
                R_noise[0] = static_cast<double>(R(0, 0));
                R_noise[1] = static_cast<double>(R(1, 0));
                R_noise[2] = static_cast<double>(R(2, 0));
            } else if (n == 1) {
                R_noise[0] = R_noise[1] = R_noise[2] = static_cast<double>(R(0, 0));
            }
        }
        
        // sensor_data構造体を構築
        const char* sd_fields[] = {"accel", "gyro", "mag", "gps_pos", "alt_baro", "dt",
            "update_accel", "update_gyro", "update_mag", "update_gps", "update_baro", "update_zupt"};
        mxArray* sensor_data = mxCreateStructMatrix(1, 1, 12, sd_fields);
        
        double zeros3[3] = {0, 0, 0};
        mxArray* accel_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(accel_arr), zeros3, 3);
        mxArray* gyro_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(gyro_arr), zeros3, 3);
        mxArray* mag_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(mag_arr), zeros3, 3);
        mxArray* gps_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(gps_arr), meas, 3);
        
        mxSetField(sensor_data, 0, "accel", accel_arr);
        mxSetField(sensor_data, 0, "gyro", gyro_arr);
        mxSetField(sensor_data, 0, "mag", mag_arr);
        mxSetField(sensor_data, 0, "gps_pos", gps_arr);
        mxSetField(sensor_data, 0, "alt_baro", mxCreateDoubleScalar(0));
        mxSetField(sensor_data, 0, "dt", mxCreateDoubleScalar(s->dt));
        mxSetField(sensor_data, 0, "update_accel", mxCreateLogicalScalar(false));
        mxSetField(sensor_data, 0, "update_gyro", mxCreateLogicalScalar(false));
        mxSetField(sensor_data, 0, "update_mag", mxCreateLogicalScalar(false));
        mxSetField(sensor_data, 0, "update_gps", mxCreateLogicalScalar(true));
        mxSetField(sensor_data, 0, "update_baro", mxCreateLogicalScalar(false));
        mxSetField(sensor_data, 0, "update_zupt", mxCreateLogicalScalar(false));
        
        // mex_params構造体
        const char* mp_fields[] = {"g", "mag_ref", "noise_accel", "noise_gyro", "noise_ba", "noise_bg",
            "noise_mag", "noise_gps", "noise_baro", "noise_zupt", "alpha", "beta", "kappa", "trace_sample"};
        mxArray* mex_params = mxCreateStructMatrix(1, 1, 14, mp_fields);
        
        mxArray* g_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(g_arr), s->g, 3);
        mxSetField(mex_params, 0, "g", g_arr);
        double mag_ref[3] = {50, 0, 0};
        mxArray* mag_ref_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(mag_ref_arr), mag_ref, 3);
        mxSetField(mex_params, 0, "mag_ref", mag_ref_arr);
        
        mxArray* na = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(na), zeros3, 3);
        mxArray* ng = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(ng), zeros3, 3);
        mxArray* nba = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(nba), zeros3, 3);
        mxArray* nbg = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(nbg), zeros3, 3);
        mxArray* nm = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(nm), zeros3, 3);
        mxArray* ngps = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(ngps), R_noise, 3);
        mxArray* nz = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(nz), zeros3, 3);
        
        mxSetField(mex_params, 0, "noise_accel", na);
        mxSetField(mex_params, 0, "noise_gyro", ng);
        mxSetField(mex_params, 0, "noise_ba", nba);
        mxSetField(mex_params, 0, "noise_bg", nbg);
        mxSetField(mex_params, 0, "noise_mag", nm);
        mxSetField(mex_params, 0, "noise_gps", ngps);
        mxSetField(mex_params, 0, "noise_baro", mxCreateDoubleScalar(0));
        mxSetField(mex_params, 0, "noise_zupt", nz);
        mxSetField(mex_params, 0, "alpha", mxCreateDoubleScalar(1e-3));
        mxSetField(mex_params, 0, "beta", mxCreateDoubleScalar(2));
        mxSetField(mex_params, 0, "kappa", mxCreateDoubleScalar(0));
        mxSetField(mex_params, 0, "trace_sample", mxCreateDoubleScalar(sample));
        
        // state構造体
        const char* st_fields[] = {"p", "v", "q", "ba", "bg", "P"};
        mxArray* state_s = mxCreateStructMatrix(1, 1, 6, st_fields);
        mxArray* p_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(p_arr), out_p, 3);
        mxArray* v_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(v_arr), out_v, 3);
        mxArray* q_arr = mxCreateDoubleMatrix(4, 1, mxREAL); copy_vec(mxGetPr(q_arr), out_q, 4);
        mxArray* ba_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(ba_arr), out_ba, 3);
        mxArray* bg_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(bg_arr), out_bg, 3);
        mxArray* P_arr = mxCreateDoubleMatrix(15, 15, mxREAL); memcpy(mxGetPr(P_arr), out_P, 15*15*8);
        mxSetField(state_s, 0, "p", p_arr);
        mxSetField(state_s, 0, "v", v_arr);
        mxSetField(state_s, 0, "q", q_arr);
        mxSetField(state_s, 0, "ba", ba_arr);
        mxSetField(state_s, 0, "bg", bg_arr);
        mxSetField(state_s, 0, "P", P_arr);
        
        // mex_meukf_step_v2 呼び出し（維持）
        mxArray* prhs_m[3] = {state_s, sensor_data, mex_params};
        mxArray* plhs_m[3];
        if (mexCallMATLAB(3, plhs_m, 3, prhs_m, "mex_meukf_step_v2") == 0) {
            mxArray* new_state = plhs_m[0];
            mxArray* dbg_out = plhs_m[1];
            
            // noise estimate更新（mex_sensor_filter → g_filter_lib.noise_estimator.estimate()に置き換え）
            if (mxIsStruct(dbg_out) && mxGetField(dbg_out, 0, "innov") && mxGetField(dbg_out, 0, "H")) {
                // Convert innov to cm
                Vector<3, float> innov_vec;
                if (matToVector(mxGetField(dbg_out, 0, "innov"), innov_vec)) {
                    cm innov_cm(3, 1);
                    for (int i = 0; i < 3; ++i) innov_cm(i, 0) = innov_vec(i, 0);
                    
                    // Convert H to cm
                    int H_rows = mxGetM(mxGetField(dbg_out, 0, "H"));
                    int H_cols = mxGetN(mxGetField(dbg_out, 0, "H"));
                    cm H_cm(H_rows, H_cols);
                    std::vector<float> H_tmp(static_cast<size_t>(H_rows) * static_cast<size_t>(H_cols));
                    mex_conv::mxArrayToFloatArray(mxGetField(dbg_out, 0, "H"), H_tmp.data(), static_cast<size_t>(H_rows) * static_cast<size_t>(H_cols));
                    for (int j = 0; j < H_cols; ++j) {
                        for (int i = 0; i < H_rows; ++i) {
                            H_cm(i, j) = H_tmp[static_cast<size_t>(j) * static_cast<size_t>(H_rows) + static_cast<size_t>(i)];
                        }
                    }
                    
                    // Convert P to cm
                    cm P_cm(15, 15);
                    std::vector<float> P_tmp(15 * 15);
                    mex_conv::mxArrayToFloatArray(P_arr, P_tmp.data(), 15 * 15);
                    for (int j = 0; j < 15; ++j) {
                        for (int i = 0; i < 15; ++i) {
                            P_cm(i, j) = P_tmp[static_cast<size_t>(j) * 15 + static_cast<size_t>(i)];
                        }
                    }
                    
                    // Call noise_estimator.estimate directly
                    g_filter_lib.noise_estimator.estimate(sensor_type, innov_cm, H_cm, P_cm);
                }
            }
            
            // postprocess (mex_eskf_update_postprocess を統合)
            if (mxIsStruct(dbg_out) && mxGetField(dbg_out, 0, "dx")) {
                // Get dx (15x1)
                Vector<15, float> dx;
                if (!matToVector(mxGetField(dbg_out, 0, "dx"), dx)) {
                    // Fallback: use new_state directly
                    copy_vec(out_p, mxGetPr(mxGetField(new_state, 0, "p")), 3);
                    copy_vec(out_v, mxGetPr(mxGetField(new_state, 0, "v")), 3);
                    copy_vec(out_q, mxGetPr(mxGetField(new_state, 0, "q")), 4);
                    copy_vec(out_ba, mxGetPr(mxGetField(new_state, 0, "ba")), 3);
                    copy_vec(out_bg, mxGetPr(mxGetField(new_state, 0, "bg")), 3);
                    if (mxGetField(new_state, 0, "P")) {
                        double* new_P = mxGetPr(mxGetField(new_state, 0, "P"));
                        for (int i = 0; i < 15; ++i) {
                            for (int j = 0; j < 15; ++j) {
                                out_P[i + j*15] = 0.5 * (new_P[i + j*15] + new_P[j + i*15]);
                            }
                        }
                    }
                    should_skip = false;
                } else {
                    // Get innov
                    const mxArray* innov = mxGetField(dbg_out, 0, "innov");
                    
                    // Get state
                    Vector<3, float> state_p, state_v, state_ba, state_bg;
                    Vector<4, float> state_q;
                    Matrix<15, 15, float> state_P, new_state_P;
                    
                    if (!matToVector(p_arr, state_p)) state_p = Vector<3, float>::Zero();
                    if (!matToVector(v_arr, state_v)) state_v = Vector<3, float>::Zero();
                    if (!matToVector(q_arr, state_q)) state_q = Vector<4, float>::Zero();
                    if (!matToVector(ba_arr, state_ba)) state_ba = Vector<3, float>::Zero();
                    if (!matToVector(bg_arr, state_bg)) state_bg = Vector<3, float>::Zero();
                    if (!matToMatrix(P_arr, state_P)) state_P = Matrix<15, 15, float>::Zero();
                    if (mxGetField(new_state, 0, "P")) {
                        if (!matToMatrix(mxGetField(new_state, 0, "P"), new_state_P)) {
                            new_state_P = state_P;
                        }
                    } else {
                        new_state_P = state_P;
                    }
                    
                    // Call divergence_guard.check_and_attenuate directly (mex_sensor_filter → g_filter_lib.divergence_guard.check_and_attenuate()に置き換え)
                    cm innov_cm;
                    if (mxGetField(dbg_out, 0, "innov")) {
                        int innov_len = mxGetM(mxGetField(dbg_out, 0, "innov"));
                        innov_cm.resize(innov_len, 1);
                        std::vector<float> innov_tmp(static_cast<size_t>(innov_len));
                        mex_conv::mxArrayToFloatArray(mxGetField(dbg_out, 0, "innov"), innov_tmp.data(), static_cast<size_t>(innov_len));
                        for (int i = 0; i < innov_len; ++i) innov_cm(i, 0) = innov_tmp[i];
                    }
                    
                    cm dx_cm(15, 1);
                    for (int i = 0; i < 15; ++i) dx_cm(i, 0) = dx(i, 0);
                    
                    bool was_attenuated = false;
                    bool should_skip_div = g_filter_lib.divergence_guard.check_and_attenuate(sensor_type, innov_cm, dx_cm, was_attenuated);
                    
                    // Get dx_out from dx_cm
                    Vector<15, float> dx_out;
                    for (int i = 0; i < 15; ++i) dx_out(i, 0) = dx_cm(i, 0);
                    
                    should_skip = should_skip_div;
                    
                    // Output: new_state (p, v, q, ba, bg, P), should_skip
                    Vector<3, float> new_p, new_v, new_ba, new_bg;
                    Vector<4, float> new_q;
                    Matrix<15, 15, float> out_P_mat = new_state_P;
                    
                    if (should_skip) {
                        // Return original state
                        new_p = state_p;
                        new_v = state_v;
                        new_q = state_q;
                        new_ba = state_ba;
                        new_bg = state_bg;
                        out_P_mat = state_P;
                    } else if (was_attenuated) {
                        // Apply dx_out
                        UpdatePostprocessResult updated = update_state_from_dx(dx_out, state_p, state_v, state_q, state_ba, state_bg, new_state_P);
                        new_p = updated.p;
                        new_v = updated.v;
                        new_q = updated.q;
                        new_ba = updated.ba;
                        new_bg = updated.bg;
                        out_P_mat = updated.P;
                    } else {
                        // Use new_state directly (no attenuation)
                        UpdatePostprocessResult updated = update_state_from_dx(dx, state_p, state_v, state_q, state_ba, state_bg, new_state_P);
                        new_p = updated.p;
                        new_v = updated.v;
                        new_q = updated.q;
                        new_ba = updated.ba;
                        new_bg = updated.bg;
                        out_P_mat = updated.P;
                    }
                    
                    // Convert back to double arrays
                    for (int i = 0; i < 3; ++i) {
                        out_p[i] = static_cast<double>(new_p(i, 0));
                        out_v[i] = static_cast<double>(new_v(i, 0));
                        out_ba[i] = static_cast<double>(new_ba(i, 0));
                        out_bg[i] = static_cast<double>(new_bg(i, 0));
                    }
                    for (int i = 0; i < 4; ++i) {
                        out_q[i] = static_cast<double>(new_q(i, 0));
                    }
                    for (int i = 0; i < 15; ++i) {
                        for (int j = 0; j < 15; ++j) {
                            out_P[i + j*15] = static_cast<double>(out_P_mat(i, j));
                        }
                    }
                }
            } else {
                copy_vec(out_p, mxGetPr(mxGetField(new_state, 0, "p")), 3);
                copy_vec(out_v, mxGetPr(mxGetField(new_state, 0, "v")), 3);
                copy_vec(out_q, mxGetPr(mxGetField(new_state, 0, "q")), 4);
                copy_vec(out_ba, mxGetPr(mxGetField(new_state, 0, "ba")), 3);
                copy_vec(out_bg, mxGetPr(mxGetField(new_state, 0, "bg")), 3);
                if (mxGetField(new_state, 0, "P")) {
                    double* new_P = mxGetPr(mxGetField(new_state, 0, "P"));
                    for (int i = 0; i < 15; ++i) {
                        for (int j = 0; j < 15; ++j) {
                            out_P[i + j*15] = 0.5 * (new_P[i + j*15] + new_P[j + i*15]);
                        }
                    }
                }
            }
            
            for (int i = 0; i < 3; ++i) mxDestroyArray(plhs_m[i]);
        }
        
        mxDestroyArray(sensor_data);
        mxDestroyArray(mex_params);
        mxDestroyArray(state_s);
    }
    
    // 出力を状態にコピー
    copy_vec(s->p, out_p, 3);
    copy_vec(s->v, out_v, 3);
    copy_vec(s->q, out_q, 4);
    copy_vec(s->ba, out_ba, 3);
    copy_vec(s->bg, out_bg, 3);
    memcpy(s->P, out_P, 15*15*sizeof(double));
    s->prev_gps_lat = lat;
    s->prev_gps_lon = lon;
    s->prev_gps_alt = alt;
}

// Reset check (moved to Src/ESKF/eskf_runner.cpp)
static void check_and_reset(ESKFState* s, int k) {
    ESKFRunner::check_and_reset(s, k);
}

// ZUPT check and update (moved to Src/ESKF/eskf_runner.cpp)
static void zupt_check_and_update(ESKFState* s, const double* a_meas, const double* w_meas) {
    ESKFRunner::zupt_check_and_update(s, a_meas, w_meas);
}

// Initialize (mex_eskf_constructor を統合、初期化ロジックはESKFRunner::initializeに移動)
static uint64_t do_init(const mxArray* obs, double static_time, double dt) {
    ESKFState* s = new ESKFState();
    
    // 静止サンプル数の計算
    int N_static = static_cast<int>(floor(static_time / dt));
    
    // MATLABデータの読み込み
    const mxArray* ax_arr = get_field_any(obs, "ax", "accel_x");
    const mxArray* ay_arr = get_field_any(obs, "ay", "accel_y");
    const mxArray* az_arr = get_field_any(obs, "az", "accel_z");
    const mxArray* wx_arr = get_field_any(obs, "wx", "gyro_x");
    const mxArray* wy_arr = get_field_any(obs, "wy", "gyro_y");
    const mxArray* wz_arr = get_field_any(obs, "wz", "gyro_z");
    const mxArray* mx_arr = get_field_any(obs, "mx", "mag_x");
    const mxArray* my_arr = get_field_any(obs, "my", "mag_y");
    const mxArray* mz_arr = get_field_any(obs, "mz", "mag_z");
    const mxArray* pressure_arr = get_field_any(obs, "pressure", "baro");
    const mxArray* lat_arr = get_field_any(obs, "lat", "gps_lat");
    const mxArray* lon_arr = get_field_any(obs, "lon", "gps_lon");
    const mxArray* alt_arr = get_field_any(obs, "alt", "gps_alt");
    
    int n_samples = ax_arr ? get_length(ax_arr) : 0;
    if (N_static > n_samples) N_static = n_samples;
    
    // ESKFRunner::InitParams構造体の準備
    ESKFRunner::InitParams params;
    params.dt = dt;
    params.N_static = N_static;
    
    params.has_accel = (ax_arr && ay_arr && az_arr);
    params.has_gyro = (wx_arr && wy_arr && wz_arr);
    params.has_mag = (mx_arr && my_arr && mz_arr);
    params.has_baro = (pressure_arr != nullptr);
    params.has_gps = (lat_arr && lon_arr && alt_arr);
    
    params.ax = params.has_accel ? get_data(ax_arr) : nullptr;
    params.ay = params.has_accel ? get_data(ay_arr) : nullptr;
    params.az = params.has_accel ? get_data(az_arr) : nullptr;
    params.wx = params.has_gyro ? get_data(wx_arr) : nullptr;
    params.wy = params.has_gyro ? get_data(wy_arr) : nullptr;
    params.wz = params.has_gyro ? get_data(wz_arr) : nullptr;
    params.mx = params.has_mag ? get_data(mx_arr) : nullptr;
    params.my = params.has_mag ? get_data(my_arr) : nullptr;
    params.mz = params.has_mag ? get_data(mz_arr) : nullptr;
    params.pressure = params.has_baro ? get_data(pressure_arr) : nullptr;
    params.lat = params.has_gps ? get_data(lat_arr) : nullptr;
    params.lon = params.has_gps ? get_data(lon_arr) : nullptr;
    params.alt = params.has_gps ? get_data(alt_arr) : nullptr;
    
    // ESKFRunner::initializeを呼び出し（初期化ロジックはSrc/ESKF/eskf_runner.cppに移動）
    ESKFRunner::initialize(s, params);
    
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

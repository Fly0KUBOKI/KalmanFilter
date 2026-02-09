#pragma once
#ifndef MEX_IMPL_MEX_HYBRID_FILTER_IMPL_HPP
#define MEX_IMPL_MEX_HYBRID_FILTER_IMPL_HPP

/**
 * mex_hybrid_filter.cpp用の実装関数群（Impl）
 */

#include "mex_hybrid_filter_common.hpp"
#include <cstdint>

#include "mex_hybrid_filter_sensor_updates.hpp"
#include "mex_hybrid_filter_filter_ops.hpp"
#include "../../Lib/MEUKF/inc/meukf_core.hpp"
#include "mex_type_conversion.hpp"

namespace mex_hybrid_filter_impl {

// State handle table API (fixed-size table used instead of std::map)
uint64_t allocate_handle(FilterState* s);
FilterState* find_state(uint64_t handle);
void remove_handle(uint64_t handle);
extern SensorFilterLib g_filter_lib;

inline void call_predict(FilterState* s, const float* a_meas, const float* w_meas) {
    HybridFilterRunner::predict(s, a_meas, w_meas);
}

// File-scope helper: get scalar field (logical/single/double) as double
inline double get_field_scalar_impl(const mxArray* s, const char* f) {
    if (!s) return 0.0;
    mxArray* ff = mxGetField((mxArray*)s,0,f);
    if (!ff) return 0.0;
    if (mxIsLogical(ff)) return mxIsLogicalScalarTrue(ff) ? 1.0 : 0.0;
    if (mxGetClassID(ff) == mxSINGLE_CLASS) {
        const float* pf = reinterpret_cast<const float*>(mxGetData(ff));
        return pf ? pf[0] : 0.0;
    } else if (mxGetClassID(ff) == mxDOUBLE_CLASS) {
        const double* pr = mxGetPr(ff);
        return pr ? pr[0] : 0.0;
    }
    return 0.0;
}

// File-scope helper: set vec3 into struct (single float field)
inline void set_vec3_impl(mxArray* out_new_state_local, const char* name, const float* in) {
    mxArray* f = mxGetField(out_new_state_local, 0, name);
    if(!f) return;
    if (mxGetClassID(f) != mxSINGLE_CLASS) {
        mexErrMsgIdAndTxt("mex_hybrid_filter:type_error",
            "Expected single (float) array for field '%s', but got %s.",
            name, mxGetClassName(f));
        return;
    }
    float* pf = (float*)mxGetData(f);
    pf[0] = in[0]; pf[1] = in[1]; pf[2] = in[2];
}

inline uint64_t do_init(const mxArray* obs, double static_time) {
    // Full reset of sensor filter library (includes noise_estimator and divergence_guard)
    g_filter_lib = SensorFilterLib();
    FilterState* s = initialize_eskf_from_matlab(obs, static_time);
    return allocate_handle(s);
}

inline void do_step(FilterState* s, const mxArray* obs, int k) {
    int idx = k - 1;
    double a_d[3], w_d[3], m_d[3];
    getAccel(obs, idx, a_d);
    getGyro(obs, idx, w_d);
    getMag(obs, idx, m_d);

    // Convert to float and convert gyro to rad/s
    const double deg2rad = M_PI / 180.0;
    float a_f[3], w_f[3], m_f[3];
    for (int i = 0; i < 3; ++i) {
        a_f[i] = static_cast<float>(a_d[i]);
        w_f[i] = static_cast<float>(w_d[i] * deg2rad);
        m_f[i] = static_cast<float>(m_d[i]);
    }

    // Predict (ESKF prediction step)
    call_predict(s, a_f, w_f);
    
    // ZUPT check and update
    zupt_check_and_update(s, a_d, w_d);

    // Sensor updates (MEUKF update step)
    call_sensor_update(s, "accel", a_d, 3, k);
    call_sensor_update(s, "mag", m_d, 3, k);

    // Baro
    mxArray* baro_field = mxGetField(obs, 0, "pressure");
    if (baro_field) {
        if (mxGetClassID(baro_field) != mxSINGLE_CLASS) {
            mexErrMsgIdAndTxt("mex_hybrid_filter:type_error", 
                "Expected single (float) array for field 'pressure', but got %s.", 
                mxGetClassName(baro_field));
        }
        const float* pf = (const float*)mxGetData(baro_field);
        double baro = pf[idx];
        double baro_arr[1] = {baro};
        call_sensor_update(s, "baro", baro_arr, 1, k);
    }

    // GPS
    mxArray* gps_lat = mxGetField(obs, 0, "lat");
    mxArray* gps_lon = mxGetField(obs, 0, "lon");
    mxArray* gps_alt = mxGetField(obs, 0, "alt");
    if (gps_lat && gps_lon && gps_alt) {
        if (mxGetClassID(gps_lat) != mxDOUBLE_CLASS) {
            mexErrMsgIdAndTxt("mex_hybrid_filter:type_error", 
                "Expected double array for GPS 'lat', but got %s.", 
                mxGetClassName(gps_lat));
        }
        if (mxGetClassID(gps_lon) != mxDOUBLE_CLASS) {
            mexErrMsgIdAndTxt("mex_hybrid_filter:type_error", 
                "Expected double array for GPS 'lon', but got %s.", 
                mxGetClassName(gps_lon));
        }
        if (mxGetClassID(gps_alt) != mxDOUBLE_CLASS) {
            mexErrMsgIdAndTxt("mex_hybrid_filter:type_error", 
                "Expected double array for GPS 'alt', but got %s.", 
                mxGetClassName(gps_alt));
        }
        double lat = mxGetPr(gps_lat)[idx];
        double lon = mxGetPr(gps_lon)[idx];
        double alt = mxGetPr(gps_alt)[idx];
        if (!std::isnan(lat) && !std::isnan(lon)) {
            call_gps_update(s, lat, lon, alt, k);
        }
    }
    
    // Reset check (divergence detection)
    check_and_reset(s, k);
}

inline mxArray* do_get_state(FilterState* s) {
    const char* fields[] = {"p", "v", "q", "euler", "ba", "bg", "P"};
    mxArray* out = mxCreateStructMatrix(1, 1, 7, fields);

    mxArray* p = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* p_ptr = (float*)mxGetData(p);
    for (int i = 0; i < 3; i++) p_ptr[i] = s->p[i];
    mxSetField(out, 0, "p", p);

    mxArray* v = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* v_ptr = (float*)mxGetData(v);
    for (int i = 0; i < 3; i++) v_ptr[i] = s->v[i];
    mxSetField(out, 0, "v", v);

    mxArray* q = mxCreateNumericMatrix(4, 1, mxSINGLE_CLASS, mxREAL);
    float* q_ptr = (float*)mxGetData(q);
    for (int i = 0; i < 4; i++) q_ptr[i] = s->q[i];
    mxSetField(out, 0, "q", q);

    // Compute Euler angles relative to the initialization quaternion (so displayed yaw starts at 0)
    mxArray* eu = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* eu_ptr = (float*)mxGetData(eu);
    // If q_init is available and state marked valid, compute relative quaternion q_rel = q_init^{-1} * q
    bool use_relative = s->valid;
    if (use_relative) {
        cmath_fx::Vector<4,float> q_curr; cmath_fx::Vector<4,float> q_init; cmath_fx::Vector<4,float> q_init_conj; cmath_fx::Vector<4,float> q_rel;
        for (int i=0;i<4;++i) { q_curr(i,0) = s->q[i]; q_init(i,0) = s->q_init[i]; }
        cquat::normalize_quat(q_curr);
        cquat::normalize_quat(q_init);
        cquat::conjugate_quat(q_init, q_init_conj);
        cquat::multiply_quat(q_init_conj, q_curr, q_rel);
        cquat::normalize_quat(q_rel);
        float roll_deg, pitch_deg, yaw_deg;
        cquat::to_euler_deg(q_rel, roll_deg, pitch_deg, yaw_deg);
        eu_ptr[0] = roll_deg;
        eu_ptr[1] = pitch_deg;
        eu_ptr[2] = yaw_deg;
    } else {
        double euler[3];
        double q_d[4]; for (int i=0;i<4;++i) q_d[i] = s->q[i];
        quat_to_euler(q_d, euler);
        eu_ptr[0] = static_cast<float>(euler[0] * 180.0 / M_PI);
        eu_ptr[1] = static_cast<float>(euler[1] * 180.0 / M_PI);
        eu_ptr[2] = static_cast<float>(euler[2] * 180.0 / M_PI);
    }
    mxSetField(out, 0, "euler", eu);

    mxArray* ba = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* ba_ptr = (float*)mxGetData(ba);
    for (int i = 0; i < 3; i++) ba_ptr[i] = s->ba[i];
    mxSetField(out, 0, "ba", ba);

    mxArray* bg = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* bg_ptr = (float*)mxGetData(bg);
    for (int i = 0; i < 3; i++) bg_ptr[i] = s->bg[i];
    mxSetField(out, 0, "bg", bg);

    mxArray* P = mxCreateNumericMatrix(15, 15, mxSINGLE_CLASS, mxREAL);
    float* P_ptr = (float*)mxGetData(P);
    for (int i = 0; i < 15*15; i++) P_ptr[i] = s->P[i];
    mxSetField(out, 0, "P", P);

    return out;
}

inline void do_free(uint64_t handle) {
    FilterState* s = find_state(handle);
    if (s != nullptr) {
        delete s;
        remove_handle(handle);
    }
}

inline void do_sensor_update(const mxArray* m_prev_state, const mxArray* m_sensor, const mxArray* m_params,
                         mxArray*& out_new_state, mxArray*& out_dbg_out, mxArray*& out_dbg_output) {
    meukf::MEUKFInput input;
    mex_conv::mxArrayToFloatArray(mxGetField(m_prev_state,0,"p"), input.prev_state.p, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_prev_state,0,"v"), input.prev_state.v, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_prev_state,0,"ba"), input.prev_state.ba, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_prev_state,0,"bg"), input.prev_state.bg, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_prev_state,0,"q"), input.prev_state.q, 4);
    mxArray* f_P = mxGetField(m_prev_state, 0, "P");
    if (f_P) {
        float P_tmp[15*15];
        mex_conv::mxArrayToFloatArray(f_P, P_tmp, 15*15);
        for (int r=0;r<15;++r) for (int c=0;c<15;++c) input.prev_state.P[r*15 + c] = P_tmp[c*15 + r];
    }

    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor,0,"accel"), input.sensor.accel, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor,0,"gyro"), input.sensor.gyro, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor,0,"mag"), input.sensor.mag, 3);
    mxArray* gps_pos_field = mxGetField(m_sensor, 0, "gps_pos");
    if (gps_pos_field) {
        if (mxGetClassID(gps_pos_field) != mxDOUBLE_CLASS) {
            mexErrMsgIdAndTxt("mex_hybrid_filter:type_error", 
                "Expected double array for GPS 'gps_pos', but got %s.", 
                mxGetClassName(gps_pos_field));
        }
        const double* gps_pr = mxGetPr(gps_pos_field);
        for (int i = 0; i < 3; ++i) {
            input.sensor.gps_pos[i] = static_cast<float>(gps_pr[i]);
        }
    } else {
        input.sensor.gps_pos[0] = input.sensor.gps_pos[1] = input.sensor.gps_pos[2] = 0.0f;
    }
    input.sensor.alt_baro = mex_conv::mxGetScalarAsFloat(mxGetField(m_sensor,0,"alt_baro"));
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor,0,"prev_mag"), input.sensor.prev_mag, 3);
    mxArray* prev_gps_pos_field = mxGetField(m_sensor, 0, "prev_gps_pos");
    if (prev_gps_pos_field) {
        if (mxGetClassID(prev_gps_pos_field) != mxDOUBLE_CLASS) {
            mexErrMsgIdAndTxt("mex_hybrid_filter:type_error", 
                "Expected double array for GPS 'prev_gps_pos', but got %s.", 
                mxGetClassName(prev_gps_pos_field));
        }
        const double* prev_gps_pr = mxGetPr(prev_gps_pos_field);
        for (int i = 0; i < 3; ++i) {
            input.sensor.prev_gps_pos[i] = static_cast<float>(prev_gps_pr[i]);
        }
    } else {
        input.sensor.prev_gps_pos[0] = input.sensor.prev_gps_pos[1] = input.sensor.prev_gps_pos[2] = 0.0f;
    }
    input.sensor.prev_baro_alt = mex_conv::mxGetScalarAsFloat(mxGetField(m_sensor,0,"prev_baro_alt"));
    input.sensor.update_accel = (uint8_t)get_field_scalar_impl(m_sensor, "update_accel");
    input.sensor.update_gyro = (uint8_t)get_field_scalar_impl(m_sensor, "update_gyro");
    input.sensor.update_mag = (uint8_t)get_field_scalar_impl(m_sensor, "update_mag");
    input.sensor.update_gps = (uint8_t)get_field_scalar_impl(m_sensor, "update_gps");
    input.sensor.update_baro = (uint8_t)get_field_scalar_impl(m_sensor, "update_baro");
    input.sensor.update_zupt = (uint8_t)get_field_scalar_impl(m_sensor, "update_zupt");
    
    // 時刻情報を読み込む
    input.sensor.current_time = mex_conv::mxGetScalarAsDouble(mxGetField(m_sensor, 0, "current_time"));
    input.sensor.prev_time_accel = mex_conv::mxGetScalarAsDouble(mxGetField(m_sensor, 0, "prev_time_accel"));
    input.sensor.prev_time_gyro = mex_conv::mxGetScalarAsDouble(mxGetField(m_sensor, 0, "prev_time_gyro"));
    input.sensor.prev_time_mag = mex_conv::mxGetScalarAsDouble(mxGetField(m_sensor, 0, "prev_time_mag"));
    input.sensor.prev_time_gps = mex_conv::mxGetScalarAsDouble(mxGetField(m_sensor, 0, "prev_time_gps"));
    input.sensor.prev_time_baro = mex_conv::mxGetScalarAsDouble(mxGetField(m_sensor, 0, "prev_time_baro"));
    
    // 個別のセンサーdtを計算（値が更新された場合のみ）
    // 加速度計
    float new_accel[3];
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor, 0, "accel"), new_accel, 3);
    float accel_diff = 0.0f;
    for (int i = 0; i < 3; ++i) {
        float diff = new_accel[i] - input.sensor.accel[i];
        accel_diff += diff * diff;
    }
    if (accel_diff > 1e-6f && input.sensor.current_time > input.sensor.prev_time_accel) {
        input.sensor.dt_accel = (float)(input.sensor.current_time - input.sensor.prev_time_accel);
    }
    for (int i = 0; i < 3; ++i) input.sensor.accel[i] = new_accel[i];
    
    // ジャイロ
    float new_gyro[3];
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor, 0, "gyro"), new_gyro, 3);
    float gyro_diff = 0.0f;
    for (int i = 0; i < 3; ++i) {
        float diff = new_gyro[i] - input.sensor.gyro[i];
        gyro_diff += diff * diff;
    }
    if (gyro_diff > 1e-6f && input.sensor.current_time > input.sensor.prev_time_gyro) {
        input.sensor.dt_gyro = (float)(input.sensor.current_time - input.sensor.prev_time_gyro);
    }
    for (int i = 0; i < 3; ++i) input.sensor.gyro[i] = new_gyro[i];
    
    // 磁気計
    float new_mag[3];
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor, 0, "mag"), new_mag, 3);
    float mag_diff = 0.0f;
    for (int i = 0; i < 3; ++i) {
        float diff = new_mag[i] - input.sensor.mag[i];
        mag_diff += diff * diff;
    }
    if (mag_diff > 1e-6f && input.sensor.current_time > input.sensor.prev_time_mag) {
        input.sensor.dt_mag = (float)(input.sensor.current_time - input.sensor.prev_time_mag);
    }
    for (int i = 0; i < 3; ++i) input.sensor.mag[i] = new_mag[i];
    
    // GPS位置
    mxArray* gps_pos_field_new = mxGetField(m_sensor, 0, "gps_pos");
    float new_gps_pos[3];
    if (gps_pos_field_new) {
        if (mxGetClassID(gps_pos_field_new) != mxDOUBLE_CLASS) {
            mexErrMsgIdAndTxt("mex_hybrid_filter:type_error", 
                "Expected double array for GPS 'gps_pos', but got %s.", 
                mxGetClassName(gps_pos_field_new));
        }
        const double* gps_pr = mxGetPr(gps_pos_field_new);
        for (int i = 0; i < 3; ++i) {
            new_gps_pos[i] = static_cast<float>(gps_pr[i]);
        }
    } else {
        new_gps_pos[0] = new_gps_pos[1] = new_gps_pos[2] = 0.0f;
    }
    float gps_diff = 0.0f;
    for (int i = 0; i < 3; ++i) {
        float diff = new_gps_pos[i] - input.sensor.gps_pos[i];
        gps_diff += diff * diff;
    }
    if (gps_diff > 1e-6f && input.sensor.current_time > input.sensor.prev_time_gps) {
        input.sensor.dt_gps = (float)(input.sensor.current_time - input.sensor.prev_time_gps);
    }
    for (int i = 0; i < 3; ++i) input.sensor.gps_pos[i] = new_gps_pos[i];
    
    // 気圧高度
    float new_baro_alt = mex_conv::mxGetScalarAsFloat(mxGetField(m_sensor, 0, "alt_baro"));
    if (fabsf(new_baro_alt - input.sensor.alt_baro) > 1e-3f && input.sensor.current_time > input.sensor.prev_time_baro) {
        input.sensor.dt_baro = (float)(input.sensor.current_time - input.sensor.prev_time_baro);
    }
    input.sensor.alt_baro = new_baro_alt;
    
    // Params: g, mag_ref, noise (accel/gyro/ba/bg/mag)
    mex_conv::mxArrayToFloatArray(mxGetField(m_params,0,"g"), input.params.g, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_params,0,"mag_ref"), input.params.mag_ref, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_params,0,"noise_accel"), input.params.noise_accel, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_params,0,"noise_gyro"), input.params.noise_gyro, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_params,0,"noise_ba"), input.params.noise_ba, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_params,0,"noise_bg"), input.params.noise_bg, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_params,0,"noise_mag"), input.params.noise_mag, 3);
    mxArray* noise_gps_field = mxGetField(m_params, 0, "noise_gps");
    if (noise_gps_field) {
        if (mxGetClassID(noise_gps_field) != mxDOUBLE_CLASS) {
            mexErrMsgIdAndTxt("mex_hybrid_filter:type_error", 
                "Expected double array for GPS 'noise_gps', but got %s.", 
                mxGetClassName(noise_gps_field));
        }
        const double* noise_gps_pr = mxGetPr(noise_gps_field);
        for (int i = 0; i < 3; ++i) {
            input.params.noise_gps[i] = static_cast<float>(noise_gps_pr[i]);
        }
    } else {
        input.params.noise_gps[0] = input.params.noise_gps[1] = input.params.noise_gps[2] = 0.0f;
    }
    input.params.noise_baro = mex_conv::mxGetScalarAsFloat(mxGetField(m_params,0,"noise_baro"));
    mex_conv::mxArrayToFloatArray(mxGetField(m_params,0,"noise_zupt"), input.params.noise_zupt, 3);
    input.params.alpha = mex_conv::mxGetScalarAsFloat(mxGetField(m_params,0,"alpha"));
    input.params.beta = mex_conv::mxGetScalarAsFloat(mxGetField(m_params,0,"beta"));
    input.params.kappa = mex_conv::mxGetScalarAsFloat(mxGetField(m_params,0,"kappa"));

    // Validate GPS noise
    for(int i=0;i<3;++i) {
        float v = input.params.noise_gps[i];
        if(!std::isfinite(v) || v < 0.0f) {
            mexErrMsgIdAndTxt("MEUKF:step:invalidNoiseGPS", "noise_gps must be finite non-negative variances (meters^2). Got %g at index %d", (double)v, i+1);
        }
    }

    // Call MEUKF core
    meukf::MEUKFOutput output;
    meukf::MEUKFCore::step(input, output);

    // Prepare MATLAB output: duplicate prev_state and update fields
    out_new_state = mxDuplicateArray(m_prev_state);
    // helper to set vec3
    set_vec3_impl(out_new_state, "p", output.new_state.p);
    set_vec3_impl(out_new_state, "v", output.new_state.v);
    set_vec3_impl(out_new_state, "ba", output.new_state.ba);
    set_vec3_impl(out_new_state, "bg", output.new_state.bg);
    // q
    mxArray* f_q = mxGetField(out_new_state, 0, "q");
    if(f_q) {
        if (mxGetClassID(f_q) != mxSINGLE_CLASS) {
            mexErrMsgIdAndTxt("mex_hybrid_filter:type_error", 
                "Expected single (float) array for field 'q', but got %s.", 
                mxGetClassName(f_q));
        } else {
            float* pf = (float*)mxGetData(f_q);
            pf[0] = output.new_state.q[0];
            pf[1] = output.new_state.q[1];
            pf[2] = output.new_state.q[2];
            pf[3] = output.new_state.q[3];
        }
    }
    // P
    mxArray* fP = mxGetField(out_new_state, 0, "P");
    if(fP) {
        if (mxGetClassID(fP) != mxSINGLE_CLASS) {
            mexErrMsgIdAndTxt("mex_hybrid_filter:type_error", 
                "Expected single (float) array for field 'P', but got %s.", 
                mxGetClassName(fP));
        } else {
            float* pf = (float*)mxGetData(fP);
            for(int c=0;c<15;++c) {
                for(int r=0;r<15;++r) {
                    pf[r + c*15] = output.new_state.P[r*15 + c];
                }
            }
        }
    }

    // dbg_out: handle_sensor_update_internalが期待する構造体
    const char* dbg_out_fnames[] = {"innov", "H", "dx"};
    out_dbg_out = mxCreateStructMatrix(1, 1, 3, dbg_out_fnames);

    // dbg_output: 構造体（mex_meukf_step_v2のplhs[2]と互換）
    const char* fnames[] = {"pred_P", "last_K", "last_S", "last_S_inv", "last_H", "last_y", "last_y_len", "last_sensor_type", "input_update_gps", "input_noise_gps"};
    out_dbg_output = mxCreateStructMatrix(1, 1, 10, fnames);
    // Fill dbg output from MEUKFOutput 'output'
    // pred_P: 15 x 15 float matrix
    mxArray* m_pred_P = mxCreateNumericMatrix(15, 15, mxSINGLE_CLASS, mxREAL);
    float* p_pred = (float*)mxGetData(m_pred_P);
    for(int r=0;r<15;++r) for(int c=0;c<15;++c) p_pred[r + c*15] = output.pred_P[r*15 + c];
    mxSetField(out_dbg_output, 0, "pred_P", m_pred_P);

    // last_K: 15 x 3 float matrix
    mxArray* m_last_K = mxCreateNumericMatrix(15, 3, mxSINGLE_CLASS, mxREAL);
    float* p_K = (float*)mxGetData(m_last_K);
    for(int r=0;r<15;++r) for(int c=0;c<3;++c) p_K[r + c*15] = output.last_K[r*3 + c];
    mxSetField(out_dbg_output, 0, "last_K", m_last_K);

    // last_S: 3 x 3 float matrix
    mxArray* m_last_S = mxCreateNumericMatrix(3, 3, mxSINGLE_CLASS, mxREAL);
    float* p_S = (float*)mxGetData(m_last_S);
    for(int r=0;r<3;++r) for(int c=0;c<3;++c) p_S[r + c*3] = output.last_S[r*3 + c];
    mxSetField(out_dbg_output, 0, "last_S", m_last_S);

    // last_y: variable length up to 3
    int ylen = output.last_y_len;
    if(ylen < 0) ylen = 0; if(ylen > 3) ylen = 3;
    mxArray* m_last_y = mxCreateNumericMatrix(ylen, 1, mxSINGLE_CLASS, mxREAL);
    if(ylen > 0) {
        float* p_y = (float*)mxGetData(m_last_y);
        for(int i=0;i<ylen;++i) p_y[i] = output.last_y[i];
    }
    mxSetField(out_dbg_output, 0, "last_y", m_last_y);

    // last_S_inv: 3 x 3 float matrix
    mxArray* m_last_S_inv = mxCreateNumericMatrix(3, 3, mxSINGLE_CLASS, mxREAL);
    float* p_Sinv = (float*)mxGetData(m_last_S_inv);
    for(int r=0;r<3;++r) for(int c=0;c<3;++c) p_Sinv[r + c*3] = output.last_S_inv[r*3 + c];
    mxSetField(out_dbg_output, 0, "last_S_inv", m_last_S_inv);

    // last_H: 3 x 15 float matrix
    mxArray* m_last_H = mxCreateNumericMatrix(3, 15, mxSINGLE_CLASS, mxREAL);
    float* p_H = (float*)mxGetData(m_last_H);
    for(int r=0;r<3;++r) for(int c=0;c<15;++c) p_H[r + c*3] = output.last_H[r*15 + c];
    mxSetField(out_dbg_output, 0, "last_H", m_last_H);

    // last_y_len
    mxArray* m_ylen = mxCreateDoubleScalar(output.last_y_len);
    mxSetField(out_dbg_output, 0, "last_y_len", m_ylen);

    // last_sensor_type
    mxArray* m_stype = mxCreateDoubleScalar(output.last_sensor_type);
    mxSetField(out_dbg_output, 0, "last_sensor_type", m_stype);

    // input_update_gps and input_noise_gps (echo)
    mxArray* m_input_update = mxCreateDoubleScalar(input.sensor.update_gps);
    mxSetField(out_dbg_output, 0, "input_update_gps", m_input_update);
    mxArray* m_input_noise = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* p_noise = (float*)mxGetData(m_input_noise);
    for(int i=0;i<3;++i) p_noise[i] = input.params.noise_gps[i];
    mxSetField(out_dbg_output, 0, "input_noise_gps", m_input_noise);
}

/** Sensor filter wrappers (copied from Inc implementation) **/
inline mxArray* do_sensor_filter_reset_zero() {
    try {
        g_filter_lib.reset_all_zero();
    } catch (...) {
        mexErrMsgIdAndTxt("sensor_filter:reset_zero","reset_all_zero failed");
    }
    return mxCreateLogicalScalar(true);
}

inline mxArray* do_sensor_filter_reset() {
    try {
        g_filter_lib.reset_all();
    } catch (...) {
        mexErrMsgIdAndTxt("sensor_filter:reset","reset_all failed");
    }
    return mxCreateLogicalScalar(true);
}

inline void copy_vec3_from_mx(const mxArray* a, double out[3]) {
    if (!a) { out[0]=out[1]=out[2]=0.0; return; }
    double* pr = mxGetPr(a);
    out[0]=pr[0]; out[1]=pr[1]; out[2]=pr[2];
}

inline mxArray* do_sensor_filter_update(const mxArray* m_sensor) {
    double accel_d[3]={0,0,0}, mag_d[3]={0,0,0}, gps_d[3]={0,0,0};
    copy_vec3_from_mx(mxGetField(m_sensor,0,"accel"), accel_d);
    copy_vec3_from_mx(mxGetField(m_sensor,0,"mag"), mag_d);
    copy_vec3_from_mx(mxGetField(m_sensor,0,"gps_pos"), gps_d);
    double alt_baro = 0.0;
    mxArray* f_alt = mxGetField(m_sensor,0,"alt_baro"); if(f_alt) alt_baro = mxGetScalar(f_alt);
    double dt = 0.0; mxArray* f_dt = mxGetField(m_sensor,0,"dt"); if(f_dt) dt = mxGetScalar(f_dt);

    // Convert to cm (FixedMatrix) used by sensor lib
    cm a_in; a_in.resize(3,1); for(int i=0;i<3;i++) a_in(i,0)=static_cast<float>(accel_d[i]);
    cm m_in; m_in.resize(3,1); for(int i=0;i<3;i++) m_in(i,0)=static_cast<float>(mag_d[i]);
    cm gps_in; gps_in.resize(3,1); for(int i=0;i<3;i++) gps_in(i,0)=static_cast<float>(gps_d[i]);

    bool is_outlier_accel=false, is_outlier_mag=false;
    cm a_filt = g_filter_lib.filter_accel(a_in, a_in, is_outlier_accel);
    cm m_filt = g_filter_lib.filter_mag(m_in, m_in, is_outlier_mag);
    cm pos_out, vel_out; pos_out.resize(3,1); vel_out.resize(3,1);
    g_filter_lib.filter_gps(gps_in, static_cast<float>(dt), pos_out, vel_out);
    float baro_out = g_filter_lib.filter_baro(static_cast<float>(alt_baro));

    const char* fields[] = {"accel", "mag", "gps_pos", "alt_baro", "is_outlier_accel", "is_outlier_mag"};
    mxArray* out = mxCreateStructMatrix(1,1,6,fields);
    mxArray* a_out = mxCreateNumericMatrix(3,1, mxSINGLE_CLASS, mxREAL);
    float* pa = (float*)mxGetData(a_out);
    pa[0]=static_cast<float>(a_filt(0,0)); pa[1]=static_cast<float>(a_filt(1,0)); pa[2]=static_cast<float>(a_filt(2,0));
    mxSetField(out,0,"accel", a_out);
    mxArray* m_out = mxCreateNumericMatrix(3,1, mxSINGLE_CLASS, mxREAL);
    float* pm = (float*)mxGetData(m_out);
    pm[0]=static_cast<float>(m_filt(0,0)); pm[1]=static_cast<float>(m_filt(1,0)); pm[2]=static_cast<float>(m_filt(2,0));
    mxSetField(out,0,"mag", m_out);
    mxArray* g_out = mxCreateNumericMatrix(3,1, mxSINGLE_CLASS, mxREAL);
    float* pg = (float*)mxGetData(g_out);
    pg[0]=static_cast<float>(pos_out(0,0)); pg[1]=static_cast<float>(pos_out(1,0)); pg[2]=static_cast<float>(pos_out(2,0));
    mxSetField(out,0,"gps_pos", g_out);
    mxArray* b_out = mxCreateNumericMatrix(1,1, mxSINGLE_CLASS, mxREAL);
    float* pb = (float*)mxGetData(b_out);
    pb[0] = baro_out;
    mxSetField(out,0,"alt_baro", b_out);
    mxSetField(out,0,"is_outlier_accel", mxCreateLogicalScalar(is_outlier_accel));
    mxSetField(out,0,"is_outlier_mag", mxCreateLogicalScalar(is_outlier_mag));

    return out;

} // namespace mex_hybrid_filter_impl
}

#endif // MEX_IMPL_MEX_RUN_ESKF_IMPL_HPP


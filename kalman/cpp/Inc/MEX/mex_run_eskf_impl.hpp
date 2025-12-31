#pragma once

#ifndef MEX_MEX_RUN_ESKF_IMPL_HPP
#define MEX_MEX_RUN_ESKF_IMPL_HPP

/**
 * mex_run_eskf.cpp用の実装関数群
 * 
 * このヘッダーには、mex_run_eskf.cppで使用される内部関数の定義が含まれます。
 * グローバル変数とstatic関数をinline化してヘッダーに移動しています。
 */

#include "mex_eskf_common.hpp"
#include <cstdint>

// センサー更新関数とフィルター操作関数を先にインクルード（do_stepで使用するため）
#include "mex_run_eskf_sensor_updates.hpp"
#include "mex_run_eskf_filter_ops.hpp"
#include "../MEUKF/meukf_core.hpp"
#include "mex_type_conversion.hpp"

namespace mex_run_eskf_impl {

// グローバル変数（extern宣言、実装は.cppファイルに）
extern std::map<uint64_t, ESKFState*> g_states;
extern uint64_t g_next_handle;
extern SensorFilterLib g_filter_lib;

/**
 * 予測ステップの呼び出し（ESKFRunnerを使用）
 */
inline void call_predict(ESKFState* s, const double* a_meas, const double* w_meas) {
    ESKFRunner::predict(s, a_meas, w_meas);
}

/**
 * 初期化処理
 */
inline uint64_t do_init(const mxArray* obs, double static_time, double dt) {
    ESKFState* s = initialize_eskf_from_matlab(obs, static_time, dt);
    uint64_t handle = g_next_handle++;
    g_states[handle] = s;
    return handle;
}

/**
 * ステップ処理
 */
inline void do_step(ESKFState* s, const mxArray* obs, int k) {
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
    call_sensor_update(s, "accel", a, 3, static_cast<double>(k));
    call_sensor_update(s, "mag", m, 3, static_cast<double>(k));
    
    // Baro
    mxArray* baro_field = mxGetField(obs, 0, "pressure");
    if (baro_field) {
        double baro = mxGetPr(baro_field)[idx];
        double baro_arr[1] = {baro};
        call_sensor_update(s, "baro", baro_arr, 1, static_cast<double>(k));
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
            call_gps_update(s, lat, lon, alt, static_cast<double>(k));
        }
    }
    
    // Reset check
    check_and_reset(s, k);
}

/**
 * 状態取得処理
 */
inline mxArray* do_get_state(ESKFState* s) {
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

/**
 * メモリ解放処理
 */
inline void do_free(uint64_t handle) {
    auto it = g_states.find(handle);
    if (it != g_states.end()) {
        delete it->second;
        g_states.erase(it);
    }
}

/**
 * MEUKF の単体ステップ呼び出しラッパー
 * 入力は mex_meukf_step と同様: prev_state (struct), sensor (struct), params (struct)
 * 出力は MATLAB 構造体（state更新）を plhs[0] として返すことを想定
 */
inline mxArray* do_meukf_step(const mxArray* m_prev_state, const mxArray* m_sensor, const mxArray* m_params) {
    meukf::MEUKFInput input;
    // State変換
    // p, v, ba, bg
    mex_conv::mxArrayToFloatArray(mxGetField(m_prev_state,0,"p"), input.prev_state.p, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_prev_state,0,"v"), input.prev_state.v, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_prev_state,0,"ba"), input.prev_state.ba, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_prev_state,0,"bg"), input.prev_state.bg, 3);
    // q
    mex_conv::mxArrayToFloatArray(mxGetField(m_prev_state,0,"q"), input.prev_state.q, 4);
    // P: MATLAB column-major -> internal row-major
    mxArray* f_P = mxGetField(m_prev_state, 0, "P");
    if (f_P) {
        float P_tmp[15*15];
        mex_conv::mxArrayToFloatArray(f_P, P_tmp, 15*15);
        for (int r=0;r<15;++r) for (int c=0;c<15;++c) input.prev_state.P[r*15 + c] = P_tmp[c*15 + r];
    }

    // SensorData変換（必要なフィールドのみ）
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor,0,"accel"), input.sensor.accel, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor,0,"gyro"), input.sensor.gyro, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor,0,"mag"), input.sensor.mag, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor,0,"gps_pos"), input.sensor.gps_pos, 3);
    input.sensor.alt_baro = mex_conv::mxGetScalarAsFloat(mxGetField(m_sensor,0,"alt_baro"));
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor,0,"prev_mag"), input.sensor.prev_mag, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor,0,"prev_gps_pos"), input.sensor.prev_gps_pos, 3);
    input.sensor.prev_baro_alt = mex_conv::mxGetScalarAsFloat(mxGetField(m_sensor,0,"prev_baro_alt"));
    auto get_field_scalar = [&](const mxArray* s, const char* f)->double{ mxArray* ff = mxGetField(s,0,f); return ff? static_cast<double>(mex_conv::mxGetScalarAsFloat(ff)) : 0.0; };
    input.sensor.update_accel = (uint8_t)get_field_scalar(m_sensor, "update_accel");
    input.sensor.update_gyro = (uint8_t)get_field_scalar(m_sensor, "update_gyro");
    input.sensor.update_mag = (uint8_t)get_field_scalar(m_sensor, "update_mag");
    input.sensor.update_gps = (uint8_t)get_field_scalar(m_sensor, "update_gps");
    input.sensor.update_baro = (uint8_t)get_field_scalar(m_sensor, "update_baro");
    input.sensor.update_zupt = (uint8_t)get_field_scalar(m_sensor, "update_zupt");
    input.sensor.dt = mex_conv::mxGetScalarAsFloat(mxGetField(m_sensor,0,"dt"));

    // Params
    mex_conv::mxArrayToFloatArray(mxGetField(m_params,0,"g"), input.params.g, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_params,0,"mag_ref"), input.params.mag_ref, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_params,0,"noise_accel"), input.params.noise_accel, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_params,0,"noise_gyro"), input.params.noise_gyro, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_params,0,"noise_ba"), input.params.noise_ba, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_params,0,"noise_bg"), input.params.noise_bg, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_params,0,"noise_mag"), input.params.noise_mag, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_params,0,"noise_gps"), input.params.noise_gps, 3);
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

    meukf::MEUKFOutput output;
    meukf::MEUKFCore::step(input, output);

    // Prepare MATLAB output: duplicate prev_state and update fields
    mxArray* out_state = mxDuplicateArray(m_prev_state);
    // helper to set vec3
    auto set_vec3 = [&](const char* name, const float* in) {
        mxArray* f = mxGetField(out_state, 0, name);
        if(!f) return;
        double* pr = mxGetPr(f);
        pr[0] = static_cast<double>(in[0]); pr[1] = static_cast<double>(in[1]); pr[2] = static_cast<double>(in[2]);
    };
    set_vec3("p", output.new_state.p);
    set_vec3("v", output.new_state.v);
    set_vec3("ba", output.new_state.ba);
    set_vec3("bg", output.new_state.bg);
    // q
    mxArray* f_q = mxGetField(out_state, 0, "q"); if(f_q) { double* pr = mxGetPr(f_q); pr[0]=output.new_state.q[0]; pr[1]=output.new_state.q[1]; pr[2]=output.new_state.q[2]; pr[3]=output.new_state.q[3]; }
    // P
    mxArray* fP = mxGetField(out_state, 0, "P"); if(fP) {
        double* pr = mxGetPr(fP);
        for(int c=0;c<15;++c) for(int r=0;r<15;++r) pr[c*15 + r] = static_cast<double>(output.new_state.P[r*15 + c]);
    }

    return out_state;
}

/** Sensor filter wrappers **/
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
    // Expect struct with fields accel, gyro, mag, gps_pos, alt_baro, dt
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

    // Build output struct: fields filtered and is_outlier flags
    const char* fields[] = {"accel", "mag", "gps_pos", "alt_baro", "is_outlier_accel", "is_outlier_mag"};
    mxArray* out = mxCreateStructMatrix(1,1,6,fields);
    mxArray* a_out = mxCreateDoubleMatrix(3,1,mxREAL); double* pa = mxGetPr(a_out);
    mxArray* m_out = mxCreateDoubleMatrix(3,1,mxREAL); double* pm = mxGetPr(m_out);
    mxArray* g_out = mxCreateDoubleMatrix(3,1,mxREAL); double* pg = mxGetPr(g_out);
    pa[0]=a_filt(0,0); pa[1]=a_filt(1,0); pa[2]=a_filt(2,0);
    pm[0]=m_filt(0,0); pm[1]=m_filt(1,0); pm[2]=m_filt(2,0);
    pg[0]=pos_out(0,0); pg[1]=pos_out(1,0); pg[2]=pos_out(2,0);
    mxSetField(out,0,"accel", a_out);
    mxSetField(out,0,"mag", m_out);
    mxSetField(out,0,"gps_pos", g_out);
    mxSetField(out,0,"alt_baro", mxCreateDoubleScalar(static_cast<double>(baro_out)));
    mxSetField(out,0,"is_outlier_accel", mxCreateLogicalScalar(is_outlier_accel));
    mxSetField(out,0,"is_outlier_mag", mxCreateLogicalScalar(is_outlier_mag));

    return out;
}

} // namespace mex_run_eskf_impl

#endif // MEX_MEX_RUN_ESKF_IMPL_HPP


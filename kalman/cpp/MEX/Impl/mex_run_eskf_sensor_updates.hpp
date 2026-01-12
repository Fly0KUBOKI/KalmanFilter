#pragma once

#ifndef MEX_MEX_RUN_ESKF_SENSOR_UPDATES_HPP_IMPL
#define MEX_MEX_RUN_ESKF_SENSOR_UPDATES_HPP_IMPL

/**
 * mex_run_eskf.cpp用のセンサー更新関数群
 * 
 * 長い関数（call_sensor_update, call_gps_update）の実装を含みます。
 */

#include "mex_eskf_common.hpp"
#include "mex_type_conversion.hpp"
#include <cstring>
#include "../../Lib/Common/inc/Math/portable_math.hpp"

namespace mex_run_eskf_impl {

// 前方宣言
inline void do_sensor_update_meukf(const mxArray* m_prev_state, const mxArray* m_sensor, const mxArray* m_params,
                          mxArray*& out_new_state, mxArray*& out_dbg_out, mxArray*& out_dbg_output);

inline void handle_sensor_update_internal(
    const char* sensor_type,
    const double* meas, int meas_len,
    const double* p, const double* v, const double* q,
    const double* ba, const double* bg, const double* P,
    const double* g, double dt, double sample,
    ESKFState* s,  // ESKFState pointer for accessing prev_* values
    double* out_p, double* out_v, double* out_q,
    double* out_ba, double* out_bg, double* out_P,
    bool& should_skip
);

/**
 * センサー更新処理（mexCallMATLAB実装）
 */
inline void call_sensor_update(ESKFState* s, const char* type, const double* meas, int meas_len, double sample) {
    // Revert to original implementation from mex_eskf_sensor_updates_full.cpp
    double p[3], v[3], q[4], ba[3], bg[3], P[15*15], g[3];
    // Copy from float-typed ESKFState into local double buffers
    for (int i = 0; i < 3; ++i) p[i] = s->p[i];
    for (int i = 0; i < 3; ++i) v[i] = s->v[i];
    for (int i = 0; i < 4; ++i) q[i] = s->q[i];
    for (int i = 0; i < 3; ++i) ba[i] = s->ba[i];
    for (int i = 0; i < 3; ++i) bg[i] = s->bg[i];
    for (int i = 0; i < 15*15; ++i) P[i] = s->P[i];
    for (int i = 0; i < 3; ++i) g[i] = s->g[i];
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
        // Preprocess accel (C++ direct implementation)
        cmath_fx::Vector<3, float> a_meas_f;
        cmath_fx::Vector<3, float> prev_a_f;
        for (int i = 0; i < 3; ++i) {
            a_meas_f(i, 0) = static_cast<float>(meas[i]);
            prev_a_f(i, 0) = s->prev_accel[i];
        }
        
        PreprocessResult result = preprocess_accel(a_meas_f, prev_a_f, s->buffer_tolerance);
        
        double a_corrected[3];
        for (int i = 0; i < 3; ++i) {
            a_corrected[i] = result.output(i, 0);
        }
        bool is_outlier = result.is_outlier;
        bool no_change = result.no_change;
        
        // Check w_body norm
        double w_norm = 0.0;
        for (int i = 0; i < 3; ++i) {
            double w = s->w_body[i];
            w_norm += w * w;
        }
        w_norm = common::math::portable_sqrt(w_norm);
        
        if (!no_change && !is_nan_any(a_corrected, 3) && !is_outlier && (w_norm <= 1.5)) {
            should_skip = false;
            // store prev_accel into float prev buffer
            for (int i = 0; i < 3; ++i) s->prev_accel[i] = static_cast<float>(meas[i]);
        }
        
        if (!should_skip) {
            // Call handle_sensor_update_internal directly (integrated from mex_eskf_do_update)
            handle_sensor_update_internal(
                "accel", a_corrected, 3,
                out_p, out_v, out_q, out_ba, out_bg, out_P,
                g, dt, sample, s,
                out_p, out_v, out_q, out_ba, out_bg, out_P,
                should_skip
            );
        }
    }
    else if (strcmp(type, "mag") == 0 && meas_len == 3) {
        // Preprocess mag (C++ direct implementation)
        cmath_fx::Vector<3, float> m_meas_f;
        cmath_fx::Vector<3, float> prev_m_f;
        for (int i = 0; i < 3; ++i) {
            m_meas_f(i, 0) = static_cast<float>(meas[i]);
            prev_m_f(i, 0) = s->prev_mag[i];
        }
        
        PreprocessResult result = preprocess_mag(m_meas_f, prev_m_f, s->buffer_tolerance);
        
        double m_filtered[3];
        for (int i = 0; i < 3; ++i) {
            m_filtered[i] = result.output(i, 0);
        }
        bool is_outlier = result.is_outlier;
        bool no_change = result.no_change;
        
        if (!no_change && !is_nan_any(m_filtered, 3) && !is_outlier) {
            should_skip = false;
            for (int i = 0; i < 3; ++i) s->prev_mag[i] = static_cast<float>(meas[i]);
        }
        
        if (!should_skip) {
            // Call handle_sensor_update_internal directly (integrated from mex_eskf_do_update)
            handle_sensor_update_internal(
                "mag", m_filtered, 3,
                out_p, out_v, out_q, out_ba, out_bg, out_P,
                g, dt, sample, s,
                out_p, out_v, out_q, out_ba, out_bg, out_P,
                should_skip
            );
        }
    }
    else if (strcmp(type, "baro") == 0 && meas_len == 1) {
        double pressure = meas[0];
        double prev_baro = s->prev_baro;
        
        if (fabs(pressure - prev_baro) > s->buffer_tolerance) {
            should_skip = false;
            s->prev_baro = pressure;
            
            // Preprocess baro (C++ direct implementation)
            double alt_baro = preprocess_baro(pressure);
            
            if (!should_skip) {
                // Call handle_sensor_update_internal directly (integrated from mex_eskf_do_update)
                double alt_baro_arr[1] = {alt_baro};
                handle_sensor_update_internal(
                    "baro", alt_baro_arr, 1,
                    out_p, out_v, out_q, out_ba, out_bg, out_P,
                    g, dt, sample, s,
                    out_p, out_v, out_q, out_ba, out_bg, out_P,
                    should_skip
                );
            }
        }
    }
    
    // Update state if not skipped
    if (!should_skip) {
        for (int i = 0; i < 3; ++i) s->p[i] = static_cast<float>(out_p[i]);
        for (int i = 0; i < 3; ++i) s->v[i] = static_cast<float>(out_v[i]);
        for (int i = 0; i < 4; ++i) s->q[i] = static_cast<float>(out_q[i]);
        for (int i = 0; i < 3; ++i) s->ba[i] = static_cast<float>(out_ba[i]);
        for (int i = 0; i < 3; ++i) s->bg[i] = static_cast<float>(out_bg[i]);
        for (int i = 0; i < 15*15; ++i) s->P[i] = static_cast<float>(out_P[i]);
    }
}

/**
 * GPS更新処理（mexCallMATLAB実装）
 */
inline void call_gps_update(ESKFState* s, double lat, double lon, double alt, double sample) {
    // Revert to original implementation from mex_eskf_sensor_updates_full.cpp
    double p[3], v[3], q[4], ba[3], bg[3], P[15*15], g[3];
    for (int i = 0; i < 3; ++i) p[i] = s->p[i];
    for (int i = 0; i < 3; ++i) v[i] = s->v[i];
    for (int i = 0; i < 4; ++i) q[i] = s->q[i];
    for (int i = 0; i < 3; ++i) ba[i] = s->ba[i];
    for (int i = 0; i < 3; ++i) bg[i] = s->bg[i];
    for (int i = 0; i < 15*15; ++i) P[i] = s->P[i];
    for (int i = 0; i < 3; ++i) g[i] = s->g[i];
    double dt = s->dt;
    
    double out_p[3], out_v[3], out_q[4], out_ba[3], out_bg[3], out_P[15*15];
    memcpy(out_p, p, 3 * sizeof(double));
    memcpy(out_v, v, 3 * sizeof(double));
    memcpy(out_q, q, 4 * sizeof(double));
    memcpy(out_ba, ba, 3 * sizeof(double));
    memcpy(out_bg, bg, 3 * sizeof(double));
    memcpy(out_P, P, 15*15*sizeof(double));
    
    // Preprocess GPS (C++ direct implementation)
    cmath_fx::Vector<3, float> origin_f;
    for (int i = 0; i < 3; ++i) {
        origin_f(i, 0) = static_cast<float>(s->gps_origin[i]);
    }
    
    PreprocessResult result = preprocess_gps(lat, lon, alt, origin_f, s->buffer_tolerance);
    
    bool should_skip = true;
    double z_gps[3];
    for (int i = 0; i < 3; ++i) {
        z_gps[i] = result.output(i, 0);
    }
    bool is_outlier = result.is_outlier;
    bool no_change = result.no_change;
    
    if (!no_change && !is_outlier) {
        should_skip = false;
    }
    
    // (debug prints removed)
    
    if (!should_skip) {
        // Call handle_sensor_update_internal directly (integrated from mex_eskf_do_update)
        handle_sensor_update_internal(
            "gps", z_gps, 3,
            out_p, out_v, out_q, out_ba, out_bg, out_P,
            g, dt, sample, s,
            out_p, out_v, out_q, out_ba, out_bg, out_P,
            should_skip
        );
        
        // Update prev_gps
        s->prev_gps_lat = lat;
        s->prev_gps_lon = lon;
        s->prev_gps_alt = alt;
    }
    
    // Update state if not skipped
    if (!should_skip) {
        for (int i = 0; i < 3; ++i) s->p[i] = static_cast<float>(out_p[i]);
            for (int i = 0; i < 3; ++i) s->v[i] = static_cast<float>(out_v[i]);
            for (int i = 0; i < 4; ++i) s->q[i] = static_cast<float>(out_q[i]);
            for (int i = 0; i < 3; ++i) s->ba[i] = static_cast<float>(out_ba[i]);
            for (int i = 0; i < 3; ++i) s->bg[i] = static_cast<float>(out_bg[i]);
            for (int i = 0; i < 15*15; ++i) s->P[i] = static_cast<float>(out_P[i]);
    }
}

/**
 * センサー更新の内部処理（mex_eskf_do_updateのhandle_update関数を統合）
 * 入力: sensor_type, meas, p, v, q, ba, bg, P, g, dt, [sample]
 * 出力: out_p, out_v, out_q, out_ba, out_bg, out_P, should_skip
 */
inline void handle_sensor_update_internal(
    const char* sensor_type,
    const double* meas, int meas_len,
    const double* p, const double* v, const double* q,
    const double* ba, const double* bg, const double* P,
    const double* g, double dt, double sample,
    ESKFState* s,  // ESKFState pointer for accessing prev_* values
    double* out_p, double* out_v, double* out_q,
    double* out_ba, double* out_bg, double* out_P,
    bool& should_skip
) {
    // 出力バッファを初期化
    copy_vec(out_p, p, 3);
    copy_vec(out_v, v, 3);
    copy_vec(out_q, q, 4);
    copy_vec(out_ba, ba, 3);
    copy_vec(out_bg, bg, 3);
    memcpy(out_P, P, 15*15*sizeof(double));
    should_skip = false;
    
    // R取得 (C++ direct implementation)
    double R_noise[3] = {0.01, 0.01, 0.01};
    {
        cmath_fx::FixedMatrix R = g_filter_lib.noise_estimator.get_R_matrix(sensor_type);
        int n_rows = R.rows;
        int n_cols = R.cols;
        if (n_rows == 3 && n_cols == 3) {
            // 3x3行列から対角要素を取得
            R_noise[0] = R(0, 0);
            R_noise[1] = R(1, 1);
            R_noise[2] = R(2, 2);
        } else if (n_rows >= 3 && n_cols == 1) {
            // ベクトル形式
            R_noise[0] = R(0, 0);
            R_noise[1] = R(1, 0);
            R_noise[2] = R(2, 0);
        } else if (n_rows == 1 && n_cols == 1) {
            // スカラー（baroなど）
            R_noise[0] = R_noise[1] = R_noise[2] = R(0, 0);
        }
    }

    // sensor_data構造体を構築（GPS以外はsingle、GPSはdouble）
    const char* sd_fields[] = {"accel", "gyro", "mag", "gps_pos", "alt_baro", "dt",
        "update_accel", "update_gyro", "update_mag", "update_gps", "update_baro", "update_zupt",
        "prev_mag", "prev_gps_pos", "prev_baro_alt"};
    mxArray* sensor_data = mxCreateStructMatrix(1, 1, 15, sd_fields);

    float zeros3f[3] = {0.0f, 0.0f, 0.0f};
    double zeros3d[3] = {0.0, 0.0, 0.0};
    // GPS以外のセンサーデータはsingle（float）
    mxArray* accel_arr = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* accel_ptr = (float*)mxGetData(accel_arr);
    for (int i = 0; i < 3; ++i) accel_ptr[i] = zeros3f[i];
    mxArray* gyro_arr = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* gyro_ptr = (float*)mxGetData(gyro_arr);
    for (int i = 0; i < 3; ++i) gyro_ptr[i] = zeros3f[i];
    mxArray* mag_arr = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* mag_ptr = (float*)mxGetData(mag_arr);
    for (int i = 0; i < 3; ++i) mag_ptr[i] = zeros3f[i];
    // GPSデータはdouble
    mxArray* gps_arr = mxCreateDoubleMatrix(3, 1, mxREAL); 
    copy_vec(mxGetPr(gps_arr), zeros3d, 3);

    mxSetField(sensor_data, 0, "accel", accel_arr);
    mxSetField(sensor_data, 0, "gyro", gyro_arr);
    mxSetField(sensor_data, 0, "mag", mag_arr);
    mxSetField(sensor_data, 0, "gps_pos", gps_arr);
    mxArray* alt_baro_init = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
    float* alt_baro_init_ptr = (float*)mxGetData(alt_baro_init);
    alt_baro_init_ptr[0] = 0.0f;
    mxSetField(sensor_data, 0, "alt_baro", alt_baro_init);
    mxArray* dt_arr = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
    float* dt_ptr = (float*)mxGetData(dt_arr);
    dt_ptr[0] = static_cast<float>(dt);
    mxSetField(sensor_data, 0, "dt", dt_arr);
    mxSetField(sensor_data, 0, "update_accel", mxCreateLogicalScalar(false));
    mxSetField(sensor_data, 0, "update_gyro", mxCreateLogicalScalar(false));
    mxSetField(sensor_data, 0, "update_mag", mxCreateLogicalScalar(false));
    mxSetField(sensor_data, 0, "update_gps", mxCreateLogicalScalar(false));
    mxSetField(sensor_data, 0, "update_baro", mxCreateLogicalScalar(false));
    mxSetField(sensor_data, 0, "update_zupt", mxCreateLogicalScalar(false));
    
    // Set prev_* values from ESKFState
    // prev_magはsingle（float）
    mxArray* prev_mag_arr = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* prev_mag_ptr = (float*)mxGetData(prev_mag_arr);
    for (int i = 0; i < 3; ++i) prev_mag_ptr[i] = s->prev_mag[i];
    mxSetField(sensor_data, 0, "prev_mag", prev_mag_arr);
    
    // Convert GPS lat/lon/alt to ECEF position (or use stored prev_gps_pos if available)
    // prev_gps_posはdouble（GPSデータ）
    mxArray* prev_gps_pos_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    double prev_gps_pos[3] = {0, 0, 0};
    // If GPS was previously set, we could convert lat/lon/alt to ECEF here
    // For simplicity, use zeros (mex_meukf_step_v2 will handle the change detection)
    copy_vec(mxGetPr(prev_gps_pos_arr), prev_gps_pos, 3);
    mxSetField(sensor_data, 0, "prev_gps_pos", prev_gps_pos_arr);
    
    // prev_baro_altはsingle（float）
    mxArray* prev_baro_alt_arr = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
    float* prev_baro_alt_ptr = (float*)mxGetData(prev_baro_alt_arr);
    prev_baro_alt_ptr[0] = s->prev_baro;
    mxSetField(sensor_data, 0, "prev_baro_alt", prev_baro_alt_arr);

    // mex_params構造体（GPS以外はsingle、GPSはdouble）
    const char* mp_fields[] = {"g", "mag_ref", "noise_accel", "noise_gyro", "noise_ba", "noise_bg",
        "noise_mag", "noise_gps", "noise_baro", "noise_zupt", "alpha", "beta", "kappa", "trace_sample"};
    mxArray* mex_params = mxCreateStructMatrix(1, 1, 14, mp_fields);

    // g, mag_refはsingle（float）
    mxArray* g_arr = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* g_ptr = (float*)mxGetData(g_arr);
    for (int i = 0; i < 3; ++i) g_ptr[i] = static_cast<float>(g[i]);
    mxSetField(mex_params, 0, "g", g_arr);
    float mag_ref[3] = {50.0f, 0.0f, 0.0f};
    mxArray* mag_ref_arr = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* mag_ref_ptr = (float*)mxGetData(mag_ref_arr);
    for (int i = 0; i < 3; ++i) mag_ref_ptr[i] = mag_ref[i];
    mxSetField(mex_params, 0, "mag_ref", mag_ref_arr);

    // ノイズパラメータはsingle（float）、GPSノイズはdouble
    mxArray* na = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* na_ptr = (float*)mxGetData(na);
    for (int i = 0; i < 3; ++i) na_ptr[i] = zeros3f[i];
    mxArray* ng = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* ng_ptr = (float*)mxGetData(ng);
    for (int i = 0; i < 3; ++i) ng_ptr[i] = zeros3f[i];
    mxArray* nba = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* nba_ptr = (float*)mxGetData(nba);
    for (int i = 0; i < 3; ++i) nba_ptr[i] = zeros3f[i];
    mxArray* nbg = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* nbg_ptr = (float*)mxGetData(nbg);
    for (int i = 0; i < 3; ++i) nbg_ptr[i] = zeros3f[i];
    mxArray* nm = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* nm_ptr = (float*)mxGetData(nm);
    for (int i = 0; i < 3; ++i) nm_ptr[i] = zeros3f[i];
    // noise_gpsはdouble（GPSデータ）
    mxArray* ngps = mxCreateDoubleMatrix(3, 1, mxREAL); 
    copy_vec(mxGetPr(ngps), zeros3d, 3);
    mxArray* nz = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* nz_ptr = (float*)mxGetData(nz);
    for (int i = 0; i < 3; ++i) nz_ptr[i] = zeros3f[i];

    mxSetField(mex_params, 0, "noise_accel", na);
    mxSetField(mex_params, 0, "noise_gyro", ng);
    mxSetField(mex_params, 0, "noise_ba", nba);
    mxSetField(mex_params, 0, "noise_bg", nbg);
    mxSetField(mex_params, 0, "noise_mag", nm);
    mxSetField(mex_params, 0, "noise_gps", ngps);
    mxArray* noise_baro_init = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
    float* noise_baro_init_ptr = (float*)mxGetData(noise_baro_init);
    noise_baro_init_ptr[0] = 0.0f;
    mxSetField(mex_params, 0, "noise_baro", noise_baro_init);
    mxSetField(mex_params, 0, "noise_zupt", nz);
    mxArray* alpha_arr = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
    float* alpha_ptr = (float*)mxGetData(alpha_arr);
    alpha_ptr[0] = 1e-3f;
    mxSetField(mex_params, 0, "alpha", alpha_arr);
    mxArray* beta_arr = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
    float* beta_ptr = (float*)mxGetData(beta_arr);
    beta_ptr[0] = 2.0f;
    mxSetField(mex_params, 0, "beta", beta_arr);
    mxArray* kappa_arr = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
    float* kappa_ptr = (float*)mxGetData(kappa_arr);
    kappa_ptr[0] = 0.0f;
    mxSetField(mex_params, 0, "kappa", kappa_arr);
    mxSetField(mex_params, 0, "trace_sample", mxCreateDoubleScalar(sample));

    // センサータイプ別設定
    if (strcmp(sensor_type, "accel") == 0) {
        // accelはsingle（float）
        float* accel_ptr = (float*)mxGetData(mxGetField(sensor_data, 0, "accel"));
        for (int i = 0; i < 3; ++i) accel_ptr[i] = static_cast<float>(meas[i]);
        for (int i = 0; i < 3; ++i) R_noise[i] *= 1.5;
        float* noise_accel_ptr = (float*)mxGetData(mxGetField(mex_params, 0, "noise_accel"));
        for (int i = 0; i < 3; ++i) noise_accel_ptr[i] = static_cast<float>(R_noise[i]);
        mxDestroyArray(mxGetField(sensor_data, 0, "update_accel"));
        mxSetField(sensor_data, 0, "update_accel", mxCreateLogicalScalar(true));
    } else if (strcmp(sensor_type, "mag") == 0) {
        // magはsingle（float）
        float* mag_ptr = (float*)mxGetData(mxGetField(sensor_data, 0, "mag"));
        for (int i = 0; i < 3; ++i) mag_ptr[i] = static_cast<float>(meas[i]);
        for (int i = 0; i < 3; ++i) R_noise[i] *= 1.5;
        float* noise_mag_ptr = (float*)mxGetData(mxGetField(mex_params, 0, "noise_mag"));
        for (int i = 0; i < 3; ++i) noise_mag_ptr[i] = static_cast<float>(R_noise[i]);
        mxDestroyArray(mxGetField(sensor_data, 0, "update_mag"));
        mxSetField(sensor_data, 0, "update_mag", mxCreateLogicalScalar(true));
    } else if (strcmp(sensor_type, "gps") == 0) {
        // gps_posはdouble（GPSデータ）
        copy_vec(mxGetPr(mxGetField(sensor_data, 0, "gps_pos")), meas, 3);
        copy_vec(mxGetPr(mxGetField(mex_params, 0, "noise_gps")), R_noise, 3);
        mxDestroyArray(mxGetField(sensor_data, 0, "update_gps"));
        mxSetField(sensor_data, 0, "update_gps", mxCreateLogicalScalar(true));
    } else if (strcmp(sensor_type, "baro") == 0) {
        // alt_baroはsingle（float）
        mxDestroyArray(mxGetField(sensor_data, 0, "alt_baro"));
        mxArray* alt_baro_arr = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
        float* alt_baro_ptr = (float*)mxGetData(alt_baro_arr);
        alt_baro_ptr[0] = static_cast<float>(meas[0]);
        mxSetField(sensor_data, 0, "alt_baro", alt_baro_arr);
        mxDestroyArray(mxGetField(mex_params, 0, "noise_baro"));
        mxArray* noise_baro_arr = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
        float* noise_baro_ptr = (float*)mxGetData(noise_baro_arr);
        noise_baro_ptr[0] = static_cast<float>(R_noise[0]);
        mxSetField(mex_params, 0, "noise_baro", noise_baro_arr);
        mxDestroyArray(mxGetField(sensor_data, 0, "update_baro"));
        mxSetField(sensor_data, 0, "update_baro", mxCreateLogicalScalar(true));
    } else if (strcmp(sensor_type, "zupt") == 0) {
        // noise_zuptはsingle（float）
        float zupt_noise[3] = {0.0001f, 0.0001f, 0.0001f};
        float* noise_zupt_ptr = (float*)mxGetData(mxGetField(mex_params, 0, "noise_zupt"));
        for (int i = 0; i < 3; ++i) noise_zupt_ptr[i] = zupt_noise[i];
        mxDestroyArray(mxGetField(sensor_data, 0, "update_zupt"));
        mxSetField(sensor_data, 0, "update_zupt", mxCreateLogicalScalar(true));
    } else {
        mexErrMsgTxt("Unknown sensor type");
    }

    // state構造体（すべてsingle（float））
    const char* st_fields[] = {"p", "v", "q", "ba", "bg", "P"};
    mxArray* state_s = mxCreateStructMatrix(1, 1, 6, st_fields);
    mxArray* p_arr = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* p_ptr = (float*)mxGetData(p_arr);
    for (int i = 0; i < 3; ++i) p_ptr[i] = static_cast<float>(p[i]);
    mxArray* v_arr = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* v_ptr = (float*)mxGetData(v_arr);
    for (int i = 0; i < 3; ++i) v_ptr[i] = static_cast<float>(v[i]);
    mxArray* q_arr = mxCreateNumericMatrix(4, 1, mxSINGLE_CLASS, mxREAL);
    float* q_ptr = (float*)mxGetData(q_arr);
    for (int i = 0; i < 4; ++i) q_ptr[i] = static_cast<float>(q[i]);
    mxArray* ba_arr = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* ba_ptr = (float*)mxGetData(ba_arr);
    for (int i = 0; i < 3; ++i) ba_ptr[i] = static_cast<float>(ba[i]);
    mxArray* bg_arr = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* bg_ptr = (float*)mxGetData(bg_arr);
    for (int i = 0; i < 3; ++i) bg_ptr[i] = static_cast<float>(bg[i]);
    mxArray* P_arr = mxCreateNumericMatrix(15, 15, mxSINGLE_CLASS, mxREAL);
    float* P_ptr = (float*)mxGetData(P_arr);
    for (int i = 0; i < 15*15; ++i) P_ptr[i] = static_cast<float>(P[i]);
    mxSetField(state_s, 0, "p", p_arr);
    mxSetField(state_s, 0, "v", v_arr);
    mxSetField(state_s, 0, "q", q_arr);
    mxSetField(state_s, 0, "ba", ba_arr);
    mxSetField(state_s, 0, "bg", bg_arr);
    mxSetField(state_s, 0, "P", P_arr);

    // Phase 1: MEUKF呼び出しの統合（mexCallMATLAB → do_sensor_update_meukf）
    // 最も簡単な部分から統合開始
    mxArray* new_state = nullptr;
    mxArray* dbg_out = nullptr;
    mxArray* dbg_output = nullptr;
    do_sensor_update_meukf(state_s, sensor_data, mex_params, new_state, dbg_out, dbg_output);
    
    if (new_state) {
        // 後続処理は同じ（noise estimate更新、postprocess等）

        // noise estimate更新 (C++ direct implementation)
        if (mxIsStruct(dbg_out) && mxGetField(dbg_out, 0, "innov") && mxGetField(dbg_out, 0, "H")) {
            // Get innov
            const mxArray* innov = mxGetField(dbg_out, 0, "innov");
            int innov_len = mxGetM(innov) * mxGetN(innov);
            if (innov_len < 1) innov_len = 1;
            if (innov_len > 3) innov_len = 3;
            
            cmath_fx::FixedMatrix innov_cm(innov_len, 1);
            std::vector<float> innov_tmp(static_cast<size_t>(innov_len));
            mex_conv::mxArrayToFloatArray(innov, innov_tmp.data(), static_cast<size_t>(innov_len));
            for (int i = 0; i < innov_len; ++i) {
                innov_cm(i, 0) = innov_tmp[i];
            }
            
            // Get H
            const mxArray* H = mxGetField(dbg_out, 0, "H");
            int H_rows = mxGetM(H);
            int H_cols = mxGetN(H);
            if (H_rows < 1) H_rows = 1;
            if (H_cols < 1) H_cols = 1;
            if (H_rows > 3) H_rows = 3;
            if (H_cols > 15) H_cols = 15;
            
            cmath_fx::FixedMatrix H_cm(H_rows, H_cols);
            std::vector<float> H_tmp(static_cast<size_t>(H_rows * H_cols));
            mex_conv::mxArrayToFloatArray(H, H_tmp.data(), static_cast<size_t>(H_rows * H_cols));
            for (int j = 0; j < H_cols; ++j) {
                for (int i = 0; i < H_rows; ++i) {
                    H_cm(i, j) = H_tmp[j * H_rows + i];  // MATLAB column-major to row-major
                }
            }
            
            // Get P_pred
            cmath_fx::FixedMatrix P_pred(15, 15);
            std::vector<float> P_tmp(15 * 15);
            mex_conv::mxArrayToFloatArray(P_arr, P_tmp.data(), 15 * 15);
            for (int j = 0; j < 15; ++j) {
                for (int i = 0; i < 15; ++i) {
                    P_pred(i, j) = P_tmp[j * 15 + i];  // MATLAB column-major to row-major
                }
            }
            
            // Call noise estimate directly
            g_filter_lib.noise_estimator.estimate(sensor_type, innov_cm, H_cm, P_pred);
        }

        // postprocess (mex_eskf_update_postprocess を統合)
        if (mxIsStruct(dbg_out) && mxGetField(dbg_out, 0, "dx")) {
            // Get dx (15x1)
            Vector<15, float> dx;
            if (!mex_conv::matToVector(mxGetField(dbg_out, 0, "dx"), dx)) {
                // Fallback: read `new_state` (single) fields directly into double buffers
                mxArray* p_field = mxGetField(new_state, 0, "p");
                if (p_field && mxGetClassID(p_field) == mxSINGLE_CLASS) {
                    const float* pf = (const float*)mxGetData(p_field);
                    for (int i = 0; i < 3; ++i) out_p[i] = pf[i];
                }
                mxArray* v_field = mxGetField(new_state, 0, "v");
                if (v_field && mxGetClassID(v_field) == mxSINGLE_CLASS) {
                    const float* vf = (const float*)mxGetData(v_field);
                    for (int i = 0; i < 3; ++i) out_v[i] = vf[i];
                }
                mxArray* q_field = mxGetField(new_state, 0, "q");
                if (q_field && mxGetClassID(q_field) == mxSINGLE_CLASS) {
                    const float* qf = (const float*)mxGetData(q_field);
                    for (int i = 0; i < 4; ++i) out_q[i] = qf[i];
                }
                mxArray* ba_field = mxGetField(new_state, 0, "ba");
                if (ba_field && mxGetClassID(ba_field) == mxSINGLE_CLASS) {
                    const float* baf = (const float*)mxGetData(ba_field);
                    for (int i = 0; i < 3; ++i) out_ba[i] = baf[i];
                }
                mxArray* bg_field = mxGetField(new_state, 0, "bg");
                if (bg_field && mxGetClassID(bg_field) == mxSINGLE_CLASS) {
                    const float* bgf = (const float*)mxGetData(bg_field);
                    for (int i = 0; i < 3; ++i) out_bg[i] = bgf[i];
                }
                mxArray* P_field = mxGetField(new_state, 0, "P");
                if (P_field && mxGetClassID(P_field) == mxSINGLE_CLASS) {
                    const float* Pf = (const float*)mxGetData(P_field);
                    for (int i = 0; i < 15; ++i) {
                        for (int j = 0; j < 15; ++j) {
                            // MATLAB column-major -> internal row-major, average symmetric entries
                            double val1 = Pf[j * 15 + i];
                            double val2 = Pf[i * 15 + j];
                            out_P[i + j*15] = 0.5 * (val1 + val2);
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
                
                if (!mex_conv::matToVector(p_arr, state_p)) state_p = Vector<3, float>::Zero();
                if (!mex_conv::matToVector(v_arr, state_v)) state_v = Vector<3, float>::Zero();
                if (!mex_conv::matToVector(q_arr, state_q)) state_q = Vector<4, float>::Zero();
                if (!mex_conv::matToVector(ba_arr, state_ba)) state_ba = Vector<3, float>::Zero();
                if (!mex_conv::matToVector(bg_arr, state_bg)) state_bg = Vector<3, float>::Zero();
                if (!mex_conv::matToMatrix(P_arr, state_P)) state_P = Matrix<15, 15, float>::Zero();
                if (mxGetField(new_state, 0, "P")) {
                    if (!mex_conv::matToMatrix(mxGetField(new_state, 0, "P"), new_state_P)) {
                        new_state_P = state_P;
                    }
                } else {
                    new_state_P = state_P;
                }
                
                // Divergence check (C++ direct implementation)
                // Get innov size
                int innov_len = mxGetM(innov) * mxGetN(innov);
                if (innov_len < 1) innov_len = 1;
                if (innov_len > 3) innov_len = 3;
                
                // Convert innov to FixedMatrix
                cmath_fx::FixedMatrix innov_cm(innov_len, 1);
                std::vector<float> innov_tmp(static_cast<size_t>(innov_len));
                mex_conv::mxArrayToFloatArray(innov, innov_tmp.data(), static_cast<size_t>(innov_len));
                for (int i = 0; i < innov_len; ++i) {
                    innov_cm(i, 0) = innov_tmp[i];
                }
                
                // Convert dx to FixedMatrix
                cmath_fx::FixedMatrix dx_cm(15, 1);
                for (int i = 0; i < 15; ++i) {
                    dx_cm(i, 0) = dx(i, 0);
                }
                
                // Call divergence check directly
                bool was_attenuated = false;
                bool should_skip_result = g_filter_lib.divergence_guard.check_and_attenuate(
                    sensor_type, innov_cm, dx_cm, was_attenuated);
                
                // Convert back to Vector
                Vector<15, float> dx_out;
                for (int i = 0; i < 15; ++i) {
                    dx_out(i, 0) = dx_cm(i, 0);
                }
                should_skip = should_skip_result;
                
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
                    out_p[i] = new_p(i, 0);
                    out_v[i] = new_v(i, 0);
                    out_ba[i] = new_ba(i, 0);
                    out_bg[i] = new_bg(i, 0);
                }
                for (int i = 0; i < 4; ++i) {
                    out_q[i] = new_q(i, 0);
                }
                for (int i = 0; i < 15; ++i) {
                    for (int j = 0; j < 15; ++j) {
                        out_P[i + j*15] = out_P_mat(i, j);
                    }
                }
            }
        } else {
            // new_stateはsingle型（float）で返される
            mxArray* p_field = mxGetField(new_state, 0, "p");
            if (p_field && mxGetClassID(p_field) == mxSINGLE_CLASS) {
                const float* pf = (const float*)mxGetData(p_field);
                for (int i = 0; i < 3; ++i) out_p[i] = pf[i];
            }
            mxArray* v_field = mxGetField(new_state, 0, "v");
            if (v_field && mxGetClassID(v_field) == mxSINGLE_CLASS) {
                const float* vf = (const float*)mxGetData(v_field);
                for (int i = 0; i < 3; ++i) out_v[i] = vf[i];
            }
            mxArray* q_field = mxGetField(new_state, 0, "q");
            if (q_field && mxGetClassID(q_field) == mxSINGLE_CLASS) {
                const float* qf = (const float*)mxGetData(q_field);
                for (int i = 0; i < 4; ++i) out_q[i] = qf[i];
            }
            mxArray* ba_field = mxGetField(new_state, 0, "ba");
            if (ba_field && mxGetClassID(ba_field) == mxSINGLE_CLASS) {
                const float* baf = (const float*)mxGetData(ba_field);
                for (int i = 0; i < 3; ++i) out_ba[i] = baf[i];
            }
            mxArray* bg_field = mxGetField(new_state, 0, "bg");
            if (bg_field && mxGetClassID(bg_field) == mxSINGLE_CLASS) {
                const float* bgf = (const float*)mxGetData(bg_field);
                for (int i = 0; i < 3; ++i) out_bg[i] = bgf[i];
            }
            mxArray* P_field = mxGetField(new_state, 0, "P");
            if (P_field && mxGetClassID(P_field) == mxSINGLE_CLASS) {
                const float* Pf = (const float*)mxGetData(P_field);
                for (int i = 0; i < 15; ++i) {
                    for (int j = 0; j < 15; ++j) {
                        // MATLAB column-major to row-major
                        double val1 = Pf[j * 15 + i];
                        double val2 = Pf[i * 15 + j];
                        out_P[i + j*15] = 0.5 * (val1 + val2);
                    }
                }
            }
        }
        
        // Phase 1統合完了: mexCallMATLABの代わりにdo_sensor_update_meukfを直接呼び出し
        // メモリ管理: new_state, dbg_out, dbg_outputは呼び出し側で管理
    }

    mxDestroyArray(sensor_data);
    mxDestroyArray(mex_params);
    mxDestroyArray(state_s);
}

} // namespace mex_run_eskf_impl

#endif // MEX_MEX_RUN_ESKF_SENSOR_UPDATES_HPP_IMPL


#pragma once

#ifndef MEX_MEX_RUN_ESKF_SENSOR_UPDATES_HPP
#define MEX_MEX_RUN_ESKF_SENSOR_UPDATES_HPP

/**
 * mex_run_eskf.cpp用のセンサー更新関数群
 * 
 * 長い関数（call_sensor_update, call_gps_update）の実装を含みます。
 */

#include "mex_eskf_common.hpp"
#include <cstring>

namespace mex_run_eskf_impl {

// 前方宣言
inline void handle_sensor_update_internal(
    const char* sensor_type,
    const double* meas, int meas_len,
    const double* p, const double* v, const double* q,
    const double* ba, const double* bg, const double* P,
    const double* g, double dt, double sample,
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
        // Preprocess accel (C++ direct implementation)
        cmath_fx::Vector<3, float> a_meas_f;
        cmath_fx::Vector<3, float> prev_a_f;
        for (int i = 0; i < 3; ++i) {
            a_meas_f(i, 0) = static_cast<float>(meas[i]);
            prev_a_f(i, 0) = static_cast<float>(s->prev_accel[i]);
        }
        
        PreprocessResult result = preprocess_accel(a_meas_f, prev_a_f, s->buffer_tolerance);
        
        double a_corrected[3];
        for (int i = 0; i < 3; ++i) {
            a_corrected[i] = static_cast<double>(result.output(i, 0));
        }
        bool is_outlier = result.is_outlier;
        bool no_change = result.no_change;
        
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
        
        if (!should_skip) {
            // Call handle_sensor_update_internal directly (integrated from mex_eskf_do_update)
            handle_sensor_update_internal(
                "accel", a_corrected, 3,
                out_p, out_v, out_q, out_ba, out_bg, out_P,
                g, dt, sample,
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
            prev_m_f(i, 0) = static_cast<float>(s->prev_mag[i]);
        }
        
        PreprocessResult result = preprocess_mag(m_meas_f, prev_m_f, s->buffer_tolerance);
        
        double m_filtered[3];
        for (int i = 0; i < 3; ++i) {
            m_filtered[i] = static_cast<double>(result.output(i, 0));
        }
        bool is_outlier = result.is_outlier;
        bool no_change = result.no_change;
        
        if (!no_change && !is_nan_any(m_filtered, 3) && !is_outlier) {
            should_skip = false;
            memcpy(s->prev_mag, meas, 3 * sizeof(double));
        }
        
        if (!should_skip) {
            // Call handle_sensor_update_internal directly (integrated from mex_eskf_do_update)
            handle_sensor_update_internal(
                "mag", m_filtered, 3,
                out_p, out_v, out_q, out_ba, out_bg, out_P,
                g, dt, sample,
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
                    g, dt, sample,
                    out_p, out_v, out_q, out_ba, out_bg, out_P,
                    should_skip
                );
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

/**
 * GPS更新処理（mexCallMATLAB実装）
 */
inline void call_gps_update(ESKFState* s, double lat, double lon, double alt, double sample) {
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
    
    // Preprocess GPS (C++ direct implementation)
    cmath_fx::Vector<3, float> origin_f;
    for (int i = 0; i < 3; ++i) {
        origin_f(i, 0) = static_cast<float>(s->gps_origin[i]);
    }
    
    PreprocessResult result = preprocess_gps(lat, lon, alt, origin_f, s->buffer_tolerance);
    
    bool should_skip = true;
    double z_gps[3];
    for (int i = 0; i < 3; ++i) {
        z_gps[i] = static_cast<double>(result.output(i, 0));
    }
    bool is_outlier = result.is_outlier;
    bool no_change = result.no_change;
    
    if (!no_change && !is_outlier) {
        should_skip = false;
    }
    
    if (!should_skip) {
        // Call handle_sensor_update_internal directly (integrated from mex_eskf_do_update)
        handle_sensor_update_internal(
            "gps", z_gps, 3,
            out_p, out_v, out_q, out_ba, out_bg, out_P,
            g, dt, sample,
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
        memcpy(s->p, out_p, 3 * sizeof(double));
        memcpy(s->v, out_v, 3 * sizeof(double));
        memcpy(s->q, out_q, 4 * sizeof(double));
        memcpy(s->ba, out_ba, 3 * sizeof(double));
        memcpy(s->bg, out_bg, 3 * sizeof(double));
        memcpy(s->P, out_P, 15*15*sizeof(double));
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
            R_noise[0] = static_cast<double>(R(0, 0));
            R_noise[1] = static_cast<double>(R(1, 1));
            R_noise[2] = static_cast<double>(R(2, 2));
        } else if (n_rows >= 3 && n_cols == 1) {
            // ベクトル形式
            R_noise[0] = static_cast<double>(R(0, 0));
            R_noise[1] = static_cast<double>(R(1, 0));
            R_noise[2] = static_cast<double>(R(2, 0));
        } else if (n_rows == 1 && n_cols == 1) {
            // スカラー（baroなど）
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
    mxSetField(sensor_data, 0, "alt_baro", mxCreateDoubleScalar(0));
    mxSetField(sensor_data, 0, "dt", mxCreateDoubleScalar(dt));
    mxSetField(sensor_data, 0, "update_accel", mxCreateLogicalScalar(false));
    mxSetField(sensor_data, 0, "update_gyro", mxCreateLogicalScalar(false));
    mxSetField(sensor_data, 0, "update_mag", mxCreateLogicalScalar(false));
    mxSetField(sensor_data, 0, "update_gps", mxCreateLogicalScalar(false));
    mxSetField(sensor_data, 0, "update_baro", mxCreateLogicalScalar(false));
    mxSetField(sensor_data, 0, "update_zupt", mxCreateLogicalScalar(false));

    // mex_params構造体
    const char* mp_fields[] = {"g", "mag_ref", "noise_accel", "noise_gyro", "noise_ba", "noise_bg",
        "noise_mag", "noise_gps", "noise_baro", "noise_zupt", "alpha", "beta", "kappa", "trace_sample"};
    mxArray* mex_params = mxCreateStructMatrix(1, 1, 14, mp_fields);

    mxArray* g_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(g_arr), g, 3);
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

    // センサータイプ別設定
    if (strcmp(sensor_type, "accel") == 0) {
        copy_vec(mxGetPr(mxGetField(sensor_data, 0, "accel")), meas, 3);
        for (int i = 0; i < 3; ++i) R_noise[i] *= 1.5;
        copy_vec(mxGetPr(mxGetField(mex_params, 0, "noise_accel")), R_noise, 3);
        mxDestroyArray(mxGetField(sensor_data, 0, "update_accel"));
        mxSetField(sensor_data, 0, "update_accel", mxCreateLogicalScalar(true));
    } else if (strcmp(sensor_type, "mag") == 0) {
        copy_vec(mxGetPr(mxGetField(sensor_data, 0, "mag")), meas, 3);
        for (int i = 0; i < 3; ++i) R_noise[i] *= 1.5;
        copy_vec(mxGetPr(mxGetField(mex_params, 0, "noise_mag")), R_noise, 3);
        mxDestroyArray(mxGetField(sensor_data, 0, "update_mag"));
        mxSetField(sensor_data, 0, "update_mag", mxCreateLogicalScalar(true));
    } else if (strcmp(sensor_type, "gps") == 0) {
        copy_vec(mxGetPr(mxGetField(sensor_data, 0, "gps_pos")), meas, 3);
        copy_vec(mxGetPr(mxGetField(mex_params, 0, "noise_gps")), R_noise, 3);
        mxDestroyArray(mxGetField(sensor_data, 0, "update_gps"));
        mxSetField(sensor_data, 0, "update_gps", mxCreateLogicalScalar(true));
    } else if (strcmp(sensor_type, "baro") == 0) {
        mxDestroyArray(mxGetField(sensor_data, 0, "alt_baro"));
        mxSetField(sensor_data, 0, "alt_baro", mxCreateDoubleScalar(meas[0]));
        mxDestroyArray(mxGetField(mex_params, 0, "noise_baro"));
        mxSetField(mex_params, 0, "noise_baro", mxCreateDoubleScalar(R_noise[0]));
        mxDestroyArray(mxGetField(sensor_data, 0, "update_baro"));
        mxSetField(sensor_data, 0, "update_baro", mxCreateLogicalScalar(true));
    } else if (strcmp(sensor_type, "zupt") == 0) {
        double zupt_noise[3] = {0.0001, 0.0001, 0.0001};
        copy_vec(mxGetPr(mxGetField(mex_params, 0, "noise_zupt")), zupt_noise, 3);
        mxDestroyArray(mxGetField(sensor_data, 0, "update_zupt"));
        mxSetField(sensor_data, 0, "update_zupt", mxCreateLogicalScalar(true));
    } else {
        mexErrMsgTxt("Unknown sensor type");
    }

    // state構造体
    const char* st_fields[] = {"p", "v", "q", "ba", "bg", "P"};
    mxArray* state_s = mxCreateStructMatrix(1, 1, 6, st_fields);
    mxArray* p_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(p_arr), p, 3);
    mxArray* v_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(v_arr), v, 3);
    mxArray* q_arr = mxCreateDoubleMatrix(4, 1, mxREAL); copy_vec(mxGetPr(q_arr), q, 4);
    mxArray* ba_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(ba_arr), ba, 3);
    mxArray* bg_arr = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(bg_arr), bg, 3);
    mxArray* P_arr = mxCreateDoubleMatrix(15, 15, mxREAL); memcpy(mxGetPr(P_arr), P, 15*15*8);
    mxSetField(state_s, 0, "p", p_arr);
    mxSetField(state_s, 0, "v", v_arr);
    mxSetField(state_s, 0, "q", q_arr);
    mxSetField(state_s, 0, "ba", ba_arr);
    mxSetField(state_s, 0, "bg", bg_arr);
    mxSetField(state_s, 0, "P", P_arr);

    // mex_meukf_step_v2 呼び出し（後でMEUKFCore::stepに置き換え予定）
    mxArray* prhs_m[3] = {state_s, sensor_data, mex_params};
    mxArray* plhs_m[3];
    if (mexCallMATLAB(3, plhs_m, 3, prhs_m, "mex_meukf_step_v2") == 0) {
        mxArray* new_state = plhs_m[0];
        mxArray* dbg_out = plhs_m[1];

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

} // namespace mex_run_eskf_impl

#endif // MEX_MEX_RUN_ESKF_SENSOR_UPDATES_HPP


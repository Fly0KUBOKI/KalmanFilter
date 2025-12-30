#pragma once

#ifndef MEX_MEX_RUN_ESKF_SENSOR_UPDATES_HPP
#define MEX_MEX_RUN_ESKF_SENSOR_UPDATES_HPP

// Sensor update functions for mex_run_eskf.cpp

#include "mex_eskf_common.hpp"
#include <cstring>
#include <cmath>
#include "../MEUKF/meukf_types.hpp"
#include "../MEUKF/meukf_core.hpp"

namespace mex_run_eskf_impl {

// Forward declaration
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
    // Validate input data
    for (int i = 0; i < 3; ++i) {
        if (std::isnan(p[i]) || std::isinf(p[i]) || 
            std::isnan(v[i]) || std::isinf(v[i]) ||
            std::isnan(ba[i]) || std::isinf(ba[i]) ||
            std::isnan(bg[i]) || std::isinf(bg[i])) {
            should_skip = true;
            return;
        }
    }
    for (int i = 0; i < 4; ++i) {
        if (std::isnan(q[i]) || std::isinf(q[i])) {
            should_skip = true;
            return;
        }
    }
    for (int i = 0; i < 15*15; ++i) {
        if (std::isnan(P[i]) || std::isinf(P[i])) {
            should_skip = true;
            return;
        }
    }
    
    // Initialize output buffers
    copy_vec(out_p, p, 3);
    copy_vec(out_v, v, 3);
    copy_vec(out_q, q, 4);
    copy_vec(out_ba, ba, 3);
    copy_vec(out_bg, bg, 3);
    memcpy(out_P, P, 15*15*sizeof(double));
    should_skip = false;
    
    // Get R matrix (C++ direct implementation)
    double R_noise[3] = {0.01, 0.01, 0.01};
    {
        cmath_fx::FixedMatrix R = g_filter_lib.noise_estimator.get_R_matrix(sensor_type);
        int n_rows = R.rows;
        int n_cols = R.cols;
        if (n_rows == 3 && n_cols == 3) {
            // Get diagonal elements from 3x3 matrix
            R_noise[0] = static_cast<double>(R(0, 0));
            R_noise[1] = static_cast<double>(R(1, 1));
            R_noise[2] = static_cast<double>(R(2, 2));
        } else if (n_rows >= 3 && n_cols == 1) {
            // Vector format
            R_noise[0] = static_cast<double>(R(0, 0));
            R_noise[1] = static_cast<double>(R(1, 0));
            R_noise[2] = static_cast<double>(R(2, 0));
        } else if (n_rows == 1 && n_cols == 1) {
            // Scalar (e.g., baro)
            R_noise[0] = R_noise[1] = R_noise[2] = static_cast<double>(R(0, 0));
        }
    }

    // Build sensor_data structure
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

    // Build mex_params structure
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

    // Configure by sensor type
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

    // Build MEUKFInput structure (convert directly to C++ struct without MATLAB struct)
    meukf::MEUKFInput input;
    
    // 1. Convert state (double -> float, MATLAB column-major -> C++ row-major)
    for (int i = 0; i < 3; ++i) {
        input.prev_state.p[i] = static_cast<float>(p[i]);
        input.prev_state.v[i] = static_cast<float>(v[i]);
        input.prev_state.ba[i] = static_cast<float>(ba[i]);
        input.prev_state.bg[i] = static_cast<float>(bg[i]);
    }
    for (int i = 0; i < 4; ++i) {
        input.prev_state.q[i] = static_cast<float>(q[i]);
    }
    
    // P: MATLAB column-major -> C++ row-major
    for (int r = 0; r < 15; ++r) {
        for (int c = 0; c < 15; ++c) {
            input.prev_state.P[r*15 + c] = static_cast<float>(P[c*15 + r]);
        }
    }
    
    // 2. Convert SensorData
    auto get_field_scalar = [&](const mxArray* s, const char* f) -> float {
        mxArray* field = mxGetField(s, 0, f);
        return field ? mex_conv::mxGetScalarAsFloat(field) : 0.0f;
    };
    
    mex_conv::mxArrayToFloatArray(mxGetField(sensor_data, 0, "accel"), input.sensor.accel, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(sensor_data, 0, "gyro"), input.sensor.gyro, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(sensor_data, 0, "mag"), input.sensor.mag, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(sensor_data, 0, "gps_pos"), input.sensor.gps_pos, 3);
    input.sensor.alt_baro = mex_conv::mxGetScalarAsFloat(mxGetField(sensor_data, 0, "alt_baro"));
    
    // Previous sensor values (for change detection) - currently initialized to 0
    for (int i = 0; i < 3; ++i) {
        input.sensor.prev_mag[i] = 0.0f;
        input.sensor.prev_gps_pos[i] = 0.0f;
    }
    input.sensor.prev_baro_alt = 0.0f;
    
    input.sensor.update_accel = (uint8_t)get_field_scalar(sensor_data, "update_accel");
    input.sensor.update_gyro = (uint8_t)get_field_scalar(sensor_data, "update_gyro");
    input.sensor.update_mag = (uint8_t)get_field_scalar(sensor_data, "update_mag");
    input.sensor.update_gps = (uint8_t)get_field_scalar(sensor_data, "update_gps");
    input.sensor.update_baro = (uint8_t)get_field_scalar(sensor_data, "update_baro");
    input.sensor.update_zupt = (uint8_t)get_field_scalar(sensor_data, "update_zupt");
    input.sensor.dt = mex_conv::mxGetScalarAsFloat(mxGetField(sensor_data, 0, "dt"));
    
    // 3. Convert Params
    mex_conv::mxArrayToFloatArray(mxGetField(mex_params, 0, "g"), input.params.g, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(mex_params, 0, "mag_ref"), input.params.mag_ref, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(mex_params, 0, "noise_accel"), input.params.noise_accel, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(mex_params, 0, "noise_gyro"), input.params.noise_gyro, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(mex_params, 0, "noise_ba"), input.params.noise_ba, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(mex_params, 0, "noise_bg"), input.params.noise_bg, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(mex_params, 0, "noise_mag"), input.params.noise_mag, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(mex_params, 0, "noise_gps"), input.params.noise_gps, 3);
    input.params.noise_baro = mex_conv::mxGetScalarAsFloat(mxGetField(mex_params, 0, "noise_baro"));
    mex_conv::mxArrayToFloatArray(mxGetField(mex_params, 0, "noise_zupt"), input.params.noise_zupt, 3);
    
    input.params.alpha = mex_conv::mxGetScalarAsFloat(mxGetField(mex_params, 0, "alpha"));
    input.params.beta = mex_conv::mxGetScalarAsFloat(mxGetField(mex_params, 0, "beta"));
    input.params.kappa = mex_conv::mxGetScalarAsFloat(mxGetField(mex_params, 0, "kappa"));
    
    // Validate noise_gps input
    for (int i = 0; i < 3; ++i) {
        float v = input.params.noise_gps[i];
        if (!std::isfinite(v) || v < 0.0f) {
            mexErrMsgTxt("noise_gps must be finite non-negative variances");
        }
    }
    
    // Call MEUKFCore::step directly
    meukf::MEUKFOutput output;
    meukf::MEUKFCore::step(input, output);
    
    // Validate MEUKF output
    for (int i = 0; i < 3; ++i) {
        if (std::isnan(output.new_state.p[i]) || std::isinf(output.new_state.p[i]) ||
            std::isnan(output.new_state.v[i]) || std::isinf(output.new_state.v[i]) ||
            std::isnan(output.new_state.ba[i]) || std::isinf(output.new_state.ba[i]) ||
            std::isnan(output.new_state.bg[i]) || std::isinf(output.new_state.bg[i])) {
            should_skip = true;
            return;
        }
    }
    for (int i = 0; i < 4; ++i) {
        if (std::isnan(output.new_state.q[i]) || std::isinf(output.new_state.q[i])) {
            should_skip = true;
            return;
        }
    }
    for (int i = 0; i < 15*15; ++i) {
        if (std::isnan(output.new_state.P[i]) || std::isinf(output.new_state.P[i])) {
            should_skip = true;
            return;
        }
    }
    
    // Process output
    {
        // Update noise estimate (C++ direct implementation)
        if (output.last_y_len > 0 && output.last_y_len <= 3) {
            // Get innov (last_y)
            cmath_fx::FixedMatrix innov_cm(output.last_y_len, 1);
            for (int i = 0; i < output.last_y_len; ++i) {
                innov_cm(i, 0) = output.last_y[i];
            }
            
            // Get H (last_H: row-major 3x15)
            int H_rows = (output.last_y_len <= 3) ? output.last_y_len : 3;
            int H_cols = 15;
            cmath_fx::FixedMatrix H_cm(H_rows, H_cols);
            for (int i = 0; i < H_rows; ++i) {
                for (int j = 0; j < H_cols; ++j) {
                    H_cm(i, j) = output.last_H[i*15 + j];
                }
            }
            
            // Get P_pred (row-major 15x15)
            cmath_fx::FixedMatrix P_pred(15, 15);
            for (int i = 0; i < 15; ++i) {
                for (int j = 0; j < 15; ++j) {
                    P_pred(i, j) = output.pred_P[i*15 + j];
                }
            }
            
            // Call noise estimate directly
            g_filter_lib.noise_estimator.estimate(sensor_type, innov_cm, H_cm, P_pred);
        }
        
        // Calculate dx (dx = K * y, where K is last_K and y is last_y)
        Vector<15, float> dx = Vector<15, float>::Zero();
        if (output.last_y_len > 0 && output.last_y_len <= 3) {
            // dx = K * y (K: 15x3 row-major, y: 3x1)
            for (int i = 0; i < 15; ++i) {
                float sum = 0.0f;
                for (int j = 0; j < output.last_y_len; ++j) {
                    sum += output.last_K[i*3 + j] * output.last_y[j];
                }
                dx(i, 0) = sum;
            }
        }
        
        // Postprocess: Use MEUKF output directly (MEUKF already updated the state)
        {
            // MEUKF output is already updated, so use it directly
            // Only apply divergence check and attenuation if needed
            
            // Get previous state for divergence check
            Vector<3, float> state_p, state_v, state_ba, state_bg;
            Vector<4, float> state_q;
            Matrix<15, 15, float> state_P;
            
            for (int i = 0; i < 3; ++i) {
                state_p(i, 0) = input.prev_state.p[i];
                state_v(i, 0) = input.prev_state.v[i];
                state_ba(i, 0) = input.prev_state.ba[i];
                state_bg(i, 0) = input.prev_state.bg[i];
            }
            for (int i = 0; i < 4; ++i) {
                state_q(i, 0) = input.prev_state.q[i];
            }
            for (int i = 0; i < 15; ++i) {
                for (int j = 0; j < 15; ++j) {
                    state_P(i, j) = input.prev_state.P[i*15 + j];
                }
            }
            
            // Divergence check (C++ direct implementation)
            cmath_fx::FixedMatrix innov_cm(output.last_y_len > 0 ? output.last_y_len : 1, 1);
            for (int i = 0; i < output.last_y_len && i < 3; ++i) {
                innov_cm(i, 0) = output.last_y[i];
            }
            
            cmath_fx::FixedMatrix dx_cm(15, 1);
            for (int i = 0; i < 15; ++i) {
                dx_cm(i, 0) = dx(i, 0);
            }
            
            bool was_attenuated = false;
            bool should_skip_result = g_filter_lib.divergence_guard.check_and_attenuate(
                sensor_type, innov_cm, dx_cm, was_attenuated);
            
            should_skip = should_skip_result;
            
            // Use MEUKF output directly (already updated)
            Vector<3, float> new_p, new_v, new_ba, new_bg;
            Vector<4, float> new_q;
            Matrix<15, 15, float> out_P_mat;
            
            if (should_skip) {
                // Return original state if divergence detected
                new_p = state_p;
                new_v = state_v;
                new_q = state_q;
                new_ba = state_ba;
                new_bg = state_bg;
                out_P_mat = state_P;
            } else {
                // Use MEUKF output directly
                for (int i = 0; i < 3; ++i) {
                    new_p(i, 0) = output.new_state.p[i];
                    new_v(i, 0) = output.new_state.v[i];
                    new_ba(i, 0) = output.new_state.ba[i];
                    new_bg(i, 0) = output.new_state.bg[i];
                }
                for (int i = 0; i < 4; ++i) {
                    new_q(i, 0) = output.new_state.q[i];
                }
                // Copy P to matrix for symmetrization
                for (int i = 0; i < 15; ++i) {
                    for (int j = 0; j < 15; ++j) {
                        out_P_mat(i, j) = output.new_state.P[i*15 + j];
                    }
                }
                
                // Apply symmetrization
                eskf::symmetrize_covariance(out_P_mat);
            }
            
            // Convert back to double arrays and validate
            for (int i = 0; i < 3; ++i) {
                out_p[i] = static_cast<double>(new_p(i, 0));
                out_v[i] = static_cast<double>(new_v(i, 0));
                out_ba[i] = static_cast<double>(new_ba(i, 0));
                out_bg[i] = static_cast<double>(new_bg(i, 0));
                // Final validation
                if (std::isnan(out_p[i]) || std::isinf(out_p[i]) ||
                    std::isnan(out_v[i]) || std::isinf(out_v[i]) ||
                    std::isnan(out_ba[i]) || std::isinf(out_ba[i]) ||
                    std::isnan(out_bg[i]) || std::isinf(out_bg[i])) {
                    should_skip = true;
                    return;
                }
            }
            for (int i = 0; i < 4; ++i) {
                out_q[i] = static_cast<double>(new_q(i, 0));
                if (std::isnan(out_q[i]) || std::isinf(out_q[i])) {
                    should_skip = true;
                    return;
                }
            }
            // Convert P: C++ row-major -> MATLAB column-major
            // Same as mex_meukf_step.cpp: pr[c*15 + r] = c_state.P[r*15 + c]
            // But use symmetrized out_P_mat instead of direct output.new_state.P
            for (int c = 0; c < 15; ++c) {
                for (int r = 0; r < 15; ++r) {
                    double val = static_cast<double>(out_P_mat(r, c));
                    if (std::isnan(val) || std::isinf(val)) {
                        should_skip = true;
                        return;
                    }
                    out_P[c*15 + r] = val;
                }
            }
        }
    }

    // Cleanup MATLAB structures
    mxDestroyArray(sensor_data);
    mxDestroyArray(mex_params);
}

} // namespace mex_run_eskf_impl

#endif // MEX_MEX_RUN_ESKF_SENSOR_UPDATES_HPP


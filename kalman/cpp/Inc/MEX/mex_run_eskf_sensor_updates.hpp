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
            
            // Preprocess baro (C++ direct implementation)
            double alt_baro = preprocess_baro(pressure);
            
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

} // namespace mex_run_eskf_impl

#endif // MEX_MEX_RUN_ESKF_SENSOR_UPDATES_HPP


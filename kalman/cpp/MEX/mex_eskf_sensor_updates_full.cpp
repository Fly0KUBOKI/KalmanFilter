// mex_eskf_sensor_updates_full.cpp
// sensor_updates() 全体をMEX化（前処理 + do_cpp_update統合）

#include "mex.h"
#include <cmath>
#include <cstring>

//=============================================================================
// ヘルパー関数
//=============================================================================
static double norm3(const double* v) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static bool is_nan_any(const double* v, int n) {
    for (int i = 0; i < n; ++i) {
        if (mxIsNaN(v[i])) return true;
    }
    return false;
}

static void copy_vec(double* dst, const double* src, int n) {
    memcpy(dst, src, n * sizeof(double));
}

//=============================================================================
// Accel更新
//=============================================================================
static void handle_accel(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // 入力: 'accel', a_meas, state_struct, [sample]
    // state_struct: p, v, q, ba, bg, P, g, dt, w_body, prev_accel
    
    const mxArray* a_meas_arr = prhs[1];
    const mxArray* state = prhs[2];
    double sample = (nrhs > 3 && !mxIsEmpty(prhs[3])) ? mxGetScalar(prhs[3]) : mxGetNaN();
    
    double* a_meas = mxGetPr(a_meas_arr);
    double* p = mxGetPr(mxGetField(state, 0, "p"));
    double* v = mxGetPr(mxGetField(state, 0, "v"));
    double* q = mxGetPr(mxGetField(state, 0, "q"));
    double* ba = mxGetPr(mxGetField(state, 0, "ba"));
    double* bg = mxGetPr(mxGetField(state, 0, "bg"));
    double* P = mxGetPr(mxGetField(state, 0, "P"));
    double* g = mxGetPr(mxGetField(state, 0, "g"));
    double dt = mxGetScalar(mxGetField(state, 0, "dt"));
    double* w_body = mxGetPr(mxGetField(state, 0, "w_body"));
    double* prev_accel = mxGetPr(mxGetField(state, 0, "prev_accel"));
    
    // 出力バッファ
    double out_p[3], out_v[3], out_q[4], out_ba[3], out_bg[3], out_P[15*15];
    double new_prev_accel[3];
    copy_vec(out_p, p, 3);
    copy_vec(out_v, v, 3);
    copy_vec(out_q, q, 4);
    copy_vec(out_ba, ba, 3);
    copy_vec(out_bg, bg, 3);
    memcpy(out_P, P, 15*15*sizeof(double));
    copy_vec(new_prev_accel, prev_accel, 3);
    
    // 前処理
    mxArray* prhs_pre[3];
    mxArray* plhs_pre[3];
    prhs_pre[0] = mxCreateString("preprocess_accel");
    mxArray* am = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(am), a_meas, 3);
    prhs_pre[1] = am;
    mxArray* pa = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(pa), prev_accel, 3);
    prhs_pre[2] = pa;
    
    bool should_skip = true;
    double a_corrected[3];
    
    if (mexCallMATLAB(3, plhs_pre, 3, prhs_pre, "mex_sensor_preprocessor") == 0) {
        copy_vec(a_corrected, mxGetPr(plhs_pre[0]), 3);
        bool is_outlier = mxIsLogicalScalarTrue(plhs_pre[1]);
        bool no_change = mxIsLogicalScalarTrue(plhs_pre[2]);
        
        if (!no_change && !is_nan_any(a_corrected, 3) && !is_outlier && (norm3(w_body) <= 1.5)) {
            should_skip = false;
            copy_vec(new_prev_accel, a_meas, 3);
        }
        for (int i = 0; i < 3; ++i) mxDestroyArray(plhs_pre[i]);
    }
    for (int i = 0; i < 3; ++i) mxDestroyArray(prhs_pre[i]);
    
    if (!should_skip) {
        // mex_eskf_do_update呼び出し
        mxArray* prhs_u[11];
        mxArray* plhs_u[7];
        prhs_u[0] = mxCreateString("accel");
        mxArray* ac = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(ac), a_corrected, 3);
        prhs_u[1] = ac;
        mxArray* pp = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(pp), p, 3);
        prhs_u[2] = pp;
        mxArray* vv = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(vv), v, 3);
        prhs_u[3] = vv;
        mxArray* qq = mxCreateDoubleMatrix(4, 1, mxREAL); copy_vec(mxGetPr(qq), q, 4);
        prhs_u[4] = qq;
        mxArray* bba = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(bba), ba, 3);
        prhs_u[5] = bba;
        mxArray* bbg = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(bbg), bg, 3);
        prhs_u[6] = bbg;
        mxArray* PP = mxCreateDoubleMatrix(15, 15, mxREAL); memcpy(mxGetPr(PP), P, 15*15*8);
        prhs_u[7] = PP;
        mxArray* gg = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(gg), g, 3);
        prhs_u[8] = gg;
        prhs_u[9] = mxCreateDoubleScalar(dt);
        prhs_u[10] = mxCreateDoubleScalar(sample);
        
        if (mexCallMATLAB(7, plhs_u, 11, prhs_u, "mex_eskf_do_update") == 0) {
            if (!mxIsLogicalScalarTrue(plhs_u[6])) {
                copy_vec(out_p, mxGetPr(plhs_u[0]), 3);
                copy_vec(out_v, mxGetPr(plhs_u[1]), 3);
                copy_vec(out_q, mxGetPr(plhs_u[2]), 4);
                copy_vec(out_ba, mxGetPr(plhs_u[3]), 3);
                copy_vec(out_bg, mxGetPr(plhs_u[4]), 3);
                memcpy(out_P, mxGetPr(plhs_u[5]), 15*15*8);
            }
            for (int i = 0; i < 7; ++i) mxDestroyArray(plhs_u[i]);
        }
        for (int i = 0; i < 11; ++i) mxDestroyArray(prhs_u[i]);
    }
    
    // 出力
    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[0]), out_p, 3);
    plhs[1] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[1]), out_v, 3);
    plhs[2] = mxCreateDoubleMatrix(4, 1, mxREAL); copy_vec(mxGetPr(plhs[2]), out_q, 4);
    plhs[3] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[3]), out_ba, 3);
    plhs[4] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[4]), out_bg, 3);
    plhs[5] = mxCreateDoubleMatrix(15, 15, mxREAL); memcpy(mxGetPr(plhs[5]), out_P, 15*15*8);
    plhs[6] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[6]), new_prev_accel, 3);
}

//=============================================================================
// Mag更新
//=============================================================================
static void handle_mag(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    const mxArray* m_meas_arr = prhs[1];
    const mxArray* state = prhs[2];
    double sample = (nrhs > 3 && !mxIsEmpty(prhs[3])) ? mxGetScalar(prhs[3]) : mxGetNaN();
    
    double* m_meas = mxGetPr(m_meas_arr);
    double* p = mxGetPr(mxGetField(state, 0, "p"));
    double* v = mxGetPr(mxGetField(state, 0, "v"));
    double* q = mxGetPr(mxGetField(state, 0, "q"));
    double* ba = mxGetPr(mxGetField(state, 0, "ba"));
    double* bg = mxGetPr(mxGetField(state, 0, "bg"));
    double* P = mxGetPr(mxGetField(state, 0, "P"));
    double* g = mxGetPr(mxGetField(state, 0, "g"));
    double dt = mxGetScalar(mxGetField(state, 0, "dt"));
    double* prev_mag = mxGetPr(mxGetField(state, 0, "prev_mag"));
    
    double out_p[3], out_v[3], out_q[4], out_ba[3], out_bg[3], out_P[15*15];
    double new_prev_mag[3];
    copy_vec(out_p, p, 3);
    copy_vec(out_v, v, 3);
    copy_vec(out_q, q, 4);
    copy_vec(out_ba, ba, 3);
    copy_vec(out_bg, bg, 3);
    memcpy(out_P, P, 15*15*sizeof(double));
    copy_vec(new_prev_mag, prev_mag, 3);
    
    mxArray* prhs_pre[3];
    mxArray* plhs_pre[3];
    prhs_pre[0] = mxCreateString("preprocess_mag");
    mxArray* mm = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(mm), m_meas, 3);
    prhs_pre[1] = mm;
    mxArray* pm = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(pm), prev_mag, 3);
    prhs_pre[2] = pm;
    
    bool should_skip = true;
    double m_filtered[3];
    
    if (mexCallMATLAB(3, plhs_pre, 3, prhs_pre, "mex_sensor_preprocessor") == 0) {
        copy_vec(m_filtered, mxGetPr(plhs_pre[0]), 3);
        bool is_outlier = mxIsLogicalScalarTrue(plhs_pre[1]);
        bool no_change = mxIsLogicalScalarTrue(plhs_pre[2]);
        
        if (!no_change && !is_nan_any(m_filtered, 3) && !is_outlier) {
            should_skip = false;
            copy_vec(new_prev_mag, m_meas, 3);
        }
        for (int i = 0; i < 3; ++i) mxDestroyArray(plhs_pre[i]);
    }
    for (int i = 0; i < 3; ++i) mxDestroyArray(prhs_pre[i]);
    
    if (!should_skip) {
        mxArray* prhs_u[11];
        mxArray* plhs_u[7];
        prhs_u[0] = mxCreateString("mag");
        mxArray* mf = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(mf), m_filtered, 3);
        prhs_u[1] = mf;
        mxArray* pp = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(pp), p, 3);
        prhs_u[2] = pp;
        mxArray* vv = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(vv), v, 3);
        prhs_u[3] = vv;
        mxArray* qq = mxCreateDoubleMatrix(4, 1, mxREAL); copy_vec(mxGetPr(qq), q, 4);
        prhs_u[4] = qq;
        mxArray* bba = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(bba), ba, 3);
        prhs_u[5] = bba;
        mxArray* bbg = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(bbg), bg, 3);
        prhs_u[6] = bbg;
        mxArray* PP = mxCreateDoubleMatrix(15, 15, mxREAL); memcpy(mxGetPr(PP), P, 15*15*8);
        prhs_u[7] = PP;
        mxArray* gg = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(gg), g, 3);
        prhs_u[8] = gg;
        prhs_u[9] = mxCreateDoubleScalar(dt);
        prhs_u[10] = mxCreateDoubleScalar(sample);
        
        if (mexCallMATLAB(7, plhs_u, 11, prhs_u, "mex_eskf_do_update") == 0) {
            if (!mxIsLogicalScalarTrue(plhs_u[6])) {
                copy_vec(out_p, mxGetPr(plhs_u[0]), 3);
                copy_vec(out_v, mxGetPr(plhs_u[1]), 3);
                copy_vec(out_q, mxGetPr(plhs_u[2]), 4);
                copy_vec(out_ba, mxGetPr(plhs_u[3]), 3);
                copy_vec(out_bg, mxGetPr(plhs_u[4]), 3);
                memcpy(out_P, mxGetPr(plhs_u[5]), 15*15*8);
            }
            for (int i = 0; i < 7; ++i) mxDestroyArray(plhs_u[i]);
        }
        for (int i = 0; i < 11; ++i) mxDestroyArray(prhs_u[i]);
    }
    
    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[0]), out_p, 3);
    plhs[1] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[1]), out_v, 3);
    plhs[2] = mxCreateDoubleMatrix(4, 1, mxREAL); copy_vec(mxGetPr(plhs[2]), out_q, 4);
    plhs[3] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[3]), out_ba, 3);
    plhs[4] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[4]), out_bg, 3);
    plhs[5] = mxCreateDoubleMatrix(15, 15, mxREAL); memcpy(mxGetPr(plhs[5]), out_P, 15*15*8);
    plhs[6] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[6]), new_prev_mag, 3);
}

//=============================================================================
// GPS更新
//=============================================================================
static void handle_gps(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    double lat = mxGetScalar(prhs[1]);
    double lon = mxGetScalar(prhs[2]);
    double alt = mxGetScalar(prhs[3]);
    const mxArray* state = prhs[4];
    double sample = (nrhs > 5 && !mxIsEmpty(prhs[5])) ? mxGetScalar(prhs[5]) : mxGetNaN();
    
    double* p = mxGetPr(mxGetField(state, 0, "p"));
    double* v = mxGetPr(mxGetField(state, 0, "v"));
    double* q = mxGetPr(mxGetField(state, 0, "q"));
    double* ba = mxGetPr(mxGetField(state, 0, "ba"));
    double* bg = mxGetPr(mxGetField(state, 0, "bg"));
    double* P = mxGetPr(mxGetField(state, 0, "P"));
    double* g = mxGetPr(mxGetField(state, 0, "g"));
    double dt = mxGetScalar(mxGetField(state, 0, "dt"));
    double* gps_origin = mxGetPr(mxGetField(state, 0, "gps_origin"));
    
    double out_p[3], out_v[3], out_q[4], out_ba[3], out_bg[3], out_P[15*15];
    copy_vec(out_p, p, 3);
    copy_vec(out_v, v, 3);
    copy_vec(out_q, q, 4);
    copy_vec(out_ba, ba, 3);
    copy_vec(out_bg, bg, 3);
    memcpy(out_P, P, 15*15*sizeof(double));
    
    mxArray* prhs_pre[5];
    mxArray* plhs_pre[3];
    prhs_pre[0] = mxCreateString("preprocess_gps");
    prhs_pre[1] = mxCreateDoubleScalar(lat);
    prhs_pre[2] = mxCreateDoubleScalar(lon);
    prhs_pre[3] = mxCreateDoubleScalar(alt);
    mxArray* go = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(go), gps_origin, 3);
    prhs_pre[4] = go;
    
    bool should_skip = true;
    double z_gps[3];
    
    if (mexCallMATLAB(3, plhs_pre, 5, prhs_pre, "mex_sensor_preprocessor") == 0) {
        copy_vec(z_gps, mxGetPr(plhs_pre[0]), 3);
        bool is_outlier = mxIsLogicalScalarTrue(plhs_pre[1]);
        bool no_change = mxIsLogicalScalarTrue(plhs_pre[2]);
        
        if (!no_change && !is_outlier) {
            should_skip = false;
        }
        for (int i = 0; i < 3; ++i) mxDestroyArray(plhs_pre[i]);
    }
    for (int i = 0; i < 5; ++i) mxDestroyArray(prhs_pre[i]);
    
    if (!should_skip) {
        mxArray* prhs_u[11];
        mxArray* plhs_u[7];
        prhs_u[0] = mxCreateString("gps");
        mxArray* zg = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(zg), z_gps, 3);
        prhs_u[1] = zg;
        mxArray* pp = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(pp), p, 3);
        prhs_u[2] = pp;
        mxArray* vv = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(vv), v, 3);
        prhs_u[3] = vv;
        mxArray* qq = mxCreateDoubleMatrix(4, 1, mxREAL); copy_vec(mxGetPr(qq), q, 4);
        prhs_u[4] = qq;
        mxArray* bba = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(bba), ba, 3);
        prhs_u[5] = bba;
        mxArray* bbg = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(bbg), bg, 3);
        prhs_u[6] = bbg;
        mxArray* PP = mxCreateDoubleMatrix(15, 15, mxREAL); memcpy(mxGetPr(PP), P, 15*15*8);
        prhs_u[7] = PP;
        mxArray* gg = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(gg), g, 3);
        prhs_u[8] = gg;
        prhs_u[9] = mxCreateDoubleScalar(dt);
        prhs_u[10] = mxCreateDoubleScalar(sample);
        
        if (mexCallMATLAB(7, plhs_u, 11, prhs_u, "mex_eskf_do_update") == 0) {
            if (!mxIsLogicalScalarTrue(plhs_u[6])) {
                copy_vec(out_p, mxGetPr(plhs_u[0]), 3);
                copy_vec(out_v, mxGetPr(plhs_u[1]), 3);
                copy_vec(out_q, mxGetPr(plhs_u[2]), 4);
                copy_vec(out_ba, mxGetPr(plhs_u[3]), 3);
                copy_vec(out_bg, mxGetPr(plhs_u[4]), 3);
                memcpy(out_P, mxGetPr(plhs_u[5]), 15*15*8);
            }
            for (int i = 0; i < 7; ++i) mxDestroyArray(plhs_u[i]);
        }
        for (int i = 0; i < 11; ++i) mxDestroyArray(prhs_u[i]);
    }
    
    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[0]), out_p, 3);
    plhs[1] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[1]), out_v, 3);
    plhs[2] = mxCreateDoubleMatrix(4, 1, mxREAL); copy_vec(mxGetPr(plhs[2]), out_q, 4);
    plhs[3] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[3]), out_ba, 3);
    plhs[4] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[4]), out_bg, 3);
    plhs[5] = mxCreateDoubleMatrix(15, 15, mxREAL); memcpy(mxGetPr(plhs[5]), out_P, 15*15*8);
    plhs[6] = mxCreateDoubleScalar(lat);
    plhs[7] = mxCreateDoubleScalar(lon);
    plhs[8] = mxCreateDoubleScalar(alt);
}

//=============================================================================
// Baro更新
//=============================================================================
static void handle_baro(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    double pressure = mxGetScalar(prhs[1]);
    const mxArray* state = prhs[2];
    double sample = (nrhs > 3 && !mxIsEmpty(prhs[3])) ? mxGetScalar(prhs[3]) : mxGetNaN();
    
    double* p = mxGetPr(mxGetField(state, 0, "p"));
    double* v = mxGetPr(mxGetField(state, 0, "v"));
    double* q = mxGetPr(mxGetField(state, 0, "q"));
    double* ba = mxGetPr(mxGetField(state, 0, "ba"));
    double* bg = mxGetPr(mxGetField(state, 0, "bg"));
    double* P = mxGetPr(mxGetField(state, 0, "P"));
    double* g = mxGetPr(mxGetField(state, 0, "g"));
    double dt = mxGetScalar(mxGetField(state, 0, "dt"));
    double prev_baro = mxGetScalar(mxGetField(state, 0, "prev_baro"));
    double buffer_tolerance = mxGetScalar(mxGetField(state, 0, "buffer_tolerance"));
    double baro_weight = mxGetScalar(mxGetField(state, 0, "baro_weight"));
    
    double out_p[3], out_v[3], out_q[4], out_ba[3], out_bg[3], out_P[15*15];
    double new_prev_baro = prev_baro;
    copy_vec(out_p, p, 3);
    copy_vec(out_v, v, 3);
    copy_vec(out_q, q, 4);
    copy_vec(out_ba, ba, 3);
    copy_vec(out_bg, bg, 3);
    memcpy(out_P, P, 15*15*sizeof(double));
    
    bool should_skip = true;
    
    if (fabs(pressure - prev_baro) > buffer_tolerance) {
        new_prev_baro = pressure;
        
        mxArray* prhs_pre[2];
        mxArray* plhs_pre[2];
        prhs_pre[0] = mxCreateString("preprocess_baro");
        prhs_pre[1] = mxCreateDoubleScalar(pressure);
        
        double alt_baro = 0;
        if (mexCallMATLAB(2, plhs_pre, 2, prhs_pre, "mex_sensor_preprocessor") == 0) {
            alt_baro = mxGetScalar(plhs_pre[0]);
            bool is_outlier = mxIsLogicalScalarTrue(plhs_pre[1]);
            
            if (!mxIsNaN(alt_baro) && !is_outlier) {
                should_skip = false;
            }
            mxDestroyArray(plhs_pre[0]);
            mxDestroyArray(plhs_pre[1]);
        }
        mxDestroyArray(prhs_pre[0]);
        mxDestroyArray(prhs_pre[1]);
        
        if (!should_skip) {
            // baro_weight適用
            double weight_factor = 1.0 / baro_weight;
            out_P[2 + 2*15] *= weight_factor;  // P(3,3)
            
            mxArray* prhs_u[11];
            mxArray* plhs_u[7];
            prhs_u[0] = mxCreateString("baro");
            prhs_u[1] = mxCreateDoubleScalar(alt_baro);
            mxArray* pp = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(pp), out_p, 3);
            prhs_u[2] = pp;
            mxArray* vv = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(vv), out_v, 3);
            prhs_u[3] = vv;
            mxArray* qq = mxCreateDoubleMatrix(4, 1, mxREAL); copy_vec(mxGetPr(qq), out_q, 4);
            prhs_u[4] = qq;
            mxArray* bba = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(bba), out_ba, 3);
            prhs_u[5] = bba;
            mxArray* bbg = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(bbg), out_bg, 3);
            prhs_u[6] = bbg;
            mxArray* PP = mxCreateDoubleMatrix(15, 15, mxREAL); memcpy(mxGetPr(PP), out_P, 15*15*8);
            prhs_u[7] = PP;
            mxArray* gg = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(gg), g, 3);
            prhs_u[8] = gg;
            prhs_u[9] = mxCreateDoubleScalar(dt);
            prhs_u[10] = mxCreateDoubleScalar(sample);
            
            if (mexCallMATLAB(7, plhs_u, 11, prhs_u, "mex_eskf_do_update") == 0) {
                if (!mxIsLogicalScalarTrue(plhs_u[6])) {
                    copy_vec(out_p, mxGetPr(plhs_u[0]), 3);
                    copy_vec(out_v, mxGetPr(plhs_u[1]), 3);
                    copy_vec(out_q, mxGetPr(plhs_u[2]), 4);
                    copy_vec(out_ba, mxGetPr(plhs_u[3]), 3);
                    copy_vec(out_bg, mxGetPr(plhs_u[4]), 3);
                    memcpy(out_P, mxGetPr(plhs_u[5]), 15*15*8);
                }
                for (int i = 0; i < 7; ++i) mxDestroyArray(plhs_u[i]);
            }
            for (int i = 0; i < 11; ++i) mxDestroyArray(prhs_u[i]);
            
            // baro_weight復元
            out_P[2 + 2*15] /= weight_factor;
        }
    }
    
    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[0]), out_p, 3);
    plhs[1] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[1]), out_v, 3);
    plhs[2] = mxCreateDoubleMatrix(4, 1, mxREAL); copy_vec(mxGetPr(plhs[2]), out_q, 4);
    plhs[3] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[3]), out_ba, 3);
    plhs[4] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[4]), out_bg, 3);
    plhs[5] = mxCreateDoubleMatrix(15, 15, mxREAL); memcpy(mxGetPr(plhs[5]), out_P, 15*15*8);
    plhs[6] = mxCreateDoubleScalar(new_prev_baro);
}

//=============================================================================
// MEXエントリーポイント
//=============================================================================
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 3) {
        mexErrMsgTxt("Usage: mex_eskf_sensor_updates_full(sensor_type, meas/args, state, [sample])");
    }
    
    char sensor_type[32];
    mxGetString(prhs[0], sensor_type, sizeof(sensor_type));
    
    if (strcmp(sensor_type, "accel") == 0) {
        handle_accel(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(sensor_type, "mag") == 0) {
        handle_mag(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(sensor_type, "gps") == 0) {
        handle_gps(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(sensor_type, "baro") == 0) {
        handle_baro(nlhs, plhs, nrhs, prhs);
    } else {
        mexErrMsgTxt("Unknown sensor type");
    }
}



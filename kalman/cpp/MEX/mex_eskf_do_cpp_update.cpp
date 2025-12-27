// mex_eskf_do_cpp_update.cpp
// Phase 3: do_cpp_update() の完全MEX化
// センサー更新処理全体を1つのMEX関数で実行

#include "mex.h"
#include <cmath>
#include <cstring>

//=============================================================================
// 定数
//=============================================================================
static const double GRAVITY = 9.80665;
static const double DEG2RAD = 0.017453292519943295;

//=============================================================================
// ヘルパー関数
//=============================================================================
static void copy_vector(double* dst, const double* src, int n) {
    memcpy(dst, src, n * sizeof(double));
}

static void copy_matrix(double* dst, const double* src, int rows, int cols) {
    memcpy(dst, src, rows * cols * sizeof(double));
}

//=============================================================================
// 内部関数: mex_sensor_filter を呼び出して R を取得
//=============================================================================
static void get_R_noise(const char* sensor_type, double* R_out) {
    // mex_sensor_filter('get_R', sensor_type) を呼び出し
    mxArray* prhs[2];
    mxArray* plhs[1];
    
    prhs[0] = mxCreateString("get_R");
    prhs[1] = mxCreateString(sensor_type);
    
    if (mexCallMATLAB(1, plhs, 2, prhs, "mex_sensor_filter") != 0) {
        mexWarnMsgTxt("Failed to call mex_sensor_filter for get_R");
        R_out[0] = R_out[1] = R_out[2] = 0.01;
        mxDestroyArray(prhs[0]);
        mxDestroyArray(prhs[1]);
        return;
    }
    
    double* R = mxGetPr(plhs[0]);
    int n = (int)mxGetNumberOfElements(plhs[0]);
    
    // 3x3行列の場合は対角要素を取得
    if (n == 9) {
        R_out[0] = R[0];  // R(1,1)
        R_out[1] = R[4];  // R(2,2)
        R_out[2] = R[8];  // R(3,3)
    } else if (n >= 3) {
        R_out[0] = R[0];
        R_out[1] = R[1];
        R_out[2] = R[2];
    } else if (n == 1) {
        R_out[0] = R_out[1] = R_out[2] = R[0];
    } else {
        R_out[0] = R_out[1] = R_out[2] = 0.01;
    }
    
    mxDestroyArray(prhs[0]);
    mxDestroyArray(prhs[1]);
    mxDestroyArray(plhs[0]);
}

static double get_R_noise_scalar(const char* sensor_type) {
    double R[3];
    get_R_noise(sensor_type, R);
    return R[0];
}

//=============================================================================
// 内部関数: mex_sensor_filter を呼び出してノイズ推定を更新
//=============================================================================
static void update_noise_estimate(const char* sensor_type, const double* innov, int innov_len,
                                   const double* H, int H_rows, int H_cols,
                                   const double* P, int P_size) {
    // mex_sensor_filter('noise_estimate', sensor_type, innov, H, P) を呼び出し
    mxArray* prhs[5];
    mxArray* plhs[1];
    
    prhs[0] = mxCreateString("noise_estimate");
    prhs[1] = mxCreateString(sensor_type);
    
    mxArray* innov_arr = mxCreateDoubleMatrix(innov_len, 1, mxREAL);
    memcpy(mxGetPr(innov_arr), innov, innov_len * sizeof(double));
    prhs[2] = innov_arr;
    
    mxArray* H_arr = mxCreateDoubleMatrix(H_rows, H_cols, mxREAL);
    memcpy(mxGetPr(H_arr), H, H_rows * H_cols * sizeof(double));
    prhs[3] = H_arr;
    
    mxArray* P_arr = mxCreateDoubleMatrix(P_size, P_size, mxREAL);
    memcpy(mxGetPr(P_arr), P, P_size * P_size * sizeof(double));
    prhs[4] = P_arr;
    
    mexCallMATLAB(1, plhs, 5, prhs, "mex_sensor_filter");
    
    mxDestroyArray(prhs[0]);
    mxDestroyArray(prhs[1]);
    mxDestroyArray(prhs[2]);
    mxDestroyArray(prhs[3]);
    mxDestroyArray(prhs[4]);
    if (plhs[0]) mxDestroyArray(plhs[0]);
}

//=============================================================================
// 内部関数: mex_meukf_step_v2 を呼び出し
//=============================================================================
static bool call_meukf_step(const mxArray* state, const mxArray* sensor_data, const mxArray* mex_params,
                            mxArray** new_state_out, mxArray** dbg_out_ptr) {
    mxArray* prhs[3];
    mxArray* plhs[3];
    
    prhs[0] = const_cast<mxArray*>(state);
    prhs[1] = const_cast<mxArray*>(sensor_data);
    prhs[2] = const_cast<mxArray*>(mex_params);
    
    if (mexCallMATLAB(3, plhs, 3, prhs, "mex_meukf_step_v2") != 0) {
        return false;
    }
    
    *new_state_out = plhs[0];
    *dbg_out_ptr = plhs[1];
    // plhs[2] = mex_debug (not used)
    if (plhs[2]) mxDestroyArray(plhs[2]);
    
    return true;
}

//=============================================================================
// 内部関数: mex_eskf_update_postprocess を呼び出し
//=============================================================================
static bool call_update_postprocess(const char* sensor_type, const double* dx, int dx_len,
                                     const double* innov, int innov_len,
                                     const double* state_p, const double* state_v,
                                     const double* state_q, const double* state_ba, const double* state_bg,
                                     const double* state_P, const double* new_state_P,
                                     double sample,
                                     double* out_p, double* out_v, double* out_q,
                                     double* out_ba, double* out_bg, double* out_P,
                                     bool* should_skip) {
    mxArray* prhs[12];
    mxArray* plhs[7];
    
    prhs[0] = mxCreateString("postprocess");
    prhs[1] = mxCreateString(sensor_type);
    
    mxArray* dx_arr = mxCreateDoubleMatrix(dx_len, 1, mxREAL);
    memcpy(mxGetPr(dx_arr), dx, dx_len * sizeof(double));
    prhs[2] = dx_arr;
    
    mxArray* innov_arr = mxCreateDoubleMatrix(innov_len, 1, mxREAL);
    memcpy(mxGetPr(innov_arr), innov, innov_len * sizeof(double));
    prhs[3] = innov_arr;
    
    mxArray* p_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(p_arr), state_p, 3 * sizeof(double));
    prhs[4] = p_arr;
    
    mxArray* v_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(v_arr), state_v, 3 * sizeof(double));
    prhs[5] = v_arr;
    
    mxArray* q_arr = mxCreateDoubleMatrix(4, 1, mxREAL);
    memcpy(mxGetPr(q_arr), state_q, 4 * sizeof(double));
    prhs[6] = q_arr;
    
    mxArray* ba_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(ba_arr), state_ba, 3 * sizeof(double));
    prhs[7] = ba_arr;
    
    mxArray* bg_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(bg_arr), state_bg, 3 * sizeof(double));
    prhs[8] = bg_arr;
    
    mxArray* P_arr = mxCreateDoubleMatrix(15, 15, mxREAL);
    memcpy(mxGetPr(P_arr), state_P, 15 * 15 * sizeof(double));
    prhs[9] = P_arr;
    
    mxArray* new_P_arr = mxCreateDoubleMatrix(15, 15, mxREAL);
    memcpy(mxGetPr(new_P_arr), new_state_P, 15 * 15 * sizeof(double));
    prhs[10] = new_P_arr;
    
    prhs[11] = mxCreateDoubleScalar(sample);
    
    if (mexCallMATLAB(7, plhs, 12, prhs, "mex_eskf_update_postprocess") != 0) {
        // Clean up
        for (int i = 0; i < 12; ++i) mxDestroyArray(prhs[i]);
        return false;
    }
    
    memcpy(out_p, mxGetPr(plhs[0]), 3 * sizeof(double));
    memcpy(out_v, mxGetPr(plhs[1]), 3 * sizeof(double));
    memcpy(out_q, mxGetPr(plhs[2]), 4 * sizeof(double));
    memcpy(out_ba, mxGetPr(plhs[3]), 3 * sizeof(double));
    memcpy(out_bg, mxGetPr(plhs[4]), 3 * sizeof(double));
    memcpy(out_P, mxGetPr(plhs[5]), 15 * 15 * sizeof(double));
    *should_skip = mxIsLogicalScalarTrue(plhs[6]);
    
    // Clean up
    for (int i = 0; i < 12; ++i) mxDestroyArray(prhs[i]);
    for (int i = 0; i < 7; ++i) mxDestroyArray(plhs[i]);
    
    return true;
}

//=============================================================================
// メインハンドラ
//=============================================================================
static void handle_update(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // 入力: 'update', sensor_type, meas, state_struct, params_struct, sample
    // state_struct: p, v, q, ba, bg, P, g, dt
    // params_struct: noiseEstimator (R_accel, R_gyro, R_mag, R_baro, R_gps)
    
    if (nrhs < 6) {
        mexErrMsgTxt("update requires: sensor_type, meas, state, params, sample");
    }
    
    char sensor_type[32];
    mxGetString(prhs[1], sensor_type, sizeof(sensor_type));
    
    const mxArray* meas_arr = prhs[2];
    const mxArray* state = prhs[3];
    const mxArray* params = prhs[4];
    double sample = mxGetScalar(prhs[5]);
    
    // 状態を取得
    double p[3], v[3], q[4], ba[3], bg[3], P[15*15], g[3];
    double dt;
    
    copy_vector(p, mxGetPr(mxGetField(state, 0, "p")), 3);
    copy_vector(v, mxGetPr(mxGetField(state, 0, "v")), 3);
    copy_vector(q, mxGetPr(mxGetField(state, 0, "q")), 4);
    copy_vector(ba, mxGetPr(mxGetField(state, 0, "ba")), 3);
    copy_vector(bg, mxGetPr(mxGetField(state, 0, "bg")), 3);
    copy_matrix(P, mxGetPr(mxGetField(state, 0, "P")), 15, 15);
    copy_vector(g, mxGetPr(mxGetField(state, 0, "g")), 3);
    dt = mxGetScalar(mxGetField(state, 0, "dt"));
    
    // sensor_data 構造体を構築
    const char* sensor_data_fields[] = {"accel", "gyro", "mag", "gps_pos", "alt_baro", "dt",
                                         "update_accel", "update_gyro", "update_mag", "update_gps", "update_baro", "update_zupt"};
    mxArray* sensor_data = mxCreateStructMatrix(1, 1, 12, sensor_data_fields);
    
    mxArray* zeros3 = mxCreateDoubleMatrix(3, 1, mxREAL);
    memset(mxGetPr(zeros3), 0, 3 * sizeof(double));
    
    mxSetField(sensor_data, 0, "accel", mxDuplicateArray(zeros3));
    mxSetField(sensor_data, 0, "gyro", mxDuplicateArray(zeros3));
    mxSetField(sensor_data, 0, "mag", mxDuplicateArray(zeros3));
    mxSetField(sensor_data, 0, "gps_pos", mxDuplicateArray(zeros3));
    mxSetField(sensor_data, 0, "alt_baro", mxCreateDoubleScalar(0));
    mxSetField(sensor_data, 0, "dt", mxCreateDoubleScalar(dt));
    mxSetField(sensor_data, 0, "update_accel", mxCreateLogicalScalar(false));
    mxSetField(sensor_data, 0, "update_gyro", mxCreateLogicalScalar(false));
    mxSetField(sensor_data, 0, "update_mag", mxCreateLogicalScalar(false));
    mxSetField(sensor_data, 0, "update_gps", mxCreateLogicalScalar(false));
    mxSetField(sensor_data, 0, "update_baro", mxCreateLogicalScalar(false));
    mxSetField(sensor_data, 0, "update_zupt", mxCreateLogicalScalar(false));
    mxDestroyArray(zeros3);
    
    // mex_params 構造体を構築
    const char* mex_params_fields[] = {"g", "mag_ref", "noise_accel", "noise_gyro", "noise_ba", "noise_bg",
                                        "noise_mag", "noise_gps", "noise_baro", "noise_zupt", "alpha", "beta", "kappa", "trace_sample"};
    mxArray* mex_params = mxCreateStructMatrix(1, 1, 14, mex_params_fields);
    
    mxArray* g_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(g_arr), g, 3 * sizeof(double));
    mxSetField(mex_params, 0, "g", g_arr);
    
    double mag_ref[3] = {50, 0, 0};
    mxArray* mag_ref_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(mag_ref_arr), mag_ref, 3 * sizeof(double));
    mxSetField(mex_params, 0, "mag_ref", mag_ref_arr);
    
    double zeros3_data[3] = {0, 0, 0};
    mxArray* na = mxCreateDoubleMatrix(3, 1, mxREAL); memcpy(mxGetPr(na), zeros3_data, 24); mxSetField(mex_params, 0, "noise_accel", na);
    mxArray* ng = mxCreateDoubleMatrix(3, 1, mxREAL); memcpy(mxGetPr(ng), zeros3_data, 24); mxSetField(mex_params, 0, "noise_gyro", ng);
    mxArray* nba = mxCreateDoubleMatrix(3, 1, mxREAL); memcpy(mxGetPr(nba), zeros3_data, 24); mxSetField(mex_params, 0, "noise_ba", nba);
    mxArray* nbg = mxCreateDoubleMatrix(3, 1, mxREAL); memcpy(mxGetPr(nbg), zeros3_data, 24); mxSetField(mex_params, 0, "noise_bg", nbg);
    mxArray* nm = mxCreateDoubleMatrix(3, 1, mxREAL); memcpy(mxGetPr(nm), zeros3_data, 24); mxSetField(mex_params, 0, "noise_mag", nm);
    mxArray* ngps = mxCreateDoubleMatrix(3, 1, mxREAL); memcpy(mxGetPr(ngps), zeros3_data, 24); mxSetField(mex_params, 0, "noise_gps", ngps);
    mxSetField(mex_params, 0, "noise_baro", mxCreateDoubleScalar(0));
    mxArray* nz = mxCreateDoubleMatrix(3, 1, mxREAL); memcpy(mxGetPr(nz), zeros3_data, 24); mxSetField(mex_params, 0, "noise_zupt", nz);
    mxSetField(mex_params, 0, "alpha", mxCreateDoubleScalar(1e-3));
    mxSetField(mex_params, 0, "beta", mxCreateDoubleScalar(2));
    mxSetField(mex_params, 0, "kappa", mxCreateDoubleScalar(0));
    mxSetField(mex_params, 0, "trace_sample", mxIsNaN(sample) ? mxCreateDoubleScalar(mxGetNaN()) : mxCreateDoubleScalar(sample));
    
    // センサータイプ別処理
    double R_noise[3];
    double* meas = mxGetPr(meas_arr);
    
    if (strcmp(sensor_type, "accel") == 0) {
        get_R_noise("accel", R_noise);
        for (int i = 0; i < 3; ++i) R_noise[i] *= 1.5;
        
        mxArray* accel_data = mxCreateDoubleMatrix(3, 1, mxREAL);
        memcpy(mxGetPr(accel_data), meas, 3 * sizeof(double));
        mxDestroyArray(mxGetField(sensor_data, 0, "accel"));
        mxSetField(sensor_data, 0, "accel", accel_data);
        
        memcpy(mxGetPr(mxGetField(mex_params, 0, "noise_accel")), R_noise, 3 * sizeof(double));
        
        mxDestroyArray(mxGetField(sensor_data, 0, "update_accel"));
        mxSetField(sensor_data, 0, "update_accel", mxCreateLogicalScalar(true));
    }
    else if (strcmp(sensor_type, "mag") == 0) {
        get_R_noise("mag", R_noise);
        for (int i = 0; i < 3; ++i) R_noise[i] *= 1.5;
        
        mxArray* mag_data = mxCreateDoubleMatrix(3, 1, mxREAL);
        memcpy(mxGetPr(mag_data), meas, 3 * sizeof(double));
        mxDestroyArray(mxGetField(sensor_data, 0, "mag"));
        mxSetField(sensor_data, 0, "mag", mag_data);
        
        memcpy(mxGetPr(mxGetField(mex_params, 0, "noise_mag")), R_noise, 3 * sizeof(double));
        
        mxDestroyArray(mxGetField(sensor_data, 0, "update_mag"));
        mxSetField(sensor_data, 0, "update_mag", mxCreateLogicalScalar(true));
    }
    else if (strcmp(sensor_type, "gps") == 0) {
        get_R_noise("gps", R_noise);
        
        mxArray* gps_data = mxCreateDoubleMatrix(3, 1, mxREAL);
        memcpy(mxGetPr(gps_data), meas, 3 * sizeof(double));
        mxDestroyArray(mxGetField(sensor_data, 0, "gps_pos"));
        mxSetField(sensor_data, 0, "gps_pos", gps_data);
        
        memcpy(mxGetPr(mxGetField(mex_params, 0, "noise_gps")), R_noise, 3 * sizeof(double));
        
        mxDestroyArray(mxGetField(sensor_data, 0, "update_gps"));
        mxSetField(sensor_data, 0, "update_gps", mxCreateLogicalScalar(true));
    }
    else if (strcmp(sensor_type, "baro") == 0) {
        double R_baro = get_R_noise_scalar("baro");
        
        mxDestroyArray(mxGetField(sensor_data, 0, "alt_baro"));
        mxSetField(sensor_data, 0, "alt_baro", mxCreateDoubleScalar(meas[0]));
        
        mxDestroyArray(mxGetField(mex_params, 0, "noise_baro"));
        mxSetField(mex_params, 0, "noise_baro", mxCreateDoubleScalar(R_baro));
        
        mxDestroyArray(mxGetField(sensor_data, 0, "update_baro"));
        mxSetField(sensor_data, 0, "update_baro", mxCreateLogicalScalar(true));
    }
    else if (strcmp(sensor_type, "zupt") == 0) {
        double zupt_noise[3] = {0.01*0.01, 0.01*0.01, 0.01*0.01};
        memcpy(mxGetPr(mxGetField(mex_params, 0, "noise_zupt")), zupt_noise, 3 * sizeof(double));
        
        mxDestroyArray(mxGetField(sensor_data, 0, "update_zupt"));
        mxSetField(sensor_data, 0, "update_zupt", mxCreateLogicalScalar(true));
    }
    
    // state構造体を構築
    const char* state_fields[] = {"p", "v", "q", "ba", "bg", "P"};
    mxArray* state_struct = mxCreateStructMatrix(1, 1, 6, state_fields);
    
    mxArray* p_arr = mxCreateDoubleMatrix(3, 1, mxREAL); memcpy(mxGetPr(p_arr), p, 24); mxSetField(state_struct, 0, "p", p_arr);
    mxArray* v_arr = mxCreateDoubleMatrix(3, 1, mxREAL); memcpy(mxGetPr(v_arr), v, 24); mxSetField(state_struct, 0, "v", v_arr);
    mxArray* q_arr = mxCreateDoubleMatrix(4, 1, mxREAL); memcpy(mxGetPr(q_arr), q, 32); mxSetField(state_struct, 0, "q", q_arr);
    mxArray* ba_arr = mxCreateDoubleMatrix(3, 1, mxREAL); memcpy(mxGetPr(ba_arr), ba, 24); mxSetField(state_struct, 0, "ba", ba_arr);
    mxArray* bg_arr = mxCreateDoubleMatrix(3, 1, mxREAL); memcpy(mxGetPr(bg_arr), bg, 24); mxSetField(state_struct, 0, "bg", bg_arr);
    mxArray* P_arr = mxCreateDoubleMatrix(15, 15, mxREAL); memcpy(mxGetPr(P_arr), P, 15*15*8); mxSetField(state_struct, 0, "P", P_arr);
    
    // mex_meukf_step_v2 を呼び出し
    mxArray* new_state = nullptr;
    mxArray* dbg_out = nullptr;
    
    if (!call_meukf_step(state_struct, sensor_data, mex_params, &new_state, &dbg_out)) {
        mexErrMsgTxt("Failed to call mex_meukf_step_v2");
    }
    
    // noiseEstimator.estimate を呼び出し
    if (mxIsStruct(dbg_out) && mxGetField(dbg_out, 0, "innov") && mxGetField(dbg_out, 0, "H")) {
        double* innov = mxGetPr(mxGetField(dbg_out, 0, "innov"));
        int innov_len = (int)mxGetNumberOfElements(mxGetField(dbg_out, 0, "innov"));
        double* H = mxGetPr(mxGetField(dbg_out, 0, "H"));
        int H_rows = (int)mxGetM(mxGetField(dbg_out, 0, "H"));
        int H_cols = (int)mxGetN(mxGetField(dbg_out, 0, "H"));
        
        update_noise_estimate(sensor_type, innov, innov_len, H, H_rows, H_cols, P, 15);
    }
    
    // 出力を構築
    double out_p[3], out_v[3], out_q[4], out_ba[3], out_bg[3], out_P[15*15];
    bool should_skip = false;
    
    if (mxIsStruct(dbg_out) && mxGetField(dbg_out, 0, "dx")) {
        double* dx = mxGetPr(mxGetField(dbg_out, 0, "dx"));
        int dx_len = (int)mxGetNumberOfElements(mxGetField(dbg_out, 0, "dx"));
        double* innov = mxGetPr(mxGetField(dbg_out, 0, "innov"));
        int innov_len = (int)mxGetNumberOfElements(mxGetField(dbg_out, 0, "innov"));
        
        double* new_state_P = mxGetPr(mxGetField(new_state, 0, "P"));
        if (!new_state_P) new_state_P = P;
        
        if (!call_update_postprocess(sensor_type, dx, dx_len, innov, innov_len,
                                      p, v, q, ba, bg, P, new_state_P, sample,
                                      out_p, out_v, out_q, out_ba, out_bg, out_P, &should_skip)) {
            // フォールバック: new_state を使用
            copy_vector(out_p, mxGetPr(mxGetField(new_state, 0, "p")), 3);
            copy_vector(out_v, mxGetPr(mxGetField(new_state, 0, "v")), 3);
            copy_vector(out_q, mxGetPr(mxGetField(new_state, 0, "q")), 4);
            copy_vector(out_ba, mxGetPr(mxGetField(new_state, 0, "ba")), 3);
            copy_vector(out_bg, mxGetPr(mxGetField(new_state, 0, "bg")), 3);
            if (mxGetField(new_state, 0, "P")) {
                copy_matrix(out_P, mxGetPr(mxGetField(new_state, 0, "P")), 15, 15);
            } else {
                copy_matrix(out_P, P, 15, 15);
            }
        }
    } else {
        copy_vector(out_p, mxGetPr(mxGetField(new_state, 0, "p")), 3);
        copy_vector(out_v, mxGetPr(mxGetField(new_state, 0, "v")), 3);
        copy_vector(out_q, mxGetPr(mxGetField(new_state, 0, "q")), 4);
        copy_vector(out_ba, mxGetPr(mxGetField(new_state, 0, "ba")), 3);
        copy_vector(out_bg, mxGetPr(mxGetField(new_state, 0, "bg")), 3);
        if (mxGetField(new_state, 0, "P")) {
            double* new_P = mxGetPr(mxGetField(new_state, 0, "P"));
            // 対称化
            for (int i = 0; i < 15; ++i) {
                for (int j = 0; j < 15; ++j) {
                    out_P[i + j*15] = 0.5 * (new_P[i + j*15] + new_P[j + i*15]);
                }
            }
        } else {
            copy_matrix(out_P, P, 15, 15);
        }
    }
    
    // 出力配列を作成
    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL); memcpy(mxGetPr(plhs[0]), out_p, 24);
    plhs[1] = mxCreateDoubleMatrix(3, 1, mxREAL); memcpy(mxGetPr(plhs[1]), out_v, 24);
    plhs[2] = mxCreateDoubleMatrix(4, 1, mxREAL); memcpy(mxGetPr(plhs[2]), out_q, 32);
    plhs[3] = mxCreateDoubleMatrix(3, 1, mxREAL); memcpy(mxGetPr(plhs[3]), out_ba, 24);
    plhs[4] = mxCreateDoubleMatrix(3, 1, mxREAL); memcpy(mxGetPr(plhs[4]), out_bg, 24);
    plhs[5] = mxCreateDoubleMatrix(15, 15, mxREAL); memcpy(mxGetPr(plhs[5]), out_P, 15*15*8);
    plhs[6] = mxCreateLogicalScalar(should_skip);
    
    // クリーンアップ
    mxDestroyArray(sensor_data);
    mxDestroyArray(mex_params);
    mxDestroyArray(state_struct);
    mxDestroyArray(new_state);
    mxDestroyArray(dbg_out);
}

//=============================================================================
// MEXエントリーポイント
//=============================================================================
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgTxt("Usage: [p,v,q,ba,bg,P,should_skip] = mex_eskf_do_cpp_update('update', sensor_type, meas, state, params, sample)");
    }
    
    if (!mxIsChar(prhs[0])) {
        mexErrMsgTxt("First argument must be a command string");
    }
    
    char cmd[64];
    mxGetString(prhs[0], cmd, sizeof(cmd));
    
    if (strcmp(cmd, "update") == 0) {
        handle_update(nlhs, plhs, nrhs, prhs);
    } else {
        mexErrMsgTxt("Unknown command. Use 'update'");
    }
}



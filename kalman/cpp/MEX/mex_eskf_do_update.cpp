// mex_eskf_do_update.cpp
// do_cpp_update() を完全にMEX化
// 構造体構築のオーバーヘッドを削減

#include "mex.h"
#include <cmath>
#include <cstring>

//=============================================================================
// ヘルパー関数
//=============================================================================
static void copy_vec(double* dst, const double* src, int n) {
    memcpy(dst, src, n * sizeof(double));
}

//=============================================================================
// メインハンドラ
// 入力: sensor_type, meas, p, v, q, ba, bg, P, g, dt, [sample]
// 出力: [p, v, q, ba, bg, P, should_skip]
//=============================================================================
static void handle_update(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 10) {
        mexErrMsgTxt("update requires: sensor_type, meas, p, v, q, ba, bg, P, g, dt, [sample]");
    }

    // 入力を解析
    char sensor_type[32];
    mxGetString(prhs[0], sensor_type, sizeof(sensor_type));
    
    double* meas = mxGetPr(prhs[1]);
    int meas_len = (int)mxGetNumberOfElements(prhs[1]);

    double* p = mxGetPr(prhs[2]);
    double* v = mxGetPr(prhs[3]);
    double* q = mxGetPr(prhs[4]);
    double* ba = mxGetPr(prhs[5]);
    double* bg = mxGetPr(prhs[6]);
    double* P = mxGetPr(prhs[7]);
    double* g = mxGetPr(prhs[8]);
    double dt = mxGetScalar(prhs[9]);
    
    double sample = mxGetNaN();
    if (nrhs > 10 && !mxIsEmpty(prhs[10])) {
        sample = mxGetScalar(prhs[10]);
    }

    // 出力バッファ
    double out_p[3], out_v[3], out_q[4], out_ba[3], out_bg[3], out_P[15*15];
    copy_vec(out_p, p, 3);
    copy_vec(out_v, v, 3);
    copy_vec(out_q, q, 4);
    copy_vec(out_ba, ba, 3);
    copy_vec(out_bg, bg, 3);
    memcpy(out_P, P, 15*15*sizeof(double));
    bool should_skip = false;
    
    // R取得
    double R_noise[3] = {0.01, 0.01, 0.01};
    {
        mxArray* prhs_r[2];
        mxArray* plhs_r[1];
        prhs_r[0] = mxCreateString("get_R");
        prhs_r[1] = mxCreateString(sensor_type);
        if (mexCallMATLAB(1, plhs_r, 2, prhs_r, "mex_sensor_filter") == 0) {
            double* R = mxGetPr(plhs_r[0]);
            int n = (int)mxGetNumberOfElements(plhs_r[0]);
            if (n == 9) {
                R_noise[0] = R[0]; R_noise[1] = R[4]; R_noise[2] = R[8];
            } else if (n >= 3) {
                R_noise[0] = R[0]; R_noise[1] = R[1]; R_noise[2] = R[2];
            } else if (n == 1) {
                R_noise[0] = R_noise[1] = R_noise[2] = R[0];
            }
            mxDestroyArray(plhs_r[0]);
        }
        mxDestroyArray(prhs_r[0]);
        mxDestroyArray(prhs_r[1]);
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

    // mex_meukf_step_v2 呼び出し
    mxArray* prhs_m[3] = {state_s, sensor_data, mex_params};
    mxArray* plhs_m[3];
    if (mexCallMATLAB(3, plhs_m, 3, prhs_m, "mex_meukf_step_v2") == 0) {
        mxArray* new_state = plhs_m[0];
        mxArray* dbg_out = plhs_m[1];

        // noise estimate更新
        if (mxIsStruct(dbg_out) && mxGetField(dbg_out, 0, "innov") && mxGetField(dbg_out, 0, "H")) {
            mxArray* prhs_ne[5];
            mxArray* plhs_ne[1];
            prhs_ne[0] = mxCreateString("noise_estimate");
            prhs_ne[1] = mxCreateString(sensor_type);
            prhs_ne[2] = mxDuplicateArray(mxGetField(dbg_out, 0, "innov"));
            prhs_ne[3] = mxDuplicateArray(mxGetField(dbg_out, 0, "H"));
            prhs_ne[4] = mxDuplicateArray(P_arr);
            mexCallMATLAB(1, plhs_ne, 5, prhs_ne, "mex_sensor_filter");
            for (int i = 0; i < 5; ++i) mxDestroyArray(prhs_ne[i]);
            if (plhs_ne[0]) mxDestroyArray(plhs_ne[0]);
        }

        // postprocess
        if (mxIsStruct(dbg_out) && mxGetField(dbg_out, 0, "dx")) {
            mxArray* prhs_pp[12];
            mxArray* plhs_pp[7];
            prhs_pp[0] = mxCreateString("postprocess");
            prhs_pp[1] = mxCreateString(sensor_type);
            prhs_pp[2] = mxDuplicateArray(mxGetField(dbg_out, 0, "dx"));
            prhs_pp[3] = mxDuplicateArray(mxGetField(dbg_out, 0, "innov"));
            prhs_pp[4] = mxDuplicateArray(p_arr);
            prhs_pp[5] = mxDuplicateArray(v_arr);
            prhs_pp[6] = mxDuplicateArray(q_arr);
            prhs_pp[7] = mxDuplicateArray(ba_arr);
            prhs_pp[8] = mxDuplicateArray(bg_arr);
            prhs_pp[9] = mxDuplicateArray(P_arr);
            prhs_pp[10] = mxDuplicateArray(mxGetField(new_state, 0, "P"));
            prhs_pp[11] = mxCreateDoubleScalar(sample);

            if (mexCallMATLAB(7, plhs_pp, 12, prhs_pp, "mex_eskf_update_postprocess") == 0) {
                should_skip = mxIsLogicalScalarTrue(plhs_pp[6]);
                if (!should_skip) {
                    copy_vec(out_p, mxGetPr(plhs_pp[0]), 3);
                    copy_vec(out_v, mxGetPr(plhs_pp[1]), 3);
                    copy_vec(out_q, mxGetPr(plhs_pp[2]), 4);
                    copy_vec(out_ba, mxGetPr(plhs_pp[3]), 3);
                    copy_vec(out_bg, mxGetPr(plhs_pp[4]), 3);
                    memcpy(out_P, mxGetPr(plhs_pp[5]), 15*15*8);
                }
                for (int i = 0; i < 7; ++i) mxDestroyArray(plhs_pp[i]);
            }
            for (int i = 0; i < 12; ++i) mxDestroyArray(prhs_pp[i]);
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

    // 出力
    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[0]), out_p, 3);
    plhs[1] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[1]), out_v, 3);
    plhs[2] = mxCreateDoubleMatrix(4, 1, mxREAL); copy_vec(mxGetPr(plhs[2]), out_q, 4);
    plhs[3] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[3]), out_ba, 3);
    plhs[4] = mxCreateDoubleMatrix(3, 1, mxREAL); copy_vec(mxGetPr(plhs[4]), out_bg, 3);
    plhs[5] = mxCreateDoubleMatrix(15, 15, mxREAL); memcpy(mxGetPr(plhs[5]), out_P, 15*15*8);
    plhs[6] = mxCreateLogicalScalar(should_skip);
}

//=============================================================================
// MEXエントリーポイント
//=============================================================================
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 10) {
        mexErrMsgTxt("Usage: [p,v,q,ba,bg,P,skip] = mex_eskf_do_update(sensor_type, meas, p, v, q, ba, bg, P, g, dt, [sample])");
    }

    handle_update(nlhs, plhs, nrhs, prhs);
}



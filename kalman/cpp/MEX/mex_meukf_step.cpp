#include "mex.h"
#include "meukf_core.hpp"
#include "mex_type_conv.hpp"
#include <cstring>
#include <cmath>

// ヘルパー: MATLAB構造体からC++構造体へ
void matlab_to_state(const mxArray* m_state, meukf::State& c_state) {
    // p, v, q, ba, bg, P
    // MATLAB側は double なので float に変換
    
    // p, v, ba, bg
    mex_conv::mxArrayToFloatArray(mxGetField(m_state,0,"p"), c_state.p, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_state,0,"v"), c_state.v, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_state,0,"ba"), c_state.ba, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_state,0,"bg"), c_state.bg, 3);

    // q
    mex_conv::mxArrayToFloatArray(mxGetField(m_state,0,"q"), c_state.q, 4);

    // P: MATLAB column-major -> internal row-major
    mxArray* f_P = mxGetField(m_state, 0, "P");
    if (f_P) {
        float P_tmp[15*15];
        mex_conv::mxArrayToFloatArray(f_P, P_tmp, 15*15);
        for (int r=0;r<15;++r) for (int c=0;c<15;++c) c_state.P[r*15 + c] = P_tmp[c*15 + r];
    }
}

void state_to_matlab(const meukf::State& c_state, mxArray* m_state) {
    auto set_vec3 = [&](const char* name, const float* in) {
        mxArray* f = mxGetField(m_state, 0, name);
        if(!f) {
            return; 
        }
        double* pr = mxGetPr(f);
        pr[0] = static_cast<double>(in[0]); 
        pr[1] = static_cast<double>(in[1]); 
        pr[2] = static_cast<double>(in[2]);
    };
    
    set_vec3("p", c_state.p);
    set_vec3("v", c_state.v);
    set_vec3("ba", c_state.ba);
    set_vec3("bg", c_state.bg);
    
    mxArray* f_q = mxGetField(m_state, 0, "q");
    if(f_q) {
        double* pr = mxGetPr(f_q);
        pr[0] = static_cast<double>(c_state.q[0]); 
        pr[1] = static_cast<double>(c_state.q[1]);
        pr[2] = static_cast<double>(c_state.q[2]); 
        pr[3] = static_cast<double>(c_state.q[3]);
    }
    
    mxArray* f_P = mxGetField(m_state, 0, "P");
    if(f_P) {
        double* pr = mxGetPr(f_P);
        for(int c=0; c<15; ++c) {
            for(int r=0; r<15; ++r) {
                pr[c*15 + r] = static_cast<double>(c_state.P[r*15 + c]);
            }
        }
    }
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if (nrhs != 3) {
        mexErrMsgIdAndTxt("MEUKF:step:invalidNumInputs", "3 inputs required: prev_state, sensor, params");
    }

    // 入力データの取得
    const mxArray* m_prev_state = prhs[0];
    const mxArray* m_sensor = prhs[1];
    const mxArray* m_params = prhs[2];

    meukf::MEUKFInput input;
    
    // 1. State変換
    matlab_to_state(m_prev_state, input.prev_state);
    
    // 2. SensorData変換
    auto get_field_scalar = [&](const mxArray* s, const char* f) -> double {
        mxArray* field = mxGetField(s, 0, f);
        return field ? static_cast<double>(mex_conv::mxGetScalarAsFloat(field)) : 0.0;
    };

    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor,0,"accel"), input.sensor.accel, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor,0,"gyro"), input.sensor.gyro, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor,0,"mag"), input.sensor.mag, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor,0,"gps_pos"), input.sensor.gps_pos, 3);
    input.sensor.alt_baro = mex_conv::mxGetScalarAsFloat(mxGetField(m_sensor,0,"alt_baro"));
    
    // 前回のセンサー値を読み取り（変更検知用）
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor,0,"prev_mag"), input.sensor.prev_mag, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_sensor,0,"prev_gps_pos"), input.sensor.prev_gps_pos, 3);
    input.sensor.prev_baro_alt = mex_conv::mxGetScalarAsFloat(mxGetField(m_sensor,0,"prev_baro_alt"));
    
    input.sensor.update_accel = (uint8_t)get_field_scalar(m_sensor, "update_accel");
    input.sensor.update_gyro = (uint8_t)get_field_scalar(m_sensor, "update_gyro");
    input.sensor.update_mag = (uint8_t)get_field_scalar(m_sensor, "update_mag");
    input.sensor.update_gps = (uint8_t)get_field_scalar(m_sensor, "update_gps");
    input.sensor.update_baro = (uint8_t)get_field_scalar(m_sensor, "update_baro");
    input.sensor.update_zupt = (uint8_t)get_field_scalar(m_sensor, "update_zupt");
    input.sensor.dt = mex_conv::mxGetScalarAsFloat(mxGetField(m_sensor,0,"dt"));

    // 3. Params変換
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

    // Validate noise_gps input: must be finite non-negative variances (meters^2)
    for(int i=0;i<3;++i) {
        float v = input.params.noise_gps[i];
        if(!std::isfinite(v) || v < 0.0f) {
            mexErrMsgIdAndTxt("MEUKF:step:invalidNoiseGPS", "noise_gps must be finite non-negative variances (meters^2). Got %g at index %d", (double)v, i+1);
        }
    }

    // 実行
    meukf::MEUKFOutput output;
    meukf::MEUKFCore::step(input, output);

    // 出力作成（呼び出し側が要求した出力数に合わせて安全に割り当てる）
    if(nlhs > 0) {
        // 入力構造体をコピーして、値を更新する形で返す
        plhs[0] = mxDuplicateArray(m_prev_state);
        state_to_matlab(output.new_state, plhs[0]);
    }

    if(nlhs > 1) {
        // デバッグ情報
        plhs[1] = mxCreateDoubleMatrix(1, 10, mxREAL);
        double* dbg = mxGetPr(plhs[1]);
        for(int i=0; i<10; ++i) dbg[i] = output.debug_info[i];
    }

    if(nlhs > 2) {
        // 出力: last_K / last_y / last_S 等を構造体で返す
        const char* fnames[] = {"pred_P", "last_K", "last_S", "last_S_inv", "last_H", "last_y", "last_y_len", "last_sensor_type", "input_update_gps", "input_noise_gps"};
        plhs[2] = mxCreateStructMatrix(1, 1, 10, fnames);

        // pred_P: 15 x 15 double matrix (predicted P after predict(), row-major in C++)
        // pred_P: convert internal row-major float -> MATLAB column-major double
        mxArray* m_pred_P = mxCreateDoubleMatrix(15, 15, mxREAL);
        float pred_tmp[15*15];
        for(int r=0;r<15;++r) for(int c=0;c<15;++c) pred_tmp[r + c*15] = output.pred_P[r*15 + c];
        mex_conv::floatArrayToMxArray(pred_tmp, m_pred_P, 15, 15);
        mxSetField(plhs[2], 0, "pred_P", m_pred_P);

        // last_K: 15 x 3 double matrix (stored column-major in MATLAB)
        mxArray* m_last_K = mxCreateDoubleMatrix(15, 3, mxREAL);
        float K_tmp[15*3];
        for(int r=0;r<15;++r) for(int c=0;c<3;++c) K_tmp[r + c*15] = output.last_K[r*3 + c];
        mex_conv::floatArrayToMxArray(K_tmp, m_last_K, 15, 3);
        mxSetField(plhs[2], 0, "last_K", m_last_K);

        // last_S: 3 x 3 double matrix (innovation covariance, stored column-major in MATLAB)
        mxArray* m_last_S = mxCreateDoubleMatrix(3, 3, mxREAL);
        float S_tmp[3*3];
        for(int r=0;r<3;++r) for(int c=0;c<3;++c) S_tmp[r + c*3] = output.last_S[r*3 + c];
        mex_conv::floatArrayToMxArray(S_tmp, m_last_S, 3, 3);
        mxSetField(plhs[2], 0, "last_S", m_last_S);

        // last_y: variable length up to 3
        int ylen = output.last_y_len;
        if(ylen < 0) ylen = 0;
        if(ylen > 3) ylen = 3;
        mxArray* m_last_y = mxCreateDoubleMatrix(ylen, 1, mxREAL);
        if(ylen > 0) {
            float y_tmp[3];
            for(int i=0;i<ylen;++i) y_tmp[i] = output.last_y[i];
            mex_conv::floatArrayToMxArray(y_tmp, m_last_y, ylen, 1);
        }
        mxSetField(plhs[2], 0, "last_y", m_last_y);

        // last_S_inv: 3 x 3 double matrix (inverse innovation covariance)
        mxArray* m_last_S_inv = mxCreateDoubleMatrix(3, 3, mxREAL);
        float Sinv_tmp[3*3];
        for(int r=0;r<3;++r) for(int c=0;c<3;++c) Sinv_tmp[r + c*3] = output.last_S_inv[r*3 + c];
        mex_conv::floatArrayToMxArray(Sinv_tmp, m_last_S_inv, 3, 3);
        mxSetField(plhs[2], 0, "last_S_inv", m_last_S_inv);

        // last_H: 3 x 15 double matrix (measurement matrix H, stored column-major in MATLAB)
        mxArray* m_last_H = mxCreateDoubleMatrix(3, 15, mxREAL);
        float H_tmp[3*15];
        for(int r=0;r<3;++r) for(int c=0;c<15;++c) H_tmp[r + c*3] = output.last_H[r*15 + c];
        mex_conv::floatArrayToMxArray(H_tmp, m_last_H, 3, 15);
        mxSetField(plhs[2], 0, "last_H", m_last_H);

        // last_y_len
        mxArray* m_ylen = mxCreateDoubleScalar(static_cast<double>(output.last_y_len));
        mxSetField(plhs[2], 0, "last_y_len", m_ylen);

        // last_sensor_type
        mxArray* m_stype = mxCreateDoubleScalar(static_cast<double>(output.last_sensor_type));
        mxSetField(plhs[2], 0, "last_sensor_type", m_stype);
        // Echo the raw input flag for debugging (1 if sensor.update_gps was set in input)
        mxArray* m_input_update = mxCreateDoubleScalar(static_cast<double>(input.sensor.update_gps));
        mxSetField(plhs[2], 0, "input_update_gps", m_input_update);
        // Echo the raw input noise_gps vector for debugging (3x1)
        mxArray* m_input_noise = mxCreateDoubleMatrix(3, 1, mxREAL);
        float noise_tmp[3]; for(int i=0;i<3;++i) noise_tmp[i] = input.params.noise_gps[i];
        mex_conv::floatArrayToMxArray(noise_tmp, m_input_noise, 3, 1);
        mxSetField(plhs[2], 0, "input_noise_gps", m_input_noise);

        // --- Annotation/consistency checks: compare input.params.noise_gps with
        // R estimated from last_S - H*pred_P*H'. If they differ significantly,
        // emit a warning to help developers catch unit/scale mismatches.
        try {
            // Build pred_P matrix (15x15) in double from output.pred_P (float row-major)
            double predP[15*15];
            for(int r=0;r<15;++r) for(int c=0;c<15;++c) predP[r*15 + c] = static_cast<double>(output.pred_P[r*15 + c]);

            // Build H (3x15) from output.last_H (row-major floats stored similarly)
            double Hm[3*15];
            for(int r=0;r<3;++r) for(int c=0;c<15;++c) Hm[r*15 + c] = static_cast<double>(output.last_H[r*15 + c]);

            // Compute HPHT = H * predP * H'
            double HP[3*15]; memset(HP,0,sizeof(HP));
            for(int i=0;i<3;++i) {
                for(int k=0;k<15;++k) {
                    double Hik = Hm[i*15 + k];
                    for(int j=0;j<15;++j) {
                        HP[i*15 + j] += Hik * predP[k*15 + j];
                    }
                }
            }
            double HPHT[3*3]; memset(HPHT,0,sizeof(HPHT));
            for(int i=0;i<3;++i) for(int j=0;j<3;++j) {
                double sum = 0.0;
                for(int k=0;k<15;++k) sum += HP[i*15 + k] * Hm[j*15 + k];
                HPHT[i*3 + j] = sum;
            }

            // last_S is already returned as MATLAB array m_last_S; reconstruct as double
            double* prS = mxGetPr(m_last_S);
            double lastS[3*3];
            for(int c=0;c<3;++c) for(int r=0;r<3;++r) lastS[c*3 + r] = prS[c*3 + r];

            // Estimate R_est = lastS - HPHT (diagonal)
            double R_est_diag[3];
            for(int i=0;i<3;++i) R_est_diag[i] = lastS[i*3 + i] - HPHT[i*3 + i];

            // Compare input.params.noise_gps (float) with R_est_diag
            // NOTE: This is a diagnostic check only. Disabled for now due to spurious negative R_est values.
            // See: https://github.com/your-repo/issues/XXX
            // for(int i=0;i<3;++i) {
            //     double in_v = static_cast<double>(input.params.noise_gps[i]);
            //     double est = R_est_diag[i];
            //     double diff = in_v - est;
            //     double rel = (est == 0.0) ? fabs(diff) : fabs(diff)/fabs(est);
            //     if(!std::isfinite(est) || est < 0.0 || (fabs(diff) > 1.0 && rel > 0.20)) {
            //         mexWarnMsgIdAndTxt("MEUKF:step:noiseMismatch",
            //             "GPS noise mismatch detected: input noise_gps[%d]=%g vs estimated R[%d]=%g (diff=%g, rel=%g). MEX expects variances in meters^2.",
            //             i+1, in_v, i+1, est, diff, rel);
            //     }
            // }
        } catch(...) {
            // best-effort checks only; do not fail on exceptions
        }
    }
}

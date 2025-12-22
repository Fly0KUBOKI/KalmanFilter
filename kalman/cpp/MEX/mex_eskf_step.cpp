// （上部のヘッダとヘルパー関数はファイル先頭に一度だけ定義されています）
// mex_eskf_step.cpp - 統合ESKF MEXインターフェース
// 1回の呼び出しで予測+全センサー更新を実行
// 使用法: [state_out] = mex_eskf_step(state_in, sensor_data, params)

#include "mex.h"
#include "../include/MEUKF/unified_filter.hpp"
#include "mex_type_conv.hpp"
#include <cstring>
#include <cmath>

// ヘルパー関数: MATLAB配列から3次元ベクトル取得
void get_vec3(const mxArray* mx_struct, const char* field_name, float* out) {
    const mxArray* field = mxGetField(mx_struct, 0, field_name);
    mex_conv::mxArrayToFloatArray(field, out, 3);
}

// ヘルパー関数: MATLAB配列からクォータニオン取得
void get_quat(const mxArray* mx_struct, const char* field_name, float* out) {
    const mxArray* field = mxGetField(mx_struct, 0, field_name);
    // default: identity quaternion if missing; mex_conv fills zeros, so set default first
    out[0] = 1.0f; out[1] = out[2] = out[3] = 0.0f;
    mex_conv::mxArrayToFloatArray(field, out, 4);
}

// ヘルパー関数: スカラー取得
double get_scalar(const mxArray* mx_struct, const char* field_name, double default_val = 0.0) {
    const mxArray* field = mxGetField(mx_struct, 0, field_name);
    if (!field) return default_val;
    return static_cast<double>(mex_conv::mxGetScalarAsFloat(field));
}

// ヘルパー関数: 3次元ベクトルをMATLABに設定
void set_vec3(mxArray* mx_struct, const char* field_name, const float* vec) {
    mxArray* field = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* pr = mxGetPr(field);
    pr[0] = static_cast<double>(vec[0]);
    pr[1] = static_cast<double>(vec[1]);
    pr[2] = static_cast<double>(vec[2]);
    mxSetField(mx_struct, 0, field_name, field);
}

// ヘルパー関数: クォータニオンをMATLABに設定
void set_quat(mxArray* mx_struct, const char* field_name, const float* q) {
    mxArray* field = mxCreateDoubleMatrix(4, 1, mxREAL);
    double* pr = mxGetPr(field);
    pr[0] = static_cast<double>(q[0]); // qw
    pr[1] = static_cast<double>(q[1]); // qx
    pr[2] = static_cast<double>(q[2]); // qy
    pr[3] = static_cast<double>(q[3]); // qz
    mxSetField(mx_struct, 0, field_name, field);
}

// ヘルパー関数: 15x15行列をMATLABに設定
void set_matrix15(mxArray* mx_struct, const char* field_name, const float P[15][15]) {
    mxArray* field = mxCreateDoubleMatrix(15, 15, mxREAL);
    double* pr = mxGetPr(field);
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            pr[j * 15 + i] = static_cast<double>(P[i][j]); // Column-major
        }
    }
    mxSetField(mx_struct, 0, field_name, field);
}

// ヘルパー関数: 15x15行列をC++に取得
void get_matrix15(const mxArray* mx_struct, const char* field_name, float P[15][15]) {
    const mxArray* field = mxGetField(mx_struct, 0, field_name);
    if (!field || mxGetM(field) != 15 || mxGetN(field) != 15) {
        for (int i = 0; i < 15; i++) for (int j = 0; j < 15; j++) P[i][j] = (i==j)?1.0f:0.0f;
        return;
    }
    float tmp[15*15];
    mex_conv::mxArrayToFloatArray(field, tmp, 15*15);
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            P[i][j] = tmp[j*15 + i]; // column-major to row-major layout
        }
    }
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // 引数チェック
    if (nrhs != 3) {
        mexErrMsgIdAndTxt("mex_eskf_step:nrhs", "3 inputs required: state_in, sensor_data, params");
    }
    if (nlhs != 1) {
        mexErrMsgIdAndTxt("mex_eskf_step:nlhs", "1 output required: state_out");
    }
    
    const mxArray* state_in = prhs[0];
    const mxArray* sensor_data = prhs[1];
    const mxArray* params = prhs[2];
    
    // --- 入力状態読み込み ---
    // state_in は現在の推定状態を含む構造体なので、FilterOutput に読み込んで初期化する
    unified::FilterOutput init_state;
    // temp buffer for covariance
    float Pbuf[15][15];
    get_vec3(state_in, "p", init_state.position);
    get_vec3(state_in, "v", init_state.velocity);
    get_quat(state_in, "q", init_state.quaternion);
    get_vec3(state_in, "ba", init_state.accel_bias);
    get_vec3(state_in, "bg", init_state.gyro_bias);
    get_matrix15(state_in, "P", Pbuf);
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            init_state.covariance(i,j) = Pbuf[i][j];
        }
    }

    // 初期状態をフィルタに設定
    unified::UnifiedFilter filter;
    filter.initialize(init_state);

    // --- センサーデータ読み込み ---
    unified::FilterInput input; // sensor-only input
    input.dt = static_cast<float>(get_scalar(sensor_data, "dt", 0.01));
    get_vec3(sensor_data, "accel", input.accel);
    get_vec3(sensor_data, "gyro", input.gyro);
    get_vec3(sensor_data, "mag", input.mag);

    double gps_lat = get_scalar(sensor_data, "gps_lat", 0.0);
    double gps_lon = get_scalar(sensor_data, "gps_lon", 0.0);
    double gps_alt = get_scalar(sensor_data, "gps_alt", 0.0);
    input.gps_pos[0] = static_cast<float>(gps_lat);
    input.gps_pos[1] = static_cast<float>(gps_lon);
    input.gps_pos[2] = static_cast<float>(gps_alt);

    input.baro_alt = static_cast<float>(get_scalar(sensor_data, "baro", 0.0));

    // 更新有無フラグは FilterInput の bool フィールドで表現
    float accel_norm = sqrtf(input.accel[0]*input.accel[0] + 
                             input.accel[1]*input.accel[1] + 
                             input.accel[2]*input.accel[2]);
    float gyro_norm = sqrtf(input.gyro[0]*input.gyro[0] + 
                           input.gyro[1]*input.gyro[1] + 
                           input.gyro[2]*input.gyro[2]);
    float mag_norm = sqrtf(input.mag[0]*input.mag[0] + 
                          input.mag[1]*input.mag[1] + 
                          input.mag[2]*input.mag[2]);

    // accel/gyro are always provided in this interface; mark validity for others
    input.mag_valid = (mag_norm > 1e-6f);
    input.gps_valid = (fabs(gps_lat) > 1e-9 || fabs(gps_lon) > 1e-9);
    input.baro_valid = (fabs(input.baro_alt) > 1e-6f);
    
    // --- パラメータ読み込み ---
    get_vec3(params, "g", input.g);
    get_vec3(params, "mag_ref", input.mag_ref);
    input.noise_accel = static_cast<float>(get_scalar(params, "noise_accel", 0.01));
    input.noise_gyro = static_cast<float>(get_scalar(params, "noise_gyro", 1.74e-4));
    input.noise_mag = static_cast<float>(get_scalar(params, "noise_mag", 0.1));
    input.noise_gps = static_cast<float>(get_scalar(params, "noise_gps", 1.0));
    input.noise_baro = static_cast<float>(get_scalar(params, "noise_baro", 0.01));
    
    input.alpha = static_cast<float>(get_scalar(params, "alpha", 1e-3));
    input.beta = static_cast<float>(get_scalar(params, "beta", 2.0));
    input.kappa = static_cast<float>(get_scalar(params, "kappa", 0.0));
    
    // --- UnifiedFilterで更新実行 ---
    unified::FilterOutput output;
    // filter は既に初期化済み（initialize を前で呼んでいる）
    filter.update(input, output);
    
    // --- 出力状態作成 ---
    const char* field_names[] = {"p", "v", "q", "ba", "bg", "P"};
    plhs[0] = mxCreateStructMatrix(1, 1, 6, field_names);
    
    set_vec3(plhs[0], "p", output.position);
    set_vec3(plhs[0], "v", output.velocity);
    set_quat(plhs[0], "q", output.quaternion);
    set_vec3(plhs[0], "ba", output.accel_bias);
    set_vec3(plhs[0], "bg", output.gyro_bias);
    // covariance -> float[15][15] バッファにコピーして出力
    float Pout[15][15];
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            Pout[i][j] = output.covariance(i,j);
        }
    }
    set_matrix15(plhs[0], "P", Pout);
}

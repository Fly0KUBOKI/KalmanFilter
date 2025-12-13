// mex_eskf_step.cpp - 統合ESKF MEXインターフェース
// 1回の呼び出しで予測+全センサー更新を実行
// 使用法: [state_out] = mex_eskf_step(state_in, sensor_data, params)

#include "mex.h"
#include "../include/MEUKF/unified_filter.hpp"
#include <cstring>
#include <cmath>

// ヘルパー関数: MATLAB配列から3次元ベクトル取得
void get_vec3(const mxArray* mx_struct, const char* field_name, float* out) {
    mxArray* field = mxGetField(mx_struct, 0, field_name);
    if (field && mxGetNumberOfElements(field) >= 3) {
        double* pr = mxGetPr(field);
        out[0] = static_cast<float>(pr[0]);
        out[1] = static_cast<float>(pr[1]);
        out[2] = static_cast<float>(pr[2]);
    } else {
        out[0] = out[1] = out[2] = 0.0f;
    }
}

// ヘルパー関数: MATLAB配列からクォータニオン取得
void get_quat(const mxArray* mx_struct, const char* field_name, float* out) {
    mxArray* field = mxGetField(mx_struct, 0, field_name);
    if (field && mxGetNumberOfElements(field) >= 4) {
        double* pr = mxGetPr(field);
        out[0] = static_cast<float>(pr[0]); // qw
        out[1] = static_cast<float>(pr[1]); // qx
        out[2] = static_cast<float>(pr[2]); // qy
        out[3] = static_cast<float>(pr[3]); // qz
    } else {
        out[0] = 1.0f; out[1] = out[2] = out[3] = 0.0f;
    }
}

// ヘルパー関数: スカラー取得
double get_scalar(const mxArray* mx_struct, const char* field_name, double default_val = 0.0) {
    mxArray* field = mxGetField(mx_struct, 0, field_name);
    if (field && mxIsDouble(field)) {
        return mxGetScalar(field);
    }
    return default_val;
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
    mxArray* field = mxGetField(mx_struct, 0, field_name);
    if (field && mxGetM(field) == 15 && mxGetN(field) == 15) {
        double* pr = mxGetPr(field);
        for (int i = 0; i < 15; i++) {
            for (int j = 0; j < 15; j++) {
                P[i][j] = static_cast<float>(pr[j * 15 + i]); // Column-major
            }
        }
    } else {
        // 初期化（単位行列）
        for (int i = 0; i < 15; i++) {
            for (int j = 0; j < 15; j++) {
                P[i][j] = (i == j) ? 1.0f : 0.0f;
            }
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
    unified::FilterInput input;
    get_vec3(state_in, "p", input.p);
    get_vec3(state_in, "v", input.v);
    get_quat(state_in, "q", input.q);
    get_vec3(state_in, "ba", input.ba);
    get_vec3(state_in, "bg", input.bg);
    get_matrix15(state_in, "P", input.P);
    
    // --- センサーデータ読み込み ---
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
    
    input.baro = static_cast<float>(get_scalar(sensor_data, "baro", 0.0));
    
    // 更新フラグ（ゼロベクトルでない場合に更新）
    float accel_norm = sqrtf(input.accel[0]*input.accel[0] + 
                             input.accel[1]*input.accel[1] + 
                             input.accel[2]*input.accel[2]);
    float gyro_norm = sqrtf(input.gyro[0]*input.gyro[0] + 
                           input.gyro[1]*input.gyro[1] + 
                           input.gyro[2]*input.gyro[2]);
    float mag_norm = sqrtf(input.mag[0]*input.mag[0] + 
                          input.mag[1]*input.mag[1] + 
                          input.mag[2]*input.mag[2]);
    
    input.update_accel = (accel_norm > 1e-6f) ? 1 : 0;
    input.update_gyro = (gyro_norm > 1e-9f) ? 1 : 0;
    input.update_mag = (mag_norm > 1e-6f) ? 1 : 0;
    input.update_gps = (fabs(gps_lat) > 1e-9 || fabs(gps_lon) > 1e-9) ? 1 : 0;
    input.update_baro = (fabs(input.baro) > 1e-6f) ? 1 : 0;
    
    // --- パラメータ読み込み ---
    get_vec3(params, "g", input.g);
    get_vec3(params, "mag_ref", input.mag_ref);
    get_vec3(params, "noise_accel", input.noise_accel);
    get_vec3(params, "noise_gyro", input.noise_gyro);
    get_vec3(params, "noise_ba", input.noise_ba);
    get_vec3(params, "noise_bg", input.noise_bg);
    get_vec3(params, "noise_mag", input.noise_mag);
    get_vec3(params, "noise_gps", input.noise_gps);
    input.noise_baro = static_cast<float>(get_scalar(params, "noise_baro", 0.01));
    
    input.alpha = static_cast<float>(get_scalar(params, "alpha", 1e-3));
    input.beta = static_cast<float>(get_scalar(params, "beta", 2.0));
    input.kappa = static_cast<float>(get_scalar(params, "kappa", 0.0));
    
    // --- UnifiedFilterで更新実行 ---
    unified::FilterOutput output;
    unified::UnifiedFilter filter;
    filter.update(input, output);
    
    // --- 出力状態作成 ---
    const char* field_names[] = {"p", "v", "q", "ba", "bg", "P"};
    plhs[0] = mxCreateStructMatrix(1, 1, 6, field_names);
    
    set_vec3(plhs[0], "p", output.p);
    set_vec3(plhs[0], "v", output.v);
    set_quat(plhs[0], "q", output.q);
    set_vec3(plhs[0], "ba", output.ba);
    set_vec3(plhs[0], "bg", output.bg);
    set_matrix15(plhs[0], "P", output.P);
}

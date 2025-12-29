// （上部のヘッダとヘルパー関数はファイル先頭に一度だけ定義されています）
// mex_eskf_step.cpp - 統合ESKF MEXインターフェース
// 1回の呼び出しで予測+全センサー更新を実行
// 使用法: [state_out] = mex_eskf_step(state_in, sensor_data, params)

#include "mex.h"
#include "../Inc/MEUKF/unified_types.hpp"  // FilterInput, FilterOutputを直接インクルード
#include "../Inc/MEUKF/unified_filter.hpp"
#include "mex_type_conv.hpp"
#include <cstring>
#include <cmath>

// ヘルパー関数: MATLAB配列から3次元ベクトル取得（Vec3用）
void get_vec3(const mxArray* mx_struct, const char* field_name, meukf::Vec3& out) {
    const mxArray* field = mxGetField(mx_struct, 0, field_name);
    float tmp[3];
    mex_conv::mxArrayToFloatArray(field, tmp, 3);
    for (int i = 0; i < 3; ++i) {
        out(i, 0) = tmp[i];
    }
}

// ヘルパー関数: MATLAB配列からクォータニオン取得（Vec4用）
void get_quat(const mxArray* mx_struct, const char* field_name, meukf::Vec4& out) {
    const mxArray* field = mxGetField(mx_struct, 0, field_name);
    float tmp[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    mex_conv::mxArrayToFloatArray(field, tmp, 4);
    for (int i = 0; i < 4; ++i) {
        out(i, 0) = tmp[i];
    }
}

// ヘルパー関数: スカラー取得
double get_scalar(const mxArray* mx_struct, const char* field_name, double default_val = 0.0) {
    const mxArray* field = mxGetField(mx_struct, 0, field_name);
    if (!field) return default_val;
    return static_cast<double>(mex_conv::mxGetScalarAsFloat(field));
}

// ヘルパー関数: 3次元ベクトルをMATLABに設定（Vec3用）
void set_vec3(mxArray* mx_struct, const char* field_name, const meukf::Vec3& vec) {
    mxArray* field = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* pr = mxGetPr(field);
    pr[0] = static_cast<double>(vec(0, 0));
    pr[1] = static_cast<double>(vec(1, 0));
    pr[2] = static_cast<double>(vec(2, 0));
    mxSetField(mx_struct, 0, field_name, field);
}

// ヘルパー関数: クォータニオンをMATLABに設定（Vec4用）
void set_quat(mxArray* mx_struct, const char* field_name, const meukf::Vec4& q) {
    mxArray* field = mxCreateDoubleMatrix(4, 1, mxREAL);
    double* pr = mxGetPr(field);
    pr[0] = static_cast<double>(q(0, 0)); // qw
    pr[1] = static_cast<double>(q(1, 0)); // qx
    pr[2] = static_cast<double>(q(2, 0)); // qy
    pr[3] = static_cast<double>(q(3, 0)); // qz
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
    // state_in は現在の推定状態を含む構造体なので、FilterState に読み込む
    meukf::FilterState state;
    get_vec3(state_in, "p", state.p);
    get_vec3(state_in, "v", state.v);
    get_quat(state_in, "q", state.q);
    get_vec3(state_in, "ba", state.ba);
    get_vec3(state_in, "bg", state.bg);
    // temp buffer for covariance
    float Pbuf[15][15];
    get_matrix15(state_in, "P", Pbuf);
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            state.P(i, j) = Pbuf[i][j];
        }
    }

    // UnifiedFilterを作成
    meukf::UnifiedFilter filter;

    // --- センサーデータ読み込み ---
    meukf::FilterInput input; // sensor-only input
    input.dt = static_cast<float>(get_scalar(sensor_data, "dt", 0.01));
    get_vec3(sensor_data, "accel", input.accel);
    get_vec3(sensor_data, "gyro", input.gyro);
    get_vec3(sensor_data, "mag", input.mag);

    double gps_lat = get_scalar(sensor_data, "gps_lat", 0.0);
    double gps_lon = get_scalar(sensor_data, "gps_lon", 0.0);
    double gps_alt = get_scalar(sensor_data, "gps_alt", 0.0);
    input.gps_pos(0, 0) = static_cast<float>(gps_lat);
    input.gps_pos(1, 0) = static_cast<float>(gps_lon);
    input.gps_pos(2, 0) = static_cast<float>(gps_alt);

    input.baro_alt = static_cast<float>(get_scalar(sensor_data, "baro", 0.0));

    // 更新有無フラグは FilterInput の bool フィールドで表現
    float accel_norm = sqrtf(input.accel(0,0)*input.accel(0,0) + 
                             input.accel(1,0)*input.accel(1,0) + 
                             input.accel(2,0)*input.accel(2,0));
    float gyro_norm = sqrtf(input.gyro(0,0)*input.gyro(0,0) + 
                           input.gyro(1,0)*input.gyro(1,0) + 
                           input.gyro(2,0)*input.gyro(2,0));
    float mag_norm = sqrtf(input.mag(0,0)*input.mag(0,0) + 
                          input.mag(1,0)*input.mag(1,0) + 
                          input.mag(2,0)*input.mag(2,0));

    // accel/gyro are always provided in this interface; mark validity for others
    input.mag_valid = (mag_norm > 1e-6f);
    input.gps_valid = (fabs(gps_lat) > 1e-9 || fabs(gps_lon) > 1e-9);
    input.baro_valid = (fabs(input.baro_alt) > 1e-6f);
    
    // --- パラメータ読み込み ---
    get_vec3(params, "g", input.g);
    get_vec3(params, "mag_ref", input.mag_ref);
    
    // ノイズパラメータ（Vec3として読み込む）
    float noise_tmp[3];
    const mxArray* noise_accel_field = mxGetField(params, 0, "noise_accel");
    if (noise_accel_field) {
        mex_conv::mxArrayToFloatArray(noise_accel_field, noise_tmp, 3);
        for (int i = 0; i < 3; ++i) input.noise_accel(i, 0) = noise_tmp[i];
    } else {
        float val = static_cast<float>(get_scalar(params, "noise_accel", 0.01));
        for (int i = 0; i < 3; ++i) input.noise_accel(i, 0) = val;
    }
    
    const mxArray* noise_gyro_field = mxGetField(params, 0, "noise_gyro");
    if (noise_gyro_field) {
        mex_conv::mxArrayToFloatArray(noise_gyro_field, noise_tmp, 3);
        for (int i = 0; i < 3; ++i) input.noise_gyro(i, 0) = noise_tmp[i];
    } else {
        float val = static_cast<float>(get_scalar(params, "noise_gyro", 1.74e-4));
        for (int i = 0; i < 3; ++i) input.noise_gyro(i, 0) = val;
    }
    
    const mxArray* noise_mag_field = mxGetField(params, 0, "noise_mag");
    if (noise_mag_field) {
        mex_conv::mxArrayToFloatArray(noise_mag_field, noise_tmp, 3);
        for (int i = 0; i < 3; ++i) input.noise_mag(i, 0) = noise_tmp[i];
    } else {
        float val = static_cast<float>(get_scalar(params, "noise_mag", 0.1));
        for (int i = 0; i < 3; ++i) input.noise_mag(i, 0) = val;
    }
    
    const mxArray* noise_gps_field = mxGetField(params, 0, "noise_gps");
    if (noise_gps_field) {
        mex_conv::mxArrayToFloatArray(noise_gps_field, noise_tmp, 3);
        for (int i = 0; i < 3; ++i) input.noise_gps(i, 0) = noise_tmp[i];
    } else {
        float val = static_cast<float>(get_scalar(params, "noise_gps", 1.0));
        for (int i = 0; i < 3; ++i) input.noise_gps(i, 0) = val;
    }
    
    input.noise_baro = static_cast<float>(get_scalar(params, "noise_baro", 0.01));
    
    input.alpha = static_cast<float>(get_scalar(params, "alpha", 1e-3));
    input.beta = static_cast<float>(get_scalar(params, "beta", 2.0));
    input.kappa = static_cast<float>(get_scalar(params, "kappa", 0.0));
    
    // --- UnifiedFilterで更新実行 ---
    meukf::FilterOutput output = filter.update(state, input);
    
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

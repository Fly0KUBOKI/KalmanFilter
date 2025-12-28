#include "mex.h"
#include "../Inc/MEUKF/unified_types.hpp"  // FilterInput, FilterOutputを直接インクルード
#include "../Inc/MEUKF/unified_filter.hpp"
#include "../Inc/MEUKF/meukf_core.hpp"
#include "mex_type_conv.hpp"
#include <cstring>

// ヘルパー: MATLAB double から C++ float への変換
inline float to_float(double d) { return static_cast<float>(d); }

// ヘルパー: MATLABからVector3取得（Vec3用）
void get_vec3(const mxArray* mx_struct, const char* field_name, meukf::Vec3& out) {
    const mxArray* field = mxGetField(mx_struct, 0, field_name);
    float tmp[3];
    mex_conv::mxArrayToFloatArray(field, tmp, 3);
    for (int i = 0; i < 3; ++i) {
        out(i, 0) = tmp[i];
    }
}

// ヘルパー: Vector3をMATLABへ設定（Vec3用）
void set_vec3(mxArray* mx_struct, const char* field_name, const meukf::Vec3& in) {
    mxArray* field = mxGetField(mx_struct, 0, field_name);
    if (field) {
        double* pr = mxGetPr(field);
        pr[0] = static_cast<double>(in(0, 0));
        pr[1] = static_cast<double>(in(1, 0));
        pr[2] = static_cast<double>(in(2, 0));
    }
}

// ヘルパー: MATLABからスカラー取得
float get_scalar(const mxArray* mx_struct, const char* field_name, float default_val = 0.0f) {
    const mxArray* field = mxGetField(mx_struct, 0, field_name);
    return field ? mex_conv::mxGetScalarAsFloat(field) : default_val;
}

// ヘルパー: MATLABからbool取得
bool get_bool(const mxArray* mx_struct, const char* field_name, bool default_val = false) {
    const mxArray* field = mxGetField(mx_struct, 0, field_name);
    if (!field) return default_val;
    return (mex_conv::mxGetScalarAsFloat(field) != 0.0f);
}

// FilterInputをMATLAB構造体から読み込み
void matlab_to_filter_input(const mxArray* mx_input, meukf::FilterInput& input) {
    input.dt = get_scalar(mx_input, "dt", 0.01f);
    
    get_vec3(mx_input, "accel", input.accel);
    get_vec3(mx_input, "gyro", input.gyro);
    get_vec3(mx_input, "mag", input.mag);
    get_vec3(mx_input, "gps_pos", input.gps_pos);
    input.baro_alt = get_scalar(mx_input, "baro_alt", 0.0f);
    
    input.mag_valid = get_bool(mx_input, "mag_valid", false);
    input.gps_valid = get_bool(mx_input, "gps_valid", false);
    input.baro_valid = get_bool(mx_input, "baro_valid", false);
    
    get_vec3(mx_input, "g", input.g);
    get_vec3(mx_input, "mag_ref", input.mag_ref);
    
    // ノイズパラメータ（Vec3として読み込む）
    float noise_tmp[3];
    const mxArray* noise_accel_field = mxGetField(mx_input, 0, "noise_accel");
    if (noise_accel_field) {
        mex_conv::mxArrayToFloatArray(noise_accel_field, noise_tmp, 3);
        for (int i = 0; i < 3; ++i) input.noise_accel(i, 0) = noise_tmp[i];
    } else {
        for (int i = 0; i < 3; ++i) input.noise_accel(i, 0) = 0.01f;
    }
    
    const mxArray* noise_gyro_field = mxGetField(mx_input, 0, "noise_gyro");
    if (noise_gyro_field) {
        mex_conv::mxArrayToFloatArray(noise_gyro_field, noise_tmp, 3);
        for (int i = 0; i < 3; ++i) input.noise_gyro(i, 0) = noise_tmp[i];
    } else {
        for (int i = 0; i < 3; ++i) input.noise_gyro(i, 0) = 1.74e-4f;
    }
    
    const mxArray* noise_mag_field = mxGetField(mx_input, 0, "noise_mag");
    if (noise_mag_field) {
        mex_conv::mxArrayToFloatArray(noise_mag_field, noise_tmp, 3);
        for (int i = 0; i < 3; ++i) input.noise_mag(i, 0) = noise_tmp[i];
    } else {
        for (int i = 0; i < 3; ++i) input.noise_mag(i, 0) = 0.1f;
    }
    
    const mxArray* noise_gps_field = mxGetField(mx_input, 0, "noise_gps");
    if (noise_gps_field) {
        mex_conv::mxArrayToFloatArray(noise_gps_field, noise_tmp, 3);
        for (int i = 0; i < 3; ++i) input.noise_gps(i, 0) = noise_tmp[i];
    } else {
        for (int i = 0; i < 3; ++i) input.noise_gps(i, 0) = 1.0f;
    }
    
    input.noise_baro = get_scalar(mx_input, "noise_baro", 1.0f);
    
    input.alpha = get_scalar(mx_input, "alpha", 0.001f);
    input.beta = get_scalar(mx_input, "beta", 2.0f);
    input.kappa = get_scalar(mx_input, "kappa", 0.0f);
}

// FilterOutputをMATLAB構造体へ書き込み
mxArray* filter_output_to_matlab(const meukf::FilterOutput& output) {
    const char* field_names[] = {
        "position", "velocity", "quaternion", "accel_bias", "gyro_bias",
        "covariance", "roll", "pitch", "yaw",
        "innovation_norm_accel", "innovation_norm_mag", 
        "innovation_norm_gps", "innovation_norm_baro",
        "divergence_detected", "reset_occurred"
    };
    
    mxArray* mx_output = mxCreateStructMatrix(1, 1, 15, field_names);
    
    // Vector3フィールド
    mxSetField(mx_output, 0, "position", mxCreateDoubleMatrix(3, 1, mxREAL));
    set_vec3(mx_output, "position", output.position);
    
    mxSetField(mx_output, 0, "velocity", mxCreateDoubleMatrix(3, 1, mxREAL));
    set_vec3(mx_output, "velocity", output.velocity);
    
    mxSetField(mx_output, 0, "accel_bias", mxCreateDoubleMatrix(3, 1, mxREAL));
    set_vec3(mx_output, "accel_bias", output.accel_bias);
    
    mxSetField(mx_output, 0, "gyro_bias", mxCreateDoubleMatrix(3, 1, mxREAL));
    set_vec3(mx_output, "gyro_bias", output.gyro_bias);
    
    // Quaternion [qw, qx, qy, qz]
    mxArray* mx_q = mxCreateDoubleMatrix(4, 1, mxREAL);
    double* q_data = mxGetPr(mx_q);
    q_data[0] = static_cast<double>(output.quaternion(0, 0));
    q_data[1] = static_cast<double>(output.quaternion(1, 0));
    q_data[2] = static_cast<double>(output.quaternion(2, 0));
    q_data[3] = static_cast<double>(output.quaternion(3, 0));
    mxSetField(mx_output, 0, "quaternion", mx_q);
    
    // Covariance 15x15
    mxArray* mx_P = mxCreateDoubleMatrix(15, 15, mxREAL);
    double* P_data = mxGetPr(mx_P);
    for (int c = 0; c < 15; ++c) {
        for (int r = 0; r < 15; ++r) {
            P_data[c*15 + r] = static_cast<double>(output.covariance(r, c));
        }
    }
    mxSetField(mx_output, 0, "covariance", mx_P);
    
    // Euler angles
    mxSetField(mx_output, 0, "roll", mxCreateDoubleScalar(output.roll));
    mxSetField(mx_output, 0, "pitch", mxCreateDoubleScalar(output.pitch));
    mxSetField(mx_output, 0, "yaw", mxCreateDoubleScalar(output.yaw));
    
    // Innovations
    mxSetField(mx_output, 0, "innovation_norm_accel", mxCreateDoubleScalar(output.innovation_norm_accel));
    mxSetField(mx_output, 0, "innovation_norm_mag", mxCreateDoubleScalar(output.innovation_norm_mag));
    mxSetField(mx_output, 0, "innovation_norm_gps", mxCreateDoubleScalar(output.innovation_norm_gps));
    mxSetField(mx_output, 0, "innovation_norm_baro", mxCreateDoubleScalar(output.innovation_norm_baro));
    
    // Flags
    mxSetField(mx_output, 0, "divergence_detected", mxCreateDoubleScalar(output.divergence_detected ? 1.0 : 0.0));
    mxSetField(mx_output, 0, "reset_occurred", mxCreateDoubleScalar(output.reset_occurred ? 1.0 : 0.0));
    
    return mx_output;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // 入力チェック: mex_unified_filter(prev_state, input_struct)
    if (nrhs != 2) {
        mexErrMsgIdAndTxt("UnifiedFilter:invalidInputs", 
            "2 inputs required: prev_state (struct), input (struct)");
    }
    if (nlhs != 1) {
        mexErrMsgIdAndTxt("UnifiedFilter:invalidOutputs", 
            "1 output required: output (struct)");
    }
    
    const mxArray* mx_prev_state = prhs[0];
    const mxArray* mx_input = prhs[1];
    
    // MATLAB構造体から変換
    meukf::FilterInput input;
    matlab_to_filter_input(mx_input, input);
    
    // 前回の状態をFilterStateに変換
    meukf::FilterState state;
    get_vec3(mx_prev_state, "p", state.p);
    get_vec3(mx_prev_state, "v", state.v);
    const mxArray* q_field = mxGetField(mx_prev_state, 0, "q");
    float q_tmp[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    mex_conv::mxArrayToFloatArray(q_field, q_tmp, 4);
    for (int i = 0; i < 4; ++i) state.q(i, 0) = q_tmp[i];
    get_vec3(mx_prev_state, "ba", state.ba);
    get_vec3(mx_prev_state, "bg", state.bg);
    const mxArray* P_field = mxGetField(mx_prev_state, 0, "P");
    float P_tmp[15*15];
    mex_conv::mxArrayToFloatArray(P_field, P_tmp, 15*15);
    for (int c = 0; c < 15; ++c) {
        for (int r = 0; r < 15; ++r) {
            state.P(r, c) = P_tmp[c*15 + r];
        }
    }
    
    // UnifiedFilterを使用してupdate
    meukf::UnifiedFilter filter;
    meukf::FilterOutput output = filter.update(state, input);
    
    // 結果をMATLABへ返す
    plhs[0] = filter_output_to_matlab(output);
}

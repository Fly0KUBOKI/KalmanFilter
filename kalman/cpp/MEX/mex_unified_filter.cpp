#include "mex.h"
#include "../include/MEUKF/unified_filter.hpp"
#include "../MEUKF/meukf_core.hpp"
#include <cstring>

// ヘルパー: MATLAB double から C++ float への変換
inline float to_float(double d) { return static_cast<float>(d); }

// ヘルパー: MATLABからVector3取得
void get_vec3(const mxArray* mx_struct, const char* field_name, float* out) {
    mxArray* field = mxGetField(mx_struct, 0, field_name);
    if (field && mxGetNumberOfElements(field) >= 3) {
        double* pr = mxGetPr(field);
        out[0] = to_float(pr[0]);
        out[1] = to_float(pr[1]);
        out[2] = to_float(pr[2]);
    }
}

// ヘルパー: Vector3をMATLABへ設定
void set_vec3(mxArray* mx_struct, const char* field_name, const float* in) {
    mxArray* field = mxGetField(mx_struct, 0, field_name);
    if (field) {
        double* pr = mxGetPr(field);
        pr[0] = static_cast<double>(in[0]);
        pr[1] = static_cast<double>(in[1]);
        pr[2] = static_cast<double>(in[2]);
    }
}

// ヘルパー: MATLABからスカラー取得
float get_scalar(const mxArray* mx_struct, const char* field_name, float default_val = 0.0f) {
    mxArray* field = mxGetField(mx_struct, 0, field_name);
    return field ? to_float(mxGetScalar(field)) : default_val;
}

// ヘルパー: MATLABからbool取得
bool get_bool(const mxArray* mx_struct, const char* field_name, bool default_val = false) {
    mxArray* field = mxGetField(mx_struct, 0, field_name);
    return field ? (mxGetScalar(field) != 0.0) : default_val;
}

// FilterInputをMATLAB構造体から読み込み
void matlab_to_filter_input(const mxArray* mx_input, unified::FilterInput& input) {
    input.dt = get_scalar(mx_input, "dt", 0.01f);
    
    get_vec3(mx_input, "accel", input.accel);
    get_vec3(mx_input, "gyro", input.gyro);
    get_vec3(mx_input, "mag", input.mag);
    get_vec3(mx_input, "gps_pos", input.gps_pos);
    input.baro_alt = get_scalar(mx_input, "baro_alt", 0.0f);
    
    input.mag_valid = get_bool(mx_input, "mag_valid", false);
    input.gps_valid = get_bool(mx_input, "gps_valid", false);
    input.baro_valid = get_bool(mx_input, "baro_valid", false);
    
    get_vec3(mx_input, "prev_mag", input.prev_mag);
    get_vec3(mx_input, "prev_gps_pos", input.prev_gps_pos);
    input.prev_baro_alt = get_scalar(mx_input, "prev_baro_alt", 0.0f);
    
    get_vec3(mx_input, "g", input.g);
    get_vec3(mx_input, "mag_ref", input.mag_ref);
    
    input.noise_accel = get_scalar(mx_input, "noise_accel", 0.01f);
    input.noise_gyro = get_scalar(mx_input, "noise_gyro", 1.74e-4f);
    input.noise_mag = get_scalar(mx_input, "noise_mag", 0.1f);
    input.noise_gps = get_scalar(mx_input, "noise_gps", 1.0f);
    input.noise_baro = get_scalar(mx_input, "noise_baro", 1.0f);
    
    input.alpha = get_scalar(mx_input, "alpha", 0.001f);
    input.beta = get_scalar(mx_input, "beta", 2.0f);
    input.kappa = get_scalar(mx_input, "kappa", 0.0f);
}

// FilterOutputをMATLAB構造体へ書き込み
mxArray* filter_output_to_matlab(const unified::FilterOutput& output) {
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
    q_data[0] = static_cast<double>(output.quaternion[0]);
    q_data[1] = static_cast<double>(output.quaternion[1]);
    q_data[2] = static_cast<double>(output.quaternion[2]);
    q_data[3] = static_cast<double>(output.quaternion[3]);
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

// 現在の状態をFilterOutputに変換 (ESKF state -> FilterOutput)
void eskf_state_to_filter_output(const mxArray* mx_state, unified::FilterOutput& output) {
    get_vec3(mx_state, "p", output.position);
    get_vec3(mx_state, "v", output.velocity);
    get_vec3(mx_state, "ba", output.accel_bias);
    get_vec3(mx_state, "bg", output.gyro_bias);
    
    // Quaternion
    mxArray* q_field = mxGetField(mx_state, 0, "q");
    if (q_field && mxGetNumberOfElements(q_field) >= 4) {
        double* q_data = mxGetPr(q_field);
        output.quaternion[0] = to_float(q_data[0]); // qw
        output.quaternion[1] = to_float(q_data[1]); // qx
        output.quaternion[2] = to_float(q_data[2]); // qy
        output.quaternion[3] = to_float(q_data[3]); // qz
    }
    
    // Covariance
    mxArray* P_field = mxGetField(mx_state, 0, "P");
    if (P_field) {
        double* P_data = mxGetPr(P_field);
        for (int c = 0; c < 15; ++c) {
            for (int r = 0; r < 15; ++r) {
                output.covariance(r, c) = to_float(P_data[c*15 + r]);
            }
        }
    }
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
    unified::FilterInput input;
    matlab_to_filter_input(mx_input, input);
    
    unified::FilterOutput prev_output;
    eskf_state_to_filter_output(mx_prev_state, prev_output);
    
    // UnifiedFilterを使用してupdate
    unified::UnifiedFilter filter;
    filter.initialize(prev_output);
    
    unified::FilterOutput output;
    filter.update(input, output);
    
    // 結果をMATLABへ返す
    plhs[0] = filter_output_to_matlab(output);
}

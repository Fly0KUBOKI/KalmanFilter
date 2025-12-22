#include "mex.h"
#include "../Common/Math/math_utils.hpp"
#include "../Common/Math/quaternion.hpp"
#include "../Common/Validation/validation.hpp"
#include "../include/Common/Sensor/sensor_filter.hpp"
#include "mex_type_conv.hpp"
#include <cstring>
#include <vector>

using cm = cmath_fx::FixedMatrix;

// ===== MathUtils Handlers =====
void handle_wrap_to_pi(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) mexErrMsgIdAndTxt("mex_common_lib:wrap_to_pi", "Need 1 input: angle");
    float anglef = mex_conv::mxGetScalarAsFloat(prhs[0]);
    float wrappedf = common::math::MathUtils::wrap_to_pi(anglef);
    plhs[0] = mxCreateDoubleScalar(static_cast<double>(wrappedf));
}

void handle_normalize_vector(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) mexErrMsgIdAndTxt("mex_common_lib:normalize_vector", "Need 1 input: vector");
    const mxArray* arr = prhs[0];
    int n = static_cast<int>(mxGetNumberOfElements(arr));
    std::vector<float> tmp(static_cast<size_t>(n));
    mex_conv::mxArrayToFloatArray(arr, tmp.data(), static_cast<size_t>(n));

    cm v; v.resize(n, 1);
    for (int i = 0; i < n; ++i) v(i,0) = tmp[i];

    cm v_norm = common::math::MathUtils::normalize_vector(v);

    plhs[0] = mxCreateDoubleMatrix(n, 1, mxREAL);
    double* v_out = mxGetPr(plhs[0]);
    for (int i = 0; i < n; ++i) v_out[i] = static_cast<double>(v_norm(i,0));
}

void handle_skew_symmetric(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) mexErrMsgIdAndTxt("mex_common_lib:skew_symmetric", "Need 1 input: vector");
    const mxArray* arr = prhs[0];
    std::vector<float> tmp(3);
    mex_conv::mxArrayToFloatArray(arr, tmp.data(), 3);

    cm v; v.resize(3, 1);
    for (int i = 0; i < 3; ++i) v(i,0) = tmp[i];

    cm S = common::math::MathUtils::skew_symmetric(v);

    plhs[0] = mxCreateDoubleMatrix(3, 3, mxREAL);
    double* S_out = mxGetPr(plhs[0]);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            S_out[i + j*3] = static_cast<double>(S(i,j));
        }
    }
}

void handle_enforce_symmetry(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) mexErrMsgIdAndTxt("mex_common_lib:enforce_symmetry", "Need 1 input: matrix");
    const mxArray* arr = prhs[0];
    int n = static_cast<int>(mxGetM(arr));

    std::vector<float> tmp(static_cast<size_t>(n) * static_cast<size_t>(n));
    mex_conv::mxArrayToFloatArray(arr, tmp.data(), static_cast<size_t>(n) * static_cast<size_t>(n));

    cm M; M.resize(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            M(i,j) = tmp[i + j*n];
        }
    }

    common::math::MathUtils::enforce_symmetry(M);

    plhs[0] = mxCreateDoubleMatrix(n, n, mxREAL);
    double* M_out = mxGetPr(plhs[0]);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            M_out[i + j*n] = static_cast<double>(M(i,j));
        }
    }
}

// ===== QuaternionLib Handlers =====
void handle_quat_multiply(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 2) mexErrMsgIdAndTxt("mex_common_lib:quat_multiply", "Need 2 inputs: q1, q2");
    std::vector<float> q1_tmp(4), q2_tmp(4);
    mex_conv::mxArrayToFloatArray(prhs[0], q1_tmp.data(), 4);
    mex_conv::mxArrayToFloatArray(prhs[1], q2_tmp.data(), 4);

    cm q1, q2; q1.resize(4,1); q2.resize(4,1);
    for (int i = 0; i < 4; ++i) {
        q1(i,0) = q1_tmp[i];
        q2(i,0) = q2_tmp[i];
    }

    cm q_out;
    cquat::multiply_quat(q1, q2, q_out);

    plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);
    double* q_data = mxGetPr(plhs[0]);
    for (int i = 0; i < 4; ++i) q_data[i] = static_cast<double>(q_out(i,0));
}

void handle_quat_normalize(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) mexErrMsgIdAndTxt("mex_common_lib:quat_normalize", "Need 1 input: q");
    std::vector<float> q_tmp(4);
    mex_conv::mxArrayToFloatArray(prhs[0], q_tmp.data(), 4);
    cm q; q.resize(4,1);
    for (int i = 0; i < 4; ++i) q(i,0) = q_tmp[i];

    cquat::normalize_quat(q);

    plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);
    double* q_out = mxGetPr(plhs[0]);
    for (int i = 0; i < 4; ++i) q_out[i] = static_cast<double>(q(i,0));
}

void handle_quat_to_rotm(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) mexErrMsgIdAndTxt("mex_common_lib:quat_to_rotm", "Need 1 input: q");
    std::vector<float> q_tmp(4);
    mex_conv::mxArrayToFloatArray(prhs[0], q_tmp.data(), 4);
    cm q; q.resize(4,1);
    for (int i = 0; i < 4; ++i) q(i,0) = q_tmp[i];

    cm R;
    cquat::quat_to_rotm(q, R);

    plhs[0] = mxCreateDoubleMatrix(3, 3, mxREAL);
    double* R_out = mxGetPr(plhs[0]);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_out[i + j*3] = static_cast<double>(R(i,j));
        }
    }
}

void handle_quat_from_euler(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 3) mexErrMsgIdAndTxt("mex_common_lib:quat_from_euler", "Need 3 inputs: roll, pitch, yaw (deg)");
    float roll = mex_conv::mxGetScalarAsFloat(prhs[0]);
    float pitch = mex_conv::mxGetScalarAsFloat(prhs[1]);
    float yaw = mex_conv::mxGetScalarAsFloat(prhs[2]);
    
    cm q;
    cquat::from_euler_deg(roll, pitch, yaw, q);
    
    plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);
    double* q_out = mxGetPr(plhs[0]);
    for (int i = 0; i < 4; ++i) q_out[i] = q(i,0);
}

void handle_quat_to_euler(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) mexErrMsgIdAndTxt("mex_common_lib:quat_to_euler", "Need 1 input: q");
    std::vector<float> q_tmp(4);
    mex_conv::mxArrayToFloatArray(prhs[0], q_tmp.data(), 4);
    cm q; q.resize(4,1);
    for (int i = 0; i < 4; ++i) q(i,0) = q_tmp[i];

    cm euler_deg;
    cquat::to_euler_deg(q, euler_deg);

    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* euler = mxGetPr(plhs[0]);
    for (int i = 0; i < 3; ++i) euler[i] = static_cast<double>(euler_deg(i,0));
}

// ===== StateValidator Handlers =====
void handle_clip_velocity(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 2) mexErrMsgIdAndTxt("mex_common_lib:clip_velocity", "Need 2 inputs: v, max_vel");
    std::vector<float> v_tmp(3);
    mex_conv::mxArrayToFloatArray(prhs[0], v_tmp.data(), 3);
    float max_vel = mex_conv::mxGetScalarAsFloat(prhs[1]);

    cm v; v.resize(3,1);
    for (int i = 0; i < 3; ++i) v(i,0) = v_tmp[i];
    
    // ノルム計算
    float v_norm = 0.0f;
    for (int i = 0; i < 3; ++i) v_norm += v(i,0) * v(i,0);
    v_norm = sqrtf(v_norm);
    
    bool clipped = (v_norm > max_vel);
    cm v_clipped = v;
    if (clipped) {
        float scale = max_vel / v_norm;
        for (int i = 0; i < 3; ++i) v_clipped(i,0) *= scale;
    }
    
    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* v_out = mxGetPr(plhs[0]);
    for (int i = 0; i < 3; ++i) v_out[i] = v_clipped(i,0);
    
    if (nlhs > 1) {
        plhs[1] = mxCreateLogicalScalar(clipped);
    }
}

void handle_validate_covariance(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) mexErrMsgIdAndTxt("mex_common_lib:validate_covariance", "Need 1 input: P");
    int n = static_cast<int>(mxGetM(prhs[0]));
    std::vector<float> P_tmp(static_cast<size_t>(n) * static_cast<size_t>(n));
    mex_conv::mxArrayToFloatArray(prhs[0], P_tmp.data(), static_cast<size_t>(n) * static_cast<size_t>(n));

    cm P; P.resize(n,n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            P(i,j) = P_tmp[i + j*n];
        }
    }
    
    bool valid = common::validation::StateValidator::validate_covariance(P);
    cm P_valid = common::validation::CovarianceRegularizer::regularize(P);
    
    plhs[0] = mxCreateDoubleMatrix(n, n, mxREAL);
    double* P_out = mxGetPr(plhs[0]);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            P_out[i + j*n] = P_valid(i,j);
        }
    }
    
    if (nlhs > 1) {
        plhs[1] = mxCreateLogicalScalar(valid);
    }
}

// ===== SensorFilter Handlers =====
void handle_lowpass_filter(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 3) mexErrMsgIdAndTxt("mex_common_lib:lowpass_filter", "Need 3 inputs: signal, prev_filtered, alpha");
    int n = static_cast<int>(mxGetNumberOfElements(prhs[0]));
    std::vector<float> signal_tmp(static_cast<size_t>(n));
    std::vector<float> prev_tmp(static_cast<size_t>(n));
    mex_conv::mxArrayToFloatArray(prhs[0], signal_tmp.data(), static_cast<size_t>(n));
    mex_conv::mxArrayToFloatArray(prhs[1], prev_tmp.data(), static_cast<size_t>(n));
    float alpha = mex_conv::mxGetScalarAsFloat(prhs[2]);

    cm signal, prev, filtered;
    signal.resize(n,1); prev.resize(n,1); filtered.resize(n,1);
    for (int i = 0; i < n; ++i) {
        signal(i,0) = signal_tmp[i];
        prev(i,0) = prev_tmp[i];
        filtered(i,0) = alpha * signal(i,0) + (1.0f - alpha) * prev(i,0);
    }
    
    plhs[0] = mxCreateDoubleMatrix(n, 1, mxREAL);
    double* filt_out = mxGetPr(plhs[0]);
    for (int i = 0; i < n; ++i) filt_out[i] = filtered(i,0);
}

void handle_detect_outlier(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 3) mexErrMsgIdAndTxt("mex_common_lib:detect_outlier", "Need 3 inputs: measurement, expected, threshold");
    int n = static_cast<int>(mxGetNumberOfElements(prhs[0]));
    std::vector<float> meas_tmp(static_cast<size_t>(n));
    std::vector<float> exp_tmp(static_cast<size_t>(n));
    mex_conv::mxArrayToFloatArray(prhs[0], meas_tmp.data(), static_cast<size_t>(n));
    mex_conv::mxArrayToFloatArray(prhs[1], exp_tmp.data(), static_cast<size_t>(n));
    float threshold = mex_conv::mxGetScalarAsFloat(prhs[2]);

    float diff_norm = 0.0f;
    for (int i = 0; i < n; ++i) {
        float diff = meas_tmp[i] - exp_tmp[i];
        diff_norm += diff * diff;
    }
    diff_norm = sqrtf(diff_norm);
    
    bool is_outlier = (diff_norm > threshold);
    plhs[0] = mxCreateLogicalScalar(is_outlier);
}

// ===== Main MEX Function =====
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1 || !mxIsChar(prhs[0])) {
        mexErrMsgIdAndTxt("mex_common_lib:invalidInput", 
            "First argument must be a function name string");
    }
    
    char func_name[128];
    mxGetString(prhs[0], func_name, sizeof(func_name));
    
    // MathUtils
    if (strcmp(func_name, "wrap_to_pi") == 0) {
        handle_wrap_to_pi(nlhs, plhs, nrhs-1, prhs+1);
    }
    else if (strcmp(func_name, "normalize_vector") == 0) {
        handle_normalize_vector(nlhs, plhs, nrhs-1, prhs+1);
    }
    else if (strcmp(func_name, "skew_symmetric") == 0) {
        handle_skew_symmetric(nlhs, plhs, nrhs-1, prhs+1);
    }
    else if (strcmp(func_name, "enforce_symmetry") == 0) {
        handle_enforce_symmetry(nlhs, plhs, nrhs-1, prhs+1);
    }
    // QuaternionLib
    else if (strcmp(func_name, "quat_multiply") == 0) {
        handle_quat_multiply(nlhs, plhs, nrhs-1, prhs+1);
    }
    else if (strcmp(func_name, "quat_normalize") == 0) {
        handle_quat_normalize(nlhs, plhs, nrhs-1, prhs+1);
    }
    else if (strcmp(func_name, "quat_to_rotm") == 0) {
        handle_quat_to_rotm(nlhs, plhs, nrhs-1, prhs+1);
    }
    else if (strcmp(func_name, "quat_from_euler") == 0) {
        handle_quat_from_euler(nlhs, plhs, nrhs-1, prhs+1);
    }
    else if (strcmp(func_name, "quat_to_euler") == 0) {
        handle_quat_to_euler(nlhs, plhs, nrhs-1, prhs+1);
    }
    // StateValidator
    else if (strcmp(func_name, "clip_velocity") == 0) {
        handle_clip_velocity(nlhs, plhs, nrhs-1, prhs+1);
    }
    else if (strcmp(func_name, "validate_covariance") == 0) {
        handle_validate_covariance(nlhs, plhs, nrhs-1, prhs+1);
    }
    // SensorFilter
    else if (strcmp(func_name, "lowpass_filter") == 0) {
        handle_lowpass_filter(nlhs, plhs, nrhs-1, prhs+1);
    }
    else if (strcmp(func_name, "detect_outlier") == 0) {
        handle_detect_outlier(nlhs, plhs, nrhs-1, prhs+1);
    }
    else {
        mexErrMsgIdAndTxt("mex_common_lib:unknownFunction",
            "Unknown function: %s", func_name);
    }
}

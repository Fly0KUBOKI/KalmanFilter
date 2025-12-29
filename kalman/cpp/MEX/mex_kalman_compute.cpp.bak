// mex_kalman_compute.cpp
// 統一されたMEXラッパー: 状態非依存の計算関数群
//
// 使用法:
//   output = mex_kalman_compute('function_name', input);
//
// すべての関数は以下の形式:
//   - 第1引数: 関数名 (文字列)
//   - 第2引数: input (数値配列)
//   - 出力: output (数値配列)

#include "mex.h"
#include "Common/Math/quaternion_compute.hpp"
#include "Common/Math/rotation_compute.hpp"
#include <string>
#include <cstring>
#include <vector>

using namespace kalman_compute;

// ===== ヘルパー関数 =====

// MATLAB配列 -> float配列変換
static std::vector<float> mat_to_float_vec(const mxArray* arr) {
    mwSize n = mxGetNumberOfElements(arr);
    std::vector<float> vec(n);
    if (!arr) {
        mexErrMsgTxt("Input array is null");
    }
    if (mxIsDouble(arr)) {
        mex_conv::mxArrayToFloatArray(arr, vec.data(), static_cast<size_t>(n));
    } else if (mxIsSingle(arr)) {
        // Direct copy from single data
        float* pr = static_cast<float*>(mxGetData(arr));
        if (!pr) mexErrMsgTxt("Input single array data is null");
        for (mwSize i = 0; i < n; ++i) vec[i] = pr[i];
    } else {
        mexErrMsgTxt("Input must be double or single");
    }
    
    return vec;
}

// float配列 -> MATLAB配列変換
static mxArray* float_vec_to_mat(const std::vector<float>& vec, mwSize rows, mwSize cols) {
    mxArray* arr = mxCreateDoubleMatrix(rows, cols, mxREAL);
    double* pr = mxGetPr(arr);
    
    for (mwSize i = 0; i < vec.size(); ++i) {
        pr[i] = static_cast<double>(vec[i]);
    }
    
    return arr;
}

// ===== 関数ハンドラー =====

// Quaternion関数群
static void handle_quat_multiply(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("quat_multiply: input must be 8x1");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 8) mexErrMsgTxt("quat_multiply: input must be 8x1");
    
    std::vector<float> output(4);
    QuaternionCompute::multiply(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 4, 1);
}

static void handle_quat_normalize(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("quat_normalize: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 4) mexErrMsgTxt("quat_normalize: input must be 4x1");
    
    std::vector<float> output(4);
    QuaternionCompute::normalize(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 4, 1);
}

static void handle_quat_conjugate(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("quat_conjugate: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 4) mexErrMsgTxt("quat_conjugate: input must be 4x1");
    
    std::vector<float> output(4);
    QuaternionCompute::conjugate(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 4, 1);
}

static void handle_quat_inverse(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("quat_inverse: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 4) mexErrMsgTxt("quat_inverse: input must be 4x1");
    
    std::vector<float> output(4);
    QuaternionCompute::inverse(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 4, 1);
}

static void handle_quat_to_rotation_matrix(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("quat_to_rotation_matrix: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 4) mexErrMsgTxt("quat_to_rotation_matrix: input must be 4x1");
    
    std::vector<float> output(9);
    QuaternionCompute::to_rotation_matrix(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 3, 3);
}

static void handle_quat_to_euler(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("quat_to_euler: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 4) mexErrMsgTxt("quat_to_euler: input must be 4x1");
    
    std::vector<float> output(3);
    QuaternionCompute::to_euler(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 3, 1);
}

static void handle_quat_from_euler(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("quat_from_euler: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 3) mexErrMsgTxt("quat_from_euler: input must be 3x1");
    
    std::vector<float> output(4);
    QuaternionCompute::from_euler(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 4, 1);
}

static void handle_quat_from_small_angle(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("quat_from_small_angle: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 3) mexErrMsgTxt("quat_from_small_angle: input must be 3x1");
    
    std::vector<float> output(4);
    QuaternionCompute::from_small_angle(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 4, 1);
}

static void handle_quat_integrate(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("quat_integrate: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 8) mexErrMsgTxt("quat_integrate: input must be [q(4); omega(3); dt(1)]");
    
    std::vector<float> output(4);
    QuaternionCompute::integrate(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 4, 1);
}

static void handle_quat_angle_between(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("quat_angle_between: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 8) mexErrMsgTxt("quat_angle_between: input must be 8x1");
    
    std::vector<float> output(1);
    QuaternionCompute::angle_between(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 1, 1);
}

static void handle_quat_slerp(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("quat_slerp: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 9) mexErrMsgTxt("quat_slerp: input must be 9x1");
    
    std::vector<float> output(4);
    QuaternionCompute::slerp(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 4, 1);
}

static void handle_quat_from_axis_angle(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("quat_from_axis_angle: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 4) mexErrMsgTxt("quat_from_axis_angle: input must be 4x1");
    
    std::vector<float> output(4);
    QuaternionCompute::from_axis_angle(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 4, 1);
}

static void handle_quat_to_axis_angle(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("quat_to_axis_angle: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 4) mexErrMsgTxt("quat_to_axis_angle: input must be 4x1");
    
    std::vector<float> output(4);
    QuaternionCompute::to_axis_angle(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 4, 1);
}

static void handle_quat_dot(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("quat_dot: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 8) mexErrMsgTxt("quat_dot: input must be 8x1");
    
    std::vector<float> output(1);
    QuaternionCompute::dot(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 1, 1);
}

// Rotation関数群
static void handle_rot_skew_symmetric(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("rot_skew_symmetric: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 3) mexErrMsgTxt("rot_skew_symmetric: input must be 3x1");
    
    std::vector<float> output(9);
    RotationCompute::skew_symmetric(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 3, 3);
}

static void handle_rot_rotation_x(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("rot_rotation_x: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 1) mexErrMsgTxt("rot_rotation_x: input must be scalar");
    
    std::vector<float> output(9);
    RotationCompute::rotation_x(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 3, 3);
}

static void handle_rot_rotation_y(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("rot_rotation_y: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 1) mexErrMsgTxt("rot_rotation_y: input must be scalar");
    
    std::vector<float> output(9);
    RotationCompute::rotation_y(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 3, 3);
}

static void handle_rot_rotation_z(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("rot_rotation_z: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 1) mexErrMsgTxt("rot_rotation_z: input must be scalar");
    
    std::vector<float> output(9);
    RotationCompute::rotation_z(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 3, 3);
}

static void handle_rot_from_euler(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("rot_from_euler: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 3) mexErrMsgTxt("rot_from_euler: input must be 3x1");
    
    std::vector<float> output(9);
    RotationCompute::from_euler(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 3, 3);
}

static void handle_rot_to_euler(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("rot_to_euler: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 9) mexErrMsgTxt("rot_to_euler: input must be 9x1 or 3x3");
    
    std::vector<float> output(3);
    RotationCompute::to_euler(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 3, 1);
}

static void handle_rot_rodrigues(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("rot_rodrigues: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 4) mexErrMsgTxt("rot_rodrigues: input must be 4x1");
    
    std::vector<float> output(9);
    RotationCompute::rodrigues(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 3, 3);
}

static void handle_rot_orthonormalize(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("rot_orthonormalize: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 9) mexErrMsgTxt("rot_orthonormalize: input must be 9x1 or 3x3");
    
    std::vector<float> output(9);
    RotationCompute::orthonormalize(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 3, 3);
}

static void handle_rot_inverse(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("rot_inverse: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 9) mexErrMsgTxt("rot_inverse: input must be 9x1 or 3x3");
    
    std::vector<float> output(9);
    RotationCompute::inverse(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 3, 3);
}

static void handle_rot_is_valid(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("rot_is_valid: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 9) mexErrMsgTxt("rot_is_valid: input must be 9x1 or 3x3");
    
    std::vector<float> output(1);
    RotationCompute::is_valid(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 1, 1);
}

static void handle_rot_apply_rotation(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("rot_apply_rotation: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 12) mexErrMsgTxt("rot_apply_rotation: input must be 12x1");
    
    std::vector<float> output(3);
    RotationCompute::apply_rotation(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 3, 1);
}

static void handle_rot_compose(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 2) mexErrMsgTxt("rot_compose: input required");
    
    auto input = mat_to_float_vec(prhs[1]);
    if (input.size() != 18) mexErrMsgTxt("rot_compose: input must be 18x1");
    
    std::vector<float> output(9);
    RotationCompute::compose(input.data(), output.data());
    
    plhs[0] = float_vec_to_mat(output, 3, 3);
}

// ===== MEX Entry Point =====

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgTxt("Usage: output = mex_kalman_compute('function_name', input)");
    }
    
    // 関数名取得
    char func_name[128];
    if (mxGetString(prhs[0], func_name, sizeof(func_name)) != 0) {
        mexErrMsgTxt("First argument must be a function name string");
    }
    
    // 関数ディスパッチ
    // Quaternion functions
    if (strcmp(func_name, "quat_multiply") == 0) {
        handle_quat_multiply(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "quat_normalize") == 0) {
        handle_quat_normalize(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "quat_conjugate") == 0) {
        handle_quat_conjugate(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "quat_inverse") == 0) {
        handle_quat_inverse(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "quat_to_rotation_matrix") == 0) {
        handle_quat_to_rotation_matrix(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "quat_to_euler") == 0) {
        handle_quat_to_euler(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "quat_from_euler") == 0) {
        handle_quat_from_euler(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "quat_from_small_angle") == 0) {
        handle_quat_from_small_angle(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "quat_integrate") == 0) {
        handle_quat_integrate(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "quat_angle_between") == 0) {
        handle_quat_angle_between(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "quat_slerp") == 0) {
        handle_quat_slerp(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "quat_from_axis_angle") == 0) {
        handle_quat_from_axis_angle(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "quat_to_axis_angle") == 0) {
        handle_quat_to_axis_angle(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "quat_dot") == 0) {
        handle_quat_dot(nlhs, plhs, nrhs, prhs);
    }
    // Rotation functions
    else if (strcmp(func_name, "rot_skew_symmetric") == 0) {
        handle_rot_skew_symmetric(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "rot_rotation_x") == 0) {
        handle_rot_rotation_x(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "rot_rotation_y") == 0) {
        handle_rot_rotation_y(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "rot_rotation_z") == 0) {
        handle_rot_rotation_z(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "rot_from_euler") == 0) {
        handle_rot_from_euler(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "rot_to_euler") == 0) {
        handle_rot_to_euler(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "rot_rodrigues") == 0) {
        handle_rot_rodrigues(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "rot_orthonormalize") == 0) {
        handle_rot_orthonormalize(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "rot_inverse") == 0) {
        handle_rot_inverse(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "rot_is_valid") == 0) {
        handle_rot_is_valid(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "rot_apply_rotation") == 0) {
        handle_rot_apply_rotation(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "rot_compose") == 0) {
        handle_rot_compose(nlhs, plhs, nrhs, prhs);
    }
    else {
        mexErrMsgIdAndTxt("mex_kalman_compute:unknownFunction",
                          "Unknown function: %s", func_name);
    }
}

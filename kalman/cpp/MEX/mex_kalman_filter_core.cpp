// mex_kalman_filter_core.cpp
// MATLAB MEX wrapper for kalman_filter_core functions
//
// Usage:
//   K = mex_kalman_filter_core('compute_kalman_gain', P, H, S)

#include "mex.h"
#include "../KF/Core/kalman_filter_core.hpp"
#include <string>
#include <cstring>
// Safe MEX <-> float conversions
#include "mex_type_conv.hpp"
#include <vector>

// Helper: MATLAB array -> Matrix
template<int R, int C, typename T = float>
static bool matToMatrix(const mxArray* arr, cmath_fx::Matrix<R, C, T>& out) {
    if (!arr) return false;
    mwSize rows = mxGetM(arr);
    mwSize cols = mxGetN(arr);
    if (rows != R || cols != C) return false;
    std::vector<T> tmp(static_cast<size_t>(R) * static_cast<size_t>(C));
    mex_conv::mxArrayToFloatArray(arr, tmp.data(), static_cast<size_t>(R) * static_cast<size_t>(C));
    for (int j = 0; j < C; ++j) {
        for (int i = 0; i < R; ++i) {
            out(i, j) = static_cast<T>(tmp[static_cast<size_t>(j) * static_cast<size_t>(R) + static_cast<size_t>(i)]);
        }
    }
    return true;
}

// Helper: Vector -> MATLAB array
template<int R, typename T>
static mxArray* vectorToMat(const cmath_fx::Vector<R, T>& v) {
    mxArray* out = mxCreateDoubleMatrix(R, 1, mxREAL);
    double* pr = mxGetPr(out);
    for (int i = 0; i < R; ++i) {
        pr[i] = static_cast<double>(v(i, 0));
    }
    return out;
}

// Helper: MATLAB array -> Vector
template<int R, typename T = float>
static bool matToVector(const mxArray* arr, cmath_fx::Vector<R, T>& out) {
    if (!arr) return false;
    mwSize rows = mxGetM(arr);
    mwSize cols = mxGetN(arr);
    if ((rows != R || cols != 1) && (rows != 1 || cols != R)) return false;
    std::vector<T> tmp(static_cast<size_t>(R));
    mex_conv::mxArrayToFloatArray(arr, tmp.data(), static_cast<size_t>(R));
    for (int i = 0; i < R; ++i) {
        out(i, 0) = static_cast<T>(tmp[static_cast<size_t>(i)]);
    }
    return true;
}

// Helper: Matrix -> MATLAB array
template<int R, int C, typename T>
static mxArray* matrixToMat(const cmath_fx::Matrix<R, C, T>& M) {
    mxArray* out = mxCreateDoubleMatrix(R, C, mxREAL);
    double* pr = mxGetPr(out);
    for (int j = 0; j < C; ++j) {
        for (int i = 0; i < R; ++i) {
            pr[j * R + i] = static_cast<double>(M(i, j));
        }
    }
    return out;
}

// Dynamic size helper
static bool matToMatrixDynamic(const mxArray* arr, int& rows, int& cols, float* data, int max_size) {
    if (!arr) return false;
    rows = (int)mxGetM(arr);
    cols = (int)mxGetN(arr);
    if (rows * cols > max_size) return false;
    std::vector<float> tmp(static_cast<size_t>(rows) * static_cast<size_t>(cols));
    mex_conv::mxArrayToFloatArray(arr, tmp.data(), static_cast<size_t>(rows) * static_cast<size_t>(cols));
    for (int j = 0; j < cols; ++j) {
        for (int i = 0; i < rows; ++i) {
            data[i * cols + j] = tmp[static_cast<size_t>(j) * static_cast<size_t>(rows) + static_cast<size_t>(i)];
        }
    }
    return true;
}

static mxArray* matrixDynamicToMat(int rows, int cols, const float* data) {
    mxArray* out = mxCreateDoubleMatrix(rows, cols, mxREAL);
    double* pr = mxGetPr(out);
    for (int j = 0; j < cols; ++j) {
        for (int i = 0; i < rows; ++i) {
            pr[j * rows + i] = static_cast<double>(data[i * cols + j]);
        }
    }
    return out;
}

// Handler: compute_kalman_gain
// K = P * H' * inv(S)
// P: NxN, H: MxN, S: MxM -> K: NxM
static void handle_compute_kalman_gain(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 4) {
        mexErrMsgTxt("compute_kalman_gain: Expected 3 arguments (P, H, S)");
    }
    if (nlhs > 1) {
        mexErrMsgTxt("compute_kalman_gain: Only 1 output (K)");
    }

    // Get dimensions
    int N = (int)mxGetM(prhs[1]);  // P is NxN
    int M = (int)mxGetM(prhs[2]);  // H is MxN
    
    // Support common sizes
    if (N == 15 && M == 3) {
        cmath_fx::Matrix<15, 15, float> P;
        cmath_fx::Matrix<3, 15, float> H;
        cmath_fx::Matrix<3, 3, float> S;
        
        if (!matToMatrix(prhs[1], P)) mexErrMsgTxt("Failed to read P");
        if (!matToMatrix(prhs[2], H)) mexErrMsgTxt("Failed to read H");
        if (!matToMatrix(prhs[3], S)) mexErrMsgTxt("Failed to read S");
        
        auto K = kf::KalmanFilterCore::compute_kalman_gain<15, 3, float>(P, H, S);
        plhs[0] = matrixToMat(K);
    } else if (N == 15 && M == 1) {
        cmath_fx::Matrix<15, 15, float> P;
        cmath_fx::Matrix<1, 15, float> H;
        cmath_fx::Matrix<1, 1, float> S;
        
        if (!matToMatrix(prhs[1], P)) mexErrMsgTxt("Failed to read P");
        if (!matToMatrix(prhs[2], H)) mexErrMsgTxt("Failed to read H");
        if (!matToMatrix(prhs[3], S)) mexErrMsgTxt("Failed to read S");
        
        auto K = kf::KalmanFilterCore::compute_kalman_gain<15, 1, float>(P, H, S);
        plhs[0] = matrixToMat(K);
    } else {
        mexErrMsgIdAndTxt("kalman_filter_core:unsupported_size",
                          "Unsupported matrix dimensions N=%d, M=%d", N, M);
    }
}

// Handler: compute_innovation_and_S
// [y, S, R_out] = compute_innovation_and_S(z, h, H, P_pred, R, params)
static void handle_compute_innovation_and_S(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 7) {
        mexErrMsgTxt("compute_innovation_and_S: Expected 6 arguments (z, h, H, P_pred, R, params)");
    }
    if (nlhs > 3) {
        mexErrMsgTxt("compute_innovation_and_S: Max 3 outputs (y, S, R_out)");
    }

    // Get dimensions
    int M = (int)mxGetM(prhs[1]);  // z is Mx1
    int N = (int)mxGetN(prhs[3]);  // H is MxN, P_pred is NxN
    
    // Support common sizes: N=15, M=1 (baro) or M=3 (GPS)
    if (N == 15 && M == 1) {
        cmath_fx::Vector<1, float> z, h, y;
        cmath_fx::Matrix<1, 15, float> H;
        cmath_fx::Matrix<15, 15, float> P_pred;
        cmath_fx::Matrix<1, 1, float> R, S, R_out;
        
        if (!matToVector(prhs[1], z)) mexErrMsgTxt("Failed to read z");
        if (!matToVector(prhs[2], h)) mexErrMsgTxt("Failed to read h");
        if (!matToMatrix(prhs[3], H)) mexErrMsgTxt("Failed to read H");
        if (!matToMatrix(prhs[4], P_pred)) mexErrMsgTxt("Failed to read P_pred");
        if (!matToMatrix(prhs[5], R)) mexErrMsgTxt("Failed to read R");
        
        kf::KalmanFilterCore::compute_innovation_and_S(z, h, H, P_pred, R, y, S, R_out);
        
        plhs[0] = vectorToMat(y);
        if (nlhs > 1) plhs[1] = matrixToMat(S);
        if (nlhs > 2) plhs[2] = matrixToMat(R_out);
    } else if (N == 15 && M == 3) {
        cmath_fx::Vector<3, float> z, h, y;
        cmath_fx::Matrix<3, 15, float> H;
        cmath_fx::Matrix<15, 15, float> P_pred;
        cmath_fx::Matrix<3, 3, float> R, S, R_out;
        
        if (!matToVector(prhs[1], z)) mexErrMsgTxt("Failed to read z");
        if (!matToVector(prhs[2], h)) mexErrMsgTxt("Failed to read h");
        if (!matToMatrix(prhs[3], H)) mexErrMsgTxt("Failed to read H");
        if (!matToMatrix(prhs[4], P_pred)) mexErrMsgTxt("Failed to read P_pred");
        if (!matToMatrix(prhs[5], R)) mexErrMsgTxt("Failed to read R");
        
        kf::KalmanFilterCore::compute_innovation_and_S(z, h, H, P_pred, R, y, S, R_out);
        
        plhs[0] = vectorToMat(y);
        if (nlhs > 1) plhs[1] = matrixToMat(S);
        if (nlhs > 2) plhs[2] = matrixToMat(R_out);
    } else {
        mexErrMsgIdAndTxt("kalman_filter_core:unsupported_size",
                          "Unsupported dimensions N=%d, M=%d", N, M);
    }
}

// Handler: update_state_covariance
// [x_upd, P_upd] = update_state_covariance(x_pred, P_pred, K, H, y, R)
static void handle_update_state_covariance(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 7) {
        mexErrMsgTxt("update_state_covariance: Expected 6 arguments (x_pred, P_pred, K, H, y, R)");
    }
    if (nlhs > 2) {
        mexErrMsgTxt("update_state_covariance: Max 2 outputs (x_upd, P_upd)");
    }

    int N = (int)mxGetM(prhs[1]);  // x_pred is Nx1
    int M = (int)mxGetM(prhs[5]);  // y is Mx1
    
    // Support common sizes
    if (N == 15 && M == 1) {
        cmath_fx::Vector<15, float> x_pred, x_upd;
        cmath_fx::Matrix<15, 15, float> P_pred, P_upd;
        cmath_fx::Matrix<15, 1, float> K;
        cmath_fx::Matrix<1, 15, float> H;
        cmath_fx::Vector<1, float> y;
        cmath_fx::Matrix<1, 1, float> R;
        
        if (!matToVector(prhs[1], x_pred)) mexErrMsgTxt("Failed to read x_pred");
        if (!matToMatrix(prhs[2], P_pred)) mexErrMsgTxt("Failed to read P_pred");
        if (!matToMatrix(prhs[3], K)) mexErrMsgTxt("Failed to read K");
        if (!matToMatrix(prhs[4], H)) mexErrMsgTxt("Failed to read H");
        if (!matToVector(prhs[5], y)) mexErrMsgTxt("Failed to read y");
        if (!matToMatrix(prhs[6], R)) mexErrMsgTxt("Failed to read R");
        
        kf::KalmanFilterCore::update_state_covariance(x_pred, P_pred, K, H, y, R, x_upd, P_upd);
        
        plhs[0] = vectorToMat(x_upd);
        if (nlhs > 1) plhs[1] = matrixToMat(P_upd);
    } else if (N == 15 && M == 3) {
        cmath_fx::Vector<15, float> x_pred, x_upd;
        cmath_fx::Matrix<15, 15, float> P_pred, P_upd;
        cmath_fx::Matrix<15, 3, float> K;
        cmath_fx::Matrix<3, 15, float> H;
        cmath_fx::Vector<3, float> y;
        cmath_fx::Matrix<3, 3, float> R;
        
        if (!matToVector(prhs[1], x_pred)) mexErrMsgTxt("Failed to read x_pred");
        if (!matToMatrix(prhs[2], P_pred)) mexErrMsgTxt("Failed to read P_pred");
        if (!matToMatrix(prhs[3], K)) mexErrMsgTxt("Failed to read K");
        if (!matToMatrix(prhs[4], H)) mexErrMsgTxt("Failed to read H");
        if (!matToVector(prhs[5], y)) mexErrMsgTxt("Failed to read y");
        if (!matToMatrix(prhs[6], R)) mexErrMsgTxt("Failed to read R");
        
        kf::KalmanFilterCore::update_state_covariance(x_pred, P_pred, K, H, y, R, x_upd, P_upd);
        
        plhs[0] = vectorToMat(x_upd);
        if (nlhs > 1) plhs[1] = matrixToMat(P_upd);
    } else {
        mexErrMsgIdAndTxt("kalman_filter_core:unsupported_size",
                          "Unsupported dimensions N=%d, M=%d", N, M);
    }
}

// Main MEX entry point
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgTxt("Usage: mex_kalman_filter_core(function_name, ...)");
    }
    
    if (!mxIsChar(prhs[0])) {
        mexErrMsgTxt("First argument must be a string (function name)");
    }
    
    char func_name[256];
    if (mxGetString(prhs[0], func_name, sizeof(func_name)) != 0) {
        mexErrMsgTxt("Failed to read function name");
    }
    
    if (strcmp(func_name, "compute_kalman_gain") == 0) {
        handle_compute_kalman_gain(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "compute_innovation_and_S") == 0) {
        handle_compute_innovation_and_S(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "update_state_covariance") == 0) {
        handle_update_state_covariance(nlhs, plhs, nrhs, prhs);
    } else {
        mexErrMsgIdAndTxt("kalman_filter_core:unknown_function",
                          "Unknown function: %s", func_name);
    }
}

// mex_eskf_zupt.cpp
// MEX wrapper for ZUPT (Zero Velocity Update)
// Implements: velocity zeroing and covariance update using Kalman filter update
// Based on meukf_core.cpp::update_zupt() implementation

#include "mex.h"
#include "mex_type_conv.hpp"
#include "../Inc/Common/Math/fixed_matrix.hpp"
#include "../Inc/Common/Math/quaternion.hpp"
#include <string>
#include <cmath>
#include <vector>

using namespace cmath_fx;

// Type aliases
using Vector3 = Vector<3, float>;
using Vector4 = Vector<4, float>;
using Vector15 = Vector<15, float>;
using Matrix3x3 = Matrix<3, 3, float>;
using Matrix15x3 = Matrix<15, 3, float>;
using Matrix15x15 = Matrix<15, 15, float>;

// Helper: MATLAB array -> Vector
template<int R>
static bool matToVector(const mxArray* arr, Vector<R, float>& out) {
    if (!arr) return false;
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    mwSize rows = mxGetM(arr); mwSize cols = mxGetN(arr);
    if (rows != R || cols != 1) return false;
    std::vector<float> tmp(static_cast<size_t>(R));
    mex_conv::mxArrayToFloatArray(arr, tmp.data(), static_cast<size_t>(R));
    for (int i = 0; i < R; ++i) out(i, 0) = tmp[i];
    return true;
}

// Helper: MATLAB array -> Matrix
template<int R, int C>
static bool matToMatrix(const mxArray* arr, Matrix<R, C, float>& out) {
    if (!arr) return false;
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    mwSize rows = mxGetM(arr); mwSize cols = mxGetN(arr);
    if (rows != R || cols != C) return false;
    std::vector<float> tmp(static_cast<size_t>(R) * static_cast<size_t>(C));
    mex_conv::mxArrayToFloatArray(arr, tmp.data(), static_cast<size_t>(R) * static_cast<size_t>(C));
    for (int j = 0; j < C; ++j) {
        for (int i = 0; i < R; ++i) {
            out(i, j) = tmp[static_cast<size_t>(j) * static_cast<size_t>(R) + static_cast<size_t>(i)];
        }
    }
    return true;
}

// Helper: Vector -> MATLAB array
template<int R>
static mxArray* vectorToMat(const Vector<R, float>& v) {
    mxArray* out = mxCreateDoubleMatrix(R, 1, mxREAL);
    double* pr = mxGetPr(out);
    for (int i = 0; i < R; ++i) pr[i] = static_cast<double>(v(i, 0));
    return out;
}

// Helper: Matrix -> MATLAB array
template<int R, int C>
static mxArray* matrixToMat(const Matrix<R, C, float>& M) {
    mxArray* out = mxCreateDoubleMatrix(R, C, mxREAL);
    double* pr = mxGetPr(out);
    for (int j = 0; j < C; ++j) {
        for (int i = 0; i < R; ++i) {
            pr[j * R + i] = static_cast<double>(M(i, j));
        }
    }
    return out;
}

// 3x3 matrix inverse using Gaussian elimination
static bool invert3x3(const Matrix3x3& A, Matrix3x3& A_inv) {
    // Compute determinant
    float det = A(0,0) * (A(1,1)*A(2,2) - A(2,1)*A(1,2))
              - A(0,1) * (A(1,0)*A(2,2) - A(2,0)*A(1,2))
              + A(0,2) * (A(1,0)*A(2,1) - A(2,0)*A(1,1));
    
    if (std::abs(det) < 1e-10f) return false;
    
    float inv_det = 1.0f / det;
    
    // Compute adjugate matrix
    A_inv(0,0) = (A(1,1)*A(2,2) - A(2,1)*A(1,2)) * inv_det;
    A_inv(0,1) = (A(0,2)*A(2,1) - A(0,1)*A(2,2)) * inv_det;
    A_inv(0,2) = (A(0,1)*A(1,2) - A(0,2)*A(1,1)) * inv_det;
    A_inv(1,0) = (A(1,2)*A(2,0) - A(1,0)*A(2,2)) * inv_det;
    A_inv(1,1) = (A(0,0)*A(2,2) - A(0,2)*A(2,0)) * inv_det;
    A_inv(1,2) = (A(1,0)*A(0,2) - A(0,0)*A(1,2)) * inv_det;
    A_inv(2,0) = (A(1,0)*A(2,1) - A(2,0)*A(1,1)) * inv_det;
    A_inv(2,1) = (A(2,0)*A(0,1) - A(0,0)*A(2,1)) * inv_det;
    A_inv(2,2) = (A(0,0)*A(1,1) - A(1,0)*A(0,1)) * inv_det;
    
    return true;
}

// Main ZUPT update function
// Input: v_in (3x1), P_in (15x15)
// Output: v_out (3x1), P_out (15x15)
// Based on meukf_core.cpp::update_zupt() but simplified to work with v and P only
static void handle_update(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 3) mexErrMsgTxt("update requires v_in and P_in");
    
    // Parse inputs
    Vector3 v_in;
    if (!matToVector<3>(prhs[1], v_in)) mexErrMsgTxt("v_in must be a 3x1 double vector");
    
    Matrix15x15 P_in;
    if (!matToMatrix<15, 15>(prhs[2], P_in)) mexErrMsgTxt("P_in must be a 15x15 double matrix");
    
    // ZUPT: Observe velocity = 0
    // z = [0;0;0], h = v
    // y = z - h = -v
    Vector3 y;
    y(0, 0) = -v_in(0, 0);
    y(1, 0) = -v_in(1, 0);
    y(2, 0) = -v_in(2, 0);
    
    // H = [0, I, 0, 0, 0] (observation matrix)
    // S = H*P*H' + R
    // H*P*H' is simply the velocity block of P (indices 3,4,5)
    Matrix3x3 P_vv;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_vv(i, j) = P_in(3 + i, 3 + j);
        }
    }
    
    // R: ZUPT noise covariance (diagonal)
    // MATLAB実装では noise_zupt = [0.01^2; 0.01^2; 0.01^2] = [0.0001; 0.0001; 0.0001]
    Matrix3x3 R = Matrix3x3::Zero();
    float noise_zupt[3] = {0.0001f, 0.0001f, 0.0001f}; // ZUPT noise variance (0.01^2)
    for (int i = 0; i < 3; ++i) {
        R(i, i) = noise_zupt[i];
    }
    
    // S = P_vv + R
    Matrix3x3 S = P_vv + R;
    
    // Invert S
    Matrix3x3 S_inv;
    bool S_is_singular = !invert3x3(S, S_inv);
    
    Vector3 v_out;
    Matrix15x15 P_out;
    
    if (S_is_singular) {
        // Fallback: Sが特異な場合は、単純に速度を0にして共分散を減らす
        // mex_filter_management.cppのapply_zuptと同様の処理
        v_out(0, 0) = 0.0f;
        v_out(1, 0) = 0.0f;
        v_out(2, 0) = 0.0f;
        
        // P_out = P_in (copy)
        P_out = P_in;
        
        // Reduce velocity variances by factor (indices 3,4,5)
        float factor = 0.01f;
        P_out(3, 3) *= factor;
        P_out(4, 4) *= factor;
        P_out(5, 5) *= factor;
        
        // Also update off-diagonal elements for symmetry
        for (int i = 0; i < 15; ++i) {
            if (i != 3) {
                float val = 0.5f * (P_out(i, 3) + P_out(3, i));
                P_out(i, 3) = val * factor;
                P_out(3, i) = val * factor;
            }
            if (i != 4) {
                float val = 0.5f * (P_out(i, 4) + P_out(4, i));
                P_out(i, 4) = val * factor;
                P_out(4, i) = val * factor;
            }
            if (i != 5) {
                float val = 0.5f * (P_out(i, 5) + P_out(5, i));
                P_out(i, 5) = val * factor;
                P_out(5, i) = val * factor;
            }
        }
    } else {
        // Normal Kalman filter update
        // K = P * H' * S_inv
        // P * H' is the block of columns 3,4,5 of P
        Matrix15x3 PHt;
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 3; ++j) {
                PHt(i, j) = P_in(i, 3 + j);
            }
        }
        
        // K = PHt * S_inv (meukf_core.cppと同じ実装)
        Matrix15x3 K = PHt * S_inv;
        
        // dx = K * y
        Vector15 dx = K * y;
        
        // Update velocity: v_out = v_in + dx[3:5]
        // meukf_core.cppと同じ: v(0,0) += dx(3,0); v(1,0) += dx(4,0); v(2,0) += dx(5,0);
        v_out(0, 0) = v_in(0, 0) + dx(3, 0);
        v_out(1, 0) = v_in(1, 0) + dx(4, 0);
        v_out(2, 0) = v_in(2, 0) + dx(5, 0);
        
        // Update Covariance: P = (I - K*H) * P
        // meukf_core.cppと同じ実装
        // K*H is 15x15, but only columns 3,4,5 are non-zero (equal to K)
        Matrix15x15 KH = Matrix15x15::Zero();
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 3; ++j) {
                KH(i, 3 + j) = K(i, j);
            }
        }
        
        Matrix15x15 I_mat = Matrix15x15::Identity();
        P_out = (I_mat - KH) * P_in;
        
        // Symmetrize P_out (meukf_core.cppと同じ)
        for (int i = 0; i < 15; ++i) {
            for (int j = i + 1; j < 15; ++j) {
                float val = 0.5f * (P_out(i, j) + P_out(j, i));
                P_out(i, j) = val;
                P_out(j, i) = val;
            }
        }
    }
    
    // Output
    plhs[0] = vectorToMat<3>(v_out);
    plhs[1] = matrixToMat<15, 15>(P_out);
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if (nrhs < 1) mexErrMsgTxt("Usage: [v_out, P_out] = mex_eskf_zupt('update', v_in, P_in)");
    
    char cmd[128];
    if (mxGetString(prhs[0], cmd, sizeof(cmd))) mexErrMsgTxt("Command must be string");
    
    std::string cmdstr(cmd);
    
    if (cmdstr == "update") {
        if (nlhs < 2) mexErrMsgTxt("update requires 2 outputs: v_out, P_out");
        handle_update(nlhs, plhs, nrhs, prhs);
    } else {
        mexErrMsgTxt("Unknown command. Use 'update'");
    }
}


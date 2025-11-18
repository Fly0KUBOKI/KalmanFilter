// mex_kalman_filter_core.cpp
// MATLAB MEX wrapper for kalman_filter_core functions
// Unified interface matching MATLAB kalman_filter_core.m
//
// Usage:
//   P = mex_kalman_filter_core('predict_step', P, q, a_meas, ba, w_meas, bg, Q, dt)
//   K = mex_kalman_filter_core('compute_kalman_gain', P_pred, H, S)
//   [x_upd, P_upd] = mex_kalman_filter_core('update_state_covariance', x_pred, P_pred, K, H, y, R)
//   [y, S, R_out] = mex_kalman_filter_core('compute_innovation_and_S', z, h, H, P_pred, R)
//   F = mex_kalman_filter_core('compute_jacobian', q, a_meas, ba, dt)

#include "mex.h"
#include "../KF/Core/kalman_filter_core.hpp"
#include "../Common/Math/fixed_matrix.hpp"
#include <string>
#include <cstring>

using cm = cmath_fx::FixedMatrix;

// Helper: MATLAB array -> FixedMatrix
static bool matToFixed(const mxArray* arr, cm& out) {
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    mwSize rows = mxGetM(arr);
    mwSize cols = mxGetN(arr);
    if (rows > cmath_fx::MAX_N || cols > cmath_fx::MAX_N) return false;
    double* pr = mxGetPr(arr);
    out.resize((int)rows, (int)cols);
    for (mwSize j = 0; j < cols; ++j) {
        for (mwSize i = 0; i < rows; ++i) {
            out((int)i, (int)j) = static_cast<float>(pr[j * rows + i]);
        }
    }
    return true;
}

// Helper: FixedMatrix -> MATLAB array
static mxArray* fixedToMat(const cm& M) {
    mwSize rows = (mwSize)M.rows;
    mwSize cols = (mwSize)M.cols;
    mxArray* out = mxCreateDoubleMatrix(rows, cols, mxREAL);
    double* pr = mxGetPr(out);
    for (mwSize j = 0; j < cols; ++j) {
        for (mwSize i = 0; i < rows; ++i) {
            pr[j * rows + i] = static_cast<double>(M((int)i, (int)j));
        }
    }
    return out;
}

// Helper: get scalar from mxArray
static double getScalar(const mxArray* arr) {
    if (!mxIsDouble(arr) || mxIsComplex(arr) || mxGetNumberOfElements(arr) != 1) {
        mexErrMsgTxt("Expected scalar double");
    }
    return mxGetScalar(arr);
}

// Handler: predict_step
static void handle_predict_step(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Expected: P, q, a_meas, ba, w_meas, bg, Q, dt (8 args)
    if (nrhs != 9) {
        mexErrMsgTxt("predict_step: Expected 8 arguments after function name");
    }
    if (nlhs > 1) {
        mexErrMsgTxt("predict_step: Only 1 output (P)");
    }

    cm P, q, a_meas, ba, w_meas, bg, Q;
    if (!matToFixed(prhs[1], P)) mexErrMsgTxt("predict_step: Failed to read P");
    if (!matToFixed(prhs[2], q)) mexErrMsgTxt("predict_step: Failed to read q");
    if (!matToFixed(prhs[3], a_meas)) mexErrMsgTxt("predict_step: Failed to read a_meas");
    if (!matToFixed(prhs[4], ba)) mexErrMsgTxt("predict_step: Failed to read ba");
    if (!matToFixed(prhs[5], w_meas)) mexErrMsgTxt("predict_step: Failed to read w_meas");
    if (!matToFixed(prhs[6], bg)) mexErrMsgTxt("predict_step: Failed to read bg");
    if (!matToFixed(prhs[7], Q)) mexErrMsgTxt("predict_step: Failed to read Q");
    float dt = static_cast<float>(getScalar(prhs[8]));

    cm P_out = kf::KalmanFilterCore::predict_step(P, q, a_meas, ba, w_meas, bg, Q, dt);
    plhs[0] = fixedToMat(P_out);
}

// Handler: compute_kalman_gain
static void handle_compute_kalman_gain(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Expected: P_pred, H, S (3 args)
    if (nrhs != 4) {
        mexErrMsgTxt("compute_kalman_gain: Expected 3 arguments after function name");
    }
    if (nlhs > 1) {
        mexErrMsgTxt("compute_kalman_gain: Only 1 output (K)");
    }

    cm P_pred, H, S;
    if (!matToFixed(prhs[1], P_pred)) mexErrMsgTxt("compute_kalman_gain: Failed to read P_pred");
    if (!matToFixed(prhs[2], H)) mexErrMsgTxt("compute_kalman_gain: Failed to read H");
    if (!matToFixed(prhs[3], S)) mexErrMsgTxt("compute_kalman_gain: Failed to read S");

    cm K = kf::KalmanFilterCore::compute_kalman_gain(P_pred, H, S);
    plhs[0] = fixedToMat(K);
}

// Handler: update_state_covariance
static void handle_update_state_covariance(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Expected: x_pred, P_pred, K, H, y, R (6 args)
    if (nrhs != 7) {
        mexErrMsgTxt("update_state_covariance: Expected 6 arguments after function name");
    }
    if (nlhs > 2) {
        mexErrMsgTxt("update_state_covariance: Maximum 2 outputs (x_upd, P_upd)");
    }

    cm x_pred, P_pred, K, H, y, R;
    if (!matToFixed(prhs[1], x_pred)) mexErrMsgTxt("update_state_covariance: Failed to read x_pred");
    if (!matToFixed(prhs[2], P_pred)) mexErrMsgTxt("update_state_covariance: Failed to read P_pred");
    if (!matToFixed(prhs[3], K)) mexErrMsgTxt("update_state_covariance: Failed to read K");
    if (!matToFixed(prhs[4], H)) mexErrMsgTxt("update_state_covariance: Failed to read H");
    if (!matToFixed(prhs[5], y)) mexErrMsgTxt("update_state_covariance: Failed to read y");
    if (!matToFixed(prhs[6], R)) mexErrMsgTxt("update_state_covariance: Failed to read R");

    cm x_upd, P_upd;
    kf::KalmanFilterCore::update_state_covariance(x_upd, P_upd, x_pred, P_pred, K, H, y, R);
    
    plhs[0] = fixedToMat(x_upd);
    if (nlhs > 1) {
        plhs[1] = fixedToMat(P_upd);
    }
}

// Handler: compute_innovation_and_S
static void handle_compute_innovation_and_S(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Expected: z, h, H, P_pred, R (5 args, params is ignored for now)
    if (nrhs < 6) {
        mexErrMsgTxt("compute_innovation_and_S: Expected at least 5 arguments after function name");
    }
    if (nlhs > 3) {
        mexErrMsgTxt("compute_innovation_and_S: Maximum 3 outputs (y, S, R_out)");
    }

    cm z, h, H, P_pred, R;
    if (!matToFixed(prhs[1], z)) mexErrMsgTxt("compute_innovation_and_S: Failed to read z");
    if (!matToFixed(prhs[2], h)) mexErrMsgTxt("compute_innovation_and_S: Failed to read h");
    if (!matToFixed(prhs[3], H)) mexErrMsgTxt("compute_innovation_and_S: Failed to read H");
    if (!matToFixed(prhs[4], P_pred)) mexErrMsgTxt("compute_innovation_and_S: Failed to read P_pred");
    if (!matToFixed(prhs[5], R)) mexErrMsgTxt("compute_innovation_and_S: Failed to read R");

    cm y_out, S_out, R_out;
    kf::KalmanFilterCore::compute_innovation_and_S(y_out, S_out, R_out, z, h, H, P_pred, R);
    
    plhs[0] = fixedToMat(y_out);
    if (nlhs > 1) {
        plhs[1] = fixedToMat(S_out);
    }
    if (nlhs > 2) {
        plhs[2] = fixedToMat(R_out);
    }
}

// Handler: compute_jacobian
static void handle_compute_jacobian(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Expected: q, a_meas, ba, dt (4 args)
    if (nrhs != 5) {
        mexErrMsgTxt("compute_jacobian: Expected 4 arguments after function name");
    }
    if (nlhs > 1) {
        mexErrMsgTxt("compute_jacobian: Only 1 output (F)");
    }

    cm q, a_meas, ba;
    if (!matToFixed(prhs[1], q)) mexErrMsgTxt("compute_jacobian: Failed to read q");
    if (!matToFixed(prhs[2], a_meas)) mexErrMsgTxt("compute_jacobian: Failed to read a_meas");
    if (!matToFixed(prhs[3], ba)) mexErrMsgTxt("compute_jacobian: Failed to read ba");
    float dt = static_cast<float>(getScalar(prhs[4]));

    cm F = kf::KalmanFilterCore::compute_jacobian(q, a_meas, ba, dt);
    plhs[0] = fixedToMat(F);
}

// Main MEX entry point
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgTxt("Usage: mex_kalman_filter_core(function_name, ...)");
    }

    // Get function name
    if (!mxIsChar(prhs[0])) {
        mexErrMsgTxt("First argument must be a string (function name)");
    }
    char func_name[256];
    if (mxGetString(prhs[0], func_name, sizeof(func_name)) != 0) {
        mexErrMsgTxt("Failed to read function name");
    }

    // Dispatch to appropriate handler
    if (strcmp(func_name, "predict_step") == 0) {
        handle_predict_step(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "compute_kalman_gain") == 0) {
        handle_compute_kalman_gain(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "update_state_covariance") == 0) {
        handle_update_state_covariance(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "compute_innovation_and_S") == 0) {
        handle_compute_innovation_and_S(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "compute_jacobian") == 0) {
        handle_compute_jacobian(nlhs, plhs, nrhs, prhs);
    } else {
        mexErrMsgIdAndTxt("mex_kalman_filter_core:unknownFunction",
                          "Unknown function: %s", func_name);
    }
}

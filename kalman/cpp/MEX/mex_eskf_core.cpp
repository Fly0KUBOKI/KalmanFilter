// mex_eskf_core.cpp
// MATLAB MEX wrapper for ESKF core functions
//
// Usage:
//   [p,v,q,ba,bg] = mex_eskf_core('integrate_nominal', p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_thr, accel_thr)
//   q = mex_eskf_core('update_accel', q, a_meas, scale_factor)
//   [q, P, K, dx] = mex_eskf_core('update_mag', q, P, m_meas, m_world, R_mag)
//   [p, v, P, K, dx] = mex_eskf_core('update_gps', p, v, P, gps_pos, gps_origin, R_gps)
//   [p, P, K, dx] = mex_eskf_core('update_baro', p, P, pressure, gps_origin, R_baro)

#include "mex.h"
#include "../ESKF/eskf_core.hpp"
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

// Helper: get scalar
static float getScalar(const mxArray* arr) {
    if (!mxIsDouble(arr) || mxIsComplex(arr) || mxGetNumberOfElements(arr) != 1) {
        mexErrMsgTxt("Expected scalar double");
    }
    return static_cast<float>(mxGetScalar(arr));
}

// Handler: integrate_nominal
static void handle_integrate_nominal(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Expected: p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_thr, accel_thr (11 args + 1 func name = 12)
    if (nrhs != 12) {
        mexErrMsgTxt("integrate_nominal: Expected 11 arguments after function name");
    }
    if (nlhs > 5) {
        mexErrMsgTxt("integrate_nominal: Maximum 5 outputs (p, v, q, ba, bg)");
    }
    
    cm p, v, q, ba, bg, a_meas, w_meas, g, gyro_thr, accel_thr;
    if (!matToFixed(prhs[1], p)) mexErrMsgTxt("Failed to read p");
    if (!matToFixed(prhs[2], v)) mexErrMsgTxt("Failed to read v");
    if (!matToFixed(prhs[3], q)) mexErrMsgTxt("Failed to read q");
    if (!matToFixed(prhs[4], ba)) mexErrMsgTxt("Failed to read ba");
    if (!matToFixed(prhs[5], bg)) mexErrMsgTxt("Failed to read bg");
    if (!matToFixed(prhs[6], a_meas)) mexErrMsgTxt("Failed to read a_meas");
    if (!matToFixed(prhs[7], w_meas)) mexErrMsgTxt("Failed to read w_meas");
    float dt = getScalar(prhs[8]);
    if (!matToFixed(prhs[9], g)) mexErrMsgTxt("Failed to read g");
    if (!matToFixed(prhs[10], gyro_thr)) mexErrMsgTxt("Failed to read gyro_thr");
    if (!matToFixed(prhs[11], accel_thr)) mexErrMsgTxt("Failed to read accel_thr");
    
    eskf::ESKFCore::integrate_nominal(p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_thr, accel_thr);
    
    plhs[0] = fixedToMat(p);
    if (nlhs > 1) plhs[1] = fixedToMat(v);
    if (nlhs > 2) plhs[2] = fixedToMat(q);
    if (nlhs > 3) plhs[3] = fixedToMat(ba);
    if (nlhs > 4) plhs[4] = fixedToMat(bg);
}

// Handler: update_accel
static void handle_update_accel(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Expected: q, a_meas, [scale_factor] (2-3 args)
    if (nrhs < 3 || nrhs > 4) {
        mexErrMsgTxt("update_accel: Expected 2-3 arguments after function name");
    }
    if (nlhs > 1) {
        mexErrMsgTxt("update_accel: Only 1 output (q)");
    }
    
    cm q, a_meas;
    if (!matToFixed(prhs[1], q)) mexErrMsgTxt("Failed to read q");
    if (!matToFixed(prhs[2], a_meas)) mexErrMsgTxt("Failed to read a_meas");
    
    float scale_factor = (nrhs == 4) ? getScalar(prhs[3]) : 1.0f;
    
    eskf::ESKFCore::update_accel(q, a_meas, scale_factor);
    
    plhs[0] = fixedToMat(q);
}

// Handler: update_mag
static void handle_update_mag(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Expected: q, P, m_meas, m_world, R_mag (5 args)
    if (nrhs != 6) {
        mexErrMsgTxt("update_mag: Expected 5 arguments after function name");
    }
    if (nlhs > 4) {
        mexErrMsgTxt("update_mag: Maximum 4 outputs (q, P, K, dx)");
    }
    
    cm q, P, m_meas, m_world, R_mag;
    if (!matToFixed(prhs[1], q)) mexErrMsgTxt("Failed to read q");
    if (!matToFixed(prhs[2], P)) mexErrMsgTxt("Failed to read P");
    if (!matToFixed(prhs[3], m_meas)) mexErrMsgTxt("Failed to read m_meas");
    if (!matToFixed(prhs[4], m_world)) mexErrMsgTxt("Failed to read m_world");
    if (!matToFixed(prhs[5], R_mag)) mexErrMsgTxt("Failed to read R_mag");
    
    cm K, dx;
    eskf::ESKFCore::update_mag(q, P, m_meas, m_world, R_mag, K, dx);
    
    plhs[0] = fixedToMat(q);
    if (nlhs > 1) plhs[1] = fixedToMat(P);
    if (nlhs > 2) plhs[2] = fixedToMat(K);
    if (nlhs > 3) plhs[3] = fixedToMat(dx);
}

// Handler: update_gps
static void handle_update_gps(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Expected: p, v, P, gps_pos, gps_origin, R_gps (6 args)
    if (nrhs != 7) {
        mexErrMsgTxt("update_gps: Expected 6 arguments after function name");
    }
    if (nlhs > 5) {
        mexErrMsgTxt("update_gps: Maximum 5 outputs (p, v, P, K, dx)");
    }
    
    cm p, v, P, gps_pos, gps_origin, R_gps;
    if (!matToFixed(prhs[1], p)) mexErrMsgTxt("Failed to read p");
    if (!matToFixed(prhs[2], v)) mexErrMsgTxt("Failed to read v");
    if (!matToFixed(prhs[3], P)) mexErrMsgTxt("Failed to read P");
    if (!matToFixed(prhs[4], gps_pos)) mexErrMsgTxt("Failed to read gps_pos");
    if (!matToFixed(prhs[5], gps_origin)) mexErrMsgTxt("Failed to read gps_origin");
    if (!matToFixed(prhs[6], R_gps)) mexErrMsgTxt("Failed to read R_gps");
    
    cm K, dx;
    eskf::ESKFCore::update_gps(p, v, P, gps_pos, gps_origin, R_gps, K, dx);
    
    plhs[0] = fixedToMat(p);
    if (nlhs > 1) plhs[1] = fixedToMat(v);
    if (nlhs > 2) plhs[2] = fixedToMat(P);
    if (nlhs > 3) plhs[3] = fixedToMat(K);
    if (nlhs > 4) plhs[4] = fixedToMat(dx);
}

// Handler: update_baro
static void handle_update_baro(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Expected: p, P, pressure, gps_origin, R_baro (5 args)
    if (nrhs != 6) {
        mexErrMsgTxt("update_baro: Expected 5 arguments after function name");
    }
    if (nlhs > 4) {
        mexErrMsgTxt("update_baro: Maximum 4 outputs (p, P, K, dx)");
    }
    
    cm p, P, gps_origin;
    if (!matToFixed(prhs[1], p)) mexErrMsgTxt("Failed to read p");
    if (!matToFixed(prhs[2], P)) mexErrMsgTxt("Failed to read P");
    float pressure = getScalar(prhs[3]);
    if (!matToFixed(prhs[4], gps_origin)) mexErrMsgTxt("Failed to read gps_origin");
    float R_baro = getScalar(prhs[5]);
    
    cm K, dx;
    eskf::ESKFCore::update_baro(p, P, pressure, gps_origin, R_baro, K, dx);
    
    plhs[0] = fixedToMat(p);
    if (nlhs > 1) plhs[1] = fixedToMat(P);
    if (nlhs > 2) plhs[2] = fixedToMat(K);
    if (nlhs > 3) plhs[3] = fixedToMat(dx);
}

// Handler: predict_covariance
static void handle_predict_covariance(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 9) {
        mexErrMsgTxt("predict_covariance: Expected 8 arguments after function name");
    }
    if (nlhs > 1) {
        mexErrMsgTxt("predict_covariance: Maximum 1 output (P_new)");
    }
    
    cm P, q, a_meas, ba, w_meas, bg, Q;
    if (!matToFixed(prhs[1], P)) mexErrMsgTxt("Failed to read P");
    if (!matToFixed(prhs[2], q)) mexErrMsgTxt("Failed to read q");
    if (!matToFixed(prhs[3], a_meas)) mexErrMsgTxt("Failed to read a_meas");
    if (!matToFixed(prhs[4], ba)) mexErrMsgTxt("Failed to read ba");
    if (!matToFixed(prhs[5], w_meas)) mexErrMsgTxt("Failed to read w_meas");
    if (!matToFixed(prhs[6], bg)) mexErrMsgTxt("Failed to read bg");
    if (!matToFixed(prhs[7], Q)) mexErrMsgTxt("Failed to read Q");
    float dt = getScalar(prhs[8]);
    
    cm P_new;
    eskf::ESKFCore::predict_covariance(P, q, a_meas, ba, w_meas, bg, Q, dt, P_new);
    
    plhs[0] = fixedToMat(P_new);
}

// Handler: inject_error_state
static void handle_inject_error_state(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 7) {
        mexErrMsgTxt("inject_error_state: Expected 6 arguments after function name");
    }
    if (nlhs > 5) {
        mexErrMsgTxt("inject_error_state: Maximum 5 outputs (p, v, q, ba, bg)");
    }
    
    cm p, v, q, ba, bg, dx;
    if (!matToFixed(prhs[1], p)) mexErrMsgTxt("Failed to read p");
    if (!matToFixed(prhs[2], v)) mexErrMsgTxt("Failed to read v");
    if (!matToFixed(prhs[3], q)) mexErrMsgTxt("Failed to read q");
    if (!matToFixed(prhs[4], ba)) mexErrMsgTxt("Failed to read ba");
    if (!matToFixed(prhs[5], bg)) mexErrMsgTxt("Failed to read bg");
    if (!matToFixed(prhs[6], dx)) mexErrMsgTxt("Failed to read dx");
    
    eskf::ESKFCore::inject_error_state(p, v, q, ba, bg, dx);
    
    plhs[0] = fixedToMat(p);
    if (nlhs > 1) plhs[1] = fixedToMat(v);
    if (nlhs > 2) plhs[2] = fixedToMat(q);
    if (nlhs > 3) plhs[3] = fixedToMat(ba);
    if (nlhs > 4) plhs[4] = fixedToMat(bg);
}

// Main MEX entry point
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgTxt("Usage: mex_eskf_core(function_name, ...)");
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
    if (strcmp(func_name, "integrate_nominal") == 0) {
        handle_integrate_nominal(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "update_accel") == 0) {
        handle_update_accel(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "update_mag") == 0) {
        handle_update_mag(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "update_gps") == 0) {
        handle_update_gps(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "update_baro") == 0) {
        handle_update_baro(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "predict_covariance") == 0) {
        handle_predict_covariance(nlhs, plhs, nrhs, prhs);
    } else if (strcmp(func_name, "inject_error_state") == 0) {
        handle_inject_error_state(nlhs, plhs, nrhs, prhs);
    } else {
        mexErrMsgIdAndTxt("mex_eskf_core:unknownFunction",
                          "Unknown function: %s", func_name);
    }
}

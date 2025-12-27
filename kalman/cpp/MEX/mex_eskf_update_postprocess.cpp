// mex_eskf_update_postprocess.cpp
// MEX wrapper for do_cpp_update() post-processing (Phase 2)
// Implements: divergence_guard.check_and_attenuate_update, state update with dx, quaternion multiplication

#include "mex.h"
#include "mex_type_conv.hpp"
#include "../include/Common/Math/fixed_matrix.hpp"
#include <string>
#include <cmath>
#include <vector>

using namespace cmath_fx;

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

// Quaternion multiplication: q_new = q1 * q2
static void quat_multiply(const float* q1, const float* q2, float* q_out) {
    float w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    float w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
    
    q_out[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    q_out[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    q_out[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    q_out[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2;
}

// Normalize quaternion
static void quat_normalize(float* q) {
    float norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-10f) {
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }
}

// Main post-processing function
// Input: sensor_type, dbg_out (struct with dx, innov, H), state (struct with p, v, q, ba, bg, P), sample
// Output: new_state (struct with p, v, q, ba, bg, P), should_skip
static void handle_postprocess(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Expected args (after 'postprocess' command is removed):
    // sensor_type (string), dx (15x1), innov, state_p (3x1), state_v (3x1), state_q (4x1), 
    // state_ba (3x1), state_bg (3x1), state_P (15x15), new_state_P (15x15), sample
    // Total: 11 arguments
    
    if (nrhs < 11) {
        mexErrMsgTxt("postprocess: insufficient args. Need: sensor_type, dx, innov, state_p, state_v, state_q, state_ba, state_bg, state_P, new_state_P, sample");
    }
    
    // Get sensor_type
    char sensor_type[64];
    if (!mxIsChar(prhs[0]) || mxGetString(prhs[0], sensor_type, sizeof(sensor_type)) != 0) {
        mexErrMsgTxt("sensor_type must be a string");
    }
    
    // Get dx (15x1)
    Vector<15, float> dx;
    if (!matToVector(prhs[1], dx)) {
        mexErrMsgTxt("dx read failed (expected 15x1)");
    }
    
    // Get innov (variable size, pass to MATLAB)
    const mxArray* innov = prhs[2];
    
    // Get state
    Vector<3, float> state_p, state_v, state_ba, state_bg;
    Vector<4, float> state_q;
    Matrix<15, 15, float> state_P, new_state_P;
    
    if (!matToVector(prhs[3], state_p)) mexErrMsgTxt("state_p read failed");
    if (!matToVector(prhs[4], state_v)) mexErrMsgTxt("state_v read failed");
    if (!matToVector(prhs[5], state_q)) mexErrMsgTxt("state_q read failed");
    if (!matToVector(prhs[6], state_ba)) mexErrMsgTxt("state_ba read failed");
    if (!matToVector(prhs[7], state_bg)) mexErrMsgTxt("state_bg read failed");
    if (!matToMatrix(prhs[8], state_P)) mexErrMsgTxt("state_P read failed");
    if (!matToMatrix(prhs[9], new_state_P)) mexErrMsgTxt("new_state_P read failed");
    
    double sample = mxGetScalar(prhs[10]);
    
    // Call divergence_guard.check_and_attenuate_update via mex_sensor_filter
    mxArray* plhs_div[3];
    mxArray* prhs_div[5];
    prhs_div[0] = mxCreateString("divergence_check");
    prhs_div[1] = mxCreateString(sensor_type);
    prhs_div[2] = mxDuplicateArray(innov);
    prhs_div[3] = vectorToMat(dx);
    // ctx is not used in current implementation, pass empty
    
    mexCallMATLAB(3, plhs_div, 4, prhs_div, "mex_sensor_filter");
    
    // Get outputs: dx_out, should_skip, was_attenuated
    Vector<15, float> dx_out;
    if (!matToVector(plhs_div[0], dx_out)) {
        // If dx_out is empty, use original dx
        dx_out = dx;
    }
    bool should_skip = mxIsLogicalScalarTrue(plhs_div[1]);
    bool was_attenuated = mxIsLogicalScalarTrue(plhs_div[2]);
    
    // Cleanup divergence call
    mxDestroyArray(plhs_div[0]);
    mxDestroyArray(plhs_div[1]);
    mxDestroyArray(plhs_div[2]);
    mxDestroyArray(prhs_div[0]);
    mxDestroyArray(prhs_div[1]);
    mxDestroyArray(prhs_div[2]);
    mxDestroyArray(prhs_div[3]);
    
    // Output: new_state (p, v, q, ba, bg, P), should_skip
    Vector<3, float> new_p, new_v, new_ba, new_bg;
    Vector<4, float> new_q;
    Matrix<15, 15, float> out_P = new_state_P;
    
    if (should_skip) {
        // Return original state
        new_p = state_p;
        new_v = state_v;
        new_q = state_q;
        new_ba = state_ba;
        new_bg = state_bg;
        out_P = state_P;
    } else if (was_attenuated) {
        // Apply dx_out
        for (int i = 0; i < 3; ++i) {
            new_p(i, 0) = state_p(i, 0) + dx_out(i, 0);
            new_v(i, 0) = state_v(i, 0) + dx_out(i + 3, 0);
            new_ba(i, 0) = state_ba(i, 0) + dx_out(i + 9, 0);
            new_bg(i, 0) = state_bg(i, 0) + dx_out(i + 12, 0);
        }
        
        // Quaternion update: dq = [1; 0.5 * phi], q_new = dq * q
        float phi[3] = {dx_out(6, 0), dx_out(7, 0), dx_out(8, 0)};
        float dq[4] = {1.0f, 0.5f * phi[0], 0.5f * phi[1], 0.5f * phi[2]};
        float q_state[4] = {state_q(0, 0), state_q(1, 0), state_q(2, 0), state_q(3, 0)};
        float q_new[4];
        quat_multiply(dq, q_state, q_new);
        quat_normalize(q_new);
        
        for (int i = 0; i < 4; ++i) new_q(i, 0) = q_new[i];
        
        // Symmetrize P
        for (int i = 0; i < 15; ++i) {
            for (int j = i + 1; j < 15; ++j) {
                float avg = 0.5f * (out_P(i, j) + out_P(j, i));
                out_P(i, j) = avg;
                out_P(j, i) = avg;
            }
        }
    } else {
        // Use new_state directly (no attenuation)
        // In this case, we return the new_state passed in, but symmetrize P
        // The caller should have already set new_p, new_v, new_q, new_ba, new_bg
        // For now, we return original state + dx (same as attenuated case but with original dx)
        for (int i = 0; i < 3; ++i) {
            new_p(i, 0) = state_p(i, 0) + dx(i, 0);
            new_v(i, 0) = state_v(i, 0) + dx(i + 3, 0);
            new_ba(i, 0) = state_ba(i, 0) + dx(i + 9, 0);
            new_bg(i, 0) = state_bg(i, 0) + dx(i + 12, 0);
        }
        
        // Quaternion update
        float phi[3] = {dx(6, 0), dx(7, 0), dx(8, 0)};
        float dq[4] = {1.0f, 0.5f * phi[0], 0.5f * phi[1], 0.5f * phi[2]};
        float q_state[4] = {state_q(0, 0), state_q(1, 0), state_q(2, 0), state_q(3, 0)};
        float q_new[4];
        quat_multiply(dq, q_state, q_new);
        quat_normalize(q_new);
        
        for (int i = 0; i < 4; ++i) new_q(i, 0) = q_new[i];
        
        // Symmetrize P
        for (int i = 0; i < 15; ++i) {
            for (int j = i + 1; j < 15; ++j) {
                float avg = 0.5f * (out_P(i, j) + out_P(j, i));
                out_P(i, j) = avg;
                out_P(j, i) = avg;
            }
        }
    }
    
    // Output: new_p, new_v, new_q, new_ba, new_bg, new_P, should_skip
    if (nlhs >= 1) plhs[0] = vectorToMat(new_p);
    if (nlhs >= 2) plhs[1] = vectorToMat(new_v);
    if (nlhs >= 3) plhs[2] = vectorToMat(new_q);
    if (nlhs >= 4) plhs[3] = vectorToMat(new_ba);
    if (nlhs >= 5) plhs[4] = vectorToMat(new_bg);
    if (nlhs >= 6) plhs[5] = matrixToMat(out_P);
    if (nlhs >= 7) plhs[6] = mxCreateLogicalScalar(should_skip);
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgTxt("Usage: mex_eskf_update_postprocess('postprocess', sensor_type, dx, innov, state_p, state_v, state_q, state_ba, state_bg, state_P, new_state_P, sample)");
    }
    
    if (!mxIsChar(prhs[0])) {
        mexErrMsgTxt("First argument must be a string");
    }
    
    char cmd[64];
    if (mxGetString(prhs[0], cmd, sizeof(cmd))) {
        mexErrMsgTxt("Failed to read command string");
    }
    
    std::string cmdstr(cmd);
    
    if (cmdstr == "postprocess") {
        handle_postprocess(nlhs, plhs, nrhs - 1, prhs + 1);
    } else {
        mexErrMsgTxt("Unknown command. Use 'postprocess'");
    }
}


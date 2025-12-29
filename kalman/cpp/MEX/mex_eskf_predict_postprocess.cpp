// mex_eskf_predict_postprocess.cpp
// MEX wrapper for predict() post-processing (Phase 1)
// Implements: accel_z_integration, velocity_damping, P normalization, divergence_guard, velocity clipping

#include "mex.h"
#include "mex_type_conv.hpp"
#include "../Inc/Common/Math/fixed_matrix.hpp"
#include "../Common/Math/quaternion_lib.hpp"
#include <string>
#include <cmath>
#include <vector>

using namespace cmath_fx;
using Quat = quat_lib::Quaternion<float>;

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

// Get rotation matrix from quaternion using mex_quaternion_lib to match MATLAB exactly
static Matrix<3, 3, float> quaternionToRotationMatrix(const Vector<4, float>& q) {
    // Call mex_quaternion_lib('to_rotation_matrix', q) to match MATLAB implementation exactly
    mxArray* q_mat = vectorToMat(q);
    mxArray* plhs_rot[1];
    mxArray* prhs_rot[2];
    
    prhs_rot[0] = mxCreateString("to_rotation_matrix");
    prhs_rot[1] = q_mat;
    
    mexCallMATLAB(1, plhs_rot, 2, prhs_rot, "mex_quaternion_lib");
    
    Matrix<3, 3, float> R;
    if (!matToMatrix(plhs_rot[0], R)) {
        mexErrMsgTxt("Failed to get rotation matrix from quaternion");
    }
    
    mxDestroyArray(plhs_rot[0]);
    mxDestroyArray(q_mat);
    mxDestroyArray(prhs_rot[0]);
    
    return R;
}

// Main post-processing function
static void handle_postprocess(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Expected args (after 'postprocess' command is removed):
    // v, q, P, a_for_vel, dt, g,
    // enable_accel_z_integration, accel_z_threshold, accel_z_damping,
    // velocity_damping
    // Total: 10 arguments
    
    if (nrhs < 10) {
        mexErrMsgTxt("postprocess: insufficient args. Need: v, q, P, a_for_vel, dt, g, enable_accel_z, accel_z_thresh, accel_z_damp, vel_damp");
    }
    
    Vector<3, float> v, a_for_vel, g;
    Vector<4, float> q;
    Matrix<15, 15, float> P;
    
    if (!matToVector(prhs[0], v)) mexErrMsgTxt("v read failed");
    if (!matToVector(prhs[1], q)) mexErrMsgTxt("q read failed");
    if (!matToMatrix(prhs[2], P)) mexErrMsgTxt("P read failed");
    if (!matToVector(prhs[3], a_for_vel)) mexErrMsgTxt("a_for_vel read failed");
    float dt = mex_conv::mxGetScalarAsFloat(prhs[4]);
    if (!matToVector(prhs[5], g)) mexErrMsgTxt("g read failed");
    
    bool enable_accel_z = (mxGetScalar(prhs[6]) != 0.0);
    float accel_z_threshold = mex_conv::mxGetScalarAsFloat(prhs[7]);
    float accel_z_damping = mex_conv::mxGetScalarAsFloat(prhs[8]);
    float velocity_damping = mex_conv::mxGetScalarAsFloat(prhs[9]);
    
    // MATLAB螳溯｣・ｒ螳溯｡鯉ｼ医さ繝｡繝ｳ繝医い繧ｦ繝医＆繧後※縺・↑縺・ｴ蜷医・螳溯｣・ｼ・    // 1. accel_z_integration - Use MATLAB directly via mexCallMATLAB to ensure exact match
    if (enable_accel_z) {
        // Call mex_quaternion_lib('to_rotation_matrix', q) - same as MATLAB
        mxArray* q_mat = vectorToMat(q);
        mxArray* plhs_R[1];
        mxArray* prhs_R[2];
        prhs_R[0] = mxCreateString("to_rotation_matrix");
        prhs_R[1] = q_mat;
        mexCallMATLAB(1, plhs_R, 2, prhs_R, "mex_quaternion_lib");
        
        // Calculate R * a_for_vel using MATLAB mtimes
        mxArray* a_for_vel_mat = vectorToMat(a_for_vel);
        mxArray* plhs_mult[1];
        mxArray* prhs_mult[2];
        prhs_mult[0] = plhs_R[0];
        prhs_mult[1] = a_for_vel_mat;
        mexCallMATLAB(1, plhs_mult, 2, prhs_mult, "mtimes");
        
        // Calculate (R * a_for_vel) - [0; 0; g(3)] using MATLAB minus
        mxArray* g_z_vec = mxCreateDoubleMatrix(3, 1, mxREAL);
        double* g_z_data = mxGetPr(g_z_vec);
        g_z_data[0] = 0.0;
        g_z_data[1] = 0.0;
        g_z_data[2] = static_cast<double>(g(2, 0));
        
        mxArray* plhs_sub[1];
        mxArray* prhs_sub[2];
        prhs_sub[0] = plhs_mult[0];
        prhs_sub[1] = g_z_vec;
        mexCallMATLAB(1, plhs_sub, 2, prhs_sub, "minus");
        
        // Extract a_ned and get az_excess
        Vector<3, float> a_ned;
        if (!matToVector(plhs_sub[0], a_ned)) {
            mexErrMsgTxt("Failed to get a_ned from MATLAB calculation");
        }
        
        float az_excess = a_ned(2, 0);
        if (std::abs(az_excess) > accel_z_threshold) {
            v(2, 0) = v(2, 0) * (1.0f - accel_z_damping) + az_excess * dt;
        }
        
        // Cleanup
        mxDestroyArray(plhs_R[0]);
        mxDestroyArray(q_mat);
        mxDestroyArray(prhs_R[0]);
        mxDestroyArray(a_for_vel_mat);
        mxDestroyArray(plhs_mult[0]);
        mxDestroyArray(g_z_vec);
        mxDestroyArray(plhs_sub[0]);
    }
    
    // 2. velocity_damping
    if (velocity_damping > 0.0f) {
        v(0, 0) = v(0, 0) * (1.0f - velocity_damping * dt);
        v(1, 0) = v(1, 0) * (1.0f - velocity_damping * dt);
    }
    
    // 3. divergence_guard.regularize_covariance (via mex_sensor_filter)
    mxArray* plhs_reg[1];
    mxArray* prhs_reg[2];
    prhs_reg[0] = mxCreateString("divergence_regularize");
    prhs_reg[1] = matrixToMat(P);
    mexCallMATLAB(1, plhs_reg, 2, prhs_reg, "mex_sensor_filter");
    if (!matToMatrix(plhs_reg[0], P)) {
        mexErrMsgTxt("Failed to regularize covariance");
    }
    mxDestroyArray(plhs_reg[0]);
    mxDestroyArray(prhs_reg[0]);
    mxDestroyArray(prhs_reg[1]);
    
    // 4. P normalization (max_var check) - MATLAB implementation
    float max_var[15];
    max_var[0] = max_var[1] = max_var[2] = 100.0f * 100.0f;  // position
    max_var[3] = max_var[4] = max_var[5] = 20.0f * 20.0f;    // velocity
    // Use high-precision value for deg2rad(45) to match MATLAB
    // deg2rad(45) = 0.7853981633974483 (double precision)
    float deg45_rad = 0.7853981633974483f;  // deg2rad(45) in double precision, cast to float
    max_var[6] = max_var[7] = max_var[8] = deg45_rad * deg45_rad;  // attitude
    max_var[9] = max_var[10] = max_var[11] = 0.1f;  // accel bias
    max_var[12] = max_var[13] = max_var[14] = 0.01f;  // gyro bias
    
    for (int i = 0; i < 15; ++i) {
        if (P(i, i) > max_var[i]) {
            float factor = std::sqrt(max_var[i] / P(i, i));
            for (int j = 0; j < 15; ++j) {
                P(i, j) = P(i, j) * factor;
                P(j, i) = P(j, i) * factor;
            }
            P(i, i) = max_var[i];
        }
    }
    
    // 5. divergence_guard.check_and_clip_velocity (via mex_sensor_filter)
    mxArray* plhs_clip[3];
    mxArray* prhs_clip[4];
    prhs_clip[0] = mxCreateString("divergence_clip_velocity");
    prhs_clip[1] = vectorToMat(v);
    prhs_clip[2] = matrixToMat(P);
    // vel_indices = [4, 5, 6] (1-based in MATLAB, but we use 0-based internally)
    mxArray* vel_indices = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* vid = mxGetPr(vel_indices);
    vid[0] = 4.0; vid[1] = 5.0; vid[2] = 6.0;
    prhs_clip[3] = vel_indices;
    mexCallMATLAB(3, plhs_clip, 4, prhs_clip, "mex_sensor_filter");
    if (!matToVector(plhs_clip[0], v)) {
        mexErrMsgTxt("Failed to clip velocity");
    }
    if (!matToMatrix(plhs_clip[1], P)) {
        mexErrMsgTxt("Failed to update P after velocity clipping");
    }
    mxDestroyArray(plhs_clip[0]);
    mxDestroyArray(plhs_clip[1]);
    mxDestroyArray(plhs_clip[2]);
    mxDestroyArray(prhs_clip[0]);
    mxDestroyArray(prhs_clip[1]);
    mxDestroyArray(prhs_clip[2]);
    mxDestroyArray(prhs_clip[3]);
    
    // 6. Velocity norm check (clip to 10.0 m/s) - MATLAB implementation
    // Note: This is executed AFTER divergence_clip_velocity in MATLAB implementation (lines 270-272)
    // Even though divergence_clip_velocity also performs velocity clipping internally,
    // MATLAB implementation explicitly checks again here, so we must match this behavior.
    float v_norm = 0.0f;
    for (int i = 0; i < 3; ++i) v_norm += v(i, 0) * v(i, 0);
    v_norm = std::sqrt(v_norm);
    if (v_norm > 10.0f) {
        float scale = 10.0f / v_norm;
        for (int i = 0; i < 3; ++i) v(i, 0) *= scale;
    }
    
    // Output
    plhs[0] = vectorToMat(v);
    plhs[1] = matrixToMat(P);
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgTxt("Usage: mex_eskf_predict_postprocess('postprocess', v, q, P, a_for_vel, dt, g, enable_accel_z, accel_z_thresh, accel_z_damp, vel_damp)");
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


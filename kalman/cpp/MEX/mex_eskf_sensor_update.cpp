// mex_eskf_sensor_update.cpp
// MEX wrapper for sensor_updates() - integrates preprocessing + update + postprocessing (Phase 3)
// This function calls existing MEX functions to minimize code duplication

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

// Main sensor update function
// Integrates: preprocessing -> meukf_step -> update_postprocess
// Input: sensor_type, meas, prev_meas, state (p, v, q, ba, bg, P), params, sample
// Output: new_state (p, v, q, ba, bg, P), should_skip
static void handle_sensor_update(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Expected args (after 'update' command is removed):
    // sensor_type (string), meas (3x1 or scalar), prev_meas (3x1 or scalar), 
    // state_p (3x1), state_v (3x1), state_q (4x1), state_ba (3x1), state_bg (3x1), state_P (15x15),
    // params (struct), sample, w_body_norm
    // Total: 12 arguments minimum
    
    if (nrhs < 12) {
        mexErrMsgTxt("update: insufficient args. Need: sensor_type, meas, prev_meas, state_p, state_v, state_q, state_ba, state_bg, state_P, params, sample, w_body_norm");
    }
    
    // Get sensor_type
    char sensor_type[64];
    if (!mxIsChar(prhs[0]) || mxGetString(prhs[0], sensor_type, sizeof(sensor_type)) != 0) {
        mexErrMsgTxt("sensor_type must be a string");
    }
    
    // Get measurement and previous measurement
    const mxArray* meas = prhs[1];
    const mxArray* prev_meas = prhs[2];
    
    // Get state
    Vector<3, float> state_p, state_v, state_ba, state_bg;
    Vector<4, float> state_q;
    Matrix<15, 15, float> state_P;
    
    if (!matToVector(prhs[3], state_p)) mexErrMsgTxt("state_p read failed");
    if (!matToVector(prhs[4], state_v)) mexErrMsgTxt("state_v read failed");
    if (!matToVector(prhs[5], state_q)) mexErrMsgTxt("state_q read failed");
    if (!matToVector(prhs[6], state_ba)) mexErrMsgTxt("state_ba read failed");
    if (!matToVector(prhs[7], state_bg)) mexErrMsgTxt("state_bg read failed");
    if (!matToMatrix(prhs[8], state_P)) mexErrMsgTxt("state_P read failed");
    
    const mxArray* params = prhs[9];
    double sample = mxGetScalar(prhs[10]);
    double w_body_norm = mxGetScalar(prhs[11]);
    
    // Step 1: Preprocessing via mex_sensor_preprocessor
    mxArray* plhs_preproc[3];
    mxArray* prhs_preproc[3];
    
    std::string preproc_cmd;
    if (strcmp(sensor_type, "accel") == 0) {
        preproc_cmd = "preprocess_accel";
    } else if (strcmp(sensor_type, "mag") == 0) {
        preproc_cmd = "preprocess_mag";
    } else if (strcmp(sensor_type, "baro") == 0) {
        preproc_cmd = "preprocess_baro";
    } else {
        // GPS and others use different preprocessing, handle separately
        preproc_cmd = "preprocess_" + std::string(sensor_type);
    }
    
    prhs_preproc[0] = mxCreateString(preproc_cmd.c_str());
    prhs_preproc[1] = mxDuplicateArray(meas);
    prhs_preproc[2] = mxDuplicateArray(prev_meas);
    
    int preproc_nrhs = (strcmp(sensor_type, "baro") == 0) ? 2 : 3;
    mexCallMATLAB(3, plhs_preproc, preproc_nrhs, prhs_preproc, "mex_sensor_preprocessor");
    
    // Get preprocessing results: corrected_meas, is_outlier, no_change
    const mxArray* corrected_meas = plhs_preproc[0];
    bool is_outlier = mxIsLogicalScalarTrue(plhs_preproc[1]);
    bool no_change = mxIsLogicalScalarTrue(plhs_preproc[2]);
    
    // Check if we should skip this update
    bool should_skip = no_change || is_outlier;
    
    // For accel, also check w_body_norm
    if (strcmp(sensor_type, "accel") == 0 && w_body_norm > 1.5) {
        should_skip = true;
    }
    
    if (should_skip) {
        // Return original state
        if (nlhs >= 1) plhs[0] = vectorToMat(state_p);
        if (nlhs >= 2) plhs[1] = vectorToMat(state_v);
        if (nlhs >= 3) plhs[2] = vectorToMat(state_q);
        if (nlhs >= 4) plhs[3] = vectorToMat(state_ba);
        if (nlhs >= 5) plhs[4] = vectorToMat(state_bg);
        if (nlhs >= 6) plhs[5] = matrixToMat(state_P);
        if (nlhs >= 7) plhs[6] = mxCreateLogicalScalar(true);  // should_skip = true
        
        // Cleanup
        mxDestroyArray(plhs_preproc[0]);
        mxDestroyArray(plhs_preproc[1]);
        mxDestroyArray(plhs_preproc[2]);
        mxDestroyArray(prhs_preproc[0]);
        mxDestroyArray(prhs_preproc[1]);
        if (preproc_nrhs >= 3) mxDestroyArray(prhs_preproc[2]);
        return;
    }
    
    // Step 2: Call mex_meukf_step_v2 for the update
    // Build sensor_data struct
    mxArray* sensor_data = mxCreateStructMatrix(1, 1, 0, NULL);
    mxAddField(sensor_data, "accel"); mxSetField(sensor_data, 0, "accel", mxCreateDoubleMatrix(3, 1, mxREAL));
    mxAddField(sensor_data, "gyro"); mxSetField(sensor_data, 0, "gyro", mxCreateDoubleMatrix(3, 1, mxREAL));
    mxAddField(sensor_data, "mag"); mxSetField(sensor_data, 0, "mag", mxCreateDoubleMatrix(3, 1, mxREAL));
    mxAddField(sensor_data, "gps_pos"); mxSetField(sensor_data, 0, "gps_pos", mxCreateDoubleMatrix(3, 1, mxREAL));
    mxAddField(sensor_data, "alt_baro"); mxSetField(sensor_data, 0, "alt_baro", mxCreateDoubleScalar(0));
    
    // Set measurement based on sensor type
    if (strcmp(sensor_type, "accel") == 0) {
        mxSetField(sensor_data, 0, "accel", mxDuplicateArray(corrected_meas));
        mxAddField(sensor_data, "update_accel"); mxSetField(sensor_data, 0, "update_accel", mxCreateLogicalScalar(true));
        mxAddField(sensor_data, "update_mag"); mxSetField(sensor_data, 0, "update_mag", mxCreateLogicalScalar(false));
        mxAddField(sensor_data, "update_gps"); mxSetField(sensor_data, 0, "update_gps", mxCreateLogicalScalar(false));
        mxAddField(sensor_data, "update_baro"); mxSetField(sensor_data, 0, "update_baro", mxCreateLogicalScalar(false));
    } else if (strcmp(sensor_type, "mag") == 0) {
        mxSetField(sensor_data, 0, "mag", mxDuplicateArray(corrected_meas));
        mxAddField(sensor_data, "update_accel"); mxSetField(sensor_data, 0, "update_accel", mxCreateLogicalScalar(false));
        mxAddField(sensor_data, "update_mag"); mxSetField(sensor_data, 0, "update_mag", mxCreateLogicalScalar(true));
        mxAddField(sensor_data, "update_gps"); mxSetField(sensor_data, 0, "update_gps", mxCreateLogicalScalar(false));
        mxAddField(sensor_data, "update_baro"); mxSetField(sensor_data, 0, "update_baro", mxCreateLogicalScalar(false));
    } else if (strcmp(sensor_type, "baro") == 0) {
        mxSetField(sensor_data, 0, "alt_baro", mxDuplicateArray(corrected_meas));
        mxAddField(sensor_data, "update_accel"); mxSetField(sensor_data, 0, "update_accel", mxCreateLogicalScalar(false));
        mxAddField(sensor_data, "update_mag"); mxSetField(sensor_data, 0, "update_mag", mxCreateLogicalScalar(false));
        mxAddField(sensor_data, "update_gps"); mxSetField(sensor_data, 0, "update_gps", mxCreateLogicalScalar(false));
        mxAddField(sensor_data, "update_baro"); mxSetField(sensor_data, 0, "update_baro", mxCreateLogicalScalar(true));
    }
    // Add other required fields
    mxAddField(sensor_data, "update_gyro"); mxSetField(sensor_data, 0, "update_gyro", mxCreateLogicalScalar(false));
    mxAddField(sensor_data, "update_zupt"); mxSetField(sensor_data, 0, "update_zupt", mxCreateLogicalScalar(false));
    
    // Build state struct
    mxArray* state_struct = mxCreateStructMatrix(1, 1, 0, NULL);
    mxAddField(state_struct, "p"); mxSetField(state_struct, 0, "p", vectorToMat(state_p));
    mxAddField(state_struct, "v"); mxSetField(state_struct, 0, "v", vectorToMat(state_v));
    mxAddField(state_struct, "q"); mxSetField(state_struct, 0, "q", vectorToMat(state_q));
    mxAddField(state_struct, "ba"); mxSetField(state_struct, 0, "ba", vectorToMat(state_ba));
    mxAddField(state_struct, "bg"); mxSetField(state_struct, 0, "bg", vectorToMat(state_bg));
    mxAddField(state_struct, "P"); mxSetField(state_struct, 0, "P", matrixToMat(state_P));
    
    // Call mex_meukf_step_v2
    mxArray* plhs_meukf[3];
    mxArray* prhs_meukf[3];
    prhs_meukf[0] = state_struct;
    prhs_meukf[1] = sensor_data;
    prhs_meukf[2] = mxDuplicateArray(params);
    
    mexCallMATLAB(3, plhs_meukf, 3, prhs_meukf, "mex_meukf_step_v2");
    
    // Get new_state from mex_meukf_step_v2
    mxArray* new_state = plhs_meukf[0];
    mxArray* dbg_out = plhs_meukf[1];
    
    // Extract new state values
    Vector<3, float> new_p, new_v, new_ba, new_bg;
    Vector<4, float> new_q;
    Matrix<15, 15, float> new_P;
    
    if (!matToVector(mxGetField(new_state, 0, "p"), new_p)) new_p = state_p;
    if (!matToVector(mxGetField(new_state, 0, "v"), new_v)) new_v = state_v;
    if (!matToVector(mxGetField(new_state, 0, "q"), new_q)) new_q = state_q;
    if (!matToVector(mxGetField(new_state, 0, "ba"), new_ba)) new_ba = state_ba;
    if (!matToVector(mxGetField(new_state, 0, "bg"), new_bg)) new_bg = state_bg;
    if (mxGetField(new_state, 0, "P") != NULL) {
        if (!matToMatrix(mxGetField(new_state, 0, "P"), new_P)) new_P = state_P;
    } else {
        new_P = state_P;
    }
    
    // Step 3: Call mex_eskf_update_postprocess if dx is available
    if (dbg_out != NULL && mxIsStruct(dbg_out) && mxGetField(dbg_out, 0, "dx") != NULL) {
        mxArray* plhs_post[7];
        mxArray* prhs_post[12];
        prhs_post[0] = mxCreateString("postprocess");
        prhs_post[1] = mxCreateString(sensor_type);
        prhs_post[2] = mxDuplicateArray(mxGetField(dbg_out, 0, "dx"));
        prhs_post[3] = mxDuplicateArray(mxGetField(dbg_out, 0, "innov"));
        prhs_post[4] = vectorToMat(state_p);
        prhs_post[5] = vectorToMat(state_v);
        prhs_post[6] = vectorToMat(state_q);
        prhs_post[7] = vectorToMat(state_ba);
        prhs_post[8] = vectorToMat(state_bg);
        prhs_post[9] = matrixToMat(state_P);
        prhs_post[10] = matrixToMat(new_P);
        prhs_post[11] = mxCreateDoubleScalar(sample);
        
        mexCallMATLAB(7, plhs_post, 12, prhs_post, "mex_eskf_update_postprocess");
        
        // Get final state from postprocess
        if (!matToVector(plhs_post[0], new_p)) new_p = state_p;
        if (!matToVector(plhs_post[1], new_v)) new_v = state_v;
        if (!matToVector(plhs_post[2], new_q)) new_q = state_q;
        if (!matToVector(plhs_post[3], new_ba)) new_ba = state_ba;
        if (!matToVector(plhs_post[4], new_bg)) new_bg = state_bg;
        if (!matToMatrix(plhs_post[5], new_P)) new_P = state_P;
        bool post_should_skip = mxIsLogicalScalarTrue(plhs_post[6]);
        
        if (post_should_skip) {
            new_p = state_p;
            new_v = state_v;
            new_q = state_q;
            new_ba = state_ba;
            new_bg = state_bg;
            new_P = state_P;
        }
        
        // Cleanup postprocess
        for (int i = 0; i < 7; ++i) mxDestroyArray(plhs_post[i]);
        for (int i = 0; i < 12; ++i) mxDestroyArray(prhs_post[i]);
    }
    
    // Output final state
    if (nlhs >= 1) plhs[0] = vectorToMat(new_p);
    if (nlhs >= 2) plhs[1] = vectorToMat(new_v);
    if (nlhs >= 3) plhs[2] = vectorToMat(new_q);
    if (nlhs >= 4) plhs[3] = vectorToMat(new_ba);
    if (nlhs >= 5) plhs[4] = vectorToMat(new_bg);
    if (nlhs >= 6) plhs[5] = matrixToMat(new_P);
    if (nlhs >= 7) plhs[6] = mxCreateLogicalScalar(false);  // should_skip = false
    
    // Cleanup
    mxDestroyArray(plhs_preproc[0]);
    mxDestroyArray(plhs_preproc[1]);
    mxDestroyArray(plhs_preproc[2]);
    mxDestroyArray(prhs_preproc[0]);
    mxDestroyArray(prhs_preproc[1]);
    if (preproc_nrhs >= 3) mxDestroyArray(prhs_preproc[2]);
    mxDestroyArray(plhs_meukf[0]);
    mxDestroyArray(plhs_meukf[1]);
    mxDestroyArray(plhs_meukf[2]);
    mxDestroyArray(sensor_data);
    mxDestroyArray(state_struct);
    mxDestroyArray(prhs_meukf[2]);
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgTxt("Usage: mex_eskf_sensor_update('update', sensor_type, meas, prev_meas, state_p, state_v, state_q, state_ba, state_bg, state_P, params, sample, w_body_norm)");
    }
    
    if (!mxIsChar(prhs[0])) {
        mexErrMsgTxt("First argument must be a string");
    }
    
    char cmd[64];
    if (mxGetString(prhs[0], cmd, sizeof(cmd))) {
        mexErrMsgTxt("Failed to read command string");
    }
    
    std::string cmdstr(cmd);
    
    if (cmdstr == "update") {
        handle_sensor_update(nlhs, plhs, nrhs - 1, prhs + 1);
    } else {
        mexErrMsgTxt("Unknown command. Use 'update'");
    }
}


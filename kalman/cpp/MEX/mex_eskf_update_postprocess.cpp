// mex_eskf_update_postprocess.cpp
// MEX wrapper for do_cpp_update() post-processing (Phase 2)
// Implements: divergence_guard.check_and_attenuate_update, state update with dx, quaternion multiplication

#include "mex.h"
#include "mex_type_conv.hpp"
#include "../Inc/Common/Math/fixed_matrix.hpp"
#include "../Inc/Common/Math/quaternion_lib.hpp"
#include "../Inc/ESKF/eskf_postprocess.hpp"
#include "../Inc/MEX/mex_type_conversion.hpp"
#include <string>
#include <cmath>
#include <vector>

using namespace cmath_fx;
using Quat = quat_lib::Quaternion<float>;
using namespace eskf;
using namespace mex_conv;

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
        // Apply dx_out (実装はSrc/ESKF/eskf_postprocess.cppに移動)
        UpdatePostprocessResult updated = update_state_from_dx(dx_out, state_p, state_v, state_q, state_ba, state_bg, new_state_P);
        new_p = updated.p;
        new_v = updated.v;
        new_q = updated.q;
        new_ba = updated.ba;
        new_bg = updated.bg;
        out_P = updated.P;
    } else {
        // Use new_state directly (no attenuation) (実装はSrc/ESKF/eskf_postprocess.cppに移動)
        UpdatePostprocessResult updated = update_state_from_dx(dx, state_p, state_v, state_q, state_ba, state_bg, new_state_P);
        new_p = updated.p;
        new_v = updated.v;
        new_q = updated.q;
        new_ba = updated.ba;
        new_bg = updated.bg;
        out_P = updated.P;
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


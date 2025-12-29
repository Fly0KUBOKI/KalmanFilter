// mex_filter_management.cpp
// Simple filter management utilities for Phase 5

#include "mex.h"
#include "mex_type_conv.hpp"
#include "../Inc/Common/Math/fixed_matrix.hpp"
#include <string>
#include <cmath>
#include <vector>

using namespace cmath_fx;

static bool hasNaNOrInf(const Matrix<15,15,float>& P) {
    for (int j=0;j<15;++j) for (int i=0;i<15;++i) {
        float v = P(i,j);
        if (std::isnan(v) || std::isinf(v)) return true;
    }
    return false;
}

static void setIdentityScaled(Matrix<15,15,float>& P, float scale) {
    for (int j=0;j<15;++j) for (int i=0;i<15;++i) P(i,j) = 0.0f;
    for (int i=0;i<15;++i) P(i,i) = scale;
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) mexErrMsgTxt("Usage: mex_filter_management(cmd, ...)");
    if (!mxIsChar(prhs[0])) mexErrMsgTxt("cmd must be string");
    char cmd[64]; if (mxGetString(prhs[0], cmd, sizeof(cmd))) mexErrMsgTxt("failed to read cmd");
    std::string s(cmd);

    if (s == "check_divergence") {
        if (nrhs < 2) mexErrMsgTxt("check_divergence requires P matrix");
        if (!mxIsDouble(prhs[1])) mexErrMsgTxt("P must be double matrix");
        mwSize m = mxGetM(prhs[1]); mwSize n = mxGetN(prhs[1]);
        if (m != 15 || n != 15) mexErrMsgTxt("P must be 15x15");
        Matrix<15,15,float> P; std::vector<float> tmp(225);
        mex_conv::mxArrayToFloatArray(prhs[1], tmp.data(), 225);
        for (int j=0;j<15;++j) for (int i=0;i<15;++i) P(i,j) = tmp[j*15 + i];

        bool diverged = false;
        if (hasNaNOrInf(P)) diverged = true;
        // simple diag checks for large variance
        for (int i=0;i<15 && !diverged;++i) if (P(i,i) > 1e8f) diverged = true;

        plhs[0] = mxCreateLogicalScalar(diverged);
        return;
    }

    if (s == "reset_state") {
        // Usage: [p,v,q,ba,bg,P] = mex_filter_management('reset_state', p,v,q,ba,bg, P, reset_scale)
        if (nrhs < 7) mexErrMsgTxt("reset_state requires p,v,q,ba,bg,P");
        // read vectors
        std::vector<float> tmp3(3), tmp4(4);
        mex_conv::mxArrayToFloatArray(prhs[1], tmp3.data(), 3);
        mex_conv::mxArrayToFloatArray(prhs[2], tmp3.data(), 3); // reuse buffer for v
        // We'll simply return same inputs but P replaced by identity*reset_scale
        float reset_scale = 0.01f;
        if (nrhs >= 8) reset_scale = mex_conv::mxGetScalarAsFloat(prhs[7]);

        // Return inputs unchanged (caller likely updates state elsewhere)
        for (int k=1;k<=6 && k < nrhs; ++k) plhs[k-1] = mxDuplicateArray(prhs[k]);
        // Replace last output (P) with scaled identity
        plhs[5] = mxCreateDoubleMatrix(15,15,mxREAL);
        double* outP = mxGetPr(plhs[5]);
        for (int j=0;j<15;++j) for (int i=0;i<15;++i) outP[j*15 + i] = 0.0;
        for (int i=0;i<15;++i) outP[i + i*15] = (double)reset_scale;
        return;
    }

    if (s == "apply_zupt") {
        // Usage: [v_out, P_out] = mex_filter_management('apply_zupt', v_in, P_in)
        if (nrhs < 3) mexErrMsgTxt("apply_zupt requires v_in and P_in");
        if (mxGetNumberOfElements(prhs[1]) != 3) mexErrMsgTxt("v_in must be length 3");
        float v_tmp[3]; mex_conv::mxArrayToFloatArray(prhs[1], v_tmp, 3);
        // Zero velocity
        plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
        double* poutv = mxGetPr(plhs[0]); poutv[0]=0.0; poutv[1]=0.0; poutv[2]=0.0;

        // P_in -> reduce velocity covariance (indices 4:6 -> 3-based index 3..5 zero-based)
        if (!mxIsDouble(prhs[2])) mexErrMsgTxt("P must be double");
        mwSize m = mxGetM(prhs[2]); mwSize n = mxGetN(prhs[2]); if (m!=15||n!=15) mexErrMsgTxt("P must be 15x15");
        std::vector<double> Ptmp(225); double* src = mxGetPr(prhs[2]); for (int i=0;i<225;++i) Ptmp[i]=src[i];
        // reduce velocity variances by factor
        double factor = 0.01;
        for (int idx=3; idx<6; ++idx) {
            Ptmp[idx + idx*15] = Ptmp[idx + idx*15] * factor;
        }
        plhs[1] = mxCreateDoubleMatrix(15,15,mxREAL);
        double* outP = mxGetPr(plhs[1]); for (int i=0;i<225;++i) outP[i]=Ptmp[i];
        return;
    }

    mexErrMsgTxt("Unknown command for mex_filter_management");
}


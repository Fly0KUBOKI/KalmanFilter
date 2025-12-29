// mex_filter_management.cpp
// MEX wrapper for filter management utilities (Phase 5)
// Implementation code moved to Src/Common/filter_management.cpp

#include "mex.h"
#include "mex_type_conv.hpp"
#include "../Inc/Common/filter_management.hpp"
#include "../Inc/Common/Math/fixed_matrix.hpp"
#include <string>
#include <vector>

using namespace cmath_fx;
using namespace common::filter;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) mexErrMsgTxt("Usage: mex_filter_management(cmd, ...)");
    if (!mxIsChar(prhs[0])) mexErrMsgTxt("cmd must be string");
    char cmd[64];
    if (mxGetString(prhs[0], cmd, sizeof(cmd))) mexErrMsgTxt("failed to read cmd");
    std::string s(cmd);

    if (s == "check_divergence") {
        if (nrhs < 2) mexErrMsgTxt("check_divergence requires P matrix");
        if (!mxIsDouble(prhs[1])) mexErrMsgTxt("P must be double matrix");
        mwSize m = mxGetM(prhs[1]);
        mwSize n = mxGetN(prhs[1]);
        if (m != 15 || n != 15) mexErrMsgTxt("P must be 15x15");
        
        // MATLAB配列からC++型へ変換
        Matrix<15, 15, float> P;
        std::vector<float> tmp(225);
        mex_conv::mxArrayToFloatArray(prhs[1], tmp.data(), 225);
        for (int j = 0; j < 15; ++j) {
            for (int i = 0; i < 15; ++i) {
                P(i, j) = tmp[j * 15 + i];
            }
        }

        // 実装関数を呼び出す
        bool diverged = check_divergence(P);

        // 結果をMATLAB配列に変換
        plhs[0] = mxCreateLogicalScalar(diverged);
        return;
    }

    if (s == "reset_state") {
        // Usage: [p,v,q,ba,bg,P] = mex_filter_management('reset_state', p,v,q,ba,bg, P, reset_scale)
        if (nrhs < 7) mexErrMsgTxt("reset_state requires p,v,q,ba,bg,P");
        
        float reset_scale = 0.01f;
        if (nrhs >= 8) reset_scale = mex_conv::mxGetScalarAsFloat(prhs[7]);

        // Return inputs unchanged (caller likely updates state elsewhere)
        for (int k = 1; k <= 6 && k < nrhs; ++k) {
            plhs[k - 1] = mxDuplicateArray(prhs[k]);
        }
        
        // Replace last output (P) with scaled identity
        Matrix<15, 15, float> P;
        setIdentityScaled(P, reset_scale);
        
        plhs[5] = mxCreateDoubleMatrix(15, 15, mxREAL);
        double* outP = mxGetPr(plhs[5]);
        for (int j = 0; j < 15; ++j) {
            for (int i = 0; i < 15; ++i) {
                outP[j * 15 + i] = static_cast<double>(P(i, j));
            }
        }
        return;
    }

    if (s == "apply_zupt") {
        // Usage: [v_out, P_out] = mex_filter_management('apply_zupt', v_in, P_in)
        if (nrhs < 3) mexErrMsgTxt("apply_zupt requires v_in and P_in");
        if (mxGetNumberOfElements(prhs[1]) != 3) mexErrMsgTxt("v_in must be length 3");
        
        // MATLAB配列からC++型へ変換
        Vector<3, float> v_in;
        std::vector<float> v_tmp(3);
        mex_conv::mxArrayToFloatArray(prhs[1], v_tmp.data(), 3);
        for (int i = 0; i < 3; ++i) v_in(i, 0) = v_tmp[i];
        
        Matrix<15, 15, float> P_in;
        if (!mxIsDouble(prhs[2])) mexErrMsgTxt("P must be double");
        mwSize m = mxGetM(prhs[2]);
        mwSize n = mxGetN(prhs[2]);
        if (m != 15 || n != 15) mexErrMsgTxt("P must be 15x15");
        
        std::vector<float> P_tmp(225);
        mex_conv::mxArrayToFloatArray(prhs[2], P_tmp.data(), 225);
        for (int j = 0; j < 15; ++j) {
            for (int i = 0; i < 15; ++i) {
                P_in(i, j) = P_tmp[j * 15 + i];
            }
        }
        
        // 実装関数を呼び出す
        Vector<3, float> v_out;
        Matrix<15, 15, float> P_out;
        apply_zupt(v_in, P_in, v_out, P_out);
        
        // 結果をMATLAB配列に変換
        plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
        double* poutv = mxGetPr(plhs[0]);
        for (int i = 0; i < 3; ++i) {
            poutv[i] = static_cast<double>(v_out(i, 0));
        }
        
        plhs[1] = mxCreateDoubleMatrix(15, 15, mxREAL);
        double* outP = mxGetPr(plhs[1]);
        for (int j = 0; j < 15; ++j) {
            for (int i = 0; i < 15; ++i) {
                outP[j * 15 + i] = static_cast<double>(P_out(i, j));
            }
        }
        return;
    }

    mexErrMsgTxt("Unknown command for mex_filter_management");
}



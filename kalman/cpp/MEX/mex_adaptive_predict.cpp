// mex_adaptive_predict.cpp
// MEX wrapper implementing adaptive predict() (Phase 4 scaffolding)

#include "mex.h"
#include "../include/ESKF/eskf_core.hpp"
#include "mex_type_conv.hpp"
#include <string>
#include <cmath>
#include <vector>

using namespace eskf;

static float getScalarF(const mxArray* a) {
    return mex_conv::mxGetScalarAsFloat(a);
}

template<int R, typename T = float>
static bool matToVector(const mxArray* arr, cmath_fx::Vector<R, T>& out) {
    if (!arr) return false;
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    mwSize rows = mxGetM(arr); mwSize cols = mxGetN(arr);
    if (rows != R || cols != 1) return false;
    std::vector<T> tmp(static_cast<size_t>(R));
    mex_conv::mxArrayToFloatArray(arr, tmp.data(), static_cast<size_t>(R));
    for (int i = 0; i < R; ++i) out(i,0) = tmp[i];
    return true;
}

template<int R, int C, typename T = float>
static bool matToMatrix(const mxArray* arr, cmath_fx::Matrix<R, C, T>& out) {
    if (!arr) return false;
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    mwSize rows = mxGetM(arr); mwSize cols = mxGetN(arr);
    if (rows != R || cols != C) return false;
    std::vector<T> tmp(static_cast<size_t>(R) * static_cast<size_t>(C));
    mex_conv::mxArrayToFloatArray(arr, tmp.data(), static_cast<size_t>(R) * static_cast<size_t>(C));
    for (int j = 0; j < C; ++j) for (int i = 0; i < R; ++i) out(i,j) = tmp[static_cast<size_t>(j)*R + i];
    return true;
}

template<int R, typename T>
static mxArray* vectorToMat(const cmath_fx::Vector<R, T>& v) {
    mxArray* out = mxCreateDoubleMatrix(R, 1, mxREAL);
    double* pr = mxGetPr(out);
    for (int i = 0; i < R; ++i) pr[i] = static_cast<double>(v(i,0));
    return out;
}

template<int R, int C, typename T>
static mxArray* matrixToMat(const cmath_fx::Matrix<R, C, T>& M) {
    mxArray* out = mxCreateDoubleMatrix(R, C, mxREAL);
    double* pr = mxGetPr(out);
    for (int j = 0; j < C; ++j) for (int i = 0; i < R; ++i) pr[j*R + i] = static_cast<double>(M(i,j));
    return out;
}

// predict handler
static void handle_predict(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Expected args:
    // p,v,q,ba,bg,P, a_meas, w_meas, dt, Q_nominal, adaptive_q_enabled(bool), gyro_thr(3), accel_thr(3), g(3)
    const int expected_min = 11; // without optional thresholds/g
    if (nrhs < expected_min) mexErrMsgTxt("predict: insufficient args");

    Vector3 p,v,ba,bg,a_meas,w_meas,gyro_thr,accel_thr,g;
    Vector4 q;
    Matrix15x15 P, Qnom, Pnew;

    if (!matToVector(prhs[1], p)) mexErrMsgTxt("p read failed");
    if (!matToVector(prhs[2], v)) mexErrMsgTxt("v read failed");
    if (!matToVector(prhs[3], q)) mexErrMsgTxt("q read failed");
    if (!matToVector(prhs[4], ba)) mexErrMsgTxt("ba read failed");
    if (!matToVector(prhs[5], bg)) mexErrMsgTxt("bg read failed");
    if (!matToMatrix(prhs[6], P)) mexErrMsgTxt("P read failed");
    if (!matToVector(prhs[7], a_meas)) mexErrMsgTxt("a_meas read failed");
    if (!matToVector(prhs[8], w_meas)) mexErrMsgTxt("w_meas read failed");
    float dt = getScalarF(prhs[9]);
    if (!matToMatrix(prhs[10], Qnom)) mexErrMsgTxt("Q_nominal read failed");

    bool adaptive_q = false;
    if (nrhs >= 12) adaptive_q = mxIsLogical(prhs[11]) ? mxIsLogicalScalarTrue(prhs[11]) : (mxGetScalar(prhs[11]) != 0);

    // optional gyro/accel thresholds and gravity
    gyro_thr(0,0)=gyro_thr(1,0)=gyro_thr(2,0)=0.0f;
    accel_thr(0,0)=accel_thr(1,0)=accel_thr(2,0)=0.0f;
    g(0,0)=0.0f; g(1,0)=0.0f; g(2,0)=9.81f;
    if (nrhs >= 13) matToVector(prhs[12], gyro_thr);
    if (nrhs >= 14) matToVector(prhs[13], accel_thr);
    if (nrhs >= 15) matToVector(prhs[14], g);

    // Adaptive Q scaling (match MATLAB logic)
    Matrix15x15 Qadapt = Qnom;
    if (adaptive_q) {
        // compute norms
        float a_norm = 0.0f; for (int i=0;i<3;++i) a_norm += a_meas(i,0)*a_meas(i,0); a_norm = sqrtf(a_norm);
        float gravity_error = fabsf(a_norm - 9.81f);
        float accel_scale = 1.0f + (gravity_error / 3.0f);
        float w_norm = 0.0f; for (int i=0;i<3;++i) w_norm += w_meas(i,0)*w_meas(i,0); w_norm = sqrtf(w_norm);
        float deg2rad15 = 15.0f * 3.14159265f / 180.0f;
        float gyro_scale = 1.0f + (w_norm / deg2rad15);
        float q_scale = fmaxf(accel_scale, gyro_scale);
        if (q_scale > 5.0f) q_scale = 5.0f;
        // scale Qnom
        for (int j=0;j<15;++j) for (int i=0;i<15;++i) Qadapt(i,j) = Qnom(i,j) * q_scale;
    }

    // Integrate nominal state
    ESKFCore::integrate_nominal(p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_thr, accel_thr);

    // Predict covariance
    ESKFCore::predict_covariance(P, q, a_meas, ba, w_meas, bg, Qadapt, dt, Pnew);

    // Ensure symmetry
    for (int i=0;i<15;++i) for (int j=i+1;j<15;++j) { float v = 0.5f*(Pnew(i,j)+Pnew(j,i)); Pnew(i,j)=v; Pnew(j,i)=v; }

    // Outputs
    plhs[0] = vectorToMat<3>(p);
    if (nlhs > 1) plhs[1] = vectorToMat<3>(v);
    if (nlhs > 2) plhs[2] = vectorToMat<4>(q);
    if (nlhs > 3) plhs[3] = vectorToMat<3>(ba);
    if (nlhs > 4) plhs[4] = vectorToMat<3>(bg);
    if (nlhs > 5) plhs[5] = matrixToMat<15,15>(Pnew);
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) mexErrMsgTxt("Usage: mex_adaptive_predict('predict', ...)");
    if (!mxIsChar(prhs[0])) mexErrMsgTxt("First arg must be function name");
    char cmd[128]; if (mxGetString(prhs[0], cmd, sizeof(cmd))) mexErrMsgTxt("Failed to read cmd");
    std::string s(cmd);
    if (s == "predict") {
        handle_predict(nlhs, plhs, nrhs, prhs);
    } else {
        mexErrMsgTxt("Unknown command for mex_adaptive_predict");
    }
}

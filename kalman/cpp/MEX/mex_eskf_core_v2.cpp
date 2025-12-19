// ESKF C++ MEX Wrapper - Common Library版
// ビルドコマンド: mex mex_eskf_core.cpp eskf_core.cpp kalman_filter_core.cpp -Ic:\...\cpp

#include "mex.h"
#include "../ESKF/eskf_core.hpp"
#include <cstring>

// ヘルパー: MATLAB配列からFixedMatrixへ
cmath_fx::FixedMatrix matToFixed(const mxArray* mat) {
    size_t rows = mxGetM(mat);
    size_t cols = mxGetN(mat);
    double* data = mxGetPr(mat);
    
    cmath_fx::FixedMatrix result;
    result.resize(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            result(i,j) = static_cast<float>(data[j*rows + i]);
        }
    }
    return result;
}

// ヘルパー: FixedMatrixからMATLAB配列へ
mxArray* fixedToMat(const cmath_fx::FixedMatrix& mat) {
    mxArray* result = mxCreateDoubleMatrix(mat.rows(), mat.cols(), mxREAL);
    double* data = mxGetPr(result);
    
    for (int i = 0; i < mat.rows(); ++i) {
        for (int j = 0; j < mat.cols(); ++j) {
            data[j*mat.rows() + i] = static_cast<double>(mat(i,j));
        }
    }
    return result;
}

// ==================== ハンドラ関数 ====================

void handle_integrate_nominal(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // 入力: p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_thr, accel_thr
    if (nrhs != 12) mexErrMsgIdAndTxt("eskf:integrate_nominal", "11 inputs required");
    if (nlhs != 5) mexErrMsgIdAndTxt("eskf:integrate_nominal", "5 outputs required");
    
    auto p = matToFixed(prhs[1]);
    auto v = matToFixed(prhs[2]);
    auto q = matToFixed(prhs[3]);
    auto ba = matToFixed(prhs[4]);
    auto bg = matToFixed(prhs[5]);
    auto a_meas = matToFixed(prhs[6]);
    auto w_meas = matToFixed(prhs[7]);
    float dt = static_cast<float>(mxGetScalar(prhs[8]));
    auto g = matToFixed(prhs[9]);
    auto gyro_thr = matToFixed(prhs[10]);
    auto accel_thr = matToFixed(prhs[11]);
    
    cmath_fx::FixedMatrix p_new, v_new, q_new, ba_new, bg_new;
    eskf::ESKFCore::integrate_nominal(p, v, q, ba, bg, a_meas, w_meas, dt, g, 
                                     gyro_thr, accel_thr, 
                                     p_new, v_new, q_new, ba_new, bg_new);
    
    plhs[0] = fixedToMat(p_new);
    plhs[1] = fixedToMat(v_new);
    plhs[2] = fixedToMat(q_new);
    plhs[3] = fixedToMat(ba_new);
    plhs[4] = fixedToMat(bg_new);
}

void handle_update_accel(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // 入力: q, a_meas, scale
    if (nrhs != 4) mexErrMsgIdAndTxt("eskf:update_accel", "3 inputs required");
    if (nlhs != 1) mexErrMsgIdAndTxt("eskf:update_accel", "1 output required");
    
    auto q = matToFixed(prhs[1]);
    auto a_meas = matToFixed(prhs[2]);
    float scale = static_cast<float>(mxGetScalar(prhs[3]));
    
    auto q_new = eskf::ESKFCore::update_accel(q, a_meas, scale);
    
    plhs[0] = fixedToMat(q_new);
}

void handle_update_mag(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // 入力: q, P, m_meas, m_world, R_mag
    if (nrhs != 6) mexErrMsgIdAndTxt("eskf:update_mag", "5 inputs required");
    if (nlhs != 4) mexErrMsgIdAndTxt("eskf:update_mag", "4 outputs required");
    
    auto q = matToFixed(prhs[1]);
    auto P = matToFixed(prhs[2]);
    auto m_meas = matToFixed(prhs[3]);
    auto m_world = matToFixed(prhs[4]);
    auto R_mag = matToFixed(prhs[5]);
    
    cmath_fx::FixedMatrix q_new, P_new, K, dx;
    eskf::ESKFCore::update_mag(q, P, m_meas, m_world, R_mag, q_new, P_new, K, dx);
    
    plhs[0] = fixedToMat(q_new);
    plhs[1] = fixedToMat(P_new);
    plhs[2] = fixedToMat(K);
    plhs[3] = fixedToMat(dx);
}

void handle_update_gps(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // 入力: p, v, P, gps_pos, gps_origin, R_gps
    if (nrhs != 7) mexErrMsgIdAndTxt("eskf:update_gps", "6 inputs required");
    if (nlhs != 5) mexErrMsgIdAndTxt("eskf:update_gps", "5 outputs required");
    
    auto p = matToFixed(prhs[1]);
    auto v = matToFixed(prhs[2]);
    auto P = matToFixed(prhs[3]);
    auto gps_pos = matToFixed(prhs[4]);
    auto gps_origin = matToFixed(prhs[5]);
    auto R_gps = matToFixed(prhs[6]);
    
    cmath_fx::FixedMatrix p_new, v_new, P_new, K, dx;
    eskf::ESKFCore::update_gps(p, v, P, gps_pos, gps_origin, R_gps, 
                               p_new, v_new, P_new, K, dx);
    
    plhs[0] = fixedToMat(p_new);
    plhs[1] = fixedToMat(v_new);
    plhs[2] = fixedToMat(P_new);
    plhs[3] = fixedToMat(K);
    plhs[4] = fixedToMat(dx);
}

void handle_update_baro(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // 入力: p, P, pressure, origin, R_baro
    if (nrhs != 6) mexErrMsgIdAndTxt("eskf:update_baro", "5 inputs required");
    if (nlhs != 4) mexErrMsgIdAndTxt("eskf:update_baro", "4 outputs required");
    
    auto p = matToFixed(prhs[1]);
    auto P = matToFixed(prhs[2]);
    float pressure = static_cast<float>(mxGetScalar(prhs[3]));
    auto origin = matToFixed(prhs[4]);
    float R_baro = static_cast<float>(mxGetScalar(prhs[5]));
    
    cmath_fx::FixedMatrix p_new, P_new, K, dx;
    eskf::ESKFCore::update_baro(p, P, pressure, origin, R_baro, p_new, P_new, K, dx);
    
    plhs[0] = fixedToMat(p_new);
    plhs[1] = fixedToMat(P_new);
    plhs[2] = fixedToMat(K);
    plhs[3] = fixedToMat(dx);
}

// ==================== メインMEX関数 ====================

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgIdAndTxt("eskf:usage", 
            "Usage: mex_eskf_core('function_name', ...args)\n"
            "Functions:\n"
            "  integrate_nominal - Adams-Bashforth 2nd order integration\n"
            "  update_accel      - Accelerometer Roll/Pitch update\n"
            "  update_mag        - Magnetometer Yaw update\n"
            "  update_gps        - GPS position update\n"
            "  update_baro       - Barometer altitude update");
    }
    
    if (!mxIsChar(prhs[0])) {
        mexErrMsgIdAndTxt("eskf:input", "First argument must be a string");
    }
    
    char func_name[64];
    mxGetString(prhs[0], func_name, sizeof(func_name));
    
    try {
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
        } else {
            mexErrMsgIdAndTxt("eskf:unknown", "Unknown function: %s", func_name);
        }
    } catch (const std::exception& e) {
        mexErrMsgIdAndTxt("eskf:error", "C++ exception: %s", e.what());
    } catch (...) {
        mexErrMsgIdAndTxt("eskf:error", "Unknown C++ exception");
    }
}

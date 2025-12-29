/* mex_sensor_preprocessor.cpp
 * MEX wrapper for sensor preprocessor (Phase3 migration)
 * Implementation code moved to Src/Common/Sensor/sensor_preprocessor.cpp
 */

#include "mex.h"
#include "../Inc/Common/Sensor/sensor_preprocessor.hpp"
#include "mex_type_conv.hpp"
#include <string>
#include <vector>

using namespace common::sensor;

static std::string getCmd(const mxArray* a) {
    char buf[256];
    if (!mxIsChar(a)) return std::string();
    mxGetString(a, buf, sizeof(buf));
    return std::string(buf);
}

void do_preprocess_accel(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 3) {
        mexErrMsgIdAndTxt("mex_sensor_preprocessor:usage", "preprocess_accel requires (a_meas, prev_a)");
    }
    
    // MATLAB配列からC++型へ変換
    cmath_fx::Vector<3, float> a_meas;
    cmath_fx::Vector<3, float> prev_a;
    std::vector<float> tmp(3);
    
    mex_conv::mxArrayToFloatArray(prhs[1], tmp.data(), 3);
    for (int i = 0; i < 3; ++i) a_meas(i, 0) = tmp[i];
    
    mex_conv::mxArrayToFloatArray(prhs[2], tmp.data(), 3);
    for (int i = 0; i < 3; ++i) prev_a(i, 0) = tmp[i];
    
    // 実装関数を呼び出す
    PreprocessResult result = preprocess_accel(a_meas, prev_a);
    
    // 結果をMATLAB配列に変換
    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* out = mxGetPr(plhs[0]);
    for (int i = 0; i < 3; ++i) {
        out[i] = static_cast<double>(result.output(i, 0));
    }
    
    if (nlhs >= 2) {
        plhs[1] = mxCreateLogicalScalar(result.is_outlier);
    }
    if (nlhs >= 3) {
        plhs[2] = mxCreateLogicalScalar(result.no_change);
    }
}

void do_preprocess_mag(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 3) {
        mexErrMsgIdAndTxt("mex_sensor_preprocessor:usage", "preprocess_mag requires (m_meas, prev_m)");
    }
    
    // MATLAB配列からC++型へ変換
    cmath_fx::Vector<3, float> m_meas;
    cmath_fx::Vector<3, float> prev_m;
    std::vector<float> tmp(3);
    
    mex_conv::mxArrayToFloatArray(prhs[1], tmp.data(), 3);
    for (int i = 0; i < 3; ++i) m_meas(i, 0) = tmp[i];
    
    mex_conv::mxArrayToFloatArray(prhs[2], tmp.data(), 3);
    for (int i = 0; i < 3; ++i) prev_m(i, 0) = tmp[i];
    
    // 実装関数を呼び出す
    PreprocessResult result = preprocess_mag(m_meas, prev_m);
    
    // 結果をMATLAB配列に変換
    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* out = mxGetPr(plhs[0]);
    for (int i = 0; i < 3; ++i) {
        out[i] = static_cast<double>(result.output(i, 0));
    }
    
    if (nlhs >= 2) {
        plhs[1] = mxCreateLogicalScalar(result.is_outlier);
    }
    if (nlhs >= 3) {
        plhs[2] = mxCreateLogicalScalar(result.no_change);
    }
}

void do_preprocess_baro(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 2) {
        mexErrMsgIdAndTxt("mex_sensor_preprocessor:usage", "preprocess_baro requires (pressure)");
    }
    
    double pressure = mxGetScalar(prhs[1]);
    
    // 実装関数を呼び出す
    double alt = preprocess_baro(pressure);
    
    // 結果をMATLAB配列に変換
    plhs[0] = mxCreateDoubleScalar(alt);
    if (nlhs >= 2) {
        plhs[1] = mxCreateLogicalScalar(false);
    }
    if (nlhs >= 3) {
        plhs[2] = mxCreateLogicalScalar(false);
    }
}

void do_preprocess_gps(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 5) {
        mexErrMsgIdAndTxt("mex_sensor_preprocessor:usage", "preprocess_gps requires (lat, lon, alt, origin3)");
    }
    
    double lat = mxGetScalar(prhs[1]);
    double lon = mxGetScalar(prhs[2]);
    double alt = mxGetScalar(prhs[3]);
    
    // MATLAB配列からC++型へ変換
    cmath_fx::Vector<3, float> origin;
    std::vector<float> tmp(3);
    mex_conv::mxArrayToFloatArray(prhs[4], tmp.data(), 3);
    for (int i = 0; i < 3; ++i) origin(i, 0) = tmp[i];
    
    // 実装関数を呼び出す
    PreprocessResult result = preprocess_gps(lat, lon, alt, origin);
    
    // 結果をMATLAB配列に変換
    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* out = mxGetPr(plhs[0]);
    for (int i = 0; i < 3; ++i) {
        out[i] = static_cast<double>(result.output(i, 0));
    }
    
    if (nlhs >= 2) {
        plhs[1] = mxCreateLogicalScalar(result.is_outlier);
    }
    if (nlhs >= 3) {
        plhs[2] = mxCreateLogicalScalar(result.no_change);
    }
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]){
    if(nrhs < 1) mexErrMsgIdAndTxt("mex_sensor_preprocessor:usage","Command required");
    std::string cmd = getCmd(prhs[0]);
    if(cmd == "preprocess_accel") do_preprocess_accel(nlhs, plhs, nrhs, prhs);
    else if(cmd == "preprocess_mag") do_preprocess_mag(nlhs, plhs, nrhs, prhs);
    else if(cmd == "preprocess_baro") do_preprocess_baro(nlhs, plhs, nrhs, prhs);
    else if(cmd == "preprocess_gps") do_preprocess_gps(nlhs, plhs, nrhs, prhs);
    else mexErrMsgIdAndTxt("mex_sensor_preprocessor:unknown","Unknown command: %s", cmd.c_str());
}



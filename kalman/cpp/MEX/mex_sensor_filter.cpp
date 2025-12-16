#include "mex.h"
#include "../Common/Sensor/sensor_filter.hpp"
#include "../Common/Math/fixed_matrix.hpp"
#include <string>
#include <iostream>
#include <cstdlib>

using namespace common::sensor;
using namespace cmath_fx;

// 静的インスタンスで状態を保持
static SensorFilterLib filter_lib;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if (nrhs < 1) {
        mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "Command string required.");
    }

    char cmd_buffer[64];
    if (mxGetString(prhs[0], cmd_buffer, sizeof(cmd_buffer)) != 0) {
        mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "First argument must be a command string.");
    }
    std::string cmd(cmd_buffer);

    if (cmd == "reset") {
        filter_lib.reset_all();
        return;
    }

    if (cmd == "reset_zero") {
        filter_lib.reset_all_zero();
        return;
    }
    else if (cmd == "accel") {
        if (nrhs < 2) {
            mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "Accel measurement required.");
        }
        
        // 入力: a_meas [3x1]
        double* a_ptr = mxGetPr(prhs[1]);
        size_t M = mxGetM(prhs[1]);
        size_t N = mxGetN(prhs[1]);
        
        if (M * N != 3) {
            mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "Accel measurement must be 3 elements.");
        }
        
        cmath_fx::FixedMatrix a_meas(3, 1);
        a_meas(0,0) = (float)a_ptr[0];
        a_meas(1,0) = (float)a_ptr[1];
        a_meas(2,0) = (float)a_ptr[2];
        
        // 入力: a_expected [3x1] (オプション)
        cmath_fx::FixedMatrix a_expected(3, 1);
        if (nrhs >= 3) {
            double* exp_ptr = mxGetPr(prhs[2]);
            if (mxGetM(prhs[2]) * mxGetN(prhs[2]) == 3) {
                a_expected(0,0) = (float)exp_ptr[0];
                a_expected(1,0) = (float)exp_ptr[1];
                a_expected(2,0) = (float)exp_ptr[2];
            }
        } else {
            a_expected(0,0) = 0.0f;
            a_expected(1,0) = 0.0f;
            a_expected(2,0) = 0.0f;
        }
        
        bool is_outlier = false;
        cmath_fx::FixedMatrix a_filt = filter_lib.filter_accel(a_meas, a_expected, is_outlier);
        
        // 出力: a_filt [3x1]
        plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
        double* out_ptr = mxGetPr(plhs[0]);
        out_ptr[0] = (double)a_filt(0,0);
        out_ptr[1] = (double)a_filt(1,0);
        out_ptr[2] = (double)a_filt(2,0);
        
        // 出力: is_outlier
        if (nlhs >= 2) {
            plhs[1] = mxCreateLogicalScalar(is_outlier);
        }
    }
    else if (cmd == "mag") {
        if (nrhs < 2) {
            mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "Mag measurement required.");
        }
        
        // 入力: m_meas [3x1]
        double* m_ptr = mxGetPr(prhs[1]);
        if (mxGetM(prhs[1]) * mxGetN(prhs[1]) != 3) {
            mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "Mag measurement must be 3 elements.");
        }
        
        cmath_fx::FixedMatrix m_meas(3, 1);
        m_meas(0,0) = (float)m_ptr[0];
        m_meas(1,0) = (float)m_ptr[1];
        m_meas(2,0) = (float)m_ptr[2];
        
        // 入力: m_expected [3x1] (オプション)
        cmath_fx::FixedMatrix m_expected(3, 1);
        if (nrhs >= 3) {
            double* exp_ptr = mxGetPr(prhs[2]);
            if (mxGetM(prhs[2]) * mxGetN(prhs[2]) == 3) {
                m_expected(0,0) = (float)exp_ptr[0];
                m_expected(1,0) = (float)exp_ptr[1];
                m_expected(2,0) = (float)exp_ptr[2];
            }
        }
        
        bool is_outlier = false;
        cmath_fx::FixedMatrix m_filt = filter_lib.filter_mag(m_meas, m_expected, is_outlier);
        
        // 出力: m_filt [3x1]
        plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
        double* out_ptr = mxGetPr(plhs[0]);
        out_ptr[0] = (double)m_filt(0,0);
        out_ptr[1] = (double)m_filt(1,0);
        out_ptr[2] = (double)m_filt(2,0);
        
        // 出力: is_outlier
        if (nlhs >= 2) {
            plhs[1] = mxCreateLogicalScalar(is_outlier);
        }
    }
    else if (cmd == "gps") {
        if (nrhs < 3) {
            mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "GPS measurement and dt required.");
        }
        
        // 入力: p_meas [3x1]
        double* p_ptr = mxGetPr(prhs[1]);
        if (mxGetM(prhs[1]) * mxGetN(prhs[1]) != 3) {
            mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "GPS measurement must be 3 elements.");
        }
        
        cmath_fx::FixedMatrix p_meas(3, 1);
        p_meas(0,0) = (float)p_ptr[0];
        p_meas(1,0) = (float)p_ptr[1];
        p_meas(2,0) = (float)p_ptr[2];
        
        // 入力: dt
        double dt = mxGetScalar(prhs[2]);
        
        cmath_fx::FixedMatrix p_out, v_out;
        filter_lib.filter_gps(p_meas, (float)dt, p_out, v_out);
        
        // 出力: p_out [3x1]
        plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
        double* p_out_ptr = mxGetPr(plhs[0]);
        p_out_ptr[0] = (double)p_out(0,0);
        p_out_ptr[1] = (double)p_out(1,0);
        p_out_ptr[2] = (double)p_out(2,0);
        
        // 出力: v_out [3x1]
        if (nlhs >= 2) {
            plhs[1] = mxCreateDoubleMatrix(3, 1, mxREAL);
            double* v_out_ptr = mxGetPr(plhs[1]);
            v_out_ptr[0] = (double)v_out(0,0);
            v_out_ptr[1] = (double)v_out(1,0);
            v_out_ptr[2] = (double)v_out(2,0);
        }
    }
    else if (cmd == "baro") {
        if (nrhs < 2) {
            mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "Baro measurement required.");
        }
        
        // 入力: h_meas (scalar)
        double h_meas = mxGetScalar(prhs[1]);
        
        float h_filt = filter_lib.filter_baro((float)h_meas);
        
        // 出力: h_filt (scalar)
        plhs[0] = mxCreateDoubleScalar((double)h_filt);
    }
    else {
        mexErrMsgIdAndTxt("SensorFilter:UnknownCommand", "Unknown command.");
    }
}

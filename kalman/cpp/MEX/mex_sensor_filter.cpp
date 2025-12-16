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
// NoiseEstimator / DivergenceGuard の静的インスタンス
static NoiseEstimator noise_estimator;
static DivergenceGuard divergence_guard;

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
    else if (cmd == "accel_config") {
        if (nrhs < 2 || !mxIsStruct(prhs[1])) {
            mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "accel_config requires a struct argument.");
        }

        const mxArray* s = prhs[1];
        double ema_alpha = 0.3;
        double history_size = 20;
        double threshold_sigma = 3.0;
        double min_std = 0.1;

        // optional fields
        mxArray* f;
        f = mxGetField((mxArray*)s, 0, "ema_alpha");
        if (f && mxIsDouble(f)) ema_alpha = mxGetScalar(f);
        f = mxGetField((mxArray*)s, 0, "history_size");
        if (f && mxIsDouble(f)) history_size = mxGetScalar(f);
        f = mxGetField((mxArray*)s, 0, "threshold_sigma");
        if (f && mxIsDouble(f)) threshold_sigma = mxGetScalar(f);
        f = mxGetField((mxArray*)s, 0, "min_std");
        if (f && mxIsDouble(f)) min_std = mxGetScalar(f);

        filter_lib.set_accel_config((float)ema_alpha, (int)history_size, (float)threshold_sigma, (float)min_std);
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
    // Note: gyro handling removed — gyro filtering is deprecated and should be
    // performed by MATLAB-side NoOp/alternate implementation. If callers still
    // invoke 'gyro' they will receive an UnknownCommand error below.
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
    else if (cmd == "get_R") {
        if (nrhs < 2 || !mxIsChar(prhs[1])) {
            mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "get_R requires sensor type string.");
        }
        char buf[64];
        if (mxGetString(prhs[1], buf, sizeof(buf)) != 0) mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "Invalid sensor type string.");
        std::string s(buf);

        // call C++ NoiseEstimator get_R_matrix
        cmath_fx::FixedMatrix R = noise_estimator.get_R_matrix(s.c_str());

        // convert to mxArray
        plhs[0] = mxCreateDoubleMatrix(R.rows, R.cols, mxREAL);
        double* out = mxGetPr(plhs[0]);
        for (mwSize j = 0; j < (mwSize)R.cols; ++j) {
            for (mwSize i = 0; i < (mwSize)R.rows; ++i) {
                out[j*R.rows + i] = (double)R((int)i,(int)j);
            }
        }
        return;
    }
    else if (cmd == "noise_estimate") {
        // noise_estimate(sensor_type, innovation, H, P_pred)
        if (nrhs < 4) mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "noise_estimate requires 4 args.");
        if (!mxIsChar(prhs[1])) mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "sensor_type must be string.");
        char buf[64];
        if (mxGetString(prhs[1], buf, sizeof(buf)) != 0) mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "Invalid sensor type string.");
        std::string s(buf);

        // innovation vector
        double* innov_ptr = mxGetPr(prhs[2]);
        mwSize innov_m = mxGetM(prhs[2]);
        mwSize innov_n = mxGetN(prhs[2]);
        mwSize innov_len = innov_m * innov_n;
        cmath_fx::FixedMatrix innov((int)innov_len, 1);
        for (mwSize i = 0; i < innov_len; ++i) innov((int)i,0) = (float)innov_ptr[i];

        // H matrix
        double* H_ptr = mxGetPr(prhs[3]);
        mwSize H_m = mxGetM(prhs[3]);
        mwSize H_n = mxGetN(prhs[3]);
        cmath_fx::FixedMatrix H((int)H_m, (int)H_n);
        for (mwSize j = 0; j < H_n; ++j) for (mwSize i = 0; i < H_m; ++i) H((int)i,(int)j) = (float)H_ptr[j*H_m + i];

        // P_pred matrix (4th arg)
        double* P_ptr = mxGetPr(prhs[4]);
        mwSize P_m = mxGetM(prhs[4]);
        mwSize P_n = mxGetN(prhs[4]);
        cmath_fx::FixedMatrix P((int)P_m, (int)P_n);
        for (mwSize j = 0; j < P_n; ++j) for (mwSize i = 0; i < P_m; ++i) P((int)i,(int)j) = (float)P_ptr[j*P_m + i];

        noise_estimator.estimate(s.c_str(), innov, H, P);
        return;
    }
    else if (cmd == "divergence_check") {
        // divergence_check(sensor_name, innovation, dx_in)
        if (nrhs < 3) mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "divergence_check requires sensor_name and innovation and dx_in.");
        if (!mxIsChar(prhs[1])) mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "sensor_name must be string.");
        char buf2[64];
        if (mxGetString(prhs[1], buf2, sizeof(buf2)) != 0) mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "Invalid sensor name string.");
        std::string sname(buf2);

        // innovation
        double* innov_ptr = mxGetPr(prhs[2]);
        mwSize innov_len = mxGetM(prhs[2]) * mxGetN(prhs[2]);
        cmath_fx::FixedMatrix innov((int)innov_len, 1);
        for (mwSize i = 0; i < innov_len; ++i) innov((int)i,0) = (float)innov_ptr[i];

        // dx_in
        double* dx_ptr = mxGetPr(prhs[3]);
        mwSize dx_len = mxGetM(prhs[3]) * mxGetN(prhs[3]);
        cmath_fx::FixedMatrix dx((int)dx_len, 1);
        for (mwSize i = 0; i < dx_len; ++i) dx((int)i,0) = (float)dx_ptr[i];

        bool was_attenuated = false;
        bool should_skip = divergence_guard.check_and_attenuate(sname.c_str(), innov, dx, was_attenuated);

        // output dx_out
        plhs[0] = mxCreateDoubleMatrix((mwSize)dx.rows, (mwSize)dx.cols, mxREAL);
        double* out_dx = mxGetPr(plhs[0]);
        for (mwSize j = 0; j < (mwSize)dx.cols; ++j) for (mwSize i = 0; i < (mwSize)dx.rows; ++i) out_dx[j*dx.rows + i] = (double)dx((int)i,(int)j);

        // should_skip
        if (nlhs >= 2) plhs[1] = mxCreateLogicalScalar(should_skip);
        // was_attenuated
        if (nlhs >= 3) plhs[2] = mxCreateLogicalScalar(was_attenuated);

        return;
    }
    else if (cmd == "divergence_regularize") {
        // divergence_regularize(P)
        if (nrhs < 2) mexErrMsgIdAndTxt("SensorFilter:InvalidInput", "divergence_regularize requires P matrix.");
        double* P_ptr = mxGetPr(prhs[1]);
        mwSize P_m = mxGetM(prhs[1]);
        mwSize P_n = mxGetN(prhs[1]);
        cmath_fx::FixedMatrix P((int)P_m, (int)P_n);
        for (mwSize j = 0; j < P_n; ++j) for (mwSize i = 0; i < P_m; ++i) P((int)i,(int)j) = (float)P_ptr[j*P_m + i];

        divergence_guard.regularize_covariance(P);

        plhs[0] = mxCreateDoubleMatrix(P.rows, P.cols, mxREAL);
        double* out = mxGetPr(plhs[0]);
        for (mwSize j = 0; j < (mwSize)P.cols; ++j) for (mwSize i = 0; i < (mwSize)P.rows; ++i) out[j*P.rows + i] = (double)P((int)i,(int)j);
        return;
    }
    else {
        mexErrMsgIdAndTxt("SensorFilter:UnknownCommand", "Unknown command.");
    }
}

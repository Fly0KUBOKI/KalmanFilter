#include "mex.h"
#include "../include/Common/Sensor/sensor_filter.hpp"
#include "../include/Common/Math/fixed_matrix.hpp"
#include "mex_type_conv.hpp"
#include <string>
#include <cstdlib>
#include <vector>

using namespace common::sensor;

static SensorFilterLib filter_lib;



void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if (nrhs < 1) mexErrMsgIdAndTxt("SensorFilter:Usage", "Command required");
    char cmd[128];
    if (mxGetString(prhs[0], cmd, sizeof(cmd))) mexErrMsgIdAndTxt("SensorFilter:Usage", "Cmd must be string");
    
    std::string cmdstr(cmd);
    
    if (cmdstr == "reset") {
        filter_lib.reset_all();
        return;
    }
    if (cmdstr == "reset_zero") {
        filter_lib.reset_all_zero();
        return;
    }
    if (cmdstr == "log") {
        if (nrhs < 2) mexErrMsgIdAndTxt("SensorFilter:Usage", "log requires 'on' or 'off'");
        char arg[16];
        if (mxGetString(prhs[1], arg, sizeof(arg))) mexErrMsgIdAndTxt("SensorFilter:Usage", "log arg must be string");
        std::string a(arg);
        if (a == "on") {
            sensor_log_enable(true);
        } else if (a == "off") {
            sensor_log_enable(false);
        } else {
            mexErrMsgIdAndTxt("SensorFilter:Usage", "log arg must be 'on' or 'off'");
        }
        return;
    }
    if (cmdstr == "get_R") {
        if (nrhs < 2) mexErrMsgIdAndTxt("SensorFilter:Usage", "get_R requires sensor type");
        char stype[64]; 
        if (mxGetString(prhs[1], stype, sizeof(stype))) mexErrMsgIdAndTxt("SensorFilter:Usage", "sensor_type must be string");
        auto R = filter_lib.noise_estimator.get_R_matrix(stype);
        plhs[0] = mxCreateDoubleMatrix(R.rows, R.cols, mxREAL);
        double* out = mxGetPr(plhs[0]);
        for (int j=0; j<R.cols; ++j) for (int i=0; i<R.rows; ++i) out[j*R.rows + i] = (double)R(i,j);
        return;
    }
    if (cmdstr == "noise_estimate") {
        if (nrhs < 5) mexErrMsgIdAndTxt("SensorFilter:Usage", "noise_estimate requires 4 args (sensor, innov, H, P)");
        char stype[64];
        if (mxGetString(prhs[1], stype, sizeof(stype))) mexErrMsgIdAndTxt("SensorFilter:Usage", "sensor_type must be string");
        int innov_len = mxGetM(prhs[2]) * mxGetN(prhs[2]);
        auto innov = cmath_fx::FixedMatrix(innov_len, 1);
        std::vector<float> innov_tmp(static_cast<size_t>(innov_len));
        mex_conv::mxArrayToFloatArray(prhs[2], innov_tmp.data(), static_cast<size_t>(innov_len));
        for(int i=0; i<innov_len; ++i) innov(i,0) = innov_tmp[i];

        int H_m = mxGetM(prhs[3]); int H_n = mxGetN(prhs[3]);
        auto H = cmath_fx::FixedMatrix(H_m, H_n);
        std::vector<float> H_tmp(static_cast<size_t>(H_m) * static_cast<size_t>(H_n));
        mex_conv::mxArrayToFloatArray(prhs[3], H_tmp.data(), static_cast<size_t>(H_m) * static_cast<size_t>(H_n));
        for(int j=0;j<H_n;++j) for(int i=0;i<H_m;++i) H(i,j) = H_tmp[j*H_m + i];

        int P_m = mxGetM(prhs[4]); int P_n = mxGetN(prhs[4]);
        auto P = cmath_fx::FixedMatrix(P_m, P_n);
        std::vector<float> P_tmp(static_cast<size_t>(P_m) * static_cast<size_t>(P_n));
        mex_conv::mxArrayToFloatArray(prhs[4], P_tmp.data(), static_cast<size_t>(P_m) * static_cast<size_t>(P_n));
        for(int j=0;j<P_n;++j) for(int i=0;i<P_m;++i) P(i,j) = P_tmp[j*P_m + i];

        filter_lib.noise_estimator.estimate(stype, innov, H, P);
        return;
    }
    if (cmdstr == "accel_config") {
        if (nrhs < 2) mexErrMsgIdAndTxt("SensorFilter:Usage", "accel_config requires a config struct or vector");
        // Accept struct with fields or numeric vector [ema_alpha, history_size, threshold_sigma, min_std]
        if (mxIsStruct(prhs[1])) {
            mxArray* f;
            float ema_alpha = 0.3f; int history_size = 10; float thresh = 3.0f; float min_std = 0.1f;
            f = mxGetField(prhs[1], 0, "ema_alpha"); if(f) ema_alpha = mex_conv::mxGetScalarAsFloat(f);
            f = mxGetField(prhs[1], 0, "history_size"); if(f) history_size = static_cast<int>(mex_conv::mxGetScalarAsFloat(f));
            f = mxGetField(prhs[1], 0, "threshold_sigma"); if(f) thresh = mex_conv::mxGetScalarAsFloat(f);
            f = mxGetField(prhs[1], 0, "min_std"); if(f) min_std = mex_conv::mxGetScalarAsFloat(f);
            filter_lib.set_accel_config(ema_alpha, history_size, thresh, min_std);
        } else if (mxIsDouble(prhs[1])) {
            int len = mxGetNumberOfElements(prhs[1]);
            std::vector<float> dp_tmp(static_cast<size_t>(len));
            mex_conv::mxArrayToFloatArray(prhs[1], dp_tmp.data(), static_cast<size_t>(len));
            float ema_alpha = (len>0)?dp_tmp[0]:0.3f;
            int history_size = (len>1)?static_cast<int>(dp_tmp[1]):10;
            float thresh = (len>2)?dp_tmp[2]:3.0f;
            float min_std = (len>3)?dp_tmp[3]:0.1f;
            filter_lib.set_accel_config(ema_alpha, history_size, thresh, min_std);
        } else {
            mexErrMsgIdAndTxt("SensorFilter:Usage","accel_config: unsupported arg");
        }
        return;
    }

    if (cmdstr == "accel") {
        if (nrhs < 3) mexErrMsgIdAndTxt("SensorFilter:Usage","accel requires a_meas and a_expected");
        int m_m = mxGetM(prhs[1]); int m_n = mxGetN(prhs[1]);
        int e_m = mxGetM(prhs[2]); int e_n = mxGetN(prhs[2]);
        if (m_m*m_n !=3 || e_m*e_n!=3) mexErrMsgIdAndTxt("SensorFilter:Usage","accel vectors must be length 3");
        cmath_fx::FixedMatrix a_meas(3,1), a_exp(3,1);
        float a_tmp[3]; float b_tmp[3];
        mex_conv::mxArrayToFloatArray(prhs[1], a_tmp, 3);
        mex_conv::mxArrayToFloatArray(prhs[2], b_tmp, 3);
        for(int i=0;i<3;++i) a_meas(i,0) = a_tmp[i];
        for(int i=0;i<3;++i) a_exp(i,0) = b_tmp[i];
        bool is_out = false;
        auto a_filt = filter_lib.filter_accel(a_meas, a_exp, is_out);
        plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL); double* out = mxGetPr(plhs[0]); for(int i=0;i<3;++i) out[i]=(double)a_filt(i,0);
        if(nlhs>=2) plhs[1]=mxCreateLogicalScalar(is_out);
        return;
    }

    if (cmdstr == "mag") {
        if (nrhs < 3) mexErrMsgIdAndTxt("SensorFilter:Usage","mag requires m_meas and m_expected");
        int m_m = mxGetM(prhs[1]); int m_n = mxGetN(prhs[1]);
        int e_m = mxGetM(prhs[2]); int e_n = mxGetN(prhs[2]);
        if (m_m*m_n !=3 || e_m*e_n!=3) mexErrMsgIdAndTxt("SensorFilter:Usage","mag vectors must be length 3");
        cmath_fx::FixedMatrix m_meas(3,1), m_exp(3,1);
        float a_tmp[3]; float b_tmp[3];
        mex_conv::mxArrayToFloatArray(prhs[1], a_tmp, 3);
        mex_conv::mxArrayToFloatArray(prhs[2], b_tmp, 3);
        for(int i=0;i<3;++i) m_meas(i,0) = a_tmp[i];
        for(int i=0;i<3;++i) m_exp(i,0) = b_tmp[i];
        bool is_out = false;
        auto m_filt = filter_lib.filter_mag(m_meas, m_exp, is_out);
        plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL); double* out = mxGetPr(plhs[0]); for(int i=0;i<3;++i) out[i]=(double)m_filt(i,0);
        if(nlhs>=2) plhs[1]=mxCreateLogicalScalar(is_out);
        return;
    }

    if (cmdstr == "gps") {
        if (nrhs < 3) mexErrMsgIdAndTxt("SensorFilter:Usage","gps requires gps_pos (3x1) and dt");
        int p_m = mxGetM(prhs[1]); int p_n = mxGetN(prhs[1]);
        if (p_m*p_n!=3) mexErrMsgIdAndTxt("SensorFilter:Usage","gps_pos must be length 3");
        float pp_tmp[3]; cmath_fx::FixedMatrix gps_pos(3,1);
        mex_conv::mxArrayToFloatArray(prhs[1], pp_tmp, 3);
        for(int i=0;i<3;++i) gps_pos(i,0) = pp_tmp[i];
        float dt = mex_conv::mxGetScalarAsFloat(prhs[2]);
        cmath_fx::FixedMatrix pos_out(3,1), vel_out(3,1);
        filter_lib.filter_gps(gps_pos, (float)dt, pos_out, vel_out);
        plhs[0]=mxCreateDoubleMatrix(3,1,mxREAL); double* out1=mxGetPr(plhs[0]); for(int i=0;i<3;++i) out1[i]=(double)pos_out(i,0);
        if(nlhs>=2) { plhs[1]=mxCreateDoubleMatrix(3,1,mxREAL); double* out2=mxGetPr(plhs[1]); for(int i=0;i<3;++i) out2[i]=(double)vel_out(i,0); }
        return;
    }

    if (cmdstr == "baro") {
        if (nrhs < 2) mexErrMsgIdAndTxt("SensorFilter:Usage","baro requires pressure scalar");
        float p = mex_conv::mxGetScalarAsFloat(prhs[1]);
        // Convert pressure (Pa) to altitude (m) using barometric formula
        const float P0 = 101325.0f;
        const float ALT_COEFF = 44330.0f; // matches MATLAB SensorBaroFilter default
        float p_frac = (float)(p / P0);
        if (p_frac < 1e-9f) p_frac = 1e-9f;
        float alt_meas = ALT_COEFF * (1.0f - powf(p_frac, 0.1903f));
        float pf = filter_lib.filter_baro(alt_meas);
        plhs[0]=mxCreateDoubleMatrix(1,1,mxREAL); *mxGetPr(plhs[0])=(double)pf;
        if (nlhs >= 2) {
            /* Provide is_outlier flag for MATLAB callers that expect two outputs */
            plhs[1] = mxCreateLogicalScalar(false);
        }
        return;
    }
    if (cmdstr == "divergence_check") {
        if (nrhs < 4) mexErrMsgIdAndTxt("SensorFilter:Usage", "divergence_check requires sensor_name, innovation, dx_in");
        char sname[64];
        if (mxGetString(prhs[1], sname, sizeof(sname))) mexErrMsgIdAndTxt("SensorFilter:Usage", "sensor_name must be string");
        
        int innov_len = mxGetM(prhs[2]) * mxGetN(prhs[2]);
        auto innov = cmath_fx::FixedMatrix(innov_len, 1);
        std::vector<float> innov_tmp(static_cast<size_t>(innov_len));
        mex_conv::mxArrayToFloatArray(prhs[2], innov_tmp.data(), static_cast<size_t>(innov_len));
        for(int i=0; i<innov_len; ++i) innov(i,0) = innov_tmp[i];

        int dx_len = mxGetM(prhs[3]) * mxGetN(prhs[3]);
        auto dx = cmath_fx::FixedMatrix(dx_len, 1);
        std::vector<float> dx_tmp(static_cast<size_t>(dx_len));
        mex_conv::mxArrayToFloatArray(prhs[3], dx_tmp.data(), static_cast<size_t>(dx_len));
        for(int i=0; i<dx_len; ++i) dx(i,0) = dx_tmp[i];
        
        bool was_att = false;
        bool skip = filter_lib.divergence_guard.check_and_attenuate(sname, innov, dx, was_att);
        
        plhs[0] = mxCreateDoubleMatrix(dx.rows, dx.cols, mxREAL);
        double* out = mxGetPr(plhs[0]);
        for(int j=0;j<dx.cols;++j) for(int i=0;i<dx.rows;++i) out[j*dx.rows+i]=(double)dx(i,j);
        if(nlhs>=2) plhs[1] = mxCreateLogicalScalar(skip);
        if(nlhs>=3) plhs[2] = mxCreateLogicalScalar(was_att);
        return;
    }
    if (cmdstr == "divergence_regularize") {
        if (nrhs < 2) mexErrMsgIdAndTxt("SensorFilter:Usage", "divergence_regularize requires P");
        int P_m = mxGetM(prhs[1]); int P_n = mxGetN(prhs[1]);
        auto P = cmath_fx::FixedMatrix(P_m, P_n);
        std::vector<float> P_tmp(static_cast<size_t>(P_m) * static_cast<size_t>(P_n));
        mex_conv::mxArrayToFloatArray(prhs[1], P_tmp.data(), static_cast<size_t>(P_m) * static_cast<size_t>(P_n));
        for(int j=0;j<P_n;++j) for(int i=0;i<P_m;++i) P(i,j) = P_tmp[j*P_m + i];
        filter_lib.divergence_guard.regularize_covariance(P);
        plhs[0] = mxCreateDoubleMatrix(P.rows, P.cols, mxREAL);
        double* out = mxGetPr(plhs[0]);
        for(int j=0;j<P.cols;++j) for(int i=0;i<P.rows;++i) out[j*P.rows+i]=(double)P(i,j);
        return;
    }
    
    mexErrMsgIdAndTxt("SensorFilter:UnknownCmd", "Unknown command");
}




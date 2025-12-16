#include "mex.h"
#include "../include/Common/Sensor/sensor_filter.hpp"
#include "../include/Common/Math/fixed_matrix.hpp"
#include <string>
#include <cstdlib>

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
        double* ip = mxGetPr(prhs[2]);
        for(int i=0; i<innov_len; ++i) innov(i,0) = (float)ip[i];
        
        int H_m = mxGetM(prhs[3]); int H_n = mxGetN(prhs[3]);
        auto H = cmath_fx::FixedMatrix(H_m, H_n);
        double* Hp = mxGetPr(prhs[3]);
        for(int j=0;j<H_n;++j) for(int i=0;i<H_m;++i) H(i,j)=(float)Hp[j*H_m+i];
        
        int P_m = mxGetM(prhs[4]); int P_n = mxGetN(prhs[4]);
        auto P = cmath_fx::FixedMatrix(P_m, P_n);
        double* Pp = mxGetPr(prhs[4]);
        for(int j=0;j<P_n;++j) for(int i=0;i<P_m;++i) P(i,j)=(float)Pp[j*P_m+i];
        
        filter_lib.noise_estimator.estimate(stype, innov, H, P);
        return;
    }
    if (cmdstr == "divergence_check") {
        if (nrhs < 4) mexErrMsgIdAndTxt("SensorFilter:Usage", "divergence_check requires sensor_name, innovation, dx_in");
        char sname[64];
        if (mxGetString(prhs[1], sname, sizeof(sname))) mexErrMsgIdAndTxt("SensorFilter:Usage", "sensor_name must be string");
        
        int innov_len = mxGetM(prhs[2]) * mxGetN(prhs[2]);
        auto innov = cmath_fx::FixedMatrix(innov_len, 1);
        double* ip = mxGetPr(prhs[2]);
        for(int i=0; i<innov_len; ++i) innov(i,0) = (float)ip[i];
        
        int dx_len = mxGetM(prhs[3]) * mxGetN(prhs[3]);
        auto dx = cmath_fx::FixedMatrix(dx_len, 1);
        double* dxp = mxGetPr(prhs[3]);
        for(int i=0; i<dx_len; ++i) dx(i,0) = (float)dxp[i];
        
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
        double* Pp = mxGetPr(prhs[1]);
        for(int j=0;j<P_n;++j) for(int i=0;i<P_m;++i) P(i,j)=(float)Pp[j*P_m+i];
        filter_lib.divergence_guard.regularize_covariance(P);
        plhs[0] = mxCreateDoubleMatrix(P.rows, P.cols, mxREAL);
        double* out = mxGetPr(plhs[0]);
        for(int j=0;j<P.cols;++j) for(int i=0;i<P.rows;++i) out[j*P.rows+i]=(double)P(i,j);
        return;
    }
    
    mexErrMsgIdAndTxt("SensorFilter:UnknownCmd", "Unknown command");
}




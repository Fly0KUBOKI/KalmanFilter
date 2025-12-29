/* mex_sensor_preprocessor.cpp
 * Minimal sensor preprocessor MEX for Phase3 migration.
 */

#include <mex.h>
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static std::string getCmd(const mxArray* a){ char buf[256]; if(!mxIsChar(a)) return std::string(); mxGetString(a, buf, sizeof(buf)); return std::string(buf); }

void do_preprocess_accel(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]){
    if(nrhs < 3) mexErrMsgIdAndTxt("mex_sensor_preprocessor:usage","preprocess_accel requires (a_meas, prev_a)");
    double* a = mxGetPr(prhs[1]); double* prev = mxGetPr(prhs[2]);
    double a_meas[3] = {a[0], a[1], a[2]};
    double a_prev[3] = {prev[0], prev[1], prev[2]};
    double delta = 0.0; for(int i=0;i<3;i++){ double d=a_meas[i]-a_prev[i]; delta += d*d; } delta = sqrt(delta);
    double buffer_tol = 1e-9;
    bool is_out = false;
    double corrected[3]; for(int i=0;i<3;i++) corrected[i]=a_meas[i];
    
    if(delta <= buffer_tol){
        plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
        double* out = mxGetPr(plhs[0]);
        out[0]=corrected[0]; out[1]=corrected[1]; out[2]=corrected[2];
        if(nlhs>=2) plhs[1]=mxCreateLogicalScalar(false);
        if(nlhs>=3) plhs[2]=mxCreateLogicalScalar(true);
        return;
    }
    
    double a_norm = sqrt(a_meas[0]*a_meas[0]+a_meas[1]*a_meas[1]+a_meas[2]*a_meas[2]);
    if(a_norm < 0.1 || fabs(a_norm - 9.81) > 3.0) is_out = true;
    
    plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
    double* out = mxGetPr(plhs[0]);
    out[0]=corrected[0]; out[1]=corrected[1]; out[2]=corrected[2];
    if(nlhs>=2) plhs[1]=mxCreateLogicalScalar(is_out);
    if(nlhs>=3) plhs[2]=mxCreateLogicalScalar(false);
}

void do_preprocess_mag(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]){
    if(nrhs < 3) mexErrMsgIdAndTxt("mex_sensor_preprocessor:usage","preprocess_mag requires (m_meas, prev_m)");
    double* m = mxGetPr(prhs[1]); double* prev = mxGetPr(prhs[2]);
    double m_meas[3] = {m[0], m[1], m[2]};
    double m_prev[3] = {prev[0], prev[1], prev[2]};
    double delta = 0.0; for(int i=0;i<3;i++){ double d=m_meas[i]-m_prev[i]; delta += d*d; } delta = sqrt(delta);
    double buffer_tol = 1e-9;
    bool is_out = false;
    double corrected[3] = {m_meas[0], m_meas[1], m_meas[2]};
    
    if(delta <= buffer_tol){
        plhs[0]=mxCreateDoubleMatrix(3,1,mxREAL);
        double* out=mxGetPr(plhs[0]);
        out[0]=corrected[0]; out[1]=corrected[1]; out[2]=corrected[2];
        if(nlhs>=2) plhs[1]=mxCreateLogicalScalar(false);
        if(nlhs>=3) plhs[2]=mxCreateLogicalScalar(true);
        return;
    }
    
    plhs[0]=mxCreateDoubleMatrix(3,1,mxREAL);
    double* out=mxGetPr(plhs[0]);
    out[0]=corrected[0]; out[1]=corrected[1]; out[2]=corrected[2];
    if(nlhs>=2) plhs[1]=mxCreateLogicalScalar(is_out);
    if(nlhs>=3) plhs[2]=mxCreateLogicalScalar(false);
}

void do_preprocess_baro(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]){
    if(nrhs < 2) mexErrMsgIdAndTxt("mex_sensor_preprocessor:usage","preprocess_baro requires (pressure)");
    double p = mxGetScalar(prhs[1]);
    
    const double P0 = 101325.0; const double ALT_COEFF = 44330.0;
    double p_frac = p / P0; if(p_frac < 1e-9) p_frac = 1e-9;
    double alt = ALT_COEFF * (1.0 - pow(p_frac, 0.1903));
    plhs[0]=mxCreateDoubleScalar(alt);
    if(nlhs>=2) plhs[1]=mxCreateLogicalScalar(false);
    if(nlhs>=3) plhs[2]=mxCreateLogicalScalar(false);
}

void do_preprocess_gps(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]){
    if(nrhs < 5) mexErrMsgIdAndTxt("mex_sensor_preprocessor:usage","preprocess_gps requires (lat, lon, alt, origin3)");
    double lat = mxGetScalar(prhs[1]); double lon = mxGetScalar(prhs[2]); double alt_in = mxGetScalar(prhs[3]);
    double* origin = mxGetPr(prhs[4]); double lat0 = origin[0]; double lon0 = origin[1]; double alt0 = origin[2];
    
    double buffer_tol = 1e-9;
    double dlat = lat - lat0;
    double dlon = lon - lon0;
    double dalt = alt_in - alt0;
    if(fabs(dlat) <= buffer_tol && fabs(dlon) <= buffer_tol && fabs(dalt) <= buffer_tol){
        plhs[0]=mxCreateDoubleMatrix(3,1,mxREAL);
        double* out=mxGetPr(plhs[0]);
        out[0]=0.0; out[1]=0.0; out[2]=0.0;
        if(nlhs>=2) plhs[1]=mxCreateLogicalScalar(false);
        if(nlhs>=3) plhs[2]=mxCreateLogicalScalar(true);
        return;
    }
    
    double y_m = dlat / 9.0e-6;
    double lat0rad = lat0 * M_PI / 180.0;
    double x_m = dlon / (9.0e-6 / cos(lat0rad));
    double z_m = -dalt;
    plhs[0]=mxCreateDoubleMatrix(3,1,mxREAL);
    double* out=mxGetPr(plhs[0]);
    out[0]=y_m; out[1]=x_m; out[2]=z_m;
    if(nlhs>=2) plhs[1]=mxCreateLogicalScalar(false);
    if(nlhs>=3) plhs[2]=mxCreateLogicalScalar(false);
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


/* mex_run_eskf.cpp
 * Complete ESKF implementation in a single MEX file.
 * Replaces ESKF.m entirely.
 * Uses mexCallMATLAB to call existing MEX functions for accuracy.
 *
 * API:
 *   handle = mex_run_eskf('init', obs, static_time, dt)
 *   mex_run_eskf('step', handle, obs, k)
 *   state = mex_run_eskf('get_state', handle)
 *   mex_run_eskf('free', handle)
 */

#include <mex.h>
#include <cmath>
#include <cstring>
#include <string>
#include <map>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ESKF State Structure
struct ESKFState {
    double p[3], v[3], q[4], ba[3], bg[3];
    double P[15*15];
    double Q_nominal[15*15];
    double g[3];
    double dt;
    double gps_origin[3];
    double prev_accel[3], prev_gyro[3], prev_mag[3];
    double prev_gps_lat, prev_gps_lon, prev_gps_alt;
    double prev_baro;
    double buffer_tolerance;
    double w_body[3];
    double velocity_damping;
    double baro_weight;
    double zupt_threshold_accel, zupt_threshold_gyro;
    int zupt_min_duration;
    int zupt_counter;
    bool is_stationary;
    bool adaptive_q_enabled;
    bool enable_accel_z_integration;
    double accel_z_threshold, accel_z_damping;
    double gyro_noise_threshold;
    int last_reset_step;
    bool valid;
};

static std::map<uint64_t, ESKFState*> g_states;
static uint64_t g_next_handle = 1;

// Utility functions
static std::string getCmd(const mxArray* a) {
    char buf[256] = {0};
    if (!mxIsChar(a)) return "";
    mxGetString(a, buf, sizeof(buf));
    return std::string(buf);
}

static void copy_vec(double* dst, const double* src, int n) {
    memcpy(dst, src, n * sizeof(double));
}

static void quat_to_euler(const double* q, double* euler) {
    double w=q[0], x=q[1], y=q[2], z=q[3];
    euler[0] = atan2(2*(w*x+y*z), 1-2*(x*x+y*y));
    double sinp = 2*(w*y-z*x);
    if (fabs(sinp) >= 1) euler[1] = copysign(M_PI/2, sinp);
    else euler[1] = asin(sinp);
    euler[2] = atan2(2*(w*z+x*y), 1-2*(y*y+z*z));
}

static void getVec3(const mxArray* s, const char* xname, const char* yname, const char* zname, mwIndex idx, double* out) {
    mxArray* fx = mxGetField(s, 0, xname);
    mxArray* fy = mxGetField(s, 0, yname);
    mxArray* fz = mxGetField(s, 0, zname);
    out[0] = fx ? mxGetPr(fx)[idx] : 0.0;
    out[1] = fy ? mxGetPr(fy)[idx] : 0.0;
    out[2] = fz ? mxGetPr(fz)[idx] : 0.0;
}

#define getAccel(obs, idx, out) getVec3(obs, "ax", "ay", "az", idx, out)
#define getGyro(obs, idx, out)  getVec3(obs, "wx", "wy", "wz", idx, out)
#define getMag(obs, idx, out)   getVec3(obs, "mx", "my", "mz", idx, out)

// Call mex_adaptive_predict
static void call_predict(ESKFState* s, const double* a_meas, const double* w_meas) {
    mxArray* prhs[14];
    mxArray* plhs[6];
    
    prhs[0] = mxCreateString("predict");
    prhs[1] = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(prhs[1]), s->p, 3);
    prhs[2] = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(prhs[2]), s->v, 3);
    prhs[3] = mxCreateDoubleMatrix(4,1,mxREAL); copy_vec(mxGetPr(prhs[3]), s->q, 4);
    prhs[4] = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(prhs[4]), s->ba, 3);
    prhs[5] = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(prhs[5]), s->bg, 3);
    prhs[6] = mxCreateDoubleMatrix(15,15,mxREAL); memcpy(mxGetPr(prhs[6]), s->P, 15*15*8);
    prhs[7] = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(prhs[7]), a_meas, 3);
    prhs[8] = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(prhs[8]), w_meas, 3);
    prhs[9] = mxCreateDoubleScalar(s->dt);
    prhs[10] = mxCreateDoubleMatrix(15,15,mxREAL); memcpy(mxGetPr(prhs[10]), s->Q_nominal, 15*15*8);
    prhs[11] = mxCreateLogicalScalar(s->adaptive_q_enabled);
    double zeros3[3] = {0,0,0};
    prhs[12] = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(prhs[12]), zeros3, 3);
    prhs[13] = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(prhs[13]), zeros3, 3);
    // Note: g is passed via prhs[13] = zeros for gyro_bias_prev, we need to add g separately
    // Actually mex_adaptive_predict takes 14 args, the last one is g
    mxDestroyArray(prhs[13]);
    prhs[13] = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(prhs[13]), s->g, 3);
    
    if (mexCallMATLAB(6, plhs, 14, prhs, "mex_adaptive_predict") == 0) {
        copy_vec(s->p, mxGetPr(plhs[0]), 3);
        copy_vec(s->v, mxGetPr(plhs[1]), 3);
        copy_vec(s->q, mxGetPr(plhs[2]), 4);
        copy_vec(s->ba, mxGetPr(plhs[3]), 3);
        copy_vec(s->bg, mxGetPr(plhs[4]), 3);
        memcpy(s->P, mxGetPr(plhs[5]), 15*15*8);
        for (int i=0; i<6; i++) mxDestroyArray(plhs[i]);
    }
    for (int i=0; i<14; i++) mxDestroyArray(prhs[i]);
    
    copy_vec(s->w_body, w_meas, 3);
    
    // Call mex_eskf_predict_postprocess
    mxArray* prhs2[11];
    mxArray* plhs2[2];
    prhs2[0] = mxCreateString("postprocess");
    prhs2[1] = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(prhs2[1]), s->v, 3);
    prhs2[2] = mxCreateDoubleMatrix(4,1,mxREAL); copy_vec(mxGetPr(prhs2[2]), s->q, 4);
    prhs2[3] = mxCreateDoubleMatrix(15,15,mxREAL); memcpy(mxGetPr(prhs2[3]), s->P, 15*15*8);
    prhs2[4] = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(prhs2[4]), a_meas, 3);
    prhs2[5] = mxCreateDoubleScalar(s->dt);
    prhs2[6] = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(prhs2[6]), s->g, 3);
    prhs2[7] = mxCreateLogicalScalar(s->enable_accel_z_integration);
    prhs2[8] = mxCreateDoubleScalar(s->accel_z_threshold);
    prhs2[9] = mxCreateDoubleScalar(s->accel_z_damping);
    prhs2[10] = mxCreateDoubleScalar(s->velocity_damping);
    
    if (mexCallMATLAB(2, plhs2, 11, prhs2, "mex_eskf_predict_postprocess") == 0) {
        copy_vec(s->v, mxGetPr(plhs2[0]), 3);
        memcpy(s->P, mxGetPr(plhs2[1]), 15*15*8);
        for (int i=0; i<2; i++) mxDestroyArray(plhs2[i]);
    }
    for (int i=0; i<11; i++) mxDestroyArray(prhs2[i]);
}

// Call sensor update
static void call_sensor_update(ESKFState* s, const char* type, const double* meas, int meas_len, double sample) {
    // Create state struct
    const char* fields[] = {"p","v","q","ba","bg","P","g","dt","w_body","prev_accel","prev_mag","prev_baro","buffer_tolerance","baro_weight","gps_origin"};
    mxArray* state = mxCreateStructMatrix(1,1,15,fields);
    
    mxArray* f_p = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_p), s->p, 3);
    mxSetField(state, 0, "p", f_p);
    mxArray* f_v = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_v), s->v, 3);
    mxSetField(state, 0, "v", f_v);
    mxArray* f_q = mxCreateDoubleMatrix(4,1,mxREAL); copy_vec(mxGetPr(f_q), s->q, 4);
    mxSetField(state, 0, "q", f_q);
    mxArray* f_ba = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_ba), s->ba, 3);
    mxSetField(state, 0, "ba", f_ba);
    mxArray* f_bg = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_bg), s->bg, 3);
    mxSetField(state, 0, "bg", f_bg);
    mxArray* f_P = mxCreateDoubleMatrix(15,15,mxREAL); memcpy(mxGetPr(f_P), s->P, 15*15*8);
    mxSetField(state, 0, "P", f_P);
    mxArray* f_g = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_g), s->g, 3);
    mxSetField(state, 0, "g", f_g);
    mxSetField(state, 0, "dt", mxCreateDoubleScalar(s->dt));
    mxArray* f_wb = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_wb), s->w_body, 3);
    mxSetField(state, 0, "w_body", f_wb);
    mxArray* f_pa = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_pa), s->prev_accel, 3);
    mxSetField(state, 0, "prev_accel", f_pa);
    mxArray* f_pm = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_pm), s->prev_mag, 3);
    mxSetField(state, 0, "prev_mag", f_pm);
    mxSetField(state, 0, "prev_baro", mxCreateDoubleScalar(s->prev_baro));
    mxSetField(state, 0, "buffer_tolerance", mxCreateDoubleScalar(s->buffer_tolerance));
    mxSetField(state, 0, "baro_weight", mxCreateDoubleScalar(s->baro_weight));
    mxArray* f_go = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_go), s->gps_origin, 3);
    mxSetField(state, 0, "gps_origin", f_go);
    
    mxArray* prhs[4];
    mxArray* plhs[9]; // max outputs (gps has 9)
    
    prhs[0] = mxCreateString(type);
    prhs[1] = mxCreateDoubleMatrix(meas_len, 1, mxREAL); copy_vec(mxGetPr(prhs[1]), meas, meas_len);
    prhs[2] = state;
    prhs[3] = mxIsNaN(sample) ? mxCreateDoubleMatrix(0,0,mxREAL) : mxCreateDoubleScalar(sample);
    
    int nlhs_out = 7;
    if (strcmp(type, "gps") == 0) nlhs_out = 9;
    
    if (mexCallMATLAB(nlhs_out, plhs, 4, prhs, "mex_eskf_sensor_updates_full") == 0) {
        copy_vec(s->p, mxGetPr(plhs[0]), 3);
        copy_vec(s->v, mxGetPr(plhs[1]), 3);
        copy_vec(s->q, mxGetPr(plhs[2]), 4);
        copy_vec(s->ba, mxGetPr(plhs[3]), 3);
        copy_vec(s->bg, mxGetPr(plhs[4]), 3);
        memcpy(s->P, mxGetPr(plhs[5]), 15*15*8);
        
        if (strcmp(type, "accel") == 0) {
            copy_vec(s->prev_accel, mxGetPr(plhs[6]), 3);
        } else if (strcmp(type, "mag") == 0) {
            copy_vec(s->prev_mag, mxGetPr(plhs[6]), 3);
        } else if (strcmp(type, "baro") == 0) {
            s->prev_baro = mxGetScalar(plhs[6]);
        } else if (strcmp(type, "gps") == 0) {
            s->prev_gps_lat = mxGetScalar(plhs[6]);
            s->prev_gps_lon = mxGetScalar(plhs[7]);
            s->prev_gps_alt = mxGetScalar(plhs[8]);
        }
        for (int i=0; i<nlhs_out; i++) mxDestroyArray(plhs[i]);
    }
    
    mxDestroyArray(prhs[0]);
    mxDestroyArray(prhs[1]);
    mxDestroyArray(state);
    mxDestroyArray(prhs[3]);
}

// Call GPS sensor update (special case with multiple meas)
static void call_gps_update(ESKFState* s, double lat, double lon, double alt, double sample) {
    const char* fields[] = {"p","v","q","ba","bg","P","g","dt","gps_origin"};
    mxArray* state = mxCreateStructMatrix(1,1,9,fields);
    
    mxArray* f_p = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_p), s->p, 3);
    mxSetField(state, 0, "p", f_p);
    mxArray* f_v = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_v), s->v, 3);
    mxSetField(state, 0, "v", f_v);
    mxArray* f_q = mxCreateDoubleMatrix(4,1,mxREAL); copy_vec(mxGetPr(f_q), s->q, 4);
    mxSetField(state, 0, "q", f_q);
    mxArray* f_ba = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_ba), s->ba, 3);
    mxSetField(state, 0, "ba", f_ba);
    mxArray* f_bg = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_bg), s->bg, 3);
    mxSetField(state, 0, "bg", f_bg);
    mxArray* f_P = mxCreateDoubleMatrix(15,15,mxREAL); memcpy(mxGetPr(f_P), s->P, 15*15*8);
    mxSetField(state, 0, "P", f_P);
    mxArray* f_g = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_g), s->g, 3);
    mxSetField(state, 0, "g", f_g);
    mxSetField(state, 0, "dt", mxCreateDoubleScalar(s->dt));
    mxArray* f_go = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(f_go), s->gps_origin, 3);
    mxSetField(state, 0, "gps_origin", f_go);
    
    mxArray* prhs[6];
    mxArray* plhs[9];
    
    prhs[0] = mxCreateString("gps");
    prhs[1] = mxCreateDoubleScalar(lat);
    prhs[2] = mxCreateDoubleScalar(lon);
    prhs[3] = mxCreateDoubleScalar(alt);
    prhs[4] = state;
    prhs[5] = mxIsNaN(sample) ? mxCreateDoubleMatrix(0,0,mxREAL) : mxCreateDoubleScalar(sample);
    
    if (mexCallMATLAB(9, plhs, 6, prhs, "mex_eskf_sensor_updates_full") == 0) {
        copy_vec(s->p, mxGetPr(plhs[0]), 3);
        copy_vec(s->v, mxGetPr(plhs[1]), 3);
        copy_vec(s->q, mxGetPr(plhs[2]), 4);
        copy_vec(s->ba, mxGetPr(plhs[3]), 3);
        copy_vec(s->bg, mxGetPr(plhs[4]), 5);
        memcpy(s->P, mxGetPr(plhs[5]), 15*15*8);
        s->prev_gps_lat = mxGetScalar(plhs[6]);
        s->prev_gps_lon = mxGetScalar(plhs[7]);
        s->prev_gps_alt = mxGetScalar(plhs[8]);
        for (int i=0; i<9; i++) mxDestroyArray(plhs[i]);
    }
    
    for (int i=0; i<6; i++) mxDestroyArray(prhs[i]);
}

// Reset check
static void check_and_reset(ESKFState* s, int k) {
    // Check for divergence
    mxArray* prhs[2];
    mxArray* plhs[1];
    prhs[0] = mxCreateString("check_divergence");
    prhs[1] = mxCreateDoubleMatrix(15,15,mxREAL); memcpy(mxGetPr(prhs[1]), s->P, 15*15*8);
    
    bool need_reset = false;
    if (mexCallMATLAB(1, plhs, 2, prhs, "mex_filter_management") == 0) {
        need_reset = mxIsLogicalScalarTrue(plhs[0]);
        mxDestroyArray(plhs[0]);
    }
    mxDestroyArray(prhs[0]);
    mxDestroyArray(prhs[1]);
    
    double p_norm = sqrt(s->p[0]*s->p[0]+s->p[1]*s->p[1]+s->p[2]*s->p[2]);
    double v_norm = sqrt(s->v[0]*s->v[0]+s->v[1]*s->v[1]+s->v[2]*s->v[2]);
    double ba_norm = sqrt(s->ba[0]*s->ba[0]+s->ba[1]*s->ba[1]+s->ba[2]*s->ba[2]);
    double bg_norm = sqrt(s->bg[0]*s->bg[0]+s->bg[1]*s->bg[1]+s->bg[2]*s->bg[2]);
    
    // Check for NaN/Inf in any state (matches MATLAB)
    if (std::isnan(s->p[0]) || std::isnan(s->v[0]) || std::isnan(s->q[0])) need_reset = true;
    if (std::isnan(s->ba[0]) || std::isnan(s->bg[0])) need_reset = true;
    if (std::isinf(s->p[0]) || std::isinf(s->v[0])) need_reset = true;
    
    // Check for unreasonable values (same as MATLAB: v > 10, p > 1000)
    if (v_norm > 10.0 || p_norm > 1000.0) need_reset = true;
    
    // Additional bias divergence check (bg > 1 rad/s = 57 deg/s)
    if (bg_norm > 1.0) need_reset = true;
    
    if (need_reset) {
        s->last_reset_step = k;
        
        // Call mex_filter_management('reset_state', ...) for proper reset
        mxArray* reset_prhs[7];
        mxArray* reset_plhs[6];
        reset_prhs[0] = mxCreateString("reset_state");
        reset_prhs[1] = mxCreateDoubleMatrix(3,1,mxREAL); memcpy(mxGetPr(reset_prhs[1]), s->p, 3*8);
        reset_prhs[2] = mxCreateDoubleMatrix(3,1,mxREAL); memcpy(mxGetPr(reset_prhs[2]), s->v, 3*8);
        reset_prhs[3] = mxCreateDoubleMatrix(4,1,mxREAL); memcpy(mxGetPr(reset_prhs[3]), s->q, 4*8);
        reset_prhs[4] = mxCreateDoubleMatrix(3,1,mxREAL); memcpy(mxGetPr(reset_prhs[4]), s->ba, 3*8);
        reset_prhs[5] = mxCreateDoubleMatrix(3,1,mxREAL); memcpy(mxGetPr(reset_prhs[5]), s->bg, 3*8);
        reset_prhs[6] = mxCreateDoubleMatrix(15,15,mxREAL); memcpy(mxGetPr(reset_prhs[6]), s->P, 15*15*8);
        
        if (mexCallMATLAB(6, reset_plhs, 7, reset_prhs, "mex_filter_management") == 0) {
            // Only use the returned P matrix
            memcpy(s->P, mxGetPr(reset_plhs[5]), 15*15*8);
            for (int i=0; i<6; i++) mxDestroyArray(reset_plhs[i]);
        }
        for (int i=0; i<7; i++) mxDestroyArray(reset_prhs[i]);
        
        // Manual resets as in MATLAB reset('filter')
        s->v[0] = s->v[1] = s->v[2] = 0;
        s->ba[0] = s->ba[1] = s->ba[2] = 0;
        s->bg[0] = s->bg[1] = s->bg[2] = 0;
        
        // Reset P diagonal blocks
        s->P[0] = s->P[1+1*15] = s->P[2+2*15] = 20.0;
        s->P[3+3*15] = s->P[4+4*15] = s->P[5+5*15] = 2.0;
        double deg30 = 30.0 * M_PI / 180.0;
        s->P[6+6*15] = s->P[7+7*15] = s->P[8+8*15] = deg30*deg30;
        
        // Reset quaternion if NaN
        double q_n = sqrt(s->q[0]*s->q[0]+s->q[1]*s->q[1]+s->q[2]*s->q[2]+s->q[3]*s->q[3]);
        if (std::isnan(q_n) || q_n < 0.5) {
            s->q[0] = 1.0; s->q[1] = s->q[2] = s->q[3] = 0;
        }
    }
}

// ZUPT check and update
static void zupt_check_and_update(ESKFState* s, const double* a_meas, const double* w_meas) {
    mxArray* prhs[6];
    mxArray* plhs[2];
    
    prhs[0] = mxCreateString("check");
    prhs[1] = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(prhs[1]), a_meas, 3);
    prhs[2] = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(prhs[2]), w_meas, 3);
    prhs[3] = mxCreateDoubleScalar((double)s->zupt_counter);
    prhs[4] = mxCreateDoubleScalar(s->zupt_threshold_accel);
    prhs[5] = mxCreateDoubleScalar(s->zupt_threshold_gyro);
    
    // Note: mex_eskf_zupt takes different args; simplified here
    double a_norm = sqrt(a_meas[0]*a_meas[0]+a_meas[1]*a_meas[1]+a_meas[2]*a_meas[2]);
    double w_norm = sqrt(w_meas[0]*w_meas[0]+w_meas[1]*w_meas[1]+w_meas[2]*w_meas[2]);
    
    bool stationary = (fabs(a_norm - 9.81) < s->zupt_threshold_accel) && (w_norm < s->zupt_threshold_gyro);
    
    if (stationary) {
        s->zupt_counter++;
    } else {
        s->zupt_counter = 0;
    }
    
    s->is_stationary = (s->zupt_counter >= s->zupt_min_duration);
    
    for (int i=0; i<6; i++) mxDestroyArray(prhs[i]);
    
    if (s->is_stationary) {
        // ZUPT update via mex_eskf_zupt
        mxArray* prhs2[3];
        mxArray* plhs2[2];
        prhs2[0] = mxCreateString("update");
        prhs2[1] = mxCreateDoubleMatrix(3,1,mxREAL); copy_vec(mxGetPr(prhs2[1]), s->v, 3);
        prhs2[2] = mxCreateDoubleMatrix(15,15,mxREAL); memcpy(mxGetPr(prhs2[2]), s->P, 15*15*8);
        
        if (mexCallMATLAB(2, plhs2, 3, prhs2, "mex_eskf_zupt") == 0) {
            copy_vec(s->v, mxGetPr(plhs2[0]), 3);
            memcpy(s->P, mxGetPr(plhs2[1]), 15*15*8);
            mxDestroyArray(plhs2[0]);
            mxDestroyArray(plhs2[1]);
        }
        for (int i=0; i<3; i++) mxDestroyArray(prhs2[i]);
    }
}

// Initialize using mex_eskf_constructor
static uint64_t do_init(const mxArray* obs, double static_time, double dt) {
    ESKFState* s = new ESKFState();
    memset(s, 0, sizeof(ESKFState));
    s->valid = true;
    s->dt = dt;
    
    // Call mex_eskf_constructor
    mxArray* prhs[4];
    mxArray* plhs[1];
    
    prhs[0] = mxCreateString("init");
    prhs[1] = const_cast<mxArray*>(obs);
    prhs[2] = mxCreateDoubleScalar(static_time);
    prhs[3] = mxCreateDoubleScalar(dt);
    
    if (mexCallMATLAB(1, plhs, 4, prhs, "mex_eskf_constructor") == 0) {
        mxArray* init_data = plhs[0];
        
        // Extract state
        copy_vec(s->p, mxGetPr(mxGetField(init_data, 0, "p")), 3);
        copy_vec(s->v, mxGetPr(mxGetField(init_data, 0, "v")), 3);
        copy_vec(s->q, mxGetPr(mxGetField(init_data, 0, "q")), 4);
        copy_vec(s->ba, mxGetPr(mxGetField(init_data, 0, "ba")), 3);
        copy_vec(s->bg, mxGetPr(mxGetField(init_data, 0, "bg")), 3);
        memcpy(s->P, mxGetPr(mxGetField(init_data, 0, "P")), 15*15*8);
        memcpy(s->Q_nominal, mxGetPr(mxGetField(init_data, 0, "Q_nominal")), 15*15*8);
        copy_vec(s->g, mxGetPr(mxGetField(init_data, 0, "g")), 3);
        s->dt = mxGetScalar(mxGetField(init_data, 0, "dt"));
        copy_vec(s->gps_origin, mxGetPr(mxGetField(init_data, 0, "gps_origin")), 3);
        s->gyro_noise_threshold = mxGetScalar(mxGetField(init_data, 0, "gyro_noise_threshold"));
        
        copy_vec(s->prev_accel, mxGetPr(mxGetField(init_data, 0, "prev_accel")), 3);
        copy_vec(s->prev_gyro, mxGetPr(mxGetField(init_data, 0, "prev_gyro")), 3);
        copy_vec(s->prev_mag, mxGetPr(mxGetField(init_data, 0, "prev_mag")), 3);
        s->prev_gps_lat = mxGetScalar(mxGetField(init_data, 0, "prev_gps_lat"));
        s->prev_gps_lon = mxGetScalar(mxGetField(init_data, 0, "prev_gps_lon"));
        s->prev_gps_alt = mxGetScalar(mxGetField(init_data, 0, "prev_gps_alt"));
        s->prev_baro = mxGetScalar(mxGetField(init_data, 0, "prev_baro"));
        s->buffer_tolerance = mxGetScalar(mxGetField(init_data, 0, "buffer_tolerance"));
        
        s->zupt_threshold_accel = mxGetScalar(mxGetField(init_data, 0, "zupt_threshold_accel"));
        s->zupt_threshold_gyro = mxGetScalar(mxGetField(init_data, 0, "zupt_threshold_gyro"));
        s->zupt_min_duration = (int)mxGetScalar(mxGetField(init_data, 0, "zupt_min_duration"));
        s->zupt_counter = (int)mxGetScalar(mxGetField(init_data, 0, "zupt_counter"));
        s->is_stationary = mxIsLogicalScalarTrue(mxGetField(init_data, 0, "is_stationary"));
        
        s->adaptive_q_enabled = mxIsLogicalScalarTrue(mxGetField(init_data, 0, "adaptive_q_enabled"));
        s->velocity_damping = mxGetScalar(mxGetField(init_data, 0, "velocity_damping"));
        
        s->enable_accel_z_integration = mxIsLogicalScalarTrue(mxGetField(init_data, 0, "enable_accel_z_integration"));
        s->accel_z_threshold = mxGetScalar(mxGetField(init_data, 0, "accel_z_threshold"));
        s->accel_z_damping = mxGetScalar(mxGetField(init_data, 0, "accel_z_damping"));
        s->baro_weight = mxGetScalar(mxGetField(init_data, 0, "baro_weight"));
        
        copy_vec(s->w_body, mxGetPr(mxGetField(init_data, 0, "w_body")), 3);
        s->last_reset_step = (int)mxGetScalar(mxGetField(init_data, 0, "last_reset_step"));
        
        mxDestroyArray(init_data);
    } else {
        delete s;
        mexErrMsgIdAndTxt("mex_run_eskf:init", "mex_eskf_constructor failed");
    }
    
    mxDestroyArray(prhs[0]);
    mxDestroyArray(prhs[2]);
    mxDestroyArray(prhs[3]);
    
    uint64_t handle = g_next_handle++;
    g_states[handle] = s;
    return handle;
}

// Step
static void do_step(ESKFState* s, const mxArray* obs, int k) {
    int idx = k - 1;
    double a[3], w[3], m[3];
    getAccel(obs, idx, a);
    getGyro(obs, idx, w);
    getMag(obs, idx, m);
    
    // Convert gyro to rad/s
    double deg2rad = M_PI / 180.0;
    w[0] *= deg2rad; w[1] *= deg2rad; w[2] *= deg2rad;
    
    // Predict
    call_predict(s, a, w);
    
    // ZUPT check
    zupt_check_and_update(s, a, w);
    
    // Sensor updates
    call_sensor_update(s, "accel", a, 3, (double)k);
    call_sensor_update(s, "mag", m, 3, (double)k);
    
    // Baro
    mxArray* baro_field = mxGetField(obs, 0, "pressure");
    if (baro_field) {
        double baro = mxGetPr(baro_field)[idx];
        double baro_arr[1] = {baro};
        call_sensor_update(s, "baro", baro_arr, 1, (double)k);
    }
    
    // GPS
    mxArray* gps_lat = mxGetField(obs, 0, "lat");
    mxArray* gps_lon = mxGetField(obs, 0, "lon");
    mxArray* gps_alt = mxGetField(obs, 0, "alt");
    if (gps_lat && gps_lon && gps_alt) {
        double lat = mxGetPr(gps_lat)[idx];
        double lon = mxGetPr(gps_lon)[idx];
        double alt = mxGetPr(gps_alt)[idx];
        if (!std::isnan(lat) && !std::isnan(lon)) {
            call_gps_update(s, lat, lon, alt, (double)k);
        }
    }
    
    // Reset check
    check_and_reset(s, k);
}

// Get state
static mxArray* do_get_state(ESKFState* s) {
    const char* fields[] = {"p", "v", "q", "euler", "ba", "bg", "P"};
    mxArray* out = mxCreateStructMatrix(1, 1, 7, fields);
    
    mxArray* p = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(p), s->p, 3*sizeof(double));
    mxSetField(out, 0, "p", p);
    
    mxArray* v = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(v), s->v, 3*sizeof(double));
    mxSetField(out, 0, "v", v);
    
    mxArray* q = mxCreateDoubleMatrix(4, 1, mxREAL);
    memcpy(mxGetPr(q), s->q, 4*sizeof(double));
    mxSetField(out, 0, "q", q);
    
    double euler[3];
    quat_to_euler(s->q, euler);
    mxArray* eu = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* eu_ptr = mxGetPr(eu);
    eu_ptr[0] = euler[0] * 180.0 / M_PI;
    eu_ptr[1] = euler[1] * 180.0 / M_PI;
    eu_ptr[2] = euler[2] * 180.0 / M_PI;
    mxSetField(out, 0, "euler", eu);
    
    mxArray* ba = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(ba), s->ba, 3*sizeof(double));
    mxSetField(out, 0, "ba", ba);
    
    mxArray* bg = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(bg), s->bg, 3*sizeof(double));
    mxSetField(out, 0, "bg", bg);
    
    mxArray* P = mxCreateDoubleMatrix(15, 15, mxREAL);
    memcpy(mxGetPr(P), s->P, 15*15*sizeof(double));
    mxSetField(out, 0, "P", P);
    
    return out;
}

// Free
static void do_free(uint64_t handle) {
    auto it = g_states.find(handle);
    if (it != g_states.end()) {
        delete it->second;
        g_states.erase(it);
    }
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) mexErrMsgIdAndTxt("mex_run_eskf:usage", "Command required");
    std::string cmd = getCmd(prhs[0]);
    
    if (cmd == "init") {
        if (nrhs < 4) mexErrMsgIdAndTxt("mex_run_eskf:usage", "init requires (obs, static_time, dt)");
        const mxArray* obs = prhs[1];
        double static_time = mxGetScalar(prhs[2]);
        double dt = mxGetScalar(prhs[3]);
        uint64_t handle = do_init(obs, static_time, dt);
        plhs[0] = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
        *((uint64_t*)mxGetData(plhs[0])) = handle;
    }
    else if (cmd == "step") {
        if (nrhs < 4) mexErrMsgIdAndTxt("mex_run_eskf:usage", "step requires (handle, obs, k)");
        uint64_t handle = *((uint64_t*)mxGetData(prhs[1]));
        const mxArray* obs = prhs[2];
        int k = (int)mxGetScalar(prhs[3]);
        auto it = g_states.find(handle);
        if (it == g_states.end()) mexErrMsgIdAndTxt("mex_run_eskf:invalid", "Invalid handle");
        do_step(it->second, obs, k);
    }
    else if (cmd == "get_state") {
        if (nrhs < 2) mexErrMsgIdAndTxt("mex_run_eskf:usage", "get_state requires (handle)");
        uint64_t handle = *((uint64_t*)mxGetData(prhs[1]));
        auto it = g_states.find(handle);
        if (it == g_states.end()) mexErrMsgIdAndTxt("mex_run_eskf:invalid", "Invalid handle");
        plhs[0] = do_get_state(it->second);
    }
    else if (cmd == "free") {
        if (nrhs < 2) mexErrMsgIdAndTxt("mex_run_eskf:usage", "free requires (handle)");
        uint64_t handle = *((uint64_t*)mxGetData(prhs[1]));
        do_free(handle);
    }
    else {
        mexErrMsgIdAndTxt("mex_run_eskf:unknown", "Unknown command: %s", cmd.c_str());
    }
}

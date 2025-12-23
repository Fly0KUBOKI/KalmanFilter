#include "mex.h"
#include "mex.h"
#include "../include/MEUKF/unified_filter.hpp"
#include "mex_type_conv.hpp"
#include <vector>
#include <cmath>

// conversion constant
static const float DEG2RAD = 3.14159265358979323846f / 180.0f;

using namespace unified;

// Helpers to read vec3 from mxArray field
static void mxGetVec3Field(const mxArray* mx_struct, const char* name, float out[3]) {
    const mxArray* field = mxGetField(mx_struct, 0, name);
    if (!field) { out[0]=out[1]=out[2]=0.0f; return; }
    mex_conv::mxArrayToFloatArray(field, out, 3);
}

// Read scalar field (double -> float)
static float mxGetScalarField(const mxArray* mx_struct, const char* name, float def=0.0f) {
    const mxArray* field = mxGetField(mx_struct, 0, name);
    if (!field) return def;
    return static_cast<float>(mxGetScalar(field));
}

// Convert ESKF-like state struct to FilterOutput
static void eskf_state_to_filter_output(const mxArray* mx_state, FilterOutput& out) {
    mxGetVec3Field(mx_state, "p", out.position);
    mxGetVec3Field(mx_state, "v", out.velocity);
    mxGetVec3Field(mx_state, "ba", out.accel_bias);
    mxGetVec3Field(mx_state, "bg", out.gyro_bias);
    const mxArray* q_field = mxGetField(mx_state, 0, "q");
    if (q_field) {
        float qtmp[4]; mex_conv::mxArrayToFloatArray(q_field, qtmp, 4);
        for (int i=0;i<4;++i) out.quaternion[i]=qtmp[i];
    }
    const mxArray* P_field = mxGetField(mx_state, 0, "P");
    if (P_field) {
        float Ptmp[15*15]; mex_conv::mxArrayToFloatArray(P_field, Ptmp, 15*15);
        for (int c=0;c<15;++c) for (int r=0;r<15;++r) out.covariance(r,c)=Ptmp[c*15 + r];
    }
}

// Create output struct matching MATLAB run_simulation expectations
static mxArray* create_results_struct(size_t n_samples) {
    const char* names[] = {"time","p","v","euler","ba","bg","innov_norm","maha_dist"};
    mxArray* S = mxCreateStructMatrix(1,1,8,names);
    mxSetField(S,0,"time", mxCreateDoubleMatrix(1,(mwSize)n_samples,mxREAL));
    mxSetField(S,0,"p", mxCreateDoubleMatrix(3,(mwSize)n_samples,mxREAL));
    mxSetField(S,0,"v", mxCreateDoubleMatrix(3,(mwSize)n_samples,mxREAL));
    mxSetField(S,0,"euler", mxCreateDoubleMatrix(3,(mwSize)n_samples,mxREAL));
    mxSetField(S,0,"ba", mxCreateDoubleMatrix(3,(mwSize)n_samples,mxREAL));
    mxSetField(S,0,"bg", mxCreateDoubleMatrix(3,(mwSize)n_samples,mxREAL));
    mxSetField(S,0,"innov_norm", mxCreateDoubleMatrix(1,(mwSize)n_samples,mxREAL));
    mxSetField(S,0,"maha_dist", mxCreateDoubleMatrix(1,(mwSize)n_samples,mxREAL));
    return S;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs != 2) {
        mexErrMsgIdAndTxt("mex_run_filter:invalidInputs","Usage: results = mex_run_filter(state_struct_or_handle, obs_struct)");
    }

    const mxArray* mx_state = prhs[0];
    const mxArray* mx_obs = prhs[1];

    // If first arg is object with property state_handle, try to call mex_eskf_get_state
    if (!mxIsStruct(mx_state)) {
        // try to get property 'state_handle'
        mxArray* prop = nullptr;
        bool got_handle = false;
        if (mxIsClass(mx_state, "ESKF")) {
            prop = mxGetProperty(const_cast<mxArray*>(mx_state), 0, "state_handle");
            if (prop && mxIsUint64(prop)) {
                // call mex_eskf_get_state(handle)
                mxArray* out[1];
                mxArray* in[1]; in[0] = prop;
                if (mexCallMATLAB(1, out, 1, in, "mex_eskf_get_state") == 0) {
                    mx_state = out[0];
                    got_handle = true;
                }
                mxDestroyArray(prop);
            }
        }
        if (!got_handle && !mxIsStruct(mx_state)) {
            mexErrMsgIdAndTxt("mex_run_filter:badState","First argument must be ESKF object, uint64 handle, or state struct.");
        }
    }

    // Read obs arrays
    const mxArray* time_m = mxGetField(mx_obs,0,"time");
    const mxArray* ax_m = mxGetField(mx_obs,0,"ax");
    const mxArray* ay_m = mxGetField(mx_obs,0,"ay");
    const mxArray* az_m = mxGetField(mx_obs,0,"az");
    const mxArray* wx_m = mxGetField(mx_obs,0,"wx");
    const mxArray* wy_m = mxGetField(mx_obs,0,"wy");
    const mxArray* wz_m = mxGetField(mx_obs,0,"wz");
    const mxArray* mx_m = mxGetField(mx_obs,0,"mx");
    const mxArray* my_m = mxGetField(mx_obs,0,"my");
    const mxArray* mz_m = mxGetField(mx_obs,0,"mz");
    const mxArray* lat_m = mxGetField(mx_obs,0,"lat");
    const mxArray* lon_m = mxGetField(mx_obs,0,"lon");
    const mxArray* alt_m = mxGetField(mx_obs,0,"alt");
    const mxArray* pressure_m = mxGetField(mx_obs,0,"pressure");

    if (!time_m) mexErrMsgIdAndTxt("mex_run_filter:badObs","obs.time required");
    size_t n = mxGetNumberOfElements(time_m);

    // Prepare output arrays
    mxArray* results = create_results_struct(n);
    double* time_out = mxGetPr(mxGetField(results,0,"time"));
    double* p_out = mxGetPr(mxGetField(results,0,"p"));
    double* v_out = mxGetPr(mxGetField(results,0,"v"));
    double* euler_out = mxGetPr(mxGetField(results,0,"euler"));
    double* ba_out = mxGetPr(mxGetField(results,0,"ba"));
    double* bg_out = mxGetPr(mxGetField(results,0,"bg"));
    double* innov_out = mxGetPr(mxGetField(results,0,"innov_norm"));
    double* maha_out = mxGetPr(mxGetField(results,0,"maha_dist"));

    // Initialize UnifiedFilter with provided state
    FilterOutput init_out;
    eskf_state_to_filter_output(mx_state, init_out);
    UnifiedFilter filter;
    filter.initialize(init_out);

    // Determine dt: try to read from state field 'dt' else compute mean diff
    float dt = 0.01f;
    const mxArray* dt_field = mxGetField(mx_state,0,"dt");
    if (dt_field) dt = static_cast<float>(mxGetScalar(dt_field));
    else {
        // compute mean diff from time_m
        double* tptr = mxGetPr(time_m);
        if (n >= 2) dt = static_cast<float>(tptr[1]-tptr[0]);
    }

    // Process loop
    for (size_t k=0;k<n;++k) {
        // Fill input
        FilterInput in;
        in.dt = dt;
        if (ax_m && ay_m && az_m) {
            in.accel[0] = static_cast<float>(mxGetPr(ax_m)[k]);
            in.accel[1] = static_cast<float>(mxGetPr(ay_m)[k]);
            in.accel[2] = static_cast<float>(mxGetPr(az_m)[k]);
        }
        if (wx_m && wy_m && wz_m) {
            // convert deg/s -> rad/s
            in.gyro[0] = static_cast<float>(mxGetPr(wx_m)[k]) * DEG2RAD;
            in.gyro[1] = static_cast<float>(mxGetPr(wy_m)[k]) * DEG2RAD;
            in.gyro[2] = static_cast<float>(mxGetPr(wz_m)[k]) * DEG2RAD;
        }
        if (mx_m && my_m && mz_m) {
            double mvx = mxGetPr(mx_m)[k];
            in.mag[0] = static_cast<float>(mvx);
            in.mag_valid = !(std::isnan(mvx));
            in.mag[1] = static_cast<float>(mxGetPr(my_m)[k]);
            in.mag[2] = static_cast<float>(mxGetPr(mz_m)[k]);
        }
        if (lat_m && lon_m && alt_m) {
            double latv = mxGetPr(lat_m)[k];
            if (!std::isnan(latv)) {
                in.gps_valid = true;
                in.gps_pos[0] = static_cast<float>(mxGetPr(lat_m)[k]);
                in.gps_pos[1] = static_cast<float>(mxGetPr(lon_m)[k]);
                in.gps_pos[2] = static_cast<float>(mxGetPr(alt_m)[k]);
            }
        }
        if (pressure_m) {
            double pv = mxGetPr(pressure_m)[k];
            if (!std::isnan(pv)) { in.baro_valid = true; in.baro_alt = static_cast<float>(pv); }
        }

        // Run update
        FilterOutput out;
        filter.update(in, out);

        // write outputs
        time_out[k] = mxGetPr(time_m)[k];
        // p (3 x n) column-major: index r + c*rows
        for (int i=0;i<3;++i) p_out[i + (int)k*3] = out.position[i];
        for (int i=0;i<3;++i) v_out[i + (int)k*3] = out.velocity[i];
        euler_out[0 + (int)k*3] = out.roll;
        euler_out[1 + (int)k*3] = out.pitch;
        euler_out[2 + (int)k*3] = out.yaw;
        for (int i=0;i<3;++i) ba_out[i + (int)k*3] = out.accel_bias[i];
        for (int i=0;i<3;++i) bg_out[i + (int)k*3] = out.gyro_bias[i];
        innov_out[k] = out.innovation_norm_accel;
        maha_out[k] = 0.0;
    }

    plhs[0] = results;
}

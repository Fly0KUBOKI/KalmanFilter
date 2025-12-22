// mex_eskf_step_handle.cpp
// Usage: out = mex_eskf_step_handle(state_handle, input_struct)
// Builds prev_state from persistent C++ State, calls mex_unified_filter(prev_state, input_struct),
// updates the persistent State with returned output, and returns the output.

#include "mex.h"
#include <cstdint>
#include <cstring>

struct State {
    double p[3];
    double v[3];
    double q[4];
    double ba[3];
    double bg[3];
    double P[15*15];
};

static mxArray* make_prev_state(const State* s)
{
    const char *fields[] = {"p","v","q","ba","bg","P"};
    mxArray *ps = mxCreateStructMatrix(1,1,6,fields);

    mxArray *p = mxCreateDoubleMatrix(3,1,mxREAL);
    double *pd = mxGetPr(p); for (int i=0;i<3;i++) pd[i]=s->p[i]; mxSetField(ps,0,"p",p);

    mxArray *v = mxCreateDoubleMatrix(3,1,mxREAL);
    double *vd = mxGetPr(v); for (int i=0;i<3;i++) vd[i]=s->v[i]; mxSetField(ps,0,"v",v);

    mxArray *q = mxCreateDoubleMatrix(4,1,mxREAL);
    double *qd = mxGetPr(q); for (int i=0;i<4;i++) qd[i]=s->q[i]; mxSetField(ps,0,"q",q);

    mxArray *ba = mxCreateDoubleMatrix(3,1,mxREAL);
    double *bad = mxGetPr(ba); for (int i=0;i<3;i++) bad[i]=s->ba[i]; mxSetField(ps,0,"ba",ba);

    mxArray *bg = mxCreateDoubleMatrix(3,1,mxREAL);
    double *bgd = mxGetPr(bg); for (int i=0;i<3;i++) bgd[i]=s->bg[i]; mxSetField(ps,0,"bg",bg);

    mxArray *P = mxCreateDoubleMatrix(15,15,mxREAL);
    double *Pd = mxGetPr(P); for (int i=0;i<15*15;i++) Pd[i]=s->P[i]; mxSetField(ps,0,"P",P);

    return ps;
}

static void update_state_from_output(State* s, const mxArray* out)
{
    const mxArray *f;
    f = mxGetField(out,0,"position"); if (f && mxIsDouble(f)) { double *d=mxGetPr(f); for (int i=0;i<3;i++) s->p[i]=d[i]; }
    f = mxGetField(out,0,"velocity"); if (f && mxIsDouble(f)) { double *d=mxGetPr(f); for (int i=0;i<3;i++) s->v[i]=d[i]; }
    f = mxGetField(out,0,"quaternion"); if (f && mxIsDouble(f)) { double *d=mxGetPr(f); for (int i=0;i<4;i++) s->q[i]=d[i]; }
    f = mxGetField(out,0,"accel_bias"); if (f && mxIsDouble(f)) { double *d=mxGetPr(f); for (int i=0;i<3;i++) s->ba[i]=d[i]; }
    f = mxGetField(out,0,"gyro_bias"); if (f && mxIsDouble(f)) { double *d=mxGetPr(f); for (int i=0;i<3;i++) s->bg[i]=d[i]; }
    f = mxGetField(out,0,"covariance"); if (f && mxIsDouble(f) && mxGetNumberOfElements(f)>=15*15) { double *d=mxGetPr(f); for (int i=0;i<15*15;i++) s->P[i]=d[i]; }
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs != 2) mexErrMsgIdAndTxt("mex_eskf_step_handle:nrhs", "Two inputs required: state_handle, input_struct");
    if (nlhs > 1) mexErrMsgIdAndTxt("mex_eskf_step_handle:nlhs", "At most one output returned");

    if (!mxIsUint64(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1) mexErrMsgIdAndTxt("mex_eskf_step_handle:arg","state_handle must be uint64 scalar");
    uint64_t handle = *(uint64_t*)mxGetData(prhs[0]);
    State *s = (State*)(uintptr_t)handle;
    if (!s) mexErrMsgIdAndTxt("mex_eskf_step_handle:invalid","Null state handle");

    const mxArray *input_struct = prhs[1];

    // Build prev_state from C++ State
    mxArray *prev_state = make_prev_state(s);

    // Call mex_unified_filter(prev_state, input_struct)
    mxArray *call_in[2]; call_in[0] = prev_state; call_in[1] = (mxArray*)input_struct;
    mxArray *call_out[1];
    int status = mexCallMATLAB(1, call_out, 2, call_in, "mex_unified_filter");
    if (status != 0 || call_out[0] == NULL) {
        mexErrMsgIdAndTxt("mex_eskf_step_handle:callfail","Call to mex_unified_filter failed");
    }

    // Update persistent State from output
    update_state_from_output(s, call_out[0]);

    // Return the unified_filter output to MATLAB
    plhs[0] = call_out[0];
}

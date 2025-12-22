// mex_eskf_get_state.cpp
// Usage: state_struct = mex_eskf_get_state(state_handle)

#include "mex.h"
#include <cstdint>

struct State {
    double p[3];
    double v[3];
    double q[4];
    double ba[3];
    double bg[3];
    double P[15*15];
};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nlhs > 1) mexErrMsgIdAndTxt("mex_eskf_get_state:nlhs", "One output allowed: state_struct");
    if (nrhs != 1) mexErrMsgIdAndTxt("mex_eskf_get_state:nrhs", "One input required: state_handle");

    if (!mxIsUint64(prhs[0]) || mxGetNumberOfElements(prhs[0]) != 1) {
        mexErrMsgIdAndTxt("mex_eskf_get_state:arg", "state_handle must be a uint64 scalar");
    }

    uint64_t handle = *(uint64_t*)mxGetData(prhs[0]);
    State *s = (State*)(uintptr_t)handle;
    if (!s) mexErrMsgIdAndTxt("mex_eskf_get_state:invalid", "Null state handle");

    const char *fields[] = {"p","v","q","ba","bg","P"};
    plhs[0] = mxCreateStructMatrix(1,1,6,fields);

    // p
    mxArray *p = mxCreateDoubleMatrix(3,1,mxREAL);
    double *pd = mxGetPr(p); for (int i=0;i<3;i++) pd[i]=s->p[i];
    mxSetField(plhs[0],0,"p",p);

    // v
    mxArray *v = mxCreateDoubleMatrix(3,1,mxREAL);
    double *vd = mxGetPr(v); for (int i=0;i<3;i++) vd[i]=s->v[i];
    mxSetField(plhs[0],0,"v",v);

    // q
    mxArray *q = mxCreateDoubleMatrix(4,1,mxREAL);
    double *qd = mxGetPr(q); for (int i=0;i<4;i++) qd[i]=s->q[i];
    mxSetField(plhs[0],0,"q",q);

    // ba
    mxArray *ba = mxCreateDoubleMatrix(3,1,mxREAL);
    double *bad = mxGetPr(ba); for (int i=0;i<3;i++) bad[i]=s->ba[i];
    mxSetField(plhs[0],0,"ba",ba);

    // bg
    mxArray *bg = mxCreateDoubleMatrix(3,1,mxREAL);
    double *bgd = mxGetPr(bg); for (int i=0;i<3;i++) bgd[i]=s->bg[i];
    mxSetField(plhs[0],0,"bg",bg);

    // P (15x15)
    mxArray *P = mxCreateDoubleMatrix(15,15,mxREAL);
    double *Pd = mxGetPr(P);
    for (int i=0;i<15*15;i++) Pd[i]=s->P[i];
    mxSetField(plhs[0],0,"P",P);
}

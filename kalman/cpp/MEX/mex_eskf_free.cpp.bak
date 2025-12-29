// mex_eskf_free.cpp
// Usage: mex_eskf_free(state_handle)

#include "mex.h"
#include <cstdint>

struct State { double dummy; };

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs != 1) mexErrMsgIdAndTxt("mex_eskf_free:nrhs", "One input required: state_handle");
    if (!mxIsUint64(prhs[0]) || mxGetNumberOfElements(prhs[0]) != 1) {
        mexErrMsgIdAndTxt("mex_eskf_free:arg", "state_handle must be a uint64 scalar");
    }

    uint64_t handle = *(uint64_t*)mxGetData(prhs[0]);
    void *ptr = (void*)(uintptr_t)handle;
    if (!ptr) return; // nothing to do

    // Delete allocated State
    delete (State*)ptr;
}

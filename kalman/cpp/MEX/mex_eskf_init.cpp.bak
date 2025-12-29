// mex_eskf_init.cpp
// Usage: state_handle = mex_eskf_init(state_struct, params_struct)

#include "mex.h"
#include <cstdint>
#include <cstring>
#include <cmath>

struct State {
    double p[3];
    double v[3];
    double q[4];
    double ba[3];
    double bg[3];
    double P[15*15];
};

static void copy_vector_field(const mxArray *s, const char *field, double *dst, int len)
{
    if (!s) return;
    const mxArray *f = mxGetField(s, 0, field);
    if (!f) return;
    if (mxIsDouble(f) && mxGetNumberOfElements(f) >= len) {
        double *src = mxGetPr(f);
        for (int i = 0; i < len; ++i) dst[i] = src[i];
    }
}

static void copy_matrix_field(const mxArray *s, const char *field, double *dst, int rows, int cols)
{
    if (!s) return;
    const mxArray *f = mxGetField(s, 0, field);
    if (!f) return;
    if (mxIsDouble(f) && mxGetNumberOfElements(f) >= rows*cols) {
        double *src = mxGetPr(f);
        int n = rows*cols;
        for (int i = 0; i < n; ++i) dst[i] = src[i];
    }
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nlhs != 1) {
        mexErrMsgIdAndTxt("mex_eskf_init:nlhs", "One output required: state_handle");
    }

    if (nrhs < 1) {
        mexErrMsgIdAndTxt("mex_eskf_init:nrhs", "At least one input required: state_struct");
    }

    const mxArray *state_struct = prhs[0];

    // Allocate State in persistent memory
    State *s = new State();
    if (!s) mexErrMsgIdAndTxt("mex_eskf_init:alloc", "Failed to allocate State");
    // zero-init
    std::memset(s, 0, sizeof(State));

    // Copy known fields if present
    copy_vector_field(state_struct, "p", s->p, 3);
    copy_vector_field(state_struct, "v", s->v, 3);
    copy_vector_field(state_struct, "q", s->q, 4);
    copy_vector_field(state_struct, "ba", s->ba, 3);
    copy_vector_field(state_struct, "bg", s->bg, 3);
    copy_matrix_field(state_struct, "P", s->P, 15, 15);

    // Ensure quaternion normalization (basic)
    double nq = 0.0;
    for (int i = 0; i < 4; ++i) nq += s->q[i]*s->q[i];
    if (nq > 0.0) {
        double inv = 1.0 / std::sqrt(nq);
        for (int i = 0; i < 4; ++i) s->q[i] *= inv;
    } else {
        // default quaternion [1,0,0,0]
        s->q[0] = 1.0; s->q[1]=s->q[2]=s->q[3]=0.0;
    }

    // Make memory persistent so MATLAB does not free it
    mexMakeMemoryPersistent(s);

    // Return uint64 handle to pointer
    plhs[0] = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
    uint64_t *out = (uint64_t*)mxGetData(plhs[0]);
    *out = (uint64_t)(uintptr_t)s;
}

// mex_eskf_set_state.cpp
// Usage: mex_eskf_set_state(state_handle, state_struct)

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

static void copy_vector_field_to_state(const mxArray *s, const char *field, double *dst, int len)
{
    if (!s) return;
    const mxArray *f = mxGetField(s, 0, field);
    if (!f) return;
    if (mxIsDouble(f) && mxGetNumberOfElements(f) >= len) {
        double *src = mxGetPr(f);
        for (int i = 0; i < len; ++i) dst[i] = src[i];
    }
}

static void copy_matrix_field_to_state(const mxArray *s, const char *field, double *dst, int rows, int cols)
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
    if (nrhs != 2) mexErrMsgIdAndTxt("mex_eskf_set_state:nrhs", "Two inputs required: state_handle, state_struct");
    if (nlhs != 0) mexErrMsgIdAndTxt("mex_eskf_set_state:nlhs", "No outputs expected");

    if (!mxIsUint64(prhs[0]) || mxGetNumberOfElements(prhs[0]) != 1) {
        mexErrMsgIdAndTxt("mex_eskf_set_state:arg", "state_handle must be a uint64 scalar");
    }

    uint64_t handle = *(uint64_t*)mxGetData(prhs[0]);
    State *s = (State*)(uintptr_t)handle;
    if (!s) mexErrMsgIdAndTxt("mex_eskf_set_state:invalid", "Null state handle");

    const mxArray *state_struct = prhs[1];

    // Copy provided fields into the State
    copy_vector_field_to_state(state_struct, "p", s->p, 3);
    copy_vector_field_to_state(state_struct, "v", s->v, 3);
    copy_vector_field_to_state(state_struct, "q", s->q, 4);
    copy_vector_field_to_state(state_struct, "ba", s->ba, 3);
    copy_vector_field_to_state(state_struct, "bg", s->bg, 3);
    copy_matrix_field_to_state(state_struct, "P", s->P, 15, 15);

    // Normalize quaternion
    double nq = 0.0;
    for (int i = 0; i < 4; ++i) nq += s->q[i]*s->q[i];
    if (nq > 0.0) {
        double inv = 1.0 / std::sqrt(nq);
        for (int i = 0; i < 4; ++i) s->q[i] *= inv;
    }

    // Symmetrize covariance P: P = (P + P')/2 (column-major storage)
    for (int r = 0; r < 15; ++r) {
        for (int c = r+1; c < 15; ++c) {
            int idx1 = r + c*15; // (r,c)
            int idx2 = c + r*15; // (c,r)
            double avg = 0.5 * (s->P[idx1] + s->P[idx2]);
            s->P[idx1] = avg;
            s->P[idx2] = avg;
        }
    }
}

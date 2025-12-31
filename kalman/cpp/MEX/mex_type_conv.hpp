// Helper utilities for converting between MATLAB mxArray (double) and
// internal float buffers used by the C++ filter core.
#pragma once
#include "mex.h"
#include <cstddef>

namespace mex_conv {

inline void mxArrayToFloatArray(const mxArray* arr, float* out, std::size_t n) {
    if (!arr) {
        for (std::size_t i = 0; i < n; ++i) out[i] = 0.0f;
        return;
    }
    const double* pr = mxGetPr(arr);
    if (!pr) {
        for (std::size_t i = 0; i < n; ++i) out[i] = 0.0f;
        return;
    }
    for (std::size_t i = 0; i < n; ++i) out[i] = static_cast<float>(pr[i]);
}

inline void floatArrayToMxArray(const float* in, mxArray* out, std::size_t rows, std::size_t cols) {
    double* pr = mxGetPr(out);
    for (std::size_t c = 0; c < cols; ++c) {
        for (std::size_t r = 0; r < rows; ++r) {
            pr[r + c * rows] = static_cast<double>(in[r + c * rows]);
        }
    }
}

inline float mxGetScalarAsFloat(const mxArray* a) {
    if (!a) return 0.0f;
    return static_cast<float>(mxGetScalar(a));
}

} // namespace mex_conv

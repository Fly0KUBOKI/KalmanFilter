// Helper utilities for converting between MATLAB mxArray (double/single) and
// internal float buffers used by the C++ filter core.
#pragma once
#include "mex.h"
#include <cstddef>

namespace mex_conv {

// Convert mxArray (single only) to float array
// GPS以外のセンサーデータはfloatのみを受け取る（型変換を廃止）
inline void mxArrayToFloatArray(const mxArray* arr, float* out, std::size_t n) {
    if (!arr) {
        for (std::size_t i = 0; i < n; ++i) out[i] = 0.0f;
        return;
    }
    
    // single (float) のみを受け付ける
    if (mxGetClassID(arr) != mxSINGLE_CLASS) {
        mexErrMsgIdAndTxt("mex_conv:type_error", 
            "Expected single (float) array, but got %s. GPS以外のセンサーデータはfloatのみを受け取ります。", 
            mxGetClassName(arr));
        return;
    }
    
    const float* pf = (const float*)mxGetData(arr);
    if (!pf) {
        for (std::size_t i = 0; i < n; ++i) out[i] = 0.0f;
        return;
    }
    for (std::size_t i = 0; i < n; ++i) out[i] = pf[i];
}

// Convert float array to mxArray (double format - used for legacy compatibility)
inline void floatArrayToMxArray(const float* in, mxArray* out, std::size_t rows, std::size_t cols) {
    double* pr = mxGetPr(out);
    for (std::size_t c = 0; c < cols; ++c) {
        for (std::size_t r = 0; r < rows; ++r) {
            pr[r + c * rows] = static_cast<double>(in[r + c * rows]);
        }
    }
}

// Convert float array to mxArray (single format - new float output support)
inline void floatArrayToMxArrayFloat(const float* in, mxArray* out, std::size_t rows, std::size_t cols) {
    float* pf = (float*)mxGetData(out);
    for (std::size_t c = 0; c < cols; ++c) {
        for (std::size_t r = 0; r < rows; ++r) {
            pf[r + c * rows] = in[r + c * rows];
        }
    }
}

// Get scalar value as float (single only)
// GPS以外のスカラー値はfloatのみを受け取る（型変換を廃止）
inline float mxGetScalarAsFloat(const mxArray* a) {
    if (!a) return 0.0f;
    
    // single (float) のみを受け付ける
    if (mxGetClassID(a) != mxSINGLE_CLASS) {
        mexErrMsgIdAndTxt("mex_conv:type_error", 
            "Expected single (float) scalar, but got %s. GPS以外のスカラー値はfloatのみを受け取ります。", 
            mxGetClassName(a));
        return 0.0f;
    }
    
    const float* pf = (const float*)mxGetData(a);
    return pf ? pf[0] : 0.0f;
}

} // namespace mex_conv

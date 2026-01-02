#pragma once

#ifndef MEX_MEX_TYPE_CONVERSION_HPP
#define MEX_MEX_TYPE_CONVERSION_HPP

#include "mex.h"
#include "../Common/Math/fixed_matrix.hpp"
#include <cstddef>

namespace mex_conv {

using namespace cmath_fx;

// ============================================================================
// Basic type conversion functions (from mex_type_conv.hpp)
// ============================================================================

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

// ============================================================================
// Template functions for Matrix/Vector types
// ============================================================================

// Helper: MATLAB array -> Vector (single/double両対応)
template<int R>
bool matToVector(const mxArray* arr, Vector<R, float>& out) {
    if (!arr) return false;
    // single型またはdouble型を受け入れる（complexは不可）
    if (mxIsComplex(arr)) return false;
    if (mxGetClassID(arr) != mxSINGLE_CLASS && mxGetClassID(arr) != mxDOUBLE_CLASS) return false;
    mwSize rows = mxGetM(arr); 
    mwSize cols = mxGetN(arr);
    if (rows != R || cols != 1) return false;
    float tmp[R];
    mex_conv::mxArrayToFloatArray(arr, tmp, static_cast<size_t>(R));
    for (int i = 0; i < R; ++i) out(i, 0) = tmp[i];
    return true;
}

// Helper: MATLAB array -> Matrix (single/double両対応)
template<int R, int C>
bool matToMatrix(const mxArray* arr, Matrix<R, C, float>& out) {
    if (!arr) return false;
    // single型またはdouble型を受け入れる（complexは不可）
    if (mxIsComplex(arr)) return false;
    if (mxGetClassID(arr) != mxSINGLE_CLASS && mxGetClassID(arr) != mxDOUBLE_CLASS) return false;
    mwSize rows = mxGetM(arr); 
    mwSize cols = mxGetN(arr);
    if (rows != R || cols != C) return false;
    float tmp[R * C];
    mex_conv::mxArrayToFloatArray(arr, tmp, static_cast<size_t>(R) * static_cast<size_t>(C));
    for (int j = 0; j < C; ++j) {
        for (int i = 0; i < R; ++i) {
            out(i, j) = tmp[j * R + i];
        }
    }
    return true;
}

// Helper: Vector -> MATLAB array
template<int R>
mxArray* vectorToMat(const Vector<R, float>& v) {
    mxArray* out = mxCreateDoubleMatrix(R, 1, mxREAL);
    double* pr = mxGetPr(out);
    for (int i = 0; i < R; ++i) pr[i] = static_cast<double>(v(i, 0));
    return out;
}

// Helper: Matrix -> MATLAB array
template<int R, int C>
mxArray* matrixToMat(const Matrix<R, C, float>& M) {
    mxArray* out = mxCreateDoubleMatrix(R, C, mxREAL);
    double* pr = mxGetPr(out);
    for (int j = 0; j < C; ++j) {
        for (int i = 0; i < R; ++i) {
            pr[j * R + i] = static_cast<double>(M(i, j));
        }
    }
    return out;
}

} // namespace mex_conv

#endif // MEX_MEX_TYPE_CONVERSION_HPP


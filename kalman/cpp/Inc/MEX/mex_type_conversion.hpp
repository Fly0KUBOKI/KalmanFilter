#pragma once

#ifndef MEX_MEX_TYPE_CONVERSION_HPP
#define MEX_MEX_TYPE_CONVERSION_HPP

#include "mex.h"
#include "mex_type_conv.hpp"
#include "../Common/Math/fixed_matrix.hpp"
#include <vector>

namespace mex_conv {

using namespace cmath_fx;

// Helper: MATLAB array -> Vector
template<int R>
bool matToVector(const mxArray* arr, Vector<R, float>& out) {
    if (!arr) return false;
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    mwSize rows = mxGetM(arr); 
    mwSize cols = mxGetN(arr);
    if (rows != R || cols != 1) return false;
    std::vector<float> tmp(static_cast<size_t>(R));
    mex_conv::mxArrayToFloatArray(arr, tmp.data(), static_cast<size_t>(R));
    for (int i = 0; i < R; ++i) out(i, 0) = tmp[i];
    return true;
}

// Helper: MATLAB array -> Matrix
template<int R, int C>
bool matToMatrix(const mxArray* arr, Matrix<R, C, float>& out) {
    if (!arr) return false;
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    mwSize rows = mxGetM(arr); 
    mwSize cols = mxGetN(arr);
    if (rows != R || cols != C) return false;
    std::vector<float> tmp(static_cast<size_t>(R) * static_cast<size_t>(C));
    mex_conv::mxArrayToFloatArray(arr, tmp.data(), static_cast<size_t>(R) * static_cast<size_t>(C));
    for (int j = 0; j < C; ++j) {
        for (int i = 0; i < R; ++i) {
            out(i, j) = tmp[static_cast<size_t>(j) * static_cast<size_t>(R) + static_cast<size_t>(i)];
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


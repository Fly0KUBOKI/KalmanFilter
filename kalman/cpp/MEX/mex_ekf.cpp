// mex_ekf.cpp (nomalloc)
// EKF linear-update MEX wrapper using FixedMatrix

#include "mex.h"
#include "../Common/Math/fixed_matrix.hpp"

using cm = cmath_fx::FixedMatrix;

static bool matToFixed(const mxArray* arr, cm& out) {
    if (!mxIsDouble(arr) || mxIsComplex(arr)) return false;
    mwSize rows = mxGetM(arr); mwSize cols = mxGetN(arr);
    if (rows > cmath_fx::MAX_N || cols > cmath_fx::MAX_N) return false;
    double* pr = mxGetPr(arr);
    out.resize((int)rows, (int)cols);
    for (mwSize j = 0; j < cols; ++j) for (mwSize i = 0; i < rows; ++i) out((int)i,(int)j) = static_cast<float>(pr[j*rows + i]);
    return true;
}

static mxArray* fixedToMat(const cm& M) {
    mwSize rows = (mwSize)M.rows;
    mwSize cols = (mwSize)M.cols;
    mxArray* out = mxCreateDoubleMatrix(rows, cols, mxREAL);
    double* pr = mxGetPr(out);
    for (mwSize j = 0; j < cols; ++j) for (mwSize i = 0; i < rows; ++i) pr[j*rows + i] = static_cast<double>(M((int)i,(int)j));
    return out;
}

// Usage: [x_upd,P_upd] = mex_ekf(x,P,z,H,R)
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if (nrhs != 5) mexErrMsgTxt("Usage: [x_upd,P_upd] = mex_ekf(x,P,z,H,R)");
    cm x,P,z,H,R;
    if (!matToFixed(prhs[0], x)) mexErrMsgTxt("Failed to read x");
    if (!matToFixed(prhs[1], P)) mexErrMsgTxt("Failed to read P");
    if (!matToFixed(prhs[2], z)) mexErrMsgTxt("Failed to read z");
    if (!matToFixed(prhs[3], H)) mexErrMsgTxt("Failed to read H");
    if (!matToFixed(prhs[4], R)) mexErrMsgTxt("Failed to read R");

    // S = H * P * H' + R
    cm temp, temp2, Ht;
    cmath_fx::transpose(H, Ht);
    cmath_fx::multiply(H, P, temp);
    cmath_fx::multiply(temp, Ht, temp2);
    cm S; S.resize(temp2.rows, temp2.cols);
    for (int i=0;i<temp2.rows;++i) for (int j=0;j<temp2.cols;++j) S(i,j) = temp2(i,j) + R(i,j);

    // PHt
    cm PHt; cmath_fx::multiply(P, Ht, PHt);

    cm K;
    if (!cmath_fx::solve_linear_system(S, PHt, K)) mexErrMsgTxt("Failed to solve for K");

    // y = z - H*x
    cm Hx; cmath_fx::multiply(H, x, Hx);
    cm y; y.resize(z.rows, z.cols);
    for (int i=0;i<z.rows;++i) y(i,0) = z(i,0) - Hx(i,0);

    // x_upd = x + K*y
    cm dx; cmath_fx::multiply(K, y, dx);
    cm x_upd; x_upd.resize(x.rows, x.cols);
    for (int i=0;i<x.rows;++i) x_upd(i,0) = x(i,0) + dx(i,0);

    // P_upd = (I-KH) P (I-KH)' + K R K'
    cm KH; cmath_fx::multiply(K, H, KH);
    cm I; I = cm::Identity(P.rows);
    cm IKH; IKH.resize(I.rows, I.cols);
    for (int i=0;i<I.rows;++i) for (int j=0;j<I.cols;++j) IKH(i,j) = I(i,j) - KH(i,j);
    cm temp1; cmath_fx::multiply(IKH, P, temp1);
    cm IKHt; cmath_fx::transpose(IKH, IKHt);
    cm term1; cmath_fx::multiply(temp1, IKHt, term1);
    cm KR; cmath_fx::multiply(K, R, temp);
    cm KRt; cmath_fx::transpose(K, KRt);
    cm term2; cmath_fx::multiply(temp, KRt, term2);
    cm P_upd; P_upd.resize(term1.rows, term1.cols);
    for (int i=0;i<term1.rows;++i) for (int j=0;j<term1.cols;++j) P_upd(i,j) = term1(i,j) + term2(i,j);
    for (int i=0;i<P_upd.rows;++i) for (int j=0;j<P_upd.cols;++j) P_upd(i,j) = 0.5*(P_upd(i,j) + P_upd(j,i));

    plhs[0] = fixedToMat(x_upd);
    plhs[1] = fixedToMat(P_upd);
}


#include "mex.h"
#include "UKF/Core/ukf_core.hpp"
#include "mex_type_conv.hpp"
#include <vector>

// 最小テスト版: (6,3)ケースのみサポート
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs < 5) {
        mexErrMsgIdAndTxt("mex_ukf_update_minimal:nrhs",
                          "At least 5 inputs required: x, P, z, h_func, R");
    }
    
    const mxArray* mx_x = prhs[0];
    const mxArray* mx_P = prhs[1];
    const mxArray* mx_z = prhs[2];
    const mxArray* mx_h_func = prhs[3];
    const mxArray* mx_R = prhs[4];
    
    float alpha = (nrhs >= 6) ? mex_conv::mxGetScalarAsFloat(prhs[5]) : 1e-3f;
    float beta = (nrhs >= 7) ? mex_conv::mxGetScalarAsFloat(prhs[6]) : 2.0f;
    float kappa = (nrhs >= 8) ? mex_conv::mxGetScalarAsFloat(prhs[7]) : 0.0f;
    
    int n = static_cast<int>(mxGetM(mx_x));
    int m = static_cast<int>(mxGetM(mx_z));
    
    if (n != 6 || m != 3) {
        mexErrMsgIdAndTxt("mex_ukf_update_minimal:dim",
                          "Only (n=6, m=3) supported. Got n=%d, m=%d", n, m);
    }
    
    // (6,3)ケース
    constexpr int N = 6;
    constexpr int M = 3;
    
    using UKF = ukf::UKFCore<N, M, float>;
    using VectorN = typename UKF::VectorN;
    using VectorM = typename UKF::VectorM;
    using MatrixNN = typename UKF::MatrixNN;
    using MatrixMM = typename UKF::MatrixMM;
    using MatrixNM = typename UKF::MatrixNM;
    
    VectorN x, x_upd;
    MatrixNN P, P_upd;
    VectorM z;
    MatrixMM R, S;
    MatrixNM K;
    VectorM y;
    
    // Convert inputs to float temporaries
    float x_tmp[N];
    float P_tmp[N*N];
    float z_tmp[M];
    float R_tmp[M*M];
    mex_conv::mxArrayToFloatArray(mx_x, x_tmp, N);
    mex_conv::mxArrayToFloatArray(mx_P, P_tmp, N*N);
    mex_conv::mxArrayToFloatArray(mx_z, z_tmp, M);
    mex_conv::mxArrayToFloatArray(mx_R, R_tmp, M*M);

    for (int i = 0; i < N; ++i) {
        x(i, 0) = x_tmp[i];
        for (int j = 0; j < N; ++j) {
            P(i, j) = P_tmp[i + j * N];
        }
    }
    for (int i = 0; i < M; ++i) {
        z(i, 0) = z_tmp[i];
        for (int j = 0; j < M; ++j) {
            R(i, j) = R_tmp[i + j * M];
        }
    }
    
    x_upd = x;
    P_upd = P;
    
    auto h_wrapper = [&](const VectorN& x_sig) -> VectorM {
        mxArray* mx_x_sig = mxCreateDoubleMatrix(N, 1, mxREAL);
        double* x_sig_data = mxGetPr(mx_x_sig);
        for (int i = 0; i < N; ++i) {
            x_sig_data[i] = static_cast<double>(x_sig(i, 0));
        }
        mxArray* lhs[1];
        mxArray* rhs[2] = {const_cast<mxArray*>(mx_h_func), mx_x_sig};
        if (mexCallMATLAB(1, lhs, 2, rhs, "feval") != 0) {
            mxDestroyArray(mx_x_sig);
            mexErrMsgIdAndTxt("mex_ukf_update_minimal:h_func", "Error calling h_func");
        }
        std::vector<float> z_pred_tmp(M);
        mex_conv::mxArrayToFloatArray(lhs[0], z_pred_tmp.data(), static_cast<size_t>(M));
        VectorM z_pred_vec;
        for (int i = 0; i < M; ++i) {
            z_pred_vec(i, 0) = z_pred_tmp[i];
        }
        mxDestroyArray(mx_x_sig);
        mxDestroyArray(lhs[0]);
        return z_pred_vec;
    };
    
    UKF::update(x_upd, P_upd, z, h_wrapper, R, alpha, beta, kappa, &K, &S, &y);
    
    plhs[0] = mxCreateDoubleMatrix(N, 1, mxREAL);
    double* x_out = mxGetPr(plhs[0]);
    for (int i = 0; i < N; ++i) {
        x_out[i] = static_cast<double>(x_upd(i, 0));
    }
    
    if (nlhs >= 2) {
        plhs[1] = mxCreateDoubleMatrix(N, N, mxREAL);
        double* P_out = mxGetPr(plhs[1]);
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                P_out[i + j * N] = static_cast<double>(P_upd(i, j));
            }
        }
    }
    
    if (nlhs >= 3) {
        plhs[2] = mxCreateDoubleMatrix(N, M, mxREAL);
        double* K_out = mxGetPr(plhs[2]);
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < M; ++j) {
                K_out[i + j * N] = static_cast<double>(K(i, j));
            }
        }
    }
    
    if (nlhs >= 4) {
        plhs[3] = mxCreateDoubleMatrix(M, M, mxREAL);
        double* S_out = mxGetPr(plhs[3]);
        for (int i = 0; i < M; ++i) {
            for (int j = 0; j < M; ++j) {
                S_out[i + j * M] = static_cast<double>(S(i, j));
            }
        }
    }
    
    if (nlhs >= 5) {
        plhs[4] = mxCreateDoubleMatrix(M, 1, mxREAL);
        double* y_out = mxGetPr(plhs[4]);
        for (int i = 0; i < M; ++i) {
            y_out[i] = static_cast<double>(y(i, 0));
        }
    }
}

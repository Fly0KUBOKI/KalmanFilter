#include "mex.h"
#include "../Inc/UKF/ukf_core.hpp"
#include "mex_type_conv.hpp"
#include <vector>
#include <cstring>

// MEX関数: ukf_update
// [x_upd, P_upd, K, S, y] = mex_ukf_update(x, P, z, h_func, R, alpha, beta, kappa)
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // 引数チェック
    if (nrhs < 5) {
        mexErrMsgIdAndTxt("mex_ukf_update:nrhs",
                          "At least 5 inputs required: x, P, z, h_func, R");
    }
    if (nlhs > 5) {
        mexErrMsgIdAndTxt("mex_ukf_update:nlhs",
                          "Too many output arguments (max 5)");
    }
    
    // 入力パラメータ取得
    const mxArray* mx_x = prhs[0];
    const mxArray* mx_P = prhs[1];
    const mxArray* mx_z = prhs[2];
    const mxArray* mx_h_func = prhs[3];
    const mxArray* mx_R = prhs[4];
    
    float alpha = (nrhs >= 6) ? mex_conv::mxGetScalarAsFloat(prhs[5]) : 1e-3f;
    float beta = (nrhs >= 7) ? mex_conv::mxGetScalarAsFloat(prhs[6]) : 2.0f;
    float kappa = (nrhs >= 8) ? mex_conv::mxGetScalarAsFloat(prhs[7]) : 0.0f;
    
    // 次元チェック
    int n = static_cast<int>(mxGetM(mx_x));
    int m = static_cast<int>(mxGetM(mx_z));
    
    if (mxGetN(mx_x) != 1) {
        mexErrMsgIdAndTxt("mex_ukf_update:x", "x must be a column vector");
    }
    if (mxGetN(mx_z) != 1) {
        mexErrMsgIdAndTxt("mex_ukf_update:z", "z must be a column vector");
    }
    if (mxGetM(mx_P) != n || mxGetN(mx_P) != n) {
        mexErrMsgIdAndTxt("mex_ukf_update:P", "P must be n x n");
    }
    if (mxGetM(mx_R) != m || mxGetN(mx_R) != m) {
        mexErrMsgIdAndTxt("mex_ukf_update:R", "R must be m x m");
    }
    
    // 次元別の処理（テンプレート展開）
    // サポートする次元: n=15,16,17,18,19,20,21, m=1,2,3,4,6
    #define HANDLE_CASE(N, M) \
        if (n == N && m == M) { \
            using UKF = ukf::UKFCore<N, M, float>; \
            using VectorN = typename UKF::VectorN; \
            using VectorM = typename UKF::VectorM; \
            using MatrixNN = typename UKF::MatrixNN; \
            using MatrixMM = typename UKF::MatrixMM; \
            using MatrixNM = typename UKF::MatrixNM; \
            \
            VectorN x, x_upd; \
            MatrixNN P, P_upd; \
            VectorM z; \
            MatrixMM R, S; \
            MatrixNM K; \
            VectorM y; \
            \
            std::vector<float> x_tmp(N); \
            std::vector<float> P_tmp(N * N); \
            std::vector<float> z_tmp(M); \
            std::vector<float> R_tmp(M * M); \
            mex_conv::mxArrayToFloatArray(mx_x, x_tmp.data(), N); \
            mex_conv::mxArrayToFloatArray(mx_P, P_tmp.data(), N * N); \
            mex_conv::mxArrayToFloatArray(mx_z, z_tmp.data(), M); \
            mex_conv::mxArrayToFloatArray(mx_R, R_tmp.data(), M * M); \
            for (int i = 0; i < N; ++i) { \
                x(i, 0) = x_tmp[i]; \
                for (int j = 0; j < N; ++j) { \
                    P(i, j) = P_tmp[i + j * N]; \
                } \
            } \
            for (int i = 0; i < M; ++i) { \
                z(i, 0) = z_tmp[i]; \
                for (int j = 0; j < M; ++j) { \
                    R(i, j) = R_tmp[i + j * M]; \
                } \
            } \
            \
            x_upd = x; \
            P_upd = P; \
            \
            auto h_wrapper = [&](const VectorN& x_sig) -> VectorM { \
                mxArray* mx_x_sig = mxCreateDoubleMatrix(N, 1, mxREAL); \
                double* x_sig_data = mxGetPr(mx_x_sig); \
                for (int i = 0; i < N; ++i) { \
                    x_sig_data[i] = static_cast<double>(x_sig(i, 0)); \
                } \
                mxArray* mx_z_pred_args[2] = {const_cast<mxArray*>(mx_h_func), mx_x_sig}; \
                mxArray* mx_z_pred = nullptr; \
                if (mexCallMATLAB(1, &mx_z_pred, 2, mx_z_pred_args, "feval") != 0) { \
                    mxDestroyArray(mx_x_sig); \
                    mexErrMsgIdAndTxt("mex_ukf_update:h_func", "Error calling h_func"); \
                } \
                std::vector<float> z_pred_tmp(M); \
                mex_conv::mxArrayToFloatArray(mx_z_pred, z_pred_tmp.data(), static_cast<size_t>(M)); \
                VectorM z_pred_vec; \
                for (int i = 0; i < M; ++i) { \
                    z_pred_vec(i, 0) = z_pred_tmp[i]; \
                } \
                mxDestroyArray(mx_x_sig); \
                mxDestroyArray(mx_z_pred); \
                return z_pred_vec; \
            }; \
            \
            UKF::update(x_upd, P_upd, z, h_wrapper, R, alpha, beta, kappa, &K, &S, &y); \
            \
            plhs[0] = mxCreateDoubleMatrix(N, 1, mxREAL); \
            double* x_out = mxGetPr(plhs[0]); \
            for (int i = 0; i < N; ++i) { \
                x_out[i] = static_cast<double>(x_upd(i, 0)); \
            } \
            \
            if (nlhs >= 2) { \
                plhs[1] = mxCreateDoubleMatrix(N, N, mxREAL); \
                double* P_out = mxGetPr(plhs[1]); \
                for (int i = 0; i < N; ++i) { \
                    for (int j = 0; j < N; ++j) { \
                        P_out[i + j * N] = static_cast<double>(P_upd(i, j)); \
                    } \
                } \
            } \
            \
            if (nlhs >= 3) { \
                plhs[2] = mxCreateDoubleMatrix(N, M, mxREAL); \
                double* K_out = mxGetPr(plhs[2]); \
                for (int i = 0; i < N; ++i) { \
                    for (int j = 0; j < M; ++j) { \
                        K_out[i + j * N] = static_cast<double>(K(i, j)); \
                    } \
                } \
            } \
            \
            if (nlhs >= 4) { \
                plhs[3] = mxCreateDoubleMatrix(M, M, mxREAL); \
                double* S_out = mxGetPr(plhs[3]); \
                for (int i = 0; i < M; ++i) { \
                    for (int j = 0; j < M; ++j) { \
                        S_out[i + j * M] = static_cast<double>(S(i, j)); \
                    } \
                } \
            } \
            \
            if (nlhs >= 5) { \
                plhs[4] = mxCreateDoubleMatrix(M, 1, mxREAL); \
                double* y_out = mxGetPr(plhs[4]); \
                for (int i = 0; i < M; ++i) { \
                    y_out[i] = static_cast<double>(y(i, 0)); \
                } \
            } \
            return; \
        }
    
    // 一般的なUKF状態次元（6-21）と観測次元（1-6）をサポート
    HANDLE_CASE(6, 1)
    HANDLE_CASE(6, 2)
    HANDLE_CASE(6, 3)
    HANDLE_CASE(6, 4)
    HANDLE_CASE(6, 6)
    HANDLE_CASE(15, 1)
    HANDLE_CASE(15, 2)
    HANDLE_CASE(15, 3)
    HANDLE_CASE(15, 4)
    HANDLE_CASE(15, 6)
    HANDLE_CASE(16, 1)
    HANDLE_CASE(16, 2)
    HANDLE_CASE(16, 3)
    HANDLE_CASE(16, 4)
    HANDLE_CASE(16, 6)
    HANDLE_CASE(17, 1)
    HANDLE_CASE(17, 2)
    HANDLE_CASE(17, 3)
    HANDLE_CASE(17, 4)
    HANDLE_CASE(17, 6)
    HANDLE_CASE(18, 1)
    HANDLE_CASE(18, 2)
    HANDLE_CASE(18, 3)
    HANDLE_CASE(18, 4)
    HANDLE_CASE(18, 6)
    HANDLE_CASE(19, 1)
    HANDLE_CASE(19, 2)
    HANDLE_CASE(19, 3)
    HANDLE_CASE(19, 4)
    HANDLE_CASE(19, 6)
    HANDLE_CASE(20, 1)
    HANDLE_CASE(20, 2)
    HANDLE_CASE(20, 3)
    HANDLE_CASE(20, 4)
    HANDLE_CASE(20, 6)
    HANDLE_CASE(21, 1)
    HANDLE_CASE(21, 2)
    HANDLE_CASE(21, 3)
    HANDLE_CASE(21, 4)
    HANDLE_CASE(21, 6)
    
    #undef HANDLE_CASE
    
    mexErrMsgIdAndTxt("mex_ukf_update:dim",
                      "Unsupported dimensions n=%d, m=%d", n, m);
}

#include "mex.h"
#include "../ESKF/eskf_helper.hpp"

using ESKFHelperF = eskf::ESKFHelper<float>;

// MEX関数: mex_eskf_helper
// 使用法:
//   nominal = mex_eskf_helper('inject_error_state', nominal, dx)
//   nominal = mex_eskf_helper('inject_with_constraints', nominal, dx, max_v, max_ba, max_bg)
//   P = mex_eskf_helper('joseph_update', P, K, H, R)
//   P = mex_eskf_helper('regularize', P)

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs < 1) {
        mexErrMsgIdAndTxt("mex_eskf_helper:nrhs", "At least 1 input required: action");
    }
    
    char action[128];
    if (mxGetString(prhs[0], action, sizeof(action)) != 0) {
        mexErrMsgIdAndTxt("mex_eskf_helper:action", "First argument must be a string");
    }
    
    if (strcmp(action, "inject_error_state") == 0) {
        // nominal = inject_error_state(nominal, dx)
        if (nrhs < 3) {
            mexErrMsgIdAndTxt("mex_eskf_helper:nrhs", "nominal and dx required");
        }
        
        const mxArray* mx_nominal = prhs[1];
        const mxArray* mx_dx = prhs[2];
        
        // nominal構造体から読み込み
        mxArray* mx_p = mxGetField(mx_nominal, 0, "p");
        mxArray* mx_v = mxGetField(mx_nominal, 0, "v");
        mxArray* mx_q = mxGetField(mx_nominal, 0, "q");
        mxArray* mx_ba = mxGetField(mx_nominal, 0, "ba");
        mxArray* mx_bg = mxGetField(mx_nominal, 0, "bg");
        
        if (!mx_p || !mx_v || !mx_q || !mx_ba || !mx_bg) {
            mexErrMsgIdAndTxt("mex_eskf_helper:nominal", "nominal must have fields: p, v, q, ba, bg");
        }
        
        ESKFHelperF::NominalState nominal;
        const double* p_data = mxGetPr(mx_p);
        const double* v_data = mxGetPr(mx_v);
        const double* q_data = mxGetPr(mx_q);
        const double* ba_data = mxGetPr(mx_ba);
        const double* bg_data = mxGetPr(mx_bg);
        
        for (int i = 0; i < 3; ++i) {
            nominal.p(i, 0) = p_data[i];
            nominal.v(i, 0) = v_data[i];
            nominal.ba(i, 0) = ba_data[i];
            nominal.bg(i, 0) = bg_data[i];
        }
        nominal.q.w = q_data[0];
        nominal.q.x = q_data[1];
        nominal.q.y = q_data[2];
        nominal.q.z = q_data[3];
        
        // dx読み込み
        const double* dx_data = mxGetPr(mx_dx);
        ESKFHelperF::Vector15 dx;
        for (int i = 0; i < 15; ++i) {
            dx(i, 0) = dx_data[i];
        }
        
        // 注入実行
        ESKFHelperF::inject_error_state(nominal, dx);
        
        // 出力構造体作成
        const char* field_names[] = {"p", "v", "q", "ba", "bg"};
        plhs[0] = mxCreateStructMatrix(1, 1, 5, field_names);
        
        mxArray* out_p = mxCreateDoubleMatrix(3, 1, mxREAL);
        mxArray* out_v = mxCreateDoubleMatrix(3, 1, mxREAL);
        mxArray* out_q = mxCreateDoubleMatrix(4, 1, mxREAL);
        mxArray* out_ba = mxCreateDoubleMatrix(3, 1, mxREAL);
        mxArray* out_bg = mxCreateDoubleMatrix(3, 1, mxREAL);
        
        double* p_out = mxGetPr(out_p);
        double* v_out = mxGetPr(out_v);
        double* q_out = mxGetPr(out_q);
        double* ba_out = mxGetPr(out_ba);
        double* bg_out = mxGetPr(out_bg);
        
        for (int i = 0; i < 3; ++i) {
            p_out[i] = nominal.p(i, 0);
            v_out[i] = nominal.v(i, 0);
            ba_out[i] = nominal.ba(i, 0);
            bg_out[i] = nominal.bg(i, 0);
        }
        q_out[0] = nominal.q.w;
        q_out[1] = nominal.q.x;
        q_out[2] = nominal.q.y;
        q_out[3] = nominal.q.z;
        
        mxSetField(plhs[0], 0, "p", out_p);
        mxSetField(plhs[0], 0, "v", out_v);
        mxSetField(plhs[0], 0, "q", out_q);
        mxSetField(plhs[0], 0, "ba", out_ba);
        mxSetField(plhs[0], 0, "bg", out_bg);
        
    } else if (strcmp(action, "regularize") == 0) {
        // P = regularize(P)
        if (nrhs < 2) {
            mexErrMsgIdAndTxt("mex_eskf_helper:nrhs", "P required");
        }
        
        const double* P_data = mxGetPr(prhs[1]);
        ESKFHelperF::Matrix15 P;
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                P(i, j) = P_data[i + j * 15];
            }
        }
        
        float eps = (nrhs >= 3) ? mxGetScalar(prhs[2]) : 1e-9f;
        ESKFHelperF::regularize_covariance(P, eps);
        
        plhs[0] = mxCreateDoubleMatrix(15, 15, mxREAL);
        double* P_out = mxGetPr(plhs[0]);
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                P_out[i + j * 15] = P(i, j);
            }
        }
        
    } else {
        mexErrMsgIdAndTxt("mex_eskf_helper:action", "Unknown action: %s", action);
    }
}

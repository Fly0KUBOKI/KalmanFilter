#include "mex.h"
#include "../Inc/ESKF/eskf_helper.hpp"
#include "mex_type_conv.hpp"

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
        float tmp3[3];
        mex_conv::mxArrayToFloatArray(mx_p, tmp3, 3);
        for (int i = 0; i < 3; ++i) nominal.p(i, 0) = tmp3[i];
        mex_conv::mxArrayToFloatArray(mx_v, tmp3, 3);
        for (int i = 0; i < 3; ++i) nominal.v(i, 0) = tmp3[i];
        mex_conv::mxArrayToFloatArray(mx_ba, tmp3, 3);
        for (int i = 0; i < 3; ++i) nominal.ba(i, 0) = tmp3[i];
        mex_conv::mxArrayToFloatArray(mx_bg, tmp3, 3);
        for (int i = 0; i < 3; ++i) nominal.bg(i, 0) = tmp3[i];
        float tmpq[4];
        mex_conv::mxArrayToFloatArray(mx_q, tmpq, 4);
        nominal.q.w = tmpq[0];
        nominal.q.x = tmpq[1];
        nominal.q.y = tmpq[2];
        nominal.q.z = tmpq[3];

        // dx読み込み
        float dx_tmp[15];
        mex_conv::mxArrayToFloatArray(mx_dx, dx_tmp, 15);
        ESKFHelperF::Vector15 dx;
        for (int i = 0; i < 15; ++i) dx(i, 0) = dx_tmp[i];
        
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
        
        // 出力に float -> double 変換して詰める
        mex_conv::floatArrayToMxArray(reinterpret_cast<const float*>(&nominal.p(0,0)), out_p, 3, 1);
        mex_conv::floatArrayToMxArray(reinterpret_cast<const float*>(&nominal.v(0,0)), out_v, 3, 1);
        float qf[4] = { nominal.q.w, nominal.q.x, nominal.q.y, nominal.q.z };
        mex_conv::floatArrayToMxArray(qf, out_q, 4, 1);
        mex_conv::floatArrayToMxArray(reinterpret_cast<const float*>(&nominal.ba(0,0)), out_ba, 3, 1);
        mex_conv::floatArrayToMxArray(reinterpret_cast<const float*>(&nominal.bg(0,0)), out_bg, 3, 1);
        
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
        
        // P: double (MATLAB) -> float (internal)
        float P_tmp[15*15];
        mex_conv::mxArrayToFloatArray(prhs[1], P_tmp, 15*15); // column-major copy
        ESKFHelperF::Matrix15 P;
        for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) P(i,j) = P_tmp[i + j*15];

        float eps = (nrhs >= 3) ? mex_conv::mxGetScalarAsFloat(prhs[2]) : 1e-9f;
        ESKFHelperF::regularize_covariance(P, eps);

        plhs[0] = mxCreateDoubleMatrix(15, 15, mxREAL);
        mex_conv::floatArrayToMxArray(reinterpret_cast<const float*>(&P(0,0)), plhs[0], 15, 15);
        
    } else {
        mexErrMsgIdAndTxt("mex_eskf_helper:action", "Unknown action: %s", action);
    }
}

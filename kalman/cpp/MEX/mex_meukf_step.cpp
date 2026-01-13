#include "mex.h"
// meukf_core.hpp is located in kalman/cpp/Lib/MEUKF/inc
#include "../Lib/MEUKF/inc/meukf_core.hpp"
#include "Impl/mex_type_conversion.hpp"
#include <cstring>
#include <cmath>

// Helpers
static void set_vec3_float_field(mxArray* m_state, const char* name, const float* in) {
    mxArray* f = mxGetField(m_state, 0, name);
    if(!f) return;
    if (mxGetClassID(f) != mxSINGLE_CLASS) {
        mexErrMsgIdAndTxt("mex_meukf_step:type_error",
            "Expected single (float) array for field '%s', but got %s. 出力はfloatのみです。",
            name, mxGetClassName(f));
        return;
    }
    float* pf = (float*)mxGetData(f);
    pf[0] = in[0]; pf[1] = in[1]; pf[2] = in[2];
}

static double get_field_scalar_helper(const mxArray* s, const char* f) {
    mxArray* field = mxGetField(s, 0, f);
    if (!field) return 0.0;
    if (mxIsLogical(field)) return mxIsLogicalScalarTrue(field) ? 1.0 : 0.0;
    if (mxGetClassID(field) == mxSINGLE_CLASS) {
        const float* pf = (const float*)mxGetData(field);
        return pf ? pf[0] : 0.0;
    } else if (mxGetClassID(field) == mxDOUBLE_CLASS) {
        const double* pr = mxGetPr(field);
        return pr ? pr[0] : 0.0;
    }
    return 0.0;
}

// ヘルパー: MATLAB構造体からC++構造体へ
void matlab_to_state(const mxArray* m_state, meukf::State& c_state) {
    // p, v, q, ba, bg, P
    // MATLAB側は double なので float に変換
    
    // p, v, ba, bg
    mex_conv::mxArrayToFloatArray(mxGetField(m_state,0,"p"), c_state.p, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_state,0,"v"), c_state.v, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_state,0,"ba"), c_state.ba, 3);
    mex_conv::mxArrayToFloatArray(mxGetField(m_state,0,"bg"), c_state.bg, 3);

    // q
    mex_conv::mxArrayToFloatArray(mxGetField(m_state,0,"q"), c_state.q, 4);

    // P: MATLAB column-major -> internal row-major
    mxArray* f_P = mxGetField(m_state, 0, "P");
    if (f_P) {
        float P_tmp[15*15];
        mex_conv::mxArrayToFloatArray(f_P, P_tmp, 15*15);
        for (int r=0;r<15;++r) for (int c=0;c<15;++c) c_state.P[r*15 + c] = P_tmp[c*15 + r];
    }
}

// Output state to MATLAB (float only - type conversion removed)
void state_to_matlab(const meukf::State& c_state, mxArray* m_state) {
    set_vec3_float_field(m_state, "p", c_state.p);
    set_vec3_float_field(m_state, "v", c_state.v);
    set_vec3_float_field(m_state, "ba", c_state.ba);
    set_vec3_float_field(m_state, "bg", c_state.bg);
    
    mxArray* f_q = mxGetField(m_state, 0, "q");
    if(f_q) {
        if (mxGetClassID(f_q) != mxSINGLE_CLASS) {
            mexErrMsgIdAndTxt("mex_meukf_step:type_error", 
                "Expected single (float) array for field 'q', but got %s. 出力はfloatのみです。", 
                mxGetClassName(f_q));
        } else {
            float* pf = (float*)mxGetData(f_q);
            pf[0] = c_state.q[0]; 
            pf[1] = c_state.q[1];
            pf[2] = c_state.q[2]; 
            pf[3] = c_state.q[3];
        }
    }
    
    mxArray* f_P = mxGetField(m_state, 0, "P");
    if(f_P) {
        if (mxGetClassID(f_P) != mxSINGLE_CLASS) {
            mexErrMsgIdAndTxt("mex_meukf_step:type_error", 
                "Expected single (float) array for field 'P', but got %s. 出力はfloatのみです。", 
                mxGetClassName(f_P));
        } else {
            float* pf = (float*)mxGetData(f_P);
            for(int c=0; c<15; ++c) {
                for(int r=0; r<15; ++r) {
                    pf[r + c*15] = c_state.P[r*15 + c];
                }
            }
        }
    }
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if (nrhs != 3) {
        mexErrMsgIdAndTxt("MEUKF:step:invalidNumInputs", "3 inputs required: prev_state, sensor, params");
    }

    // 入力データの取得
    const mxArray* m_prev_state = prhs[0];
    const mxArray* m_sensor = prhs[1];
    const mxArray* m_params = prhs[2];

    // 【統合完了: 以下の処理はdo_sensor_update_meukfに統合済みのためコメントアウト】
    // mex_hybrid_filter内のdo_sensor_update_meukfが直接MEUKFCore::step()を呼び出すようになりました。
    // このMEXファイルは後方互換性のために残していますが、実際の処理はdo_sensor_update_meukfで行われます。
    
    
    
    // 【統合完了: 上記の処理はdo_sensor_update_meukfに統合済み】
    // 実際の処理はmex_hybrid_filter内のdo_sensor_update_meukfで行われます。
    // このMEXファイルは後方互換性のために残していますが、実際には使用されていません。
    // 
    // 注意: このMEXファイルは現在使用されていないため、エラーを返します。
    mexErrMsgIdAndTxt("mex_meukf_step:deprecated", 
        "mex_meukf_step_v2は統合済みです。mex_hybrid_filter内のdo_sensor_update_meukfを使用してください。");
}

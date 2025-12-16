#include "mex.h"
#include "../Common/Math/math_utils.hpp"
#include "../Common/Sensor/sensor_filter.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <cstring>

using cm = cmath_fx::FixedMatrix;
using namespace common::math;
using namespace common::sensor;

// ===== alpha_beta_step =====
void handle_alpha_beta_step(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 4) {
        mexErrMsgIdAndTxt("mex_filter_utils:alpha_beta_step", 
            "Need at least 4 inputs: x, v, z, dt");
    }
    
    double x_in = mxGetScalar(prhs[0]);
    double v_in = mxGetScalar(prhs[1]);
    double z_in = mxGetScalar(prhs[2]);
    double dt = mxGetScalar(prhs[3]);
    
    float alpha = 0.85f;
    float beta = 0.1f;
    if (nrhs >= 5 && !mxIsEmpty(prhs[4])) {
        alpha = static_cast<float>(mxGetScalar(prhs[4]));
    }
    if (nrhs >= 6 && !mxIsEmpty(prhs[5])) {
        beta = static_cast<float>(mxGetScalar(prhs[5]));
    }
    
    float x = static_cast<float>(x_in);
    float v = static_cast<float>(v_in);
    bool z_is_nan = mxIsNaN(z_in) || mxIsInf(z_in);
    float z = z_is_nan ? 0.0f : static_cast<float>(z_in);
    
    // 予測
    float x_pred = x + v * dt;
    float v_pred = v;
    
    // 更新
    float x_upd, v_upd;
    if (z_is_nan) {
        x_upd = x_pred;
        v_upd = v_pred;
    } else {
        float r = z - x_pred;
        x_upd = x_pred + alpha * r;
        v_upd = v_pred + (beta / dt) * r;
    }
    
    plhs[0] = mxCreateDoubleScalar(x_pred);
    if (nlhs >= 2) plhs[1] = mxCreateDoubleScalar(v_pred);
    if (nlhs >= 3) plhs[2] = mxCreateDoubleScalar(x_upd);
    if (nlhs >= 4) plhs[3] = mxCreateDoubleScalar(v_upd);
}

// ===== ema_update =====
void handle_ema_update(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 2) {
        mexErrMsgIdAndTxt("mex_filter_utils:ema_update", 
            "Need at least 2 inputs: x, y_prev");
    }
    
    double* x_in = mxGetPr(prhs[0]);
    int n_x = static_cast<int>(mxGetNumberOfElements(prhs[0]));
    
    float alpha = 0.1f;
    if (nrhs >= 3 && !mxIsEmpty(prhs[2])) {
        alpha = static_cast<float>(mxGetScalar(prhs[2]));
    }
    
    cm x, y_prev, out;
    x.resize(n_x, 1);
    for (int i = 0; i < n_x; ++i) {
        x(i,0) = static_cast<float>(x_in[i]);
    }
    
    bool y_prev_empty = mxIsEmpty(prhs[1]);
    if (!y_prev_empty) {
        double* y_prev_in = mxGetPr(prhs[1]);
        int n_y = static_cast<int>(mxGetNumberOfElements(prhs[1]));
        if (n_y != n_x) {
            mexErrMsgIdAndTxt("mex_filter_utils:ema_update", 
                "x and y_prev must have same size");
        }
        y_prev.resize(n_y, 1);
        for (int i = 0; i < n_y; ++i) {
            y_prev(i,0) = static_cast<float>(y_prev_in[i]);
        }
    }
    
    out.resize(n_x, 1);
    if (y_prev_empty) {
        for (int i = 0; i < n_x; ++i) {
            out(i,0) = x(i,0);
        }
    } else {
        for (int i = 0; i < n_x; ++i) {
            out(i,0) = alpha * x(i,0) + (1.0f - alpha) * y_prev(i,0);
        }
    }
    
    plhs[0] = mxCreateDoubleMatrix(n_x, 1, mxREAL);
    double* out_data = mxGetPr(plhs[0]);
    for (int i = 0; i < n_x; ++i) {
        out_data[i] = out(i,0);
    }
}

// ===== hampel_causal =====
void handle_hampel_causal(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 2) {
        mexErrMsgIdAndTxt("mex_filter_utils:hampel_causal", 
            "Need at least 2 inputs: buffer, new_x");
    }
    
    double* buffer_in = mxGetPr(prhs[0]);
    int n_buf = static_cast<int>(mxGetNumberOfElements(prhs[0]));
    
    double* new_x_in = mxGetPr(prhs[1]);
    int n_new = static_cast<int>(mxGetNumberOfElements(prhs[1]));
    
    int window = 5;
    if (nrhs >= 3 && !mxIsEmpty(prhs[2])) {
        window = static_cast<int>(mxGetScalar(prhs[2]));
    }
    
    float n_sigma = 3.0f;
    if (nrhs >= 4 && !mxIsEmpty(prhs[3])) {
        n_sigma = static_cast<float>(mxGetScalar(prhs[3]));
    }
    
    // バッファとnew_xを結合
    std::vector<float> buf;
    for (int i = 0; i < n_buf; ++i) {
        buf.push_back(static_cast<float>(buffer_in[i]));
    }
    for (int i = 0; i < n_new; ++i) {
        buf.push_back(static_cast<float>(new_x_in[i]));
    }
    
    // ウィンドウサイズに制限
    if (static_cast<int>(buf.size()) > window) {
        buf.erase(buf.begin(), buf.end() - window);
    }
    
    // 中央値計算
    std::vector<float> sorted_buf = buf;
    std::sort(sorted_buf.begin(), sorted_buf.end());
    float med;
    int n = static_cast<int>(sorted_buf.size());
    if (n % 2 == 0) {
        med = (sorted_buf[n/2-1] + sorted_buf[n/2]) / 2.0f;
    } else {
        med = sorted_buf[n/2];
    }
    
    // MAD計算
    std::vector<float> abs_dev;
    for (float val : buf) {
        abs_dev.push_back(fabsf(val - med));
    }
    std::sort(abs_dev.begin(), abs_dev.end());
    float madv;
    if (n % 2 == 0) {
        madv = (abs_dev[n/2-1] + abs_dev[n/2]) / 2.0f;
    } else {
        madv = abs_dev[n/2];
    }
    if (madv < 1e-9f) madv = 1e-9f;
    
    float threshold = n_sigma * 1.4826f * madv;
    
    // 出力計算
    plhs[0] = mxCreateDoubleMatrix(n_new, 1, mxREAL);
    double* out_data = mxGetPr(plhs[0]);
    
    if (n_new == 1) {
        float new_x_val = static_cast<float>(new_x_in[0]);
        if (fabsf(new_x_val - med) > threshold) {
            out_data[0] = med;
        } else {
            out_data[0] = new_x_val;
        }
    } else {
        // ベクトル版: 各要素に対して中央値と閾値を計算
        for (int k = 0; k < n_new; ++k) {
            float new_x_val = static_cast<float>(new_x_in[k]);
            // 簡易版: 全体の中央値と閾値を使用
            if (fabsf(new_x_val - med) > threshold) {
                out_data[k] = med;
            } else {
                out_data[k] = new_x_val;
            }
        }
    }
}

// ===== Main MEX Function =====
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1 || !mxIsChar(prhs[0])) {
        mexErrMsgIdAndTxt("mex_filter_utils:invalidInput", 
            "First argument must be a function name string");
    }
    
    char func_name[128];
    mxGetString(prhs[0], func_name, sizeof(func_name));
    
    if (strcmp(func_name, "alpha_beta_step") == 0) {
        handle_alpha_beta_step(nlhs, plhs, nrhs-1, prhs+1);
    }
    else if (strcmp(func_name, "ema_update") == 0) {
        handle_ema_update(nlhs, plhs, nrhs-1, prhs+1);
    }
    else if (strcmp(func_name, "hampel_causal") == 0) {
        handle_hampel_causal(nlhs, plhs, nrhs-1, prhs+1);
    }
    else {
        mexErrMsgIdAndTxt("mex_filter_utils:unknownFunction",
            "Unknown function: %s", func_name);
    }
}

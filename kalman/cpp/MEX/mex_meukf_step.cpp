#include "mex.h"
#include "meukf_core.hpp"
#include <cstring>

// ヘルパー: MATLAB構造体からC++構造体へ
void matlab_to_state(const mxArray* m_state, meukf::State& c_state) {
    // p, v, q, ba, bg, P
    // MATLAB側は double なので float に変換
    
    auto get_vec3 = [&](const char* name, float* out) {
        mxArray* f = mxGetField(m_state, 0, name);
        if(f) {
            double* pr = mxGetPr(f);
            out[0] = static_cast<float>(pr[0]); 
            out[1] = static_cast<float>(pr[1]); 
            out[2] = static_cast<float>(pr[2]);
        }
    };
    
    get_vec3("p", c_state.p);
    get_vec3("v", c_state.v);
    get_vec3("ba", c_state.ba);
    get_vec3("bg", c_state.bg);
    
    mxArray* f_q = mxGetField(m_state, 0, "q");
    if(f_q) {
        double* pr = mxGetPr(f_q);
        c_state.q[0] = static_cast<float>(pr[0]); 
        c_state.q[1] = static_cast<float>(pr[1]); 
        c_state.q[2] = static_cast<float>(pr[2]); 
        c_state.q[3] = static_cast<float>(pr[3]);
    }
    
    mxArray* f_P = mxGetField(m_state, 0, "P");
    if(f_P) {
        double* pr = mxGetPr(f_P);
        // MATLABはColumn-major, C++はRow-major
        for(int c=0; c<15; ++c) {
            for(int r=0; r<15; ++r) {
                c_state.P[r*15 + c] = static_cast<float>(pr[c*15 + r]);
            }
        }
    }
}

void state_to_matlab(const meukf::State& c_state, mxArray* m_state) {
    auto set_vec3 = [&](const char* name, const float* in) {
        mxArray* f = mxGetField(m_state, 0, name);
        if(!f) {
            return; 
        }
        double* pr = mxGetPr(f);
        pr[0] = static_cast<double>(in[0]); 
        pr[1] = static_cast<double>(in[1]); 
        pr[2] = static_cast<double>(in[2]);
    };
    
    set_vec3("p", c_state.p);
    set_vec3("v", c_state.v);
    set_vec3("ba", c_state.ba);
    set_vec3("bg", c_state.bg);
    
    mxArray* f_q = mxGetField(m_state, 0, "q");
    if(f_q) {
        double* pr = mxGetPr(f_q);
        pr[0] = static_cast<double>(c_state.q[0]); 
        pr[1] = static_cast<double>(c_state.q[1]);
        pr[2] = static_cast<double>(c_state.q[2]); 
        pr[3] = static_cast<double>(c_state.q[3]);
    }
    
    mxArray* f_P = mxGetField(m_state, 0, "P");
    if(f_P) {
        double* pr = mxGetPr(f_P);
        for(int c=0; c<15; ++c) {
            for(int r=0; r<15; ++r) {
                pr[c*15 + r] = static_cast<double>(c_state.P[r*15 + c]);
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

    meukf::MEUKFInput input;
    
    // 1. State変換
    matlab_to_state(m_prev_state, input.prev_state);
    
    // 2. SensorData変換
    auto get_field_scalar = [&](const mxArray* s, const char* f) -> double {
        mxArray* field = mxGetField(s, 0, f);
        return field ? mxGetScalar(field) : 0.0;
    };
    auto get_field_vec3 = [&](const mxArray* s, const char* f, double* out) {
        mxArray* field = mxGetField(s, 0, f);
        if(field) {
            double* pr = mxGetPr(field);
            out[0] = pr[0]; out[1] = pr[1]; out[2] = pr[2];
        }
    };
    auto get_field_vec3_float = [&](const mxArray* s, const char* f, float* out) {
        mxArray* field = mxGetField(s, 0, f);
        if(field) {
            double* pr = mxGetPr(field);
            out[0] = static_cast<float>(pr[0]); 
            out[1] = static_cast<float>(pr[1]); 
            out[2] = static_cast<float>(pr[2]);
        }
    };

    get_field_vec3_float(m_sensor, "accel", input.sensor.accel);
    get_field_vec3_float(m_sensor, "gyro", input.sensor.gyro);
    get_field_vec3_float(m_sensor, "mag", input.sensor.mag);
    get_field_vec3_float(m_sensor, "gps_pos", input.sensor.gps_pos);
    input.sensor.alt_baro = static_cast<float>(get_field_scalar(m_sensor, "alt_baro"));
    
    // 前回のセンサー値を読み取り（変更検知用）
    get_field_vec3_float(m_sensor, "prev_mag", input.sensor.prev_mag);
    get_field_vec3_float(m_sensor, "prev_gps_pos", input.sensor.prev_gps_pos);
    input.sensor.prev_baro_alt = static_cast<float>(get_field_scalar(m_sensor, "prev_baro_alt"));
    
    input.sensor.update_accel = (uint8_t)get_field_scalar(m_sensor, "update_accel");
    input.sensor.update_gyro = (uint8_t)get_field_scalar(m_sensor, "update_gyro");
    input.sensor.update_mag = (uint8_t)get_field_scalar(m_sensor, "update_mag");
    input.sensor.update_gps = (uint8_t)get_field_scalar(m_sensor, "update_gps");
    input.sensor.update_baro = (uint8_t)get_field_scalar(m_sensor, "update_baro");
    input.sensor.update_zupt = (uint8_t)get_field_scalar(m_sensor, "update_zupt");
    input.sensor.dt = static_cast<float>(get_field_scalar(m_sensor, "dt"));

    // 3. Params変換
    get_field_vec3_float(m_params, "g", input.params.g);
    get_field_vec3_float(m_params, "mag_ref", input.params.mag_ref);
    get_field_vec3_float(m_params, "noise_accel", input.params.noise_accel);
    get_field_vec3_float(m_params, "noise_gyro", input.params.noise_gyro);
    get_field_vec3_float(m_params, "noise_ba", input.params.noise_ba);
    get_field_vec3_float(m_params, "noise_bg", input.params.noise_bg);
    get_field_vec3_float(m_params, "noise_mag", input.params.noise_mag);
    get_field_vec3_float(m_params, "noise_gps", input.params.noise_gps);
    input.params.noise_baro = static_cast<float>(get_field_scalar(m_params, "noise_baro"));
    get_field_vec3_float(m_params, "noise_zupt", input.params.noise_zupt);
    
    input.params.alpha = static_cast<float>(get_field_scalar(m_params, "alpha"));
    input.params.beta = static_cast<float>(get_field_scalar(m_params, "beta"));
    input.params.kappa = static_cast<float>(get_field_scalar(m_params, "kappa"));

    // 実行
    meukf::MEUKFOutput output;
    meukf::MEUKFCore::step(input, output);

    // 出力作成
    // 入力構造体をコピーして、値を更新する形で返す
    plhs[0] = mxDuplicateArray(m_prev_state);
    state_to_matlab(output.new_state, plhs[0]);
    
    // デバッグ情報
    plhs[1] = mxCreateDoubleMatrix(1, 10, mxREAL);
    double* dbg = mxGetPr(plhs[1]);
    for(int i=0; i<10; ++i) dbg[i] = output.debug_info[i];
}

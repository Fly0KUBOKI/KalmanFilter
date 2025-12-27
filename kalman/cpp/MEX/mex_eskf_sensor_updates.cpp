// mex_eskf_sensor_updates.cpp
// Phase 2: sensor_updates() のMEX化
// センサー前処理とdo_cpp_update呼び出しを統合

#include "mex.h"
#include <cmath>
#include <cstring>

//=============================================================================
// 定数
//=============================================================================
static const double GRAVITY = 9.80665;
static const double DEG2RAD = 0.017453292519943295;

//=============================================================================
// ヘルパー関数
//=============================================================================
static double norm3(const double* v) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static bool is_nan_any(const double* v, int n) {
    for (int i = 0; i < n; ++i) {
        if (mxIsNaN(v[i])) return true;
    }
    return false;
}

//=============================================================================
// 前処理関数
//=============================================================================
static void preprocess_accel(const double* a_meas, const double* prev_accel,
                              double* a_corrected, bool* is_outlier, bool* no_change) {
    // 変化なしチェック
    double diff = 0.0;
    for (int i = 0; i < 3; ++i) {
        diff += fabs(a_meas[i] - prev_accel[i]);
    }
    *no_change = (diff < 1e-9);
    
    // 外れ値チェック（簡易版）
    double norm_a = norm3(a_meas);
    *is_outlier = (norm_a < 5.0 || norm_a > 20.0);
    
    // 重力補正
    a_corrected[0] = a_meas[0];
    a_corrected[1] = a_meas[1];
    a_corrected[2] = a_meas[2] + GRAVITY;
}

static void preprocess_mag(const double* m_meas, const double* prev_mag,
                            double* m_filtered, bool* is_outlier, bool* no_change) {
    // 変化なしチェック
    double diff = 0.0;
    for (int i = 0; i < 3; ++i) {
        diff += fabs(m_meas[i] - prev_mag[i]);
    }
    *no_change = (diff < 1e-9);
    
    // 外れ値チェック
    double norm_m = norm3(m_meas);
    *is_outlier = (norm_m < 20.0 || norm_m > 100.0);
    
    // フィルタリング（パススルー）
    m_filtered[0] = m_meas[0];
    m_filtered[1] = m_meas[1];
    m_filtered[2] = m_meas[2];
}

static void preprocess_gps(double lat, double lon, double alt, const double* gps_origin,
                            double* z_gps, bool* is_outlier, bool* no_change) {
    *no_change = false;
    *is_outlier = false;
    
    if (mxIsNaN(lat) || mxIsNaN(lon) || mxIsNaN(alt)) {
        *is_outlier = true;
        return;
    }
    
    double lat0 = gps_origin[0];
    double lon0 = gps_origin[1];
    double alt0 = gps_origin[2];
    
    double cos_lat0 = cos(lat0 * DEG2RAD);
    
    // ローカル座標に変換
    z_gps[0] = (lon - lon0) / (9.0e-6 / cos_lat0);  // X (East)
    z_gps[1] = (lat - lat0) / 9.0e-6;                // Y (North)
    z_gps[2] = alt - alt0;                           // Z (Up)
    
    // 外れ値チェック
    double dist = sqrt(z_gps[0]*z_gps[0] + z_gps[1]*z_gps[1]);
    *is_outlier = (dist > 1000.0);  // 1km以上は外れ値
}

static void preprocess_baro(double pressure, double* alt_baro, bool* is_outlier) {
    *is_outlier = false;
    
    if (mxIsNaN(pressure) || pressure < 80000 || pressure > 110000) {
        *is_outlier = true;
        *alt_baro = 0.0;
        return;
    }
    
    // 気圧→高度変換
    *alt_baro = 44330.0 * (1.0 - pow(pressure / 101325.0, 0.1903));
}

//=============================================================================
// センサー更新ハンドラ
//=============================================================================
static void handle_update(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // 入力: sensor_type, state_struct, prev_struct, params_struct, sensor_data
    // 出力: updated_state, updated_prev, should_skip
    
    if (nrhs < 6) {
        mexErrMsgTxt("update requires: sensor_type, state, prev, params, sensor_data");
    }
    
    // センサータイプを取得
    char sensor_type[32];
    mxGetString(prhs[1], sensor_type, sizeof(sensor_type));
    
    // 状態を取得
    const mxArray* state = prhs[2];
    double* p = mxGetPr(mxGetField(state, 0, "p"));
    double* v = mxGetPr(mxGetField(state, 0, "v"));
    double* q = mxGetPr(mxGetField(state, 0, "q"));
    double* ba = mxGetPr(mxGetField(state, 0, "ba"));
    double* bg = mxGetPr(mxGetField(state, 0, "bg"));
    double* P = mxGetPr(mxGetField(state, 0, "P"));
    double* w_body = mxGetPr(mxGetField(state, 0, "w_body"));
    
    // 前回値を取得
    const mxArray* prev = prhs[3];
    double* prev_accel = mxGetPr(mxGetField(prev, 0, "accel"));
    double* prev_mag = mxGetPr(mxGetField(prev, 0, "mag"));
    double prev_baro = mxGetScalar(mxGetField(prev, 0, "baro"));
    double* gps_origin = mxGetPr(mxGetField(prev, 0, "gps_origin"));
    double buffer_tolerance = mxGetScalar(mxGetField(prev, 0, "buffer_tolerance"));
    double baro_weight = mxGetScalar(mxGetField(prev, 0, "baro_weight"));
    
    // センサーデータを取得
    const mxArray* sensor_data = prhs[5];
    
    // 出力用変数
    bool should_skip = false;
    double meas[3] = {0, 0, 0};
    double new_prev_accel[3], new_prev_mag[3];
    double new_prev_baro = prev_baro;
    
    memcpy(new_prev_accel, prev_accel, 3 * sizeof(double));
    memcpy(new_prev_mag, prev_mag, 3 * sizeof(double));
    
    // センサータイプ別処理
    if (strcmp(sensor_type, "accel") == 0) {
        double* a_meas = mxGetPr(mxGetField(sensor_data, 0, "accel"));
        double a_corrected[3];
        bool is_outlier, no_change;
        
        preprocess_accel(a_meas, prev_accel, a_corrected, &is_outlier, &no_change);
        
        if (no_change || is_nan_any(a_corrected, 3) || is_outlier || (norm3(w_body) > 1.5)) {
            should_skip = true;
        } else {
            memcpy(meas, a_corrected, 3 * sizeof(double));
            memcpy(new_prev_accel, a_meas, 3 * sizeof(double));
        }
    }
    else if (strcmp(sensor_type, "mag") == 0) {
        double* m_meas = mxGetPr(mxGetField(sensor_data, 0, "mag"));
        double m_filtered[3];
        bool is_outlier, no_change;
        
        preprocess_mag(m_meas, prev_mag, m_filtered, &is_outlier, &no_change);
        
        if (no_change || is_nan_any(m_filtered, 3) || is_outlier) {
            should_skip = true;
        } else {
            memcpy(meas, m_filtered, 3 * sizeof(double));
            memcpy(new_prev_mag, m_meas, 3 * sizeof(double));
        }
    }
    else if (strcmp(sensor_type, "gps") == 0) {
        double lat = mxGetScalar(mxGetField(sensor_data, 0, "lat"));
        double lon = mxGetScalar(mxGetField(sensor_data, 0, "lon"));
        double alt = mxGetScalar(mxGetField(sensor_data, 0, "alt"));
        double z_gps[3];
        bool is_outlier, no_change;
        
        preprocess_gps(lat, lon, alt, gps_origin, z_gps, &is_outlier, &no_change);
        
        if (no_change || is_outlier) {
            should_skip = true;
        } else {
            memcpy(meas, z_gps, 3 * sizeof(double));
        }
    }
    else if (strcmp(sensor_type, "baro") == 0) {
        double pressure = mxGetScalar(mxGetField(sensor_data, 0, "pressure"));
        
        if (fabs(pressure - prev_baro) <= buffer_tolerance) {
            should_skip = true;
        } else {
            double alt_baro;
            bool is_outlier;
            preprocess_baro(pressure, &alt_baro, &is_outlier);
            
            if (mxIsNaN(alt_baro) || is_outlier) {
                should_skip = true;
            } else {
                meas[0] = alt_baro;
                new_prev_baro = pressure;
            }
        }
    }
    else {
        mexErrMsgTxt("Unknown sensor type");
    }
    
    // 出力構造体を作成
    const char* out_field_names[] = {"meas", "should_skip", "new_prev_accel", "new_prev_mag", "new_prev_baro"};
    plhs[0] = mxCreateStructMatrix(1, 1, 5, out_field_names);
    
    mxArray* meas_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(meas_arr), meas, 3 * sizeof(double));
    mxSetField(plhs[0], 0, "meas", meas_arr);
    
    mxSetField(plhs[0], 0, "should_skip", mxCreateLogicalScalar(should_skip));
    
    mxArray* new_prev_accel_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(new_prev_accel_arr), new_prev_accel, 3 * sizeof(double));
    mxSetField(plhs[0], 0, "new_prev_accel", new_prev_accel_arr);
    
    mxArray* new_prev_mag_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(new_prev_mag_arr), new_prev_mag, 3 * sizeof(double));
    mxSetField(plhs[0], 0, "new_prev_mag", new_prev_mag_arr);
    
    mxSetField(plhs[0], 0, "new_prev_baro", mxCreateDoubleScalar(new_prev_baro));
}

//=============================================================================
// MEXエントリーポイント
//=============================================================================
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgTxt("Usage: mex_eskf_sensor_updates('update', sensor_type, state, prev, params, sensor_data)");
    }
    
    if (!mxIsChar(prhs[0])) {
        mexErrMsgTxt("First argument must be a command string");
    }
    
    char cmd[64];
    mxGetString(prhs[0], cmd, sizeof(cmd));
    
    if (strcmp(cmd, "update") == 0) {
        handle_update(nlhs, plhs, nrhs, prhs);
    } else {
        mexErrMsgTxt("Unknown command. Use 'update'");
    }
}



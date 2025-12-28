// mex_eskf_constructor.cpp
// Phase 1: ESKFコンストラクタのMEX化
// ESKF.mのコンストラクタと完全に同じ処理をC++で実装

#include "mex.h"
#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>

//=============================================================================
// 定数
//=============================================================================
static const double GRAVITY = 9.80665;
static const double DEG2RAD = 0.017453292519943295;
static const double RAD2DEG = 57.29577951308232;

//=============================================================================
// ヘルパー関数: MATLABフィールド操作
//=============================================================================
static const mxArray* get_field(const mxArray* s, const char* name) {
    if (!mxIsStruct(s)) return nullptr;
    return mxGetField(s, 0, name);
}

static bool has_field(const mxArray* s, const char* name) {
    return get_field(s, name) != nullptr;
}

static bool has_field_any(const mxArray* s, const char* name1, const char* name2) {
    return has_field(s, name1) || has_field(s, name2);
}

static const mxArray* get_field_any(const mxArray* s, const char* name1, const char* name2) {
    const mxArray* f = get_field(s, name1);
    if (f) return f;
    return get_field(s, name2);
}

static double* get_data(const mxArray* arr) {
    if (!arr) return nullptr;
    return mxGetPr(arr);
}

static int get_length(const mxArray* arr) {
    if (!arr) return 0;
    return static_cast<int>(mxGetNumberOfElements(arr));
}

//=============================================================================
// 数学関数
//=============================================================================
static void compute_mean(const double* data, int n, double* mean) {
    *mean = 0.0;
    for (int i = 0; i < n; ++i) {
        *mean += data[i];
    }
    *mean /= n;
}

static void compute_mean_3d(const double* ax, const double* ay, const double* az, 
                            int n, double* mean_x, double* mean_y, double* mean_z) {
    *mean_x = *mean_y = *mean_z = 0.0;
    for (int i = 0; i < n; ++i) {
        *mean_x += ax[i];
        *mean_y += ay[i];
        *mean_z += az[i];
    }
    *mean_x /= n;
    *mean_y /= n;
    *mean_z /= n;
}

static double compute_std(const double* data, int n, double mean) {
    if (n < 2) return 0.0;
    double sum_sq = 0.0;
    for (int i = 0; i < n; ++i) {
        double diff = data[i] - mean;
        sum_sq += diff * diff;
    }
    return sqrt(sum_sq / (n - 1));
}

static double compute_std_3d(const double* ax, const double* ay, const double* az, 
                              int n, double mean_x, double mean_y, double mean_z) {
    double std_x = compute_std(ax, n, mean_x);
    double std_y = compute_std(ay, n, mean_y);
    double std_z = compute_std(az, n, mean_z);
    return (std_x + std_y + std_z) / 3.0;
}

//=============================================================================
// クォータニオン関数
//=============================================================================
static void quaternion_from_euler(double roll, double pitch, double yaw, double* q) {
    // roll, pitch, yaw はラジアン
    double cr = cos(roll * 0.5), sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5), sy = sin(yaw * 0.5);
    
    q[0] = cr * cp * cy + sr * sp * sy;  // w
    q[1] = sr * cp * cy - cr * sp * sy;  // x
    q[2] = cr * sp * cy + sr * cp * sy;  // y
    q[3] = cr * cp * sy - sr * sp * cy;  // z
}

static void quaternion_to_rotation_matrix(const double* q, double* R) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    
    // Column-major for MATLAB
    R[0] = 1.0 - 2.0*(y*y + z*z);  R[3] = 2.0*(x*y - w*z);        R[6] = 2.0*(x*z + w*y);
    R[1] = 2.0*(x*y + w*z);        R[4] = 1.0 - 2.0*(x*x + z*z);  R[7] = 2.0*(y*z - w*x);
    R[2] = 2.0*(x*z - w*y);        R[5] = 2.0*(y*z + w*x);        R[8] = 1.0 - 2.0*(x*x + y*y);
}

//=============================================================================
// 初期化処理
//=============================================================================
static void handle_init(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 4) {
        mexErrMsgTxt("init requires: obs, static_time, dt");
    }
    
    const mxArray* obs = prhs[1];
    double static_time = mxGetScalar(prhs[2]);
    double dt = mxGetScalar(prhs[3]);
    
    // 静止サンプル数の計算
    int N_static = static_cast<int>(floor(static_time / dt));
    
    // 基本状態の初期化
    double p[3] = {0, 0, 0};
    double v[3] = {0, 0, 0};
    double g[3] = {0, 0, GRAVITY};
    double q[4] = {1, 0, 0, 0};
    double ba[3] = {0, 0, 0};
    double bg[3] = {0, 0, 0};
    
    // ノイズパラメータのデフォルト値
    double sigma_a = 0.1;
    double sigma_g = DEG2RAD * 0.1;
    double sigma_mag = 10.0;
    double sigma_press = 1.0;
    double sigma_gps = 1.0;
    double gyro_noise_threshold = DEG2RAD * 0.1;
    
    // GPS原点
    double gps_origin[3] = {0, 0, 0};
    
    // 静止データがある場合の処理
    const mxArray* ax_arr = get_field_any(obs, "ax", "accel_x");
    const mxArray* ay_arr = get_field_any(obs, "ay", "accel_y");
    const mxArray* az_arr = get_field_any(obs, "az", "accel_z");
    
    int n_samples = ax_arr ? get_length(ax_arr) : 0;
    if (N_static > n_samples) N_static = n_samples;
    
    if (ax_arr && ay_arr && az_arr && N_static > 10) {
        double* ax = get_data(ax_arr);
        double* ay = get_data(ay_arr);
        double* az = get_data(az_arr);
        
        // 加速度平均と標準偏差
        double accel_mean_x, accel_mean_y, accel_mean_z;
        compute_mean_3d(ax, ay, az, N_static, &accel_mean_x, &accel_mean_y, &accel_mean_z);
        sigma_a = compute_std_3d(ax, ay, az, N_static, accel_mean_x, accel_mean_y, accel_mean_z);
        if (sigma_a < 0.01) sigma_a = 0.01;
        
        // 初期姿勢計算（Roll/Pitch）
        double phi = atan2(-accel_mean_y, -accel_mean_z);
        double theta = atan2(accel_mean_x, sqrt(accel_mean_y*accel_mean_y + accel_mean_z*accel_mean_z));
        
        // ジャイロデータ
        const mxArray* wx_arr = get_field_any(obs, "wx", "gyro_x");
        const mxArray* wy_arr = get_field_any(obs, "wy", "gyro_y");
        const mxArray* wz_arr = get_field_any(obs, "wz", "gyro_z");
        
        if (wx_arr && wy_arr && wz_arr) {
            double* wx = get_data(wx_arr);
            double* wy = get_data(wy_arr);
            double* wz = get_data(wz_arr);
            
            double gyro_mean_x, gyro_mean_y, gyro_mean_z;
            compute_mean_3d(wx, wy, wz, N_static, &gyro_mean_x, &gyro_mean_y, &gyro_mean_z);
            double sigma_g_deg = compute_std_3d(wx, wy, wz, N_static, gyro_mean_x, gyro_mean_y, gyro_mean_z);
            sigma_g = DEG2RAD * sigma_g_deg;
            if (sigma_g < 0.001) sigma_g = 0.001;
            
            // gyro_noise_threshold の計算
            double std_wx = compute_std(wx, N_static, gyro_mean_x);
            double std_wy = compute_std(wy, N_static, gyro_mean_y);
            double std_wz = compute_std(wz, N_static, gyro_mean_z);
            double max_std = std_wx;
            if (std_wy > max_std) max_std = std_wy;
            if (std_wz > max_std) max_std = std_wz;
            gyro_noise_threshold = 2.0 * DEG2RAD * max_std;
        }
        
        // 磁気データからYaw計算
        const mxArray* mx_arr = get_field_any(obs, "mx", "mag_x");
        const mxArray* my_arr = get_field_any(obs, "my", "mag_y");
        const mxArray* mz_arr = get_field_any(obs, "mz", "mag_z");
        
        double psi = 0.0;
        if (mx_arr && my_arr && mz_arr) {
            double* mx = get_data(mx_arr);
            double* my = get_data(my_arr);
            double* mz = get_data(mz_arr);
            
            double mag_mean_x, mag_mean_y, mag_mean_z;
            compute_mean_3d(mx, my, mz, N_static, &mag_mean_x, &mag_mean_y, &mag_mean_z);
            sigma_mag = compute_std_3d(mx, my, mz, N_static, mag_mean_x, mag_mean_y, mag_mean_z);
            if (sigma_mag < 0.1) sigma_mag = 0.1;
            
            // Roll/Pitchのみのクォータニオン
            double q_rp[4];
            quaternion_from_euler(phi, theta, 0.0, q_rp);
            
            // 回転行列
            double R_rp[9];
            quaternion_to_rotation_matrix(q_rp, R_rp);
            
            // 磁気を水平面に射影
            double m_level_x = R_rp[0]*mag_mean_x + R_rp[3]*mag_mean_y + R_rp[6]*mag_mean_z;
            double m_level_y = R_rp[1]*mag_mean_x + R_rp[4]*mag_mean_y + R_rp[7]*mag_mean_z;
            
            psi = -atan2(m_level_y, m_level_x);
        }
        
        // 最終クォータニオン
        quaternion_from_euler(phi, theta, psi, q);
        
        // 気圧データ
        const mxArray* pressure_arr = get_field_any(obs, "pressure", "baro");
        if (pressure_arr) {
            double* pressure = get_data(pressure_arr);
            
            // 気圧高度計算
            std::vector<double> alt_baro(N_static);
            double alt_mean = 0.0;
            for (int i = 0; i < N_static; ++i) {
                alt_baro[i] = 44330.0 * (1.0 - pow(pressure[i] / 101325.0, 0.1903));
                alt_mean += alt_baro[i];
            }
            alt_mean /= N_static;
            
            double sum_sq = 0.0;
            for (int i = 0; i < N_static; ++i) {
                double diff = alt_baro[i] - alt_mean;
                sum_sq += diff * diff;
            }
            sigma_press = sqrt(sum_sq / (N_static - 1));
            if (sigma_press < 0.1) sigma_press = 0.1;
        }
        
        // GPSデータ
        const mxArray* lat_arr = get_field_any(obs, "lat", "gps_lat");
        const mxArray* lon_arr = get_field_any(obs, "lon", "gps_lon");
        const mxArray* alt_arr = get_field_any(obs, "alt", "gps_alt");
        
        if (lat_arr && lon_arr && alt_arr) {
            double* lat = get_data(lat_arr);
            double* lon = get_data(lon_arr);
            double* alt = get_data(alt_arr);
            
            // GPS原点計算（NaNを除外）
            double lat_sum = 0.0, lon_sum = 0.0, alt_sum = 0.0;
            int valid_count = 0;
            for (int i = 0; i < N_static; ++i) {
                if (!mxIsNaN(lat[i])) {
                    lat_sum += lat[i];
                    lon_sum += lon[i];
                    alt_sum += alt[i];
                    valid_count++;
                }
            }
            
            if (valid_count > 0) {
                gps_origin[0] = lat_sum / valid_count;
                gps_origin[1] = lon_sum / valid_count;
                gps_origin[2] = alt_sum / valid_count;
                
                // GPS標準偏差計算
                double cos_lat0 = cos(gps_origin[0] * DEG2RAD);
                std::vector<double> x_m(valid_count), y_m(valid_count), z_m(valid_count);
                int idx = 0;
                for (int i = 0; i < N_static; ++i) {
                    if (!mxIsNaN(lat[i])) {
                        y_m[idx] = (lat[i] - gps_origin[0]) / 9.0e-6;
                        x_m[idx] = (lon[i] - gps_origin[1]) / (9.0e-6 / cos_lat0);
                        z_m[idx] = alt[i] - gps_origin[2];
                        idx++;
                    }
                }
                
                double mean_x = 0, mean_y = 0, mean_z = 0;
                for (int i = 0; i < valid_count; ++i) {
                    mean_x += x_m[i];
                    mean_y += y_m[i];
                    mean_z += z_m[i];
                }
                mean_x /= valid_count;
                mean_y /= valid_count;
                mean_z /= valid_count;
                
                double std_x = compute_std(x_m.data(), valid_count, mean_x);
                double std_y = compute_std(y_m.data(), valid_count, mean_y);
                double std_z = compute_std(z_m.data(), valid_count, mean_z);
                sigma_gps = (std_x + std_y + std_z) / 3.0;
                if (sigma_gps < 0.1) sigma_gps = 0.1;
            }
        }
    }
    
    // Q行列の初期化
    double Q[15*15] = {0};
    for (int i = 3; i < 6; ++i) Q[i*15 + i] = 0.003 * 0.003;  // 速度
    for (int i = 6; i < 9; ++i) Q[i*15 + i] = 0.003 * 0.003;  // 姿勢
    for (int i = 9; i < 12; ++i) Q[i*15 + i] = sigma_a * sigma_a * 1e-3;  // 加速度バイアス
    for (int i = 12; i < 15; ++i) Q[i*15 + i] = sigma_g * sigma_g * 1e-3;  // ジャイロバイアス
    
    // P行列の初期化
    double P[15*15] = {0};
    for (int i = 0; i < 15; ++i) P[i*15 + i] = 0.01;
    for (int i = 0; i < 3; ++i) P[i*15 + i] = 5.0;  // 位置
    for (int i = 3; i < 6; ++i) P[i*15 + i] = 0.5;  // 速度
    for (int i = 9; i < 12; ++i) P[i*15 + i] = 0.5;  // 加速度バイアス
    for (int i = 12; i < 15; ++i) P[i*15 + i] = 0.1;  // ジャイロバイアス
    
    // 出力構造体の作成
    const char* field_names[] = {
        "p", "v", "q", "ba", "bg", "P", "Q", "g", "dt",
        "gps_origin", "gyro_noise_threshold",
        "noiseEstimator",
        "prev_accel", "prev_gyro", "prev_mag",
        "prev_gps_lat", "prev_gps_lon", "prev_gps_alt", "prev_baro",
        "buffer_tolerance",
        "freq_accel", "freq_mag", "freq_baro", "freq_gps",
        "zupt_threshold_accel", "zupt_threshold_gyro", "zupt_min_duration",
        "zupt_counter", "is_stationary",
        "Q_nominal", "adaptive_q_enabled",
        "last_reset_step", "velocity_damping",
        "w_body", "quaternion_norm", "accel_innovation_norm",
        "enable_accel_z_integration", "accel_z_threshold", "accel_z_damping", "baro_weight"
    };
    int n_fields = sizeof(field_names) / sizeof(field_names[0]);
    
    plhs[0] = mxCreateStructMatrix(1, 1, n_fields, field_names);
    
    // 状態ベクトル
    mxArray* p_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    mxArray* v_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    mxArray* q_arr = mxCreateDoubleMatrix(4, 1, mxREAL);
    mxArray* ba_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    mxArray* bg_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    mxArray* P_arr_out = mxCreateDoubleMatrix(15, 15, mxREAL);
    mxArray* Q_arr = mxCreateDoubleMatrix(15, 15, mxREAL);
    mxArray* g_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    
    memcpy(mxGetPr(p_arr), p, 3 * sizeof(double));
    memcpy(mxGetPr(v_arr), v, 3 * sizeof(double));
    memcpy(mxGetPr(q_arr), q, 4 * sizeof(double));
    memcpy(mxGetPr(ba_arr), ba, 3 * sizeof(double));
    memcpy(mxGetPr(bg_arr), bg, 3 * sizeof(double));
    memcpy(mxGetPr(P_arr_out), P, 15 * 15 * sizeof(double));
    memcpy(mxGetPr(Q_arr), Q, 15 * 15 * sizeof(double));
    memcpy(mxGetPr(g_arr), g, 3 * sizeof(double));
    
    mxSetField(plhs[0], 0, "p", p_arr);
    mxSetField(plhs[0], 0, "v", v_arr);
    mxSetField(plhs[0], 0, "q", q_arr);
    mxSetField(plhs[0], 0, "ba", ba_arr);
    mxSetField(plhs[0], 0, "bg", bg_arr);
    mxSetField(plhs[0], 0, "P", P_arr_out);
    mxSetField(plhs[0], 0, "Q", Q_arr);
    mxSetField(plhs[0], 0, "g", g_arr);
    mxSetField(plhs[0], 0, "dt", mxCreateDoubleScalar(dt));
    
    // GPS原点
    mxArray* gps_origin_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(gps_origin_arr), gps_origin, 3 * sizeof(double));
    mxSetField(plhs[0], 0, "gps_origin", gps_origin_arr);
    mxSetField(plhs[0], 0, "gyro_noise_threshold", mxCreateDoubleScalar(gyro_noise_threshold));
    
    // noiseEstimator構造体
    const char* noise_fields[] = {"R_accel", "R_gyro", "R_mag", "R_baro", "R_gps"};
    mxArray* noise_struct = mxCreateStructMatrix(1, 1, 5, noise_fields);
    
    mxArray* R_accel_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    mxArray* R_gyro_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    mxArray* R_mag_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    mxArray* R_gps_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    
    double R_accel[3] = {sigma_a*sigma_a, sigma_a*sigma_a, sigma_a*sigma_a};
    double R_gyro[3] = {sigma_g*sigma_g, sigma_g*sigma_g, sigma_g*sigma_g};
    double R_mag[3] = {sigma_mag*sigma_mag, sigma_mag*sigma_mag, sigma_mag*sigma_mag};
    double R_gps[3] = {sigma_gps*sigma_gps, sigma_gps*sigma_gps, sigma_gps*sigma_gps};
    
    memcpy(mxGetPr(R_accel_arr), R_accel, 3 * sizeof(double));
    memcpy(mxGetPr(R_gyro_arr), R_gyro, 3 * sizeof(double));
    memcpy(mxGetPr(R_mag_arr), R_mag, 3 * sizeof(double));
    memcpy(mxGetPr(R_gps_arr), R_gps, 3 * sizeof(double));
    
    mxSetField(noise_struct, 0, "R_accel", R_accel_arr);
    mxSetField(noise_struct, 0, "R_gyro", R_gyro_arr);
    mxSetField(noise_struct, 0, "R_mag", R_mag_arr);
    mxSetField(noise_struct, 0, "R_baro", mxCreateDoubleScalar(sigma_press * sigma_press));
    mxSetField(noise_struct, 0, "R_gps", R_gps_arr);
    
    mxSetField(plhs[0], 0, "noiseEstimator", noise_struct);
    
    // prev_* 初期化
    double zeros3[3] = {0, 0, 0};
    mxArray* prev_accel_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    mxArray* prev_gyro_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    mxArray* prev_mag_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(prev_accel_arr), zeros3, 3 * sizeof(double));
    memcpy(mxGetPr(prev_gyro_arr), zeros3, 3 * sizeof(double));
    memcpy(mxGetPr(prev_mag_arr), zeros3, 3 * sizeof(double));
    
    mxSetField(plhs[0], 0, "prev_accel", prev_accel_arr);
    mxSetField(plhs[0], 0, "prev_gyro", prev_gyro_arr);
    mxSetField(plhs[0], 0, "prev_mag", prev_mag_arr);
    mxSetField(plhs[0], 0, "prev_gps_lat", mxCreateDoubleScalar(0));
    mxSetField(plhs[0], 0, "prev_gps_lon", mxCreateDoubleScalar(0));
    mxSetField(plhs[0], 0, "prev_gps_alt", mxCreateDoubleScalar(0));
    mxSetField(plhs[0], 0, "prev_baro", mxCreateDoubleScalar(0));
    mxSetField(plhs[0], 0, "buffer_tolerance", mxCreateDoubleScalar(1e-9));
    
    // freq_* 初期化
    mxSetField(plhs[0], 0, "freq_accel", mxCreateDoubleScalar(1));
    mxSetField(plhs[0], 0, "freq_mag", mxCreateDoubleScalar(1));
    mxSetField(plhs[0], 0, "freq_baro", mxCreateDoubleScalar(1));
    mxSetField(plhs[0], 0, "freq_gps", mxCreateDoubleScalar(1));
    
    // zupt_* 初期化
    mxSetField(plhs[0], 0, "zupt_threshold_accel", mxCreateDoubleScalar(1.0));
    mxSetField(plhs[0], 0, "zupt_threshold_gyro", mxCreateDoubleScalar(DEG2RAD * 3.0));
    mxSetField(plhs[0], 0, "zupt_min_duration", mxCreateDoubleScalar(10));
    mxSetField(plhs[0], 0, "zupt_counter", mxCreateDoubleScalar(0));
    mxSetField(plhs[0], 0, "is_stationary", mxCreateLogicalScalar(false));
    
    // Q_nominal, adaptive_q
    mxArray* Q_nominal_arr = mxCreateDoubleMatrix(15, 15, mxREAL);
    memcpy(mxGetPr(Q_nominal_arr), Q, 15 * 15 * sizeof(double));
    mxSetField(plhs[0], 0, "Q_nominal", Q_nominal_arr);
    mxSetField(plhs[0], 0, "adaptive_q_enabled", mxCreateLogicalScalar(true));
    
    // その他
    mxSetField(plhs[0], 0, "last_reset_step", mxCreateDoubleMatrix(0, 0, mxREAL));  // empty
    mxSetField(plhs[0], 0, "velocity_damping", mxCreateDoubleScalar(0.0));
    
    mxArray* w_body_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
    memcpy(mxGetPr(w_body_arr), zeros3, 3 * sizeof(double));
    mxSetField(plhs[0], 0, "w_body", w_body_arr);
    
    mxSetField(plhs[0], 0, "quaternion_norm", mxCreateDoubleScalar(1.0));
    mxSetField(plhs[0], 0, "accel_innovation_norm", mxCreateDoubleScalar(0.0));
    
    // accel_z 関連
    mxSetField(plhs[0], 0, "enable_accel_z_integration", mxCreateLogicalScalar(true));
    mxSetField(plhs[0], 0, "accel_z_threshold", mxCreateDoubleScalar(0.5));
    mxSetField(plhs[0], 0, "accel_z_damping", mxCreateDoubleScalar(0.1));
    mxSetField(plhs[0], 0, "baro_weight", mxCreateDoubleScalar(0.2));
}

//=============================================================================
// MEXエントリーポイント
//=============================================================================
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgTxt("Usage: mex_eskf_constructor('init', obs, static_time, dt)");
    }
    
    if (!mxIsChar(prhs[0])) {
        mexErrMsgTxt("First argument must be a command string");
    }
    
    char cmd[64];
    mxGetString(prhs[0], cmd, sizeof(cmd));
    
    if (strcmp(cmd, "init") == 0) {
        handle_init(nlhs, plhs, nrhs, prhs);
    } else {
        mexErrMsgTxt("Unknown command. Use 'init'");
    }
}




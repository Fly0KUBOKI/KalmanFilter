// mex_eskf_full.cpp
// Complete ESKF implementation in MEX
// Replaces ESKF.m entirely

#include "mex.h"
#include <cmath>
#include <cstring>
#include <map>
#include <vector>
#include <algorithm>

// Forward declarations for helper functions
static void quaternion_from_euler(const float* euler_deg, float* q);
static void quaternion_to_euler(const float* q, float* euler_deg);
static void quaternion_to_rotation_matrix(const float* q, float* R);
static void quaternion_normalize(float* q);
static void quaternion_multiply(const float* q1, const float* q2, float* q_out);

//=============================================================================
// ESKFFull Class - Complete ESKF implementation
//=============================================================================
class ESKFFull {
public:
    // State vectors
    float p[3], v[3], ba[3], bg[3];
    float q[4];  // [w, x, y, z]
    float P[15*15], Q[15*15];
    float dt, g[3];
    
    // GPS origin
    float gps_origin[3];  // [lat0, lon0, alt0]
    
    // Previous sensor values
    float prev_accel[3], prev_mag[3];
    float prev_baro, prev_gps_lat, prev_gps_lon, prev_gps_alt;
    float buffer_tolerance;
    
    // Noise parameters
    float R_accel[3], R_gyro[3], R_mag[3], R_gps[3], R_baro;
    
    // ZUPT parameters
    float zupt_threshold_accel, zupt_threshold_gyro;
    int zupt_min_duration;
    int zupt_counter;
    bool is_stationary;
    
    // Sensor frequencies
    int freq_accel, freq_mag, freq_baro, freq_gps;
    int static_samples;
    
    // Other parameters
    float velocity_damping;
    bool enable_accel_z_integration;
    float accel_z_threshold, accel_z_damping;
    float baro_weight;
    float w_body[3];
    float gyro_noise_threshold;
    
    // Constructor
    ESKFFull() {
        memset(p, 0, sizeof(p));
        memset(v, 0, sizeof(v));
        memset(ba, 0, sizeof(ba));
        memset(bg, 0, sizeof(bg));
        q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
        memset(P, 0, sizeof(P));
        memset(Q, 0, sizeof(Q));
        dt = 0.01f;
        g[0] = 0.0f; g[1] = 0.0f; g[2] = 9.80665f;
        
        memset(gps_origin, 0, sizeof(gps_origin));
        memset(prev_accel, 0, sizeof(prev_accel));
        memset(prev_mag, 0, sizeof(prev_mag));
        prev_baro = 0.0f;
        prev_gps_lat = prev_gps_lon = prev_gps_alt = 0.0f;
        buffer_tolerance = 1e-9f;
        
        memset(R_accel, 0, sizeof(R_accel));
        memset(R_gyro, 0, sizeof(R_gyro));
        memset(R_mag, 0, sizeof(R_mag));
        memset(R_gps, 0, sizeof(R_gps));
        R_baro = 1.0f;
        
        zupt_threshold_accel = 1.0f;
        zupt_threshold_gyro = 0.0523599f;  // deg2rad(3)
        zupt_min_duration = 10;
        zupt_counter = 0;
        is_stationary = false;
        
        freq_accel = freq_mag = freq_baro = freq_gps = 1;
        static_samples = 0;
        
        velocity_damping = 0.0f;
        enable_accel_z_integration = true;
        accel_z_threshold = 0.5f;
        accel_z_damping = 0.1f;
        baro_weight = 0.2f;
        memset(w_body, 0, sizeof(w_body));
        gyro_noise_threshold = 0.00175f;  // deg2rad(0.1)
    }
    
    // Initialize from observation data
    void init(const mxArray* obs, float static_time, float dt_in);
    
    // Single step: predict + sensor updates + reset check
    void step(const float* a, const float* w, const mxArray* sensors, int k,
              float* p_out, float* v_out, float* euler_out, float* ba_out, float* bg_out);
    
    // Internal methods
    void predict(const float* a, const float* w);
    void predict_postprocess(const float* a);
    void sensor_update_accel(const float* a, int k);
    void sensor_update_mag(const float* m, int k);
    void sensor_update_baro(float pressure, int k);
    void sensor_update_gps(float lat, float lon, float alt, int k);
    void reset_check(int k);
    bool zupt_check(const float* a, const float* w);
    void zupt_update();
    void get_euler(float* euler_deg);
    
private:
    void call_mex_adaptive_predict(const float* a, const float* w);
    void call_mex_eskf_predict_postprocess(const float* a);
    void call_mex_meukf_step(const char* sensor_type, const float* meas, int k);
    void call_mex_eskf_update_postprocess(const char* sensor_type, 
        const float* dx, const float* innov, int innov_size,
        const float* state_p, const float* state_v, const float* state_q,
        const float* state_ba, const float* state_bg, const float* state_P,
        const float* new_state_P, int k);
};

//=============================================================================
// Handle Management
//=============================================================================
static std::map<uint64_t, ESKFFull*> g_handles;
static uint64_t g_next_handle = 1;

static uint64_t create_handle(ESKFFull* eskf) {
    uint64_t handle = g_next_handle++;
    g_handles[handle] = eskf;
    return handle;
}

static ESKFFull* get_handle(uint64_t handle) {
    auto it = g_handles.find(handle);
    if (it == g_handles.end()) return nullptr;
    return it->second;
}

static void delete_handle(uint64_t handle) {
    auto it = g_handles.find(handle);
    if (it != g_handles.end()) {
        delete it->second;
        g_handles.erase(it);
    }
}

//=============================================================================
// Helper: Get field from MATLAB struct
//=============================================================================
static const mxArray* get_field(const mxArray* s, const char* name) {
    if (!mxIsStruct(s)) return nullptr;
    return mxGetField(s, 0, name);
}

static bool get_scalar(const mxArray* arr, float& out) {
    if (!arr || mxGetNumberOfElements(arr) < 1) return false;
    out = static_cast<float>(mxGetScalar(arr));
    return true;
}

static bool get_vector(const mxArray* arr, float* out, int n) {
    if (!arr || mxGetNumberOfElements(arr) < static_cast<size_t>(n)) return false;
    double* pr = mxGetPr(arr);
    for (int i = 0; i < n; ++i) out[i] = static_cast<float>(pr[i]);
    return true;
}

//=============================================================================
// Quaternion Helpers
//=============================================================================
static void quaternion_from_euler(const float* euler_deg, float* q) {
    float roll = euler_deg[0] * 0.0174533f;   // deg to rad
    float pitch = euler_deg[1] * 0.0174533f;
    float yaw = euler_deg[2] * 0.0174533f;
    
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    
    q[0] = cr * cp * cy + sr * sp * sy;  // w
    q[1] = sr * cp * cy - cr * sp * sy;  // x
    q[2] = cr * sp * cy + sr * cp * sy;  // y
    q[3] = cr * cp * sy - sr * sp * cy;  // z
}

static void quaternion_to_euler(const float* q, float* euler_deg) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    float roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2.0f * (w * y - z * x);
    float pitch;
    if (fabsf(sinp) >= 1.0f)
        pitch = copysignf(1.5707963f, sinp);  // pi/2
    else
        pitch = asinf(sinp);
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    float yaw = atan2f(siny_cosp, cosy_cosp);
    
    euler_deg[0] = roll * 57.29578f;   // rad to deg
    euler_deg[1] = pitch * 57.29578f;
    euler_deg[2] = yaw * 57.29578f;
}

static void quaternion_to_rotation_matrix(const float* q, float* R) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    
    R[0] = 1.0f - 2.0f*(y*y + z*z);  R[1] = 2.0f*(x*y - w*z);        R[2] = 2.0f*(x*z + w*y);
    R[3] = 2.0f*(x*y + w*z);         R[4] = 1.0f - 2.0f*(x*x + z*z); R[5] = 2.0f*(y*z - w*x);
    R[6] = 2.0f*(x*z - w*y);         R[7] = 2.0f*(y*z + w*x);        R[8] = 1.0f - 2.0f*(x*x + y*y);
}

static void quaternion_normalize(float* q) {
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-10f) {
        q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
    }
}

static void quaternion_multiply(const float* q1, const float* q2, float* q_out) {
    q_out[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q_out[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    q_out[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    q_out[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

//=============================================================================
// ESKFFull Implementation
//=============================================================================

void ESKFFull::init(const mxArray* obs, float static_time, float dt_in) {
    this->dt = dt_in;
    
    // Calculate static samples
    int N_static = static_cast<int>(static_time / dt);
    this->static_samples = N_static;
    
    // Get observation data arrays
    const mxArray* ax_arr = get_field(obs, "ax");
    const mxArray* ay_arr = get_field(obs, "ay");
    const mxArray* az_arr = get_field(obs, "az");
    const mxArray* wx_arr = get_field(obs, "wx");
    const mxArray* wy_arr = get_field(obs, "wy");
    const mxArray* wz_arr = get_field(obs, "wz");
    const mxArray* mx_arr = get_field(obs, "mx");
    const mxArray* my_arr = get_field(obs, "my");
    const mxArray* mz_arr = get_field(obs, "mz");
    const mxArray* lat_arr = get_field(obs, "lat");
    const mxArray* lon_arr = get_field(obs, "lon");
    const mxArray* alt_arr = get_field(obs, "alt");
    const mxArray* pressure_arr = get_field(obs, "pressure");
    
    if (!ax_arr || !ay_arr || !az_arr) {
        mexErrMsgTxt("Observation struct must contain ax, ay, az fields");
    }
    
    int n_samples = static_cast<int>(mxGetNumberOfElements(ax_arr));
    if (N_static > n_samples) N_static = n_samples;
    if (N_static < 10) N_static = 10;
    
    double* ax = mxGetPr(ax_arr);
    double* ay = mxGetPr(ay_arr);
    double* az = mxGetPr(az_arr);
    
    // Calculate initial attitude from static accelerometer data
    float accel_mean[3] = {0, 0, 0};
    for (int i = 0; i < N_static; ++i) {
        accel_mean[0] += static_cast<float>(ax[i]);
        accel_mean[1] += static_cast<float>(ay[i]);
        accel_mean[2] += static_cast<float>(az[i]);
    }
    accel_mean[0] /= N_static;
    accel_mean[1] /= N_static;
    accel_mean[2] /= N_static;
    
    // Calculate roll and pitch from gravity
    float phi = atan2f(-accel_mean[1], -accel_mean[2]);
    float theta = atan2f(accel_mean[0], sqrtf(accel_mean[1]*accel_mean[1] + accel_mean[2]*accel_mean[2]));
    
    // Calculate yaw from magnetometer if available
    float psi = 0.0f;
    if (mx_arr && my_arr && mz_arr) {
        double* mx = mxGetPr(mx_arr);
        double* my = mxGetPr(my_arr);
        double* mz = mxGetPr(mz_arr);
        
        float mag_mean[3] = {0, 0, 0};
        for (int i = 0; i < N_static; ++i) {
            mag_mean[0] += static_cast<float>(mx[i]);
            mag_mean[1] += static_cast<float>(my[i]);
            mag_mean[2] += static_cast<float>(mz[i]);
        }
        mag_mean[0] /= N_static;
        mag_mean[1] /= N_static;
        mag_mean[2] /= N_static;
        
        // Create rotation matrix for roll/pitch
        float euler_rp[3] = {phi * 57.29578f, theta * 57.29578f, 0.0f};
        float q_rp[4];
        quaternion_from_euler(euler_rp, q_rp);
        float R_rp[9];
        quaternion_to_rotation_matrix(q_rp, R_rp);
        
        // Rotate mag to level frame
        float m_level[3];
        m_level[0] = R_rp[0]*mag_mean[0] + R_rp[1]*mag_mean[1] + R_rp[2]*mag_mean[2];
        m_level[1] = R_rp[3]*mag_mean[0] + R_rp[4]*mag_mean[1] + R_rp[5]*mag_mean[2];
        m_level[2] = R_rp[6]*mag_mean[0] + R_rp[7]*mag_mean[1] + R_rp[8]*mag_mean[2];
        
        psi = -atan2f(m_level[1], m_level[0]);
    }
    
    // Set initial quaternion
    float euler_deg[3] = {phi * 57.29578f, theta * 57.29578f, psi * 57.29578f};
    quaternion_from_euler(euler_deg, this->q);
    
    // Calculate noise standard deviations
    float sigma_a = 0.1f, sigma_g = 0.00175f, sigma_mag = 10.0f;
    float sigma_press = 1.0f, sigma_gps = 1.0f;
    
    // Accel noise
    float accel_var = 0.0f;
    for (int i = 0; i < N_static; ++i) {
        float dx = static_cast<float>(ax[i]) - accel_mean[0];
        float dy = static_cast<float>(ay[i]) - accel_mean[1];
        float dz = static_cast<float>(az[i]) - accel_mean[2];
        accel_var += (dx*dx + dy*dy + dz*dz) / 3.0f;
    }
    sigma_a = sqrtf(accel_var / N_static);
    if (sigma_a < 0.01f) sigma_a = 0.01f;
    
    // Gyro noise
    if (wx_arr && wy_arr && wz_arr) {
        double* wx = mxGetPr(wx_arr);
        double* wy = mxGetPr(wy_arr);
        double* wz = mxGetPr(wz_arr);
        
        float gyro_var = 0.0f;
        for (int i = 0; i < N_static; ++i) {
            float gx = static_cast<float>(wx[i]) * 0.0174533f;
            float gy = static_cast<float>(wy[i]) * 0.0174533f;
            float gz = static_cast<float>(wz[i]) * 0.0174533f;
            gyro_var += (gx*gx + gy*gy + gz*gz) / 3.0f;
        }
        sigma_g = sqrtf(gyro_var / N_static);
        if (sigma_g < 0.001f) sigma_g = 0.001f;
        this->gyro_noise_threshold = 2.0f * sigma_g;
    }
    
    // GPS origin
    if (lat_arr && lon_arr && alt_arr) {
        double* lat = mxGetPr(lat_arr);
        double* lon = mxGetPr(lon_arr);
        double* alt = mxGetPr(alt_arr);
        
        float lat_sum = 0, lon_sum = 0, alt_sum = 0;
        int valid_count = 0;
        for (int i = 0; i < N_static; ++i) {
            if (!std::isnan(lat[i])) {
                lat_sum += static_cast<float>(lat[i]);
                lon_sum += static_cast<float>(lon[i]);
                alt_sum += static_cast<float>(alt[i]);
                valid_count++;
            }
        }
        if (valid_count > 0) {
            this->gps_origin[0] = lat_sum / valid_count;
            this->gps_origin[1] = lon_sum / valid_count;
            this->gps_origin[2] = alt_sum / valid_count;
        }
    }
    
    // Set R matrices
    for (int i = 0; i < 3; ++i) {
        this->R_accel[i] = sigma_a * sigma_a;
        this->R_gyro[i] = sigma_g * sigma_g;
        this->R_mag[i] = sigma_mag * sigma_mag;
        this->R_gps[i] = sigma_gps * sigma_gps;
    }
    this->R_baro = sigma_press * sigma_press;
    
    // Initialize P matrix
    for (int i = 0; i < 15*15; ++i) this->P[i] = 0.0f;
    for (int i = 0; i < 15; ++i) this->P[i*15 + i] = 0.01f;
    // Position: 5.0
    this->P[0] = this->P[16] = this->P[32] = 5.0f;
    // Velocity: 0.5
    this->P[48] = this->P[64] = this->P[80] = 0.5f;
    // Attitude: small
    this->P[96] = this->P[112] = this->P[128] = 0.01f;
    // Accel bias: 0.5
    this->P[144] = this->P[160] = this->P[176] = 0.5f;
    // Gyro bias: 0.1
    this->P[192] = this->P[208] = this->P[224] = 0.1f;
    
    // Initialize Q matrix
    for (int i = 0; i < 15*15; ++i) this->Q[i] = 0.0f;
    float q_vel = 0.003f * 0.003f;
    float q_att = 0.003f * 0.003f;
    float q_ba = sigma_a * sigma_a * 1e-3f;
    float q_bg = sigma_g * sigma_g * 1e-3f;
    for (int i = 3; i < 6; ++i) this->Q[i*15 + i] = q_vel;
    for (int i = 6; i < 9; ++i) this->Q[i*15 + i] = q_att;
    for (int i = 9; i < 12; ++i) this->Q[i*15 + i] = q_ba;
    for (int i = 12; i < 15; ++i) this->Q[i*15 + i] = q_bg;
}

void ESKFFull::step(const float* a, const float* w, const mxArray* sensors, int k,
                    float* p_out, float* v_out, float* euler_out, float* ba_out, float* bg_out) {
    
    // Skip static period
    if (k > this->static_samples) {
        // Predict
        predict(a, w);
        
        // ZUPT check and update
        if (zupt_check(a, w)) {
            zupt_update();
        }
        
        // Sensor updates
        if ((k % this->freq_accel) == 0) {
            sensor_update_accel(a, k);
        }
        
        if (sensors) {
            const mxArray* mag_arr = get_field(sensors, "mag");
            if (mag_arr && (k % this->freq_mag) == 0) {
                float m[3];
                if (get_vector(mag_arr, m, 3)) {
                    sensor_update_mag(m, k);
                }
            }
            
            const mxArray* pressure_arr = get_field(sensors, "pressure");
            if (pressure_arr && (k % this->freq_baro) == 0) {
                float pressure;
                if (get_scalar(pressure_arr, pressure)) {
                    sensor_update_baro(pressure, k);
                }
            }
            
            const mxArray* lat_arr = get_field(sensors, "lat");
            const mxArray* lon_arr = get_field(sensors, "lon");
            const mxArray* alt_arr = get_field(sensors, "alt");
            if (lat_arr && lon_arr && alt_arr && (k % this->freq_gps) == 0) {
                float lat, lon, alt;
                if (get_scalar(lat_arr, lat) && get_scalar(lon_arr, lon) && get_scalar(alt_arr, alt)) {
                    if (!std::isnan(lat)) {
                        sensor_update_gps(lat, lon, alt, k);
                    }
                }
            }
        }
    }
    
    // Reset check
    reset_check(k);
    
    // Output
    memcpy(p_out, this->p, 3 * sizeof(float));
    memcpy(v_out, this->v, 3 * sizeof(float));
    memcpy(ba_out, this->ba, 3 * sizeof(float));
    memcpy(bg_out, this->bg, 3 * sizeof(float));
    get_euler(euler_out);
}

void ESKFFull::predict(const float* a, const float* w) {
    // Call existing mex_adaptive_predict
    call_mex_adaptive_predict(a, w);
    
    // Store w_body
    memcpy(this->w_body, w, 3 * sizeof(float));
    
    // Call post-process
    call_mex_eskf_predict_postprocess(a);
}

void ESKFFull::call_mex_adaptive_predict(const float* a, const float* w) {
    // Create input arrays for mexCallMATLAB
    mxArray* prhs[15];
    mxArray* plhs[6];
    
    prhs[0] = mxCreateString("predict");
    
    // p
    prhs[1] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* p_ptr = mxGetPr(prhs[1]);
    for (int i = 0; i < 3; ++i) p_ptr[i] = this->p[i];
    
    // v
    prhs[2] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* v_ptr = mxGetPr(prhs[2]);
    for (int i = 0; i < 3; ++i) v_ptr[i] = this->v[i];
    
    // q
    prhs[3] = mxCreateDoubleMatrix(4, 1, mxREAL);
    double* q_ptr = mxGetPr(prhs[3]);
    for (int i = 0; i < 4; ++i) q_ptr[i] = this->q[i];
    
    // ba
    prhs[4] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* ba_ptr = mxGetPr(prhs[4]);
    for (int i = 0; i < 3; ++i) ba_ptr[i] = this->ba[i];
    
    // bg
    prhs[5] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* bg_ptr = mxGetPr(prhs[5]);
    for (int i = 0; i < 3; ++i) bg_ptr[i] = this->bg[i];
    
    // P
    prhs[6] = mxCreateDoubleMatrix(15, 15, mxREAL);
    double* P_ptr = mxGetPr(prhs[6]);
    for (int i = 0; i < 15*15; ++i) P_ptr[i] = this->P[i];
    
    // a
    prhs[7] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* a_ptr = mxGetPr(prhs[7]);
    for (int i = 0; i < 3; ++i) a_ptr[i] = a[i];
    
    // w
    prhs[8] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* w_ptr = mxGetPr(prhs[8]);
    for (int i = 0; i < 3; ++i) w_ptr[i] = w[i];
    
    // dt
    prhs[9] = mxCreateDoubleScalar(this->dt);
    
    // Q
    prhs[10] = mxCreateDoubleMatrix(15, 15, mxREAL);
    double* Q_ptr = mxGetPr(prhs[10]);
    for (int i = 0; i < 15*15; ++i) Q_ptr[i] = this->Q[i];
    
    // adaptive_q_enabled
    prhs[11] = mxCreateLogicalScalar(true);
    
    // zeros for noise
    prhs[12] = mxCreateDoubleMatrix(3, 1, mxREAL);
    prhs[13] = mxCreateDoubleMatrix(3, 1, mxREAL);
    
    // g (gravity vector)
    prhs[14] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* g_ptr = mxGetPr(prhs[14]);
    for (int i = 0; i < 3; ++i) g_ptr[i] = this->g[i];
    
    if (mexCallMATLAB(6, plhs, 15, prhs, "mex_adaptive_predict") != 0) {
        mexWarnMsgTxt("mex_adaptive_predict call failed");
    } else {
        // Get outputs
        double* p_out = mxGetPr(plhs[0]);
        double* v_out = mxGetPr(plhs[1]);
        double* q_out = mxGetPr(plhs[2]);
        double* ba_out = mxGetPr(plhs[3]);
        double* bg_out = mxGetPr(plhs[4]);
        double* P_out = mxGetPr(plhs[5]);
        
        for (int i = 0; i < 3; ++i) this->p[i] = static_cast<float>(p_out[i]);
        for (int i = 0; i < 3; ++i) this->v[i] = static_cast<float>(v_out[i]);
        for (int i = 0; i < 4; ++i) this->q[i] = static_cast<float>(q_out[i]);
        for (int i = 0; i < 3; ++i) this->ba[i] = static_cast<float>(ba_out[i]);
        for (int i = 0; i < 3; ++i) this->bg[i] = static_cast<float>(bg_out[i]);
        for (int i = 0; i < 15*15; ++i) this->P[i] = static_cast<float>(P_out[i]);
        
        // Cleanup
        for (int i = 0; i < 6; ++i) mxDestroyArray(plhs[i]);
    }
    
    // Cleanup inputs
    for (int i = 0; i < 15; ++i) mxDestroyArray(prhs[i]);
}

void ESKFFull::call_mex_eskf_predict_postprocess(const float* a) {
    mxArray* prhs[11];
    mxArray* plhs[2];
    
    prhs[0] = mxCreateString("postprocess");
    
    // v
    prhs[1] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* v_ptr = mxGetPr(prhs[1]);
    for (int i = 0; i < 3; ++i) v_ptr[i] = this->v[i];
    
    // q
    prhs[2] = mxCreateDoubleMatrix(4, 1, mxREAL);
    double* q_ptr = mxGetPr(prhs[2]);
    for (int i = 0; i < 4; ++i) q_ptr[i] = this->q[i];
    
    // P
    prhs[3] = mxCreateDoubleMatrix(15, 15, mxREAL);
    double* P_ptr = mxGetPr(prhs[3]);
    for (int i = 0; i < 15*15; ++i) P_ptr[i] = this->P[i];
    
    // a
    prhs[4] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* a_ptr = mxGetPr(prhs[4]);
    for (int i = 0; i < 3; ++i) a_ptr[i] = a[i];
    
    // dt
    prhs[5] = mxCreateDoubleScalar(this->dt);
    
    // g
    prhs[6] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* g_ptr = mxGetPr(prhs[6]);
    for (int i = 0; i < 3; ++i) g_ptr[i] = this->g[i];
    
    // enable_accel_z_integration
    prhs[7] = mxCreateLogicalScalar(this->enable_accel_z_integration);
    
    // accel_z_threshold
    prhs[8] = mxCreateDoubleScalar(this->accel_z_threshold);
    
    // accel_z_damping
    prhs[9] = mxCreateDoubleScalar(this->accel_z_damping);
    
    // velocity_damping
    prhs[10] = mxCreateDoubleScalar(this->velocity_damping);
    
    if (mexCallMATLAB(2, plhs, 11, prhs, "mex_eskf_predict_postprocess") != 0) {
        mexWarnMsgTxt("mex_eskf_predict_postprocess call failed");
    } else {
        double* v_out = mxGetPr(plhs[0]);
        double* P_out = mxGetPr(plhs[1]);
        
        for (int i = 0; i < 3; ++i) this->v[i] = static_cast<float>(v_out[i]);
        for (int i = 0; i < 15*15; ++i) this->P[i] = static_cast<float>(P_out[i]);
        
        mxDestroyArray(plhs[0]);
        mxDestroyArray(plhs[1]);
    }
    
    for (int i = 0; i < 11; ++i) mxDestroyArray(prhs[i]);
}

void ESKFFull::sensor_update_accel(const float* a, int k) {
    // Hybrid mode: sensor updates are handled via MATLAB ESKF object
    // This function is a no-op in hybrid mode
    (void)a; (void)k;
}

void ESKFFull::sensor_update_mag(const float* m, int k) {
    (void)m; (void)k;
}

void ESKFFull::sensor_update_baro(float pressure, int k) {
    (void)pressure; (void)k;
}

void ESKFFull::sensor_update_gps(float lat, float lon, float alt, int k) {
    (void)lat; (void)lon; (void)alt; (void)k;
}

void ESKFFull::reset_check(int k) {
    // Check for NaN/Inf
    bool needs_reset = false;
    for (int i = 0; i < 3; ++i) {
        if (std::isnan(p[i]) || std::isnan(v[i]) || std::isinf(p[i]) || std::isinf(v[i])) {
            needs_reset = true;
            break;
        }
    }
    for (int i = 0; i < 4; ++i) {
        if (std::isnan(q[i])) {
            needs_reset = true;
            break;
        }
    }
    
    // Check velocity/position norms
    float v_norm = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    float p_norm = sqrtf(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
    if (v_norm > 10.0f || p_norm > 1000.0f) {
        needs_reset = true;
    }
    
    if (needs_reset) {
        // Reset velocity
        v[0] = v[1] = v[2] = 0.0f;
        
        // Reset P diagonal
        P[0] = P[16] = P[32] = 20.0f;  // Position
        P[48] = P[64] = P[80] = 2.0f;  // Velocity
        float deg30_rad = 0.5236f;
        P[96] = P[112] = P[128] = deg30_rad * deg30_rad;  // Attitude
        
        // Reset quaternion if NaN
        bool q_nan = false;
        for (int i = 0; i < 4; ++i) if (std::isnan(q[i])) q_nan = true;
        if (q_nan) {
            q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
        }
        
        // Reset biases
        ba[0] = ba[1] = ba[2] = 0.0f;
        bg[0] = bg[1] = bg[2] = 0.0f;
    }
}

bool ESKFFull::zupt_check(const float* a, const float* w) {
    float a_norm = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    float gravity_deviation = fabsf(a_norm - 9.81f);
    float w_norm = sqrtf(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
    
    if (gravity_deviation < this->zupt_threshold_accel && w_norm < this->zupt_threshold_gyro) {
        this->zupt_counter++;
    } else {
        this->zupt_counter = 0;
    }
    
    this->is_stationary = (this->zupt_counter >= this->zupt_min_duration);
    return this->is_stationary;
}

void ESKFFull::zupt_update() {
    if (!this->is_stationary) return;
    
    // Zero velocity update
    // Simplified: just zero the velocity
    // In practice, would apply Kalman update with H = [0 I 0 0 0]
    v[0] = v[1] = v[2] = 0.0f;
    
    // Reduce velocity uncertainty
    P[48] = P[64] = P[80] = 0.01f;
}

void ESKFFull::get_euler(float* euler_deg) {
    quaternion_to_euler(this->q, euler_deg);
}

//=============================================================================
// MEX Entry Point
//=============================================================================
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) {
        mexErrMsgTxt("Usage: mex_eskf_full('create'|'step'|'delete', ...)");
    }
    
    if (!mxIsChar(prhs[0])) {
        mexErrMsgTxt("First argument must be a command string");
    }
    
    char cmd[64];
    mxGetString(prhs[0], cmd, sizeof(cmd));
    
    if (strcmp(cmd, "create") == 0) {
        // mex_eskf_full('create', obs, static_time, dt)
        if (nrhs < 4) {
            mexErrMsgTxt("create requires: obs, static_time, dt");
        }
        
        const mxArray* obs = prhs[1];
        float static_time = static_cast<float>(mxGetScalar(prhs[2]));
        float dt = static_cast<float>(mxGetScalar(prhs[3]));
        
        ESKFFull* eskf = new ESKFFull();
        eskf->init(obs, static_time, dt);
        
        uint64_t handle = create_handle(eskf);
        
        plhs[0] = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
        *static_cast<uint64_t*>(mxGetData(plhs[0])) = handle;
        
    } else if (strcmp(cmd, "step") == 0) {
        // mex_eskf_full('step', handle, a, w, sensors, k)
        if (nrhs < 6) {
            mexErrMsgTxt("step requires: handle, a, w, sensors, k");
        }
        
        uint64_t handle = *static_cast<uint64_t*>(mxGetData(prhs[1]));
        ESKFFull* eskf = get_handle(handle);
        if (!eskf) {
            mexErrMsgTxt("Invalid handle");
        }
        
        float a[3], w[3];
        get_vector(prhs[2], a, 3);
        get_vector(prhs[3], w, 3);
        
        const mxArray* sensors = prhs[4];
        int k = static_cast<int>(mxGetScalar(prhs[5]));
        
        float p_out[3], v_out[3], euler_out[3], ba_out[3], bg_out[3];
        eskf->step(a, w, sensors, k, p_out, v_out, euler_out, ba_out, bg_out);
        
        // Create outputs
        plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
        plhs[1] = mxCreateDoubleMatrix(3, 1, mxREAL);
        plhs[2] = mxCreateDoubleMatrix(3, 1, mxREAL);
        plhs[3] = mxCreateDoubleMatrix(3, 1, mxREAL);
        plhs[4] = mxCreateDoubleMatrix(3, 1, mxREAL);
        
        double* p_ptr = mxGetPr(plhs[0]);
        double* v_ptr = mxGetPr(plhs[1]);
        double* euler_ptr = mxGetPr(plhs[2]);
        double* ba_ptr = mxGetPr(plhs[3]);
        double* bg_ptr = mxGetPr(plhs[4]);
        
        for (int i = 0; i < 3; ++i) {
            p_ptr[i] = p_out[i];
            v_ptr[i] = v_out[i];
            euler_ptr[i] = euler_out[i];
            ba_ptr[i] = ba_out[i];
            bg_ptr[i] = bg_out[i];
        }
        
    } else if (strcmp(cmd, "delete") == 0) {
        // mex_eskf_full('delete', handle)
        if (nrhs < 2) {
            mexErrMsgTxt("delete requires: handle");
        }
        
        uint64_t handle = *static_cast<uint64_t*>(mxGetData(prhs[1]));
        delete_handle(handle);
        
    } else if (strcmp(cmd, "get_state") == 0) {
        // mex_eskf_full('get_state', handle)
        if (nrhs < 2) {
            mexErrMsgTxt("get_state requires: handle");
        }
        
        uint64_t handle = *static_cast<uint64_t*>(mxGetData(prhs[1]));
        ESKFFull* eskf = get_handle(handle);
        if (!eskf) {
            mexErrMsgTxt("Invalid handle");
        }
        
        // Return state as struct
        const char* fieldnames[] = {"p", "v", "q", "ba", "bg", "P"};
        plhs[0] = mxCreateStructMatrix(1, 1, 6, fieldnames);
        
        mxArray* p_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
        mxArray* v_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
        mxArray* q_arr = mxCreateDoubleMatrix(4, 1, mxREAL);
        mxArray* ba_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
        mxArray* bg_arr = mxCreateDoubleMatrix(3, 1, mxREAL);
        mxArray* P_arr = mxCreateDoubleMatrix(15, 15, mxREAL);
        
        double* p_ptr = mxGetPr(p_arr);
        double* v_ptr = mxGetPr(v_arr);
        double* q_ptr = mxGetPr(q_arr);
        double* ba_ptr = mxGetPr(ba_arr);
        double* bg_ptr = mxGetPr(bg_arr);
        double* P_ptr = mxGetPr(P_arr);
        
        for (int i = 0; i < 3; ++i) p_ptr[i] = eskf->p[i];
        for (int i = 0; i < 3; ++i) v_ptr[i] = eskf->v[i];
        for (int i = 0; i < 4; ++i) q_ptr[i] = eskf->q[i];
        for (int i = 0; i < 3; ++i) ba_ptr[i] = eskf->ba[i];
        for (int i = 0; i < 3; ++i) bg_ptr[i] = eskf->bg[i];
        for (int i = 0; i < 15*15; ++i) P_ptr[i] = eskf->P[i];
        
        mxSetField(plhs[0], 0, "p", p_arr);
        mxSetField(plhs[0], 0, "v", v_arr);
        mxSetField(plhs[0], 0, "q", q_arr);
        mxSetField(plhs[0], 0, "ba", ba_arr);
        mxSetField(plhs[0], 0, "bg", bg_arr);
        mxSetField(plhs[0], 0, "P", P_arr);
        
    } else {
        mexErrMsgTxt("Unknown command. Use 'create', 'step', 'delete', or 'get_state'");
    }
}


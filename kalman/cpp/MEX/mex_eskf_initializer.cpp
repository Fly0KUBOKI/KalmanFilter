#include "Impl/mex_hybrid_filter_initializer.hpp"
#include "../Lib/ESKF/inc/eskf_initializer.hpp"
#include <mex.h>
#include <vector>
#include <cmath>

namespace eskf {

// Helper functions for MATLAB struct access
static const mxArray* get_field(const mxArray* s, const char* name) {
    if (!mxIsStruct(s)) return nullptr;
    return mxGetField(s, 0, name);
}

static const mxArray* get_field_any(const mxArray* s, const char* name1, const char* name2) {
    const mxArray* f = get_field(s, name1);
    if (f) return f;
    return get_field(s, name2);
}

// Get value from single (float) array
static double get_value_at(const mxArray* arr, int idx, const char* field_name = nullptr) {
    if (!arr) return 0.0;
    
    // single型のみを受け取る
    if (mxGetClassID(arr) != mxSINGLE_CLASS) {
        const char* name = field_name ? field_name : "field";
        mexErrMsgIdAndTxt("eskf_initializer:type_error", 
            "Expected single (float) array for field '%s', but got %s.", 
            name, mxGetClassName(arr));
        return 0.0;
    }
    
    const float* pf = (const float*)mxGetData(arr);
    return pf[idx];
}

// Get value from double array (for GPS data)
static double get_gps_value_at(const mxArray* arr, int idx, const char* field_name) {
    if (!arr) return 0.0;
    
    // GPSデータはdouble型のみを受け取る
    if (mxGetClassID(arr) != mxDOUBLE_CLASS) {
        mexErrMsgIdAndTxt("eskf_initializer:type_error", 
            "Expected double array for GPS field '%s', but got %s.", 
            field_name, mxGetClassName(arr));
        return 0.0;
    }
    
    const double* pr = mxGetPr(arr);
    return pr[idx];
}

static int get_length(const mxArray* arr) {
    if (!arr) return 0;
    return static_cast<int>(mxGetNumberOfElements(arr));
}

FilterState* initialize_eskf_from_matlab(const mxArray* obs, double static_time, double dt) {
    // Calculate number of static samples
    int N_static = static_cast<int>(floor(static_time / dt));
    
    // Get field arrays
    const mxArray* ax_arr = get_field_any(obs, "ax", "accel_x");
    const mxArray* ay_arr = get_field_any(obs, "ay", "accel_y");
    const mxArray* az_arr = get_field_any(obs, "az", "accel_z");
    
    int n_samples = ax_arr ? get_length(ax_arr) : 0;
    if (N_static > n_samples) N_static = n_samples;
    
    // Prepare initialization data structure
    ESKFInitializationData init_data;
    init_data.n_samples = n_samples;
    init_data.n_static = N_static;
    init_data.static_time = static_time;
    init_data.dt = dt;
    
    // Convert MATLAB arrays to C++ arrays
    // Use temporary vectors to store converted data
    std::vector<double> accel_x_vec, accel_y_vec, accel_z_vec;
    std::vector<double> gyro_x_vec, gyro_y_vec, gyro_z_vec;
    std::vector<double> mag_x_vec, mag_y_vec, mag_z_vec;
    std::vector<double> pressure_vec;
    std::vector<double> gps_lat_vec, gps_lon_vec, gps_alt_vec;
    
    // Convert acceleration data
    if (ax_arr && ay_arr && az_arr && N_static > 10) {
        accel_x_vec.resize(N_static);
        accel_y_vec.resize(N_static);
        accel_z_vec.resize(N_static);
        for (int i = 0; i < N_static; ++i) {
            accel_x_vec[i] = get_value_at(ax_arr, i, "ax");
            accel_y_vec[i] = get_value_at(ay_arr, i, "ay");
            accel_z_vec[i] = get_value_at(az_arr, i, "az");
        }
        init_data.accel_x = accel_x_vec.data();
        init_data.accel_y = accel_y_vec.data();
        init_data.accel_z = accel_z_vec.data();
    } else {
        init_data.accel_x = nullptr;
        init_data.accel_y = nullptr;
        init_data.accel_z = nullptr;
    }
    
    // Convert gyro data
    const mxArray* wx_arr = get_field_any(obs, "wx", "gyro_x");
    const mxArray* wy_arr = get_field_any(obs, "wy", "gyro_y");
    const mxArray* wz_arr = get_field_any(obs, "wz", "gyro_z");
    if (wx_arr && wy_arr && wz_arr && N_static > 10) {
        gyro_x_vec.resize(N_static);
        gyro_y_vec.resize(N_static);
        gyro_z_vec.resize(N_static);
        for (int i = 0; i < N_static; ++i) {
            gyro_x_vec[i] = get_value_at(wx_arr, i, "wx");
            gyro_y_vec[i] = get_value_at(wy_arr, i, "wy");
            gyro_z_vec[i] = get_value_at(wz_arr, i, "wz");
        }
        init_data.gyro_x = gyro_x_vec.data();
        init_data.gyro_y = gyro_y_vec.data();
        init_data.gyro_z = gyro_z_vec.data();
    } else {
        init_data.gyro_x = nullptr;
        init_data.gyro_y = nullptr;
        init_data.gyro_z = nullptr;
    }
    
    // Convert magnetometer data
    const mxArray* mx_arr = get_field_any(obs, "mx", "mag_x");
    const mxArray* my_arr = get_field_any(obs, "my", "mag_y");
    const mxArray* mz_arr = get_field_any(obs, "mz", "mag_z");
    if (mx_arr && my_arr && mz_arr && N_static > 10) {
        mag_x_vec.resize(N_static);
        mag_y_vec.resize(N_static);
        mag_z_vec.resize(N_static);
        for (int i = 0; i < N_static; ++i) {
            mag_x_vec[i] = get_value_at(mx_arr, i, "mx");
            mag_y_vec[i] = get_value_at(my_arr, i, "my");
            mag_z_vec[i] = get_value_at(mz_arr, i, "mz");
        }
        init_data.mag_x = mag_x_vec.data();
        init_data.mag_y = mag_y_vec.data();
        init_data.mag_z = mag_z_vec.data();
    } else {
        init_data.mag_x = nullptr;
        init_data.mag_y = nullptr;
        init_data.mag_z = nullptr;
    }
    
    // Convert barometric pressure data
    const mxArray* pressure_arr = get_field_any(obs, "pressure", "baro");
    if (pressure_arr && N_static > 10) {
        pressure_vec.resize(N_static);
        for (int i = 0; i < N_static; ++i) {
            pressure_vec[i] = get_value_at(pressure_arr, i, "pressure");
        }
        init_data.pressure = pressure_vec.data();
    } else {
        init_data.pressure = nullptr;
    }
    
    // Convert GPS data
    const mxArray* lat_arr = get_field_any(obs, "lat", "gps_lat");
    const mxArray* lon_arr = get_field_any(obs, "lon", "gps_lon");
    const mxArray* alt_arr = get_field_any(obs, "alt", "gps_alt");
    if (lat_arr && lon_arr && alt_arr && N_static > 10) {
        gps_lat_vec.resize(N_static);
        gps_lon_vec.resize(N_static);
        gps_alt_vec.resize(N_static);
        for (int i = 0; i < N_static; ++i) {
            gps_lat_vec[i] = get_gps_value_at(lat_arr, i, "lat");
            gps_lon_vec[i] = get_gps_value_at(lon_arr, i, "lon");
            gps_alt_vec[i] = get_gps_value_at(alt_arr, i, "alt");
        }
        init_data.gps_lat = gps_lat_vec.data();
        init_data.gps_lon = gps_lon_vec.data();
        init_data.gps_alt = gps_alt_vec.data();
    } else {
        init_data.gps_lat = nullptr;
        init_data.gps_lon = nullptr;
        init_data.gps_alt = nullptr;
    }
    
    // Call pure C++ initialization function
    return initialize_eskf_state(init_data);
}

} // namespace eskf

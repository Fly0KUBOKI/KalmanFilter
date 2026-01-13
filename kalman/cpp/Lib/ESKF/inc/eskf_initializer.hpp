#pragma once
#ifndef LIB_ESKF_INC_ESKF_INITIALIZER_HPP
#define LIB_ESKF_INC_ESKF_INITIALIZER_HPP


#include "eskf_state.hpp"

namespace eskf {

struct ESKFInitializationData {
    const double* accel_x;   
    const double* accel_y;   
    const double* accel_z;   
    const double* gyro_x;    
    const double* gyro_y;    
    const double* gyro_z;    
    const double* mag_x;     
    const double* mag_y;     
    const double* mag_z;     
    const double* pressure;  
    const double* gps_lat;   
    const double* gps_lon;   
    const double* gps_alt;   
    
    int n_samples;           
    int n_static;            
    
    double static_time;      
    double dt;               
};

FilterState* initialize_eskf_state(const ESKFInitializationData& data);

} // namespace eskf

#endif // ESKF_ESKF_INITIALIZER_HPP

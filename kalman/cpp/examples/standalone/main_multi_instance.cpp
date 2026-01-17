/**
 * @file main_multi_instance.cpp
 * @brief Multiple filter instances example using new handle-based API
 *
 * This example demonstrates the new ハンドルベース API that allows
 * running multiple independent filter instances simultaneously.
 * 
 * Key differences from main_simple.cpp:
 * - Uses filter_create/filter_destroy instead of global filter_init/reset
 * - Can create and run multiple filter instances in parallel
 * - Parameters can be customized per instance
 * - GPS origin can be set independently
 * 
 * ビルド方法:
 *   cd kalman/cpp/examples/standalone
 *   g++ -std=c++11 -I../../Lib \
 *       ../../Lib/Common/src/standalone.cpp \
 *       ../../Lib/ESKF/src/*.cpp \
 *       ../../Lib/MEUKF/src/*.cpp \
 *       ../../Lib/Matrix/src/*.cpp \
 *       main_multi_instance.cpp -o kalman_multi
 * 
 * 実行:
 *   ./kalman_multi
 */

#include "../../Lib/examples/standalone/inc/standalone.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace kalman;

int main() {
    std::cout << "=== Kalman Filter Multi-Instance Example ===\n\n";
    
    // ========================================
    // Create TWO independent filter instances
    // ========================================
    std::cout << "1. Creating two independent ESKF filter instances...\n";
    
    FilterHandle filter1 = filter_create(FILTER_ESKF);
    FilterHandle filter2 = filter_create(FILTER_ESKF);
    
    if (!filter1 || !filter2) {
        std::cerr << "ERROR: Failed to create filter instances!\n";
        return 1;
    }
    std::cout << "   Created filter1 and filter2.\n\n";
    
    // ========================================
    // Initialize both instances with different parameters
    // ========================================
    std::cout << "2. Initializing filters with different configurations...\n";
    
    // Initialize both with default parameters
    if (filter_init(filter1, nullptr, 0, 0.0f) != 0) {
        std::cerr << "ERROR: Filter1 initialization failed!\n";
        filter_destroy(filter1);
        filter_destroy(filter2);
        return 1;
    }
    
    if (filter_init(filter2, nullptr, 0, 0.0f) != 0) {
        std::cerr << "ERROR: Filter2 initialization failed!\n";
        filter_destroy(filter1);
        filter_destroy(filter2);
        return 1;
    }
    std::cout << "   Both filters initialized.\n\n";
    
    // ========================================
    // Set different GPS origins for each instance
    // ========================================
    std::cout << "3. Setting different GPS origins...\n";
    
    // Filter 1: Tokyo Tower location
    if (filter_set_gps_origin(filter1, 35.6586, 139.7454, 40.0) != 0) {
        std::cerr << "ERROR: Failed to set GPS origin for filter1!\n";
    }
    std::cout << "   Filter1 GPS origin: Tokyo (35.6586°N, 139.7454°E, 40m)\n";
    
    // Filter 2: Statue of Liberty location (simulated)
    if (filter_set_gps_origin(filter2, 40.6892, -74.0445, 93.0) != 0) {
        std::cerr << "ERROR: Failed to set GPS origin for filter2!\n";
    }
    std::cout << "   Filter2 GPS origin: NYC (40.6892°N, -74.0445°E, 93m)\n\n";
    
    // ========================================
    // Run both filters in parallel with different sensor data
    // ========================================
    std::cout << "4. Running filters with different sensor data...\n";
    
    const int num_steps = 5;
    
    for (int i = 0; i < num_steps; ++i) {
        // Sensor data for Filter 1: moving eastward
        SensorData obs1;
        obs1.accel[0] = 0.5f * std::sin(i * 0.5f);  // eastward acceleration
        obs1.accel[1] = 0.0f;
        obs1.accel[2] = -9.81f;
        obs1.gyro[0] = obs1.gyro[1] = obs1.gyro[2] = 0.0f;
        obs1.mag[0] = 30.0f; obs1.mag[1] = 0.0f; obs1.mag[2] = -50.0f;
        obs1.baro_alt = 40.0f;
        obs1.gps_lat = 35.6586; obs1.gps_lon = 139.7454 + 0.001f * i; obs1.gps_alt = 40.0;
        
        // Sensor data for Filter 2: moving northward
        SensorData obs2;
        obs2.accel[0] = 0.0f;
        obs2.accel[1] = 0.5f * std::cos(i * 0.5f);  // northward acceleration
        obs2.accel[2] = -9.81f;
        obs2.gyro[0] = obs2.gyro[1] = obs2.gyro[2] = 0.0f;
        obs2.mag[0] = 30.0f; obs2.mag[1] = 0.0f; obs2.mag[2] = -50.0f;
        obs2.baro_alt = 93.0f;
        obs2.gps_lat = 40.6892 + 0.001f * i; obs2.gps_lon = -74.0445; obs2.gps_alt = 93.0;
        
        // Update both filters
        if (filter_update(filter1, obs1) != 0) {
            std::cerr << "ERROR: Filter1 update failed at step " << i << "!\n";
        }
        if (filter_update(filter2, obs2) != 0) {
            std::cerr << "ERROR: Filter2 update failed at step " << i << "!\n";
        }
        
        std::cout << "   Step " << i << " completed for both filters.\n";
    }
    std::cout << "   All updates completed.\n\n";
    
    // ========================================
    // Get final states from both filters
    // ========================================
    std::cout << "5. Retrieving final estimates from both filters...\n";
    
    State state1, state2;
    if (filter_get_state(filter1, state1) != 0) {
        std::cerr << "ERROR: Failed to get state from filter1!\n";
    }
    if (filter_get_state(filter2, state2) != 0) {
        std::cerr << "ERROR: Failed to get state from filter2!\n";
    }
    
    std::cout << "\n=== Filter 1 (Tokyo) ===\n\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Position [m] (ENU frame):\n";
    std::cout << "  East:  " << std::setw(8) << state1.p[0] << "\n";
    std::cout << "  North: " << std::setw(8) << state1.p[1] << "\n";
    std::cout << "  Up:    " << std::setw(8) << state1.p[2] << "\n\n";
    
    std::cout << "Velocity [m/s]:\n";
    std::cout << "  East:  " << std::setw(8) << state1.v[0] << "\n";
    std::cout << "  North: " << std::setw(8) << state1.v[1] << "\n";
    std::cout << "  Up:    " << std::setw(8) << state1.v[2] << "\n\n";
    
    std::cout << "Euler [deg]: [" << state1.euler[0] << ", " 
              << state1.euler[1] << ", " << state1.euler[2] << "]\n\n";
    
    std::cout << "=== Filter 2 (NYC) ===\n\n";
    std::cout << "Position [m] (ENU frame):\n";
    std::cout << "  East:  " << std::setw(8) << state2.p[0] << "\n";
    std::cout << "  North: " << std::setw(8) << state2.p[1] << "\n";
    std::cout << "  Up:    " << std::setw(8) << state2.p[2] << "\n\n";
    
    std::cout << "Velocity [m/s]:\n";
    std::cout << "  East:  " << std::setw(8) << state2.v[0] << "\n";
    std::cout << "  North: " << std::setw(8) << state2.v[1] << "\n";
    std::cout << "  Up:    " << std::setw(8) << state2.v[2] << "\n\n";
    
    std::cout << "Euler [deg]: [" << state2.euler[0] << ", " 
              << state2.euler[1] << ", " << state2.euler[2] << "]\n\n";
    
    // ========================================
    // Cleanup: destroy both instances
    // ========================================
    std::cout << "6. Cleaning up resources...\n";
    filter_destroy(filter1);
    filter_destroy(filter2);
    std::cout << "   Both filter instances destroyed.\n\n";
    
    std::cout << "=== Multi-Instance Example Completed Successfully ===\n";
    
    return 0;
}

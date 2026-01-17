/**
 * @file main_simple.cpp
 * @brief 最小限のKalman FilterライブラリUsageサンプル
 * 
 * このファイルはスタンドアロンC++アプリケーションとして
 * Kalman Filterライブラリを使用する最もシンプルな例です。
 * 
 * ビルド方法:
 *   cd kalman/cpp/examples/standalone
 *   g++ -std=c++11 -I../../Lib \
 *       ../../Lib/Common/src/*.cpp \
 *       ../../Lib/ESKF/src/*.cpp \
 *       ../../Lib/MEUKF/src/*.cpp \
 *       ../../Lib/Matrix/src/*.cpp \
 *       main_simple.cpp -o kalman_simple
 * 
 * 実行:
 *   ./kalman_simple
 */

#include "../../Lib/examples/standalone/inc/standalone.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace kalman;

int main() {
    std::cout << "=== Kalman Filter Standalone Example ===\n\n";
    
    // ========================================
    // 1. フィルタタイプを設定
    // ========================================
    std::cout << "1. Creating and initializing filter instance...\n";
    FilterHandle h = filter_create(FILTER_ESKF);
    if (!h) {
        std::cerr << "ERROR: Failed to create filter instance!\n";
        return 1;
    }
    if (filter_init(h, nullptr, 0, 0.0f) != 0) {
        std::cerr << "ERROR: Filter initialization failed!\n";
        filter_destroy(h);
        return 1;
    }
    std::cout << "   Filter initialized successfully.\n\n";
    
    // ========================================
    // 3. センサーデータを準備
    // ========================================
    std::cout << "3. Preparing sensor data...\n";
    SensorData obs;
    
    // IMU（静止状態）
    obs.accel[0] = 0.0f;   // x軸加速度 [m/s²]
    obs.accel[1] = 0.0f;   // y軸加速度
    obs.accel[2] = -9.81f; // z軸加速度（重力）
    
    obs.gyro[0] = 0.0f;    // x軸角速度 [rad/s]
    obs.gyro[1] = 0.0f;    // y軸角速度
    obs.gyro[2] = 0.0f;    // z軸角速度
    
    // 磁気計（北向き、下向きの地磁気を仮定）
    obs.mag[0] = 30.0f;    // x軸磁気 [µT]
    obs.mag[1] = 0.0f;     // y軸磁気
    obs.mag[2] = -50.0f;   // z軸磁気
    
    // 気圧計
    obs.baro_alt = 100.0f; // 高度 [m]
    
    // GPS（東京タワー付近の座標）
    obs.gps_lat = 35.6586; // 緯度 [deg]
    obs.gps_lon = 139.7454; // 経度 [deg]
    obs.gps_alt = 40.0;     // 高度 [m]
    
    std::cout << "   Sensor data prepared:\n";
    std::cout << "   - Accel: [" << obs.accel[0] << ", " << obs.accel[1] 
              << ", " << obs.accel[2] << "] m/s²\n";
    std::cout << "   - Gyro:  [" << obs.gyro[0] << ", " << obs.gyro[1] 
              << ", " << obs.gyro[2] << "] rad/s\n";
    std::cout << "   - Mag:   [" << obs.mag[0] << ", " << obs.mag[1] 
              << ", " << obs.mag[2] << "] µT\n";
    std::cout << "   - GPS:   [" << obs.gps_lat << ", " << obs.gps_lon 
              << ", " << obs.gps_alt << "] deg/m\n\n";
    
    // ========================================
    // 4. フィルタ更新（複数ステップ）
    // ========================================
    std::cout << "4. Running filter updates...\n";
    const int num_steps = 10;
    
    for (int i = 0; i < num_steps; ++i) {
        // ノイズを追加（簡易的なシミュレーション）
        obs.accel[0] = 0.01f * std::sin(i * 0.1f);
        obs.accel[1] = 0.01f * std::cos(i * 0.1f);
        obs.gyro[2] = 0.001f * std::sin(i * 0.2f);
        
        if (filter_update(h, obs) != 0) {
            std::cerr << "ERROR: Filter update failed at step " << i << "!\n";
            filter_reset(h);
            filter_destroy(h);
            return 1;
        }
        
        if (i % 3 == 0) {
            std::cout << "   Step " << i << " completed.\n";
        }
    }
    std::cout << "   All " << num_steps << " updates completed successfully.\n\n";
    
    // ========================================
    // 5. 推定状態を取得
    // ========================================
    std::cout << "5. Retrieving estimated state...\n";
    State state;
    if (filter_get_state(h, state) != 0) {
        std::cerr << "ERROR: Failed to get state!\n";
        filter_reset(h);
        filter_destroy(h);
        return 1;
    }
    
    // ========================================
    // 6. 結果を表示
    // ========================================
    std::cout << "\n=== Estimation Results ===\n\n";
    
    std::cout << std::fixed << std::setprecision(3);
    
    std::cout << "Position [m] (ENU frame):\n";
    std::cout << "  East:  " << std::setw(8) << state.p[0] << "\n";
    std::cout << "  North: " << std::setw(8) << state.p[1] << "\n";
    std::cout << "  Up:    " << std::setw(8) << state.p[2] << "\n\n";
    
    std::cout << "Velocity [m/s] (ENU frame):\n";
    std::cout << "  East:  " << std::setw(8) << state.v[0] << "\n";
    std::cout << "  North: " << std::setw(8) << state.v[1] << "\n";
    std::cout << "  Up:    " << std::setw(8) << state.v[2] << "\n\n";
    
    std::cout << "Quaternion [w, x, y, z] (Body→ENU):\n";
    std::cout << "  w: " << std::setw(8) << state.q[0] << "\n";
    std::cout << "  x: " << std::setw(8) << state.q[1] << "\n";
    std::cout << "  y: " << std::setw(8) << state.q[2] << "\n";
    std::cout << "  z: " << std::setw(8) << state.q[3] << "\n\n";
    
    std::cout << "Euler Angles [deg]:\n";
    std::cout << "  Roll:  " << std::setw(8) << state.euler[0] << "\n";
    std::cout << "  Pitch: " << std::setw(8) << state.euler[1] << "\n";
    std::cout << "  Yaw:   " << std::setw(8) << state.euler[2] << "\n\n";
    
    std::cout << "IMU Biases:\n";
    std::cout << "  Accel Bias [m/s²]: [" << state.ba[0] << ", " 
              << state.ba[1] << ", " << state.ba[2] << "]\n";
    std::cout << "  Gyro Bias [rad/s]: [" << state.bg[0] << ", " 
              << state.bg[1] << ", " << state.bg[2] << "]\n\n";
    
    // 共分散の一部を表示（対角成分）
    std::cout << "Covariance (diagonal elements):\n";
    std::cout << "  P(0,0) = " << state.P[0] << " (position x)\n";
    std::cout << "  P(1,1) = " << state.P[16] << " (position y)\n";
    std::cout << "  P(2,2) = " << state.P[32] << " (position z)\n";
    std::cout << "  P(3,3) = " << state.P[48] << " (velocity x)\n\n";
    
    // ========================================
    // 7. クリーンアップ
    // ========================================
    std::cout << "6. Cleaning up...\n";
    filter_reset(h);
    filter_destroy(h);
    std::cout << "   Filter reset and destroyed successfully.\n\n";
    
    std::cout << "=== Example completed successfully! ===\n";
    
    return 0;
}

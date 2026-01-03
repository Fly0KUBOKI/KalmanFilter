#pragma once

#ifndef COMMON_SENSOR_SENSOR_PREPROCESSOR_HPP
#define COMMON_SENSOR_SENSOR_PREPROCESSOR_HPP

#include "../../Lib/Matrix/fixed_matrix.hpp"
#include <cmath>

namespace common {
namespace sensor {

// センサー前処理の結果
struct PreprocessResult {
    cmath_fx::Vector<3, float> output;  // 処理後のデータ (3x1)
    bool is_outlier;                     // 外れ値フラグ
    bool no_change;                      // 変更なしフラグ
};

// 加速度センサー前処理
// a_meas: 測定値 (3x1)
// prev_a: 前回値 (3x1)
// buffer_tolerance: バッファ許容誤差（デフォルト: 1e-9）
PreprocessResult preprocess_accel(
    const cmath_fx::Vector<3, float>& a_meas,
    const cmath_fx::Vector<3, float>& prev_a,
    double buffer_tolerance = 1e-9
);

// 磁気センサー前処理
// m_meas: 測定値 (3x1)
// prev_m: 前回値 (3x1)
// buffer_tolerance: バッファ許容誤差（デフォルト: 1e-9）
PreprocessResult preprocess_mag(
    const cmath_fx::Vector<3, float>& m_meas,
    const cmath_fx::Vector<3, float>& prev_m,
    double buffer_tolerance = 1e-9
);

// 気圧センサー前処理
// pressure: 気圧値 (Pa)
// 戻り値: 高度 (m)
double preprocess_baro(double pressure);

// GPS前処理
// lat, lon, alt: GPS座標
// origin: GPS原点 [lat0, lon0, alt0] (3x1)
// buffer_tolerance: バッファ許容誤差（デフォルト: 1e-9）
// 戻り値: 処理後の位置 [y_m, x_m, z_m] (m)
PreprocessResult preprocess_gps(
    double lat, double lon, double alt,
    const cmath_fx::Vector<3, float>& origin,
    double buffer_tolerance = 1e-9
);

} // namespace sensor
} // namespace common

#endif // COMMON_SENSOR_SENSOR_PREPROCESSOR_HPP


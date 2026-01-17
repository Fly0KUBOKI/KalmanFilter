#include "sensor_preprocessor.hpp"
#include "../Matrix/fixed_matrix.hpp"
#include "../Matrix/Math/statistics.hpp"
#include <cmath>

namespace common {
namespace sensor {

PreprocessResult preprocess_accel(
    const cmath_fx::Vector<3, float>& a_meas,
    const cmath_fx::Vector<3, float>& prev_a,
    double buffer_tolerance
) {
    PreprocessResult result;
    result.is_outlier = false;
    result.no_change = false;
    
    // 変更量の計算
    double delta = 0.0;
    for (int i = 0; i < 3; ++i) {
        double d = a_meas(i, 0) - prev_a(i, 0);
        delta += d * d;
    }
    delta = std::sqrt(delta);
    
    // 出力値の初期化（測定値をそのまま使用）
    for (int i = 0; i < 3; ++i) {
        result.output(i, 0) = a_meas(i, 0);
    }
    
    // 変更がない場合
    if (delta <= buffer_tolerance) {
        result.no_change = true;
        return result;
    }
    
    // 外れ値チェック
    double a_norm = std::sqrt(
        a_meas(0, 0) * a_meas(0, 0) +
        a_meas(1, 0) * a_meas(1, 0) +
        a_meas(2, 0) * a_meas(2, 0)
    );
    if (a_norm < 0.1 || std::fabs(a_norm - 9.81) > 3.0) {
        result.is_outlier = true;
    }
    
    return result;
}

PreprocessResult preprocess_mag(
    const cmath_fx::Vector<3, float>& m_meas,
    const cmath_fx::Vector<3, float>& prev_m,
    double buffer_tolerance
) {
    PreprocessResult result;
    result.is_outlier = false;
    result.no_change = false;
    
    // 変更量の計算
    double delta = 0.0;
    for (int i = 0; i < 3; ++i) {
        double d = m_meas(i, 0) - prev_m(i, 0);
        delta += d * d;
    }
    delta = std::sqrt(delta);
    
    // 出力値の初期化（測定値をそのまま使用）
    for (int i = 0; i < 3; ++i) {
        result.output(i, 0) = m_meas(i, 0);
    }
    
    // 変更がない場合
    if (delta <= buffer_tolerance) {
        result.no_change = true;
        return result;
    }
    
    return result;
}

double preprocess_baro(double pressure) {
    const double P0 = 101325.0;
    const double ALT_COEFF = 44330.0;
    
    double p_frac = pressure / P0;
    if (p_frac < 1e-9) {
        p_frac = 1e-9;
    }

    double ratio = p_frac;
    double alt = 44330.0 * (1.0 - std::pow(ratio, 1.0/5.255));
    return alt;
}

PreprocessResult preprocess_gps(
    double lat, double lon, double alt,
    const cmath_fx::Vector<3, float>& origin,
    double buffer_tolerance
) {
    PreprocessResult result;
    result.is_outlier = false;
    result.no_change = false;
    
    double lat0 = origin(0, 0);
    double lon0 = origin(1, 0);
    double alt0 = origin(2, 0);
    
    double dlat = lat - lat0;
    double dlon = lon - lon0;
    double dalt = alt - alt0;
    
    // 変更がない場合
    if (std::fabs(dlat) <= buffer_tolerance &&
        std::fabs(dlon) <= buffer_tolerance &&
        std::fabs(dalt) <= buffer_tolerance) {
        result.output(0, 0) = 0.0;
        result.output(1, 0) = 0.0;
        result.output(2, 0) = 0.0;
        result.no_change = true;
        return result;
    }
    
    // GPS座標をメートル単位に変換
    double y_m = dlat / 9.0e-6;
    double lat0rad = lat0 * common::math::PI / 180.0;
    double x_m = dlon / (9.0e-6 / std::cos(lat0rad));
    double z_m = -dalt;
    
    result.output(0, 0) = y_m;
    result.output(1, 0) = x_m;
    result.output(2, 0) = z_m;
    
    return result;
}

} // namespace sensor
} // namespace common


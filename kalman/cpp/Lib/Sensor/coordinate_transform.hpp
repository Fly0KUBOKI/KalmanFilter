#pragma once
#ifndef LIB_SENSOR_COORDINATE_TRANSFORM_HPP
#define LIB_SENSOR_COORDINATE_TRANSFORM_HPP

#include "../Matrix/fixed_matrix.hpp"
#include <cmath>

namespace sensor {
namespace coord {

using Vector3 = cmath_fx::Vector<3, float>;

/**
 * GPS座標をローカル座標系に変換
 * @param gps_pos GPS座標 [m] (すでにENUメートルに変換済み想定)
 * @param origin_pos 原点のGPS座標 [m]
 * @param local_pos 出力：ローカル座標 [m]
 */
inline void gps_to_local(const Vector3& gps_pos, const Vector3& origin_pos, Vector3& local_pos) {
    for (int i = 0; i < 3; ++i) {
        local_pos(i, 0) = gps_pos(i, 0) - origin_pos(i, 0);
    }
}

/**
 * 気圧から高度への変換（標準大気モデル）
 * @param pressure 気圧 [Pa]
 * @return 高度 [m]
 */
inline float pressure_to_altitude(float pressure) {
    const float p0 = 101325.0f; // 海面気圧 [Pa]
    if (pressure <= 0.0f) return 0.0f;
    float ratio = pressure / p0;
    float alt = 44330.0f * (1.0f - std::pow(ratio, 1.0f / 5.255f));
    return alt;
}

/**
 * 気圧から高度への変換（double精度版）
 * @param pressure 気圧 [Pa]
 * @return 高度 [m]
 */
inline double pressure_to_altitude_d(double pressure) {
    const double p0 = 101325.0;
    if (pressure <= 0.0) return 0.0;
    double ratio = pressure / p0;
    double alt = 44330.0 * (1.0 - std::pow(ratio, 1.0 / 5.255));
    return alt;
}

/**
 * 緯度経度高度(LLA)からENU座標への変換（簡易版）
 * 注：短距離のみ有効。長距離では測地系の正確な変換が必要
 * @param lat 緯度 [rad]
 * @param lon 経度 [rad]
 * @param alt 高度 [m]
 * @param lat0 原点緯度 [rad]
 * @param lon0 原点経度 [rad]
 * @param alt0 原点高度 [m]
 * @param enu 出力：ENU座標 [m] (East, North, Up)
 */
inline void lla_to_enu_simple(
    double lat, double lon, double alt,
    double lat0, double lon0, double alt0,
    Vector3& enu
) {
    // WGS84楕円体パラメータ
    const double a = 6378137.0;           // 長半径 [m]
    const double f = 1.0 / 298.257223563; // 扁平率
    const double e2 = 2.0 * f - f * f;    // 第一離心率の二乗
    
    // 原点での曲率半径
    double sin_lat0 = std::sin(lat0);
    double N0 = a / std::sqrt(1.0 - e2 * sin_lat0 * sin_lat0);
    
    // 座標差分
    double dlat = lat - lat0;
    double dlon = lon - lon0;
    double dalt = alt - alt0;
    
    // ENU変換（小角度近似）
    double cos_lat0 = std::cos(lat0);
    enu(1, 0) = static_cast<float>(N0 * dlat);                    // North
    enu(0, 0) = static_cast<float>(N0 * cos_lat0 * dlon);          // East
    enu(2, 0) = static_cast<float>(dalt);                          // Up
}

/**
 * ENUからLLA への逆変換（簡易版）
 * @param enu ENU座標 [m]
 * @param lat0 原点緯度 [rad]
 * @param lon0 原点経度 [rad]
 * @param alt0 原点高度 [m]
 * @param lat 出力：緯度 [rad]
 * @param lon 出力：経度 [rad]
 * @param alt 出力：高度 [m]
 */
inline void enu_to_lla_simple(
    const Vector3& enu,
    double lat0, double lon0, double alt0,
    double& lat, double& lon, double& alt
) {
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double e2 = 2.0 * f - f * f;
    
    double sin_lat0 = std::sin(lat0);
    double N0 = a / std::sqrt(1.0 - e2 * sin_lat0 * sin_lat0);
    double cos_lat0 = std::cos(lat0);
    
    double north = static_cast<double>(enu(1, 0));
    double east = static_cast<double>(enu(0, 0));
    double up = static_cast<double>(enu(2, 0));
    
    lat = lat0 + north / N0;
    lon = lon0 + east / (N0 * cos_lat0);
    alt = alt0 + up;
}

} // namespace coord
} // namespace sensor

#endif // LIB_SENSOR_COORDINATE_TRANSFORM_HPP

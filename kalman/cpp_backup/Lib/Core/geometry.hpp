#pragma once

#ifndef CORE_MATH_GEOMETRY_HPP
#define CORE_MATH_GEOMETRY_HPP

#include <cmath>
#include "math_utils.hpp"

namespace common {
namespace math {

// Geometry helpers use canonical implementations in math_utils.hpp

// LLA <-> ENU (simple spherical approx)
inline void lla_to_enu(float lat, float lon, float alt,
                      float lat0, float lon0, float alt0,
                      float& x_enu, float& y_enu, float& z_enu) {
    float deg_to_m_lat = 1.0f / 9.0e-6f;
    float cos_lat0 = cosf(lat0 * PI / 180.0f);
    float deg_to_m_lon = 1.0f / (9.0e-6f / cos_lat0);
    x_enu = (lon - lon0) * deg_to_m_lon;
    y_enu = (lat - lat0) * deg_to_m_lat;
    z_enu = alt - alt0;
}

inline void enu_to_lla(float x_enu, float y_enu, float z_enu,
                      float lat0, float lon0, float alt0,
                      float& lat, float& lon, float& alt) {
    float m_to_deg_lat = 9.0e-6f;
    float cos_lat0 = cosf(lat0 * PI / 180.0f);
    float m_to_deg_lon = 9.0e-6f / cos_lat0;
    lon = lon0 + x_enu * m_to_deg_lon;
    lat = lat0 + y_enu * m_to_deg_lat;
    alt = alt0 + z_enu;
}

} // namespace math
} // namespace common

#endif // CORE_MATH_GEOMETRY_HPP

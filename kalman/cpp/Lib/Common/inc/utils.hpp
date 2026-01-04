#pragma once

#include "interface.hpp"
#include "filter_mgmt.hpp"
#include <cmath>
#include "../../Quaternion/quaternion_functions.hpp"

namespace kalman {

// 対称化: P = (P + P^T)/2
inline void symmetrizeCov(State &s) {
  // Copy flat P into cmath_fx matrix, call canonical symmetrize, copy back
  cmath_fx::Matrix<15, 15, float> P;
  for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) P(i,j) = s.P[i*15 + j];
  common::filter::symmetrize_covariance(P);
  for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) s.P[i*15 + j] = P(i,j);
}

// q: [w,x,y,z] -- delegate to canonical quaternion implementation
inline void normalizeQuat(float q[4]) {
  cmath_fx::Vector<4, float> v;
  for (int i = 0; i < 4; ++i) v(i,0) = q[i];
  cquat::normalize_quat(v);
  for (int i = 0; i < 4; ++i) q[i] = v(i,0);
}

} // namespace kalman

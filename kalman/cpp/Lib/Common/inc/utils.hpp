#pragma once

#include "interface.hpp"
#include <cmath>

namespace kalman {

// 対称化: P = (P + P^T)/2
inline void symmetrizeCov(State &s) {
  for (int r = 0; r < 15; ++r) {
    for (int c = r+1; c < 15; ++c) {
      float a = s.P[r*15 + c];
      float b = s.P[c*15 + r];
      float m = 0.5f*(a + b);
      s.P[r*15 + c] = m;
      s.P[c*15 + r] = m;
    }
  }
}

// q: [w,x,y,z]
inline void normalizeQuat(float q[4]) {
  float n = 0.0f;
  for (int i=0;i<4;i++) n += q[i]*q[i];
  if (n <= 0.0f) { q[0]=1.0f; q[1]=q[2]=q[3]=0.0f; return; }
  float inv = 1.0f / std::sqrt(n);
  for (int i=0;i<4;i++) q[i] *= inv;
}

} // namespace kalman

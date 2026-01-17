#include "../inc/interface.hpp"
#include "../../Matrix/fixed_matrix.hpp"
#include <cmath>
#include <cstring>

namespace kalman {

// 簡易スタブ: State 初期化とユーティリティのデモ
void initStateZero(State &s) {
  std::memset(&s, 0, sizeof(s));
  s.q[0] = 1.0f;
}

// P を対称化して小さい負数をクリップ
void finalizeCov(State &s) {
  // Inline symmetrize: copy into cmath_fx::Matrix, call canonical symmetrize, copy back
  cmath_fx::Matrix<15,15,float> Pmat;
  for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) Pmat(i,j) = s.P[i*15 + j];
  cmath_fx::utils::symmetrize<15,float>(Pmat);
  for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) s.P[i*15 + j] = Pmat(i,j);
  for (int i=0;i<15*15;i++) {
    if (s.P[i] > -1e-12f && s.P[i] < 0.0f) s.P[i] = 0.0f;
  }
}

} // namespace kalman

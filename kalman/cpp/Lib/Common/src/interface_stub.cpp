#include "../inc/interface.hpp"
#include "../inc/utils.hpp"
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
  symmetrizeCov(s);
  for (int i=0;i<15*15;i++) {
    if (s.P[i] > -1e-12f && s.P[i] < 0.0f) s.P[i] = 0.0f;
  }
}

} // namespace kalman

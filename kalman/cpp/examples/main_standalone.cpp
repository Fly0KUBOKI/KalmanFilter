#include <cstdio>
#include <cstring>
#include "../Inc/kalman_all.hpp"

using namespace kalman;

int main() {
  if (kalman::filter_init() != 0) {
    std::printf("filter_init failed\n");
    return 1;
  }

  SensorData obs;
  std::memset(&obs, 0, sizeof(obs));

  for (int k = 0; k < 10; ++k) {
    if (kalman::filter_update(obs) != 0) {
      std::printf("filter_update failed at step %d\n", k);
      break;
    }

    State s;
    if (kalman::filter_getState(s) == 0) {
      std::printf("step %d: p=%.3f %.3f %.3f q=%.3f %.3f %.3f %.3f\n",
                  k, s.p[0], s.p[1], s.p[2], s.q[0], s.q[1], s.q[2], s.q[3]);
    }
  }

  kalman::filter_reset();
  return 0;
}

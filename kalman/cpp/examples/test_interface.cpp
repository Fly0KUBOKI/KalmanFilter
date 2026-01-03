#include <cstdio>
#include "../Inc/kalman_all.hpp"

using namespace kalman;

class DummyFilter : public Filter {
public:
  DummyFilter() {}
  ~DummyFilter() override {}
  uint8_t init(const SensorData& obs, float static_time) override { (void)obs; (void)static_time; return 0; }
  uint8_t update(const SensorData& obs) override { (void)obs; return 0; }
  uint8_t getState(State& out) override {
    out.p[0]=1.0f; out.p[1]=2.0f; out.p[2]=3.0f;
    out.v[0]=0.1f; out.v[1]=0.2f; out.v[2]=0.3f;
    out.q[0]=1.0f; out.q[1]=0.0f; out.q[2]=0.0f; out.q[3]=0.0f;
    for(int i=0;i<15*15;i++) out.P[i]=0.0f;
    return 0;
  }
  uint8_t setParams(const Params& p) override { (void)p; return 0; }
  uint8_t reset() override { return 0; }
};

int main() {
  DummyFilter f;
  SensorData obs = {};
  f.init(obs, 1.0f);
  f.update(obs);
  State s;
  f.getState(s);
  std::printf("p: %.2f %.2f %.2f\n", s.p[0], s.p[1], s.p[2]);
  return 0;
}

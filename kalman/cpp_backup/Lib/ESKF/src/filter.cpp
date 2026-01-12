#include "../inc/filter.hpp"
#include "../inc/eskf_core.hpp"
#include <cstring>

namespace kalman {

using namespace eskf;

ESKFFilter::ESKFFilter() {
  std::memset(&state_, 0, sizeof(state_));
  std::memset(&params_, 0, sizeof(params_));
  state_.q[0] = 1.0f; // identity quaternion
  params_.dt = 0.01f;
}

ESKFFilter::~ESKFFilter() {}

uint8_t ESKFFilter::init(const SensorData& obs, float /*static_time*/) {
  // Initialize position/velocity/quat from initial observation if available
  for (int i = 0; i < 3; ++i) state_.p[i] = obs.gps_alt == 0.0 ? 0.0f : static_cast<float>(obs.gps_lat);
  state_.q[0] = 1.0f;
  // default P
  for (int i = 0; i < 15*15; ++i) state_.P[i] = 0.0f;
  for (int i = 0; i < 15; ++i) state_.P[i*15 + i] = 1e-3f;
  return 0;
}

uint8_t ESKFFilter::update(const SensorData& obs) {
  // Convert to eskf types
  Vector3 p, v, ba, bg, a_meas, w_meas;
  Vector4 q;
  // load current state
  for (int i = 0; i < 3; ++i) {
    p(i,0) = state_.p[i];
    v(i,0) = state_.v[i];
    ba(i,0) = state_.ba[i];
    bg(i,0) = state_.bg[i];
    a_meas(i,0) = obs.accel[i];
    w_meas(i,0) = obs.gyro[i];
  }
  for (int i = 0; i < 4; ++i) q(i,0) = state_.q[i];

  Vector3 g; g(0,0)=0; g(1,0)=0; g(2,0)=-9.80665f;
  Vector3 gyro_thr; gyro_thr(0,0)=gyro_thr(1,0)=gyro_thr(2,0)=0.0f;
  Vector3 accel_thr; accel_thr(0,0)=accel_thr(1,0)=accel_thr(2,0)=0.0f;

  Scalar dt = params_.dt > 0.0f ? params_.dt : 0.01f;


  // simple nominal integration
  ESKFCore::integrate_nominal(p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_thr, accel_thr);

  // --- Predict covariance ---
  eskf::Matrix15x15 Pmat;
  for (int r = 0; r < 15; ++r)
    for (int c = 0; c < 15; ++c)
      Pmat(r, c) = (Scalar)state_.P[r * 15 + c];

  // Simple process noise matrix Q (tunable). Use small diagonal values.
  eskf::Matrix15x15 Qmat;
  for (int r = 0; r < 15; ++r)
    for (int c = 0; c < 15; ++c)
      Qmat(r, c) = (r == c) ? (Scalar)1e-5 : (Scalar)0.0;

  ESKFCore::predict_covariance(Pmat, q, a_meas, ba, w_meas, bg, Qmat, (Scalar)dt, Pmat);

  // --- Magnetometer update (if available) ---
  bool has_mag = (obs.mag[0] != 0.0f) || (obs.mag[1] != 0.0f) || (obs.mag[2] != 0.0f);
  if (has_mag) {
    eskf::Vector3 m_meas;
    m_meas(0,0) = (Scalar)obs.mag[0];
    m_meas(1,0) = (Scalar)obs.mag[1];
    m_meas(2,0) = (Scalar)obs.mag[2];

    eskf::Vector3 m_world;
    m_world(0,0) = (Scalar)params_.mag_ref[0];
    m_world(1,0) = (Scalar)params_.mag_ref[1];
    m_world(2,0) = (Scalar)params_.mag_ref[2];

    eskf::Matrix3x3 Rmag;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) Rmag(i,j) = (i==j) ? (Scalar)1e-2 : (Scalar)0.0;

    cmath_fx::Matrix<15,3,Scalar> K_out;
    eskf::Vector15 dx_out;
    ESKFCore::update_mag(q, Pmat, m_meas, m_world, Rmag, K_out, dx_out);
  }

  // --- Barometer update (if available) ---
  bool has_baro = (obs.baro_alt != 0.0f);
  if (has_baro) {
    Scalar altitude = (Scalar)obs.baro_alt;
    eskf::Vector3 gps_origin; gps_origin(0,0)=gps_origin(1,0)=gps_origin(2,0)=(Scalar)0.0;

    cmath_fx::Matrix<15,1,Scalar> Kb;
    eskf::Vector15 dxb;
    ESKFCore::update_baro(p, Pmat, altitude, gps_origin, (Scalar)1e-1, Kb, dxb);
  }

  // --- GPS update (if available) ---
  bool has_gps = (obs.gps_lat != 0.0 || obs.gps_lon != 0.0 || obs.gps_alt != 0.0);
  if (has_gps) {
    eskf::Vector3 gps_pos;
    gps_pos(0,0) = (Scalar)obs.gps_lat;
    gps_pos(1,0) = (Scalar)obs.gps_lon;
    gps_pos(2,0) = (Scalar)obs.gps_alt;

    eskf::Vector3 gps_origin; gps_origin(0,0)=gps_origin(1,0)=gps_origin(2,0)=(Scalar)0.0;

    eskf::Matrix3x3 Rgps;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) Rgps(i,j) = (i==j) ? (Scalar)1e-1 : (Scalar)0.0;

    cmath_fx::Matrix<15,3,Scalar> Kg;
    eskf::Vector15 dxg;
    ESKFCore::update_gps(p, v, Pmat, gps_pos, gps_origin, Rgps, Kg, dxg);
  }

  // copy Pmat back to state_.P and symmetrize
  for (int r = 0; r < 15; ++r)
    for (int c = 0; c < 15; ++c)
      state_.P[r*15 + c] = (float)Pmat(r,c);

  for (int r = 0; r < 15; ++r)
    for (int c = r+1; c < 15; ++c) {
      float a = state_.P[r*15 + c];
      float b = state_.P[c*15 + r];
      float sym = 0.5f * (a + b);
      state_.P[r*15 + c] = sym;
      state_.P[c*15 + r] = sym;
    }

  // write back
  for (int i = 0; i < 3; ++i) {
    state_.p[i] = p(i,0);
    state_.v[i] = v(i,0);
    state_.ba[i] = ba(i,0);
    state_.bg[i] = bg(i,0);
  }
  for (int i = 0; i < 4; ++i) state_.q[i] = q(i,0);

  return 0;
}

uint8_t ESKFFilter::getState(State& out) {
  out = state_;
  return 0;
}

uint8_t ESKFFilter::setParams(const Params& p) {
  params_ = p;
  return 0;
}

uint8_t ESKFFilter::reset() {
  std::memset(&state_, 0, sizeof(state_));
  state_.q[0] = 1.0f;
  return 0;
}

} // namespace kalman

#include "../inc/meukf_core.hpp"
#include "../../Matrix/fixed_matrix.hpp"
#include "../../Matrix/Math/statistics.hpp"
#include <cmath>
#include <cmath>
#include "../../Matrix/fixed_matrix.hpp"
#include "../../Matrix/Math/statistics.hpp"
#include "../../Matrix/fixed_matrix.hpp"
#include "../../Quaternion/quaternion_functions.hpp"
#include "../../UKF/inc/ukf_update.hpp"
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <algorithm>

#include <cstddef>
#include "../inc/meukf_helpers.hpp"

namespace meukf {

// -----------------------------------------------------------------
// GPS/Baro/ZUPT UKF-version wrappers
// -----------------------------------------------------------------

// File-scope measurement mapping helpers (avoid block-scope function definitions)
static ukf::UKFUpdate<15,3,float>::VectorM h_func_gps_fn(const ukf::UKFUpdate<15,3,float>::VectorN& xv) {
    ukf::UKFUpdate<15,3,float>::VectorM zv;
    zv(0,0) = xv(0,0);
    zv(1,0) = xv(1,0);
    zv(2,0) = xv(2,0);
    return zv;
}

static ukf::UKFUpdate<15,1,float>::VectorM h_func_baro_fn(const ukf::UKFUpdate<15,1,float>::VectorN& xv) {
    ukf::UKFUpdate<15,1,float>::VectorM zv;
    zv(0,0) = xv(2,0);
    return zv;
}

static ukf::UKFUpdate<15,3,float>::VectorM h_func_zupt_fn(const ukf::UKFUpdate<15,3,float>::VectorN& xv) {
    ukf::UKFUpdate<15,3,float>::VectorM zv;
    zv(0,0) = xv(3,0);
    zv(1,0) = xv(4,0);
    zv(2,0) = xv(5,0);
    return zv;
}

void MEUKFCore::update_gps_meukf_ukf_version(State& state, const Vector3& gps_meas, const Params& params, MEUKFOutput& output)
{
    using UKF = ukf::UKFUpdate<15, 3, float>;
    using Vector15 = UKF::VectorN;
    using Matrix15x15 = UKF::MatrixNN;
    using Matrix15x3 = UKF::MatrixNM;
    using Matrix3x3 = UKF::MatrixMM;
    using Vector3f = UKF::VectorM;

    Vector15 x; Matrix15x15 P;
    for (int i = 0; i < 3; ++i) x(i,0) = state.p[i];
    for (int i = 0; i < 3; ++i) x(3+i,0) = state.v[i];
    for (int i = 0; i < 3; ++i) x(6+i,0) = 0.0f;
    for (int i = 0; i < 3; ++i) x(9+i,0) = state.ba[i];
    for (int i = 0; i < 3; ++i) x(12+i,0) = state.bg[i];
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) P(i,j) = state.P[i*15 + j];

    Vector3f z;
    z(0,0) = gps_meas(0,0);
    z(1,0) = gps_meas(1,0);
    z(2,0) = gps_meas(2,0);

    // GPS measurement mapping: state -> measurement (uses file-scope helper)

    Matrix3x3 R = Matrix3x3::Zero();
    for (int i = 0; i < 3; ++i) R(i,i) = params.noise_gps[i];

    ukf::UKFParams up; up.alpha = params.alpha; up.beta = params.beta; up.kappa = params.kappa;

    Matrix15x3 K_out;
    Matrix3x3 S_out;
    Vector3f y_out;

    bool ok = UKF::update(x, P, z, h_func_gps_fn, R, up, &K_out, &S_out, &y_out);
    if (!ok) {
        output.status = 1;
        return;
    }

    for (int i = 0; i < 3; ++i) state.p[i] = x(i,0);
    for (int i = 0; i < 3; ++i) state.v[i] = x(3+i,0);
    Vector4 q_in; q_in(0,0)=state.q[0]; q_in(1,0)=state.q[1]; q_in(2,0)=state.q[2]; q_in(3,0)=state.q[3];
    Vector4 dq; dq(0,0) = 1.0f; dq(1,0) = 0.5f * x(6,0); dq(2,0) = 0.5f * x(7,0); dq(3,0) = 0.5f * x(8,0);
    cquat::normalize_quat(dq);
    Vector4 q_new; cquat::multiply_quat(q_in, dq, q_new);
    cquat::normalize_quat(q_new);
    state.q[0] = q_new(0,0); state.q[1] = q_new(1,0); state.q[2] = q_new(2,0); state.q[3] = q_new(3,0);
    for (int i = 0; i < 3; ++i) state.ba[i] = x(9+i,0);
    for (int i = 0; i < 3; ++i) state.bg[i] = x(12+i,0);

    for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) state.P[i*15 + j] = P(i,j);

    for (int i = 0; i < 15; ++i) for (int j = 0; j < 3; ++j) output.last_K[i*3 + j] = K_out(i,j);
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) output.last_S[i*3 + j] = S_out(i,j);
    for (int i = 0; i < 3; ++i) output.last_y[i] = y_out(i,0);
    output.last_y_len = 3;
    output.last_sensor_type = 3; // GPS
}

void MEUKFCore::update_baro_meukf_ukf_version(State& state, float alt_baro, const Params& params, MEUKFOutput& output)
{
    using UKF1 = ukf::UKFUpdate<15, 1, float>;
    using Vector15 = UKF1::VectorN;
    using Matrix15x15 = UKF1::MatrixNN;
    using Matrix15x1 = UKF1::MatrixNM;
    using Matrix1x1 = UKF1::MatrixMM;
    using Vector1f = UKF1::VectorM;

    Vector15 x; Matrix15x15 P;
    for (int i = 0; i < 3; ++i) x(i,0) = state.p[i];
    for (int i = 0; i < 3; ++i) x(3+i,0) = state.v[i];
    for (int i = 0; i < 3; ++i) x(6+i,0) = 0.0f;
    for (int i = 0; i < 3; ++i) x(9+i,0) = state.ba[i];
    for (int i = 0; i < 3; ++i) x(12+i,0) = state.bg[i];
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) P(i,j) = state.P[i*15 + j];

    Vector1f z; z(0,0) = alt_baro;

    // Baro measurement mapping: state -> altitude (uses file-scope helper)

    Matrix1x1 R = Matrix1x1::Zero(); R(0,0) = params.noise_baro;
    ukf::UKFParams up; up.alpha = params.alpha; up.beta = params.beta; up.kappa = params.kappa;

    Matrix15x1 K_out; Matrix1x1 S_out; Vector1f y_out;
    bool ok = UKF1::update(x, P, z, h_func_baro_fn, R, up, &K_out, &S_out, &y_out);
    if (!ok) { output.status = 1; return; }

    for (int i = 0; i < 3; ++i) state.p[i] = x(i,0);
    for (int i = 0; i < 3; ++i) state.v[i] = x(3+i,0);
    Vector4 q_in; q_in(0,0)=state.q[0]; q_in(1,0)=state.q[1]; q_in(2,0)=state.q[2]; q_in(3,0)=state.q[3];
    Vector4 dq; dq(0,0)=1.0f; dq(1,0)=0.5f * x(6,0); dq(2,0)=0.5f * x(7,0); dq(3,0)=0.5f * x(8,0);
    cquat::normalize_quat(dq); Vector4 q_new; cquat::multiply_quat(q_in, dq, q_new); cquat::normalize_quat(q_new);
    state.q[0]=q_new(0,0); state.q[1]=q_new(1,0); state.q[2]=q_new(2,0); state.q[3]=q_new(3,0);
    for (int i = 0; i < 3; ++i) state.ba[i] = x(9+i,0);
    for (int i = 0; i < 3; ++i) state.bg[i] = x(12+i,0);
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) state.P[i*15 + j] = P(i,j);

    for (int i = 0; i < 15; ++i) for (int j = 0; j < 3; ++j) output.last_K[i*3 + j] = 0.0f;
    for (int i = 0; i < 15; ++i) output.last_K[i*3 + 0] = K_out(i,0);
    output.last_S[0] = S_out(0,0);
    output.last_y[0] = y_out(0,0); output.last_y_len = 1; output.last_sensor_type = 4; // baro
}

void MEUKFCore::update_zupt_meukf_ukf_version(State& state, const Params& params, MEUKFOutput& output)
{
    using UKF3 = ukf::UKFUpdate<15, 3, float>;
    using Vector15 = UKF3::VectorN;
    using Matrix15x15 = UKF3::MatrixNN;
    using Matrix15x3 = UKF3::MatrixNM;
    using Matrix3x3 = UKF3::MatrixMM;
    using Vector3f = UKF3::VectorM;

    Vector15 x; Matrix15x15 P;
    for (int i = 0; i < 3; ++i) x(i,0) = state.p[i];
    for (int i = 0; i < 3; ++i) x(3+i,0) = state.v[i];
    for (int i = 0; i < 3; ++i) x(6+i,0) = 0.0f;
    for (int i = 0; i < 3; ++i) x(9+i,0) = state.ba[i];
    for (int i = 0; i < 3; ++i) x(12+i,0) = state.bg[i];
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) P(i,j) = state.P[i*15 + j];

    Vector3f z; z(0,0)=0.0f; z(1,0)=0.0f; z(2,0)=0.0f;

    // ZUPT measurement mapping: velocity components (uses file-scope helper)

    Matrix3x3 R = Matrix3x3::Zero();
    for (int i = 0; i < 3; ++i) R(i,i) = params.noise_zupt[i];
    ukf::UKFParams up; up.alpha = params.alpha; up.beta = params.beta; up.kappa = params.kappa;

    Matrix15x3 K_out; Matrix3x3 S_out; Vector3f y_out;
    bool ok = UKF3::update(x, P, z, h_func_zupt_fn, R, up, &K_out, &S_out, &y_out);
    if (!ok) { output.status = 1; return; }

    for (int i = 0; i < 3; ++i) state.p[i] = x(i,0);
    for (int i = 0; i < 3; ++i) state.v[i] = x(3+i,0);
    Vector4 q_in; q_in(0,0)=state.q[0]; q_in(1,0)=state.q[1]; q_in(2,0)=state.q[2]; q_in(3,0)=state.q[3];
    Vector4 dq; dq(0,0)=1.0f; dq(1,0)=0.5f * x(6,0); dq(2,0)=0.5f * x(7,0); dq(3,0)=0.5f * x(8,0);
    cquat::normalize_quat(dq); Vector4 q_new; cquat::multiply_quat(q_in, dq, q_new); cquat::normalize_quat(q_new);
    state.q[0]=q_new(0,0); state.q[1]=q_new(1,0); state.q[2]=q_new(2,0); state.q[3]=q_new(3,0);
    for (int i = 0; i < 3; ++i) state.ba[i] = x(9+i,0);
    for (int i = 0; i < 3; ++i) state.bg[i] = x(12+i,0);
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) state.P[i*15 + j] = P(i,j);

    for (int i = 0; i < 15*3; ++i) output.last_K[i] = 0.0f;
    for (int i = 0; i < 15; ++i) for (int j = 0; j < 3; ++j) output.last_K[i*3 + j] = K_out(i,j);
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) output.last_S[i*3 + j] = S_out(i,j);
    for (int i = 0; i < 3; ++i) output.last_y[i] = y_out(i,0);
    output.last_y_len = 3; output.last_sensor_type = 5; // zupt
}

bool MEUKFCore::compare_accel_updates(const State& init_state, const SensorData& sensor, const Params& params, MEUKFOutput& out_orig, MEUKFOutput& out_ukf) {
    out_orig.new_state = init_state;
    out_orig.status = 1; // deprecated

    MEUKFOutput tmp_ukf; tmp_ukf.new_state = init_state; tmp_ukf.status = 0;
    Vector3 a_meas; a_meas(0,0)=sensor.accel[0]; a_meas(1,0)=sensor.accel[1]; a_meas(2,0)=sensor.accel[2];
    try {
        update_accel_meukf_ukf_version(tmp_ukf.new_state, a_meas, params, tmp_ukf);
    } catch(...) { tmp_ukf.status = 1; }
    out_ukf = tmp_ukf;
    return true;
}

void MEUKFCore::update_accel_meukf_ukf_version(State& state, const Vector3& a_meas, const Params& params, MEUKFOutput& output) {
    Vector3 p, v, ba, bg;
    Vector4 q_nom;
    Matrix15x15 P_full;
    state_to_vars(state, p, v, q_nom, ba, bg, P_full);

    Matrix15x15 P_full_pre = P_full;

    Matrix3x3 P_att;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) P_att(i, j) = P_full(6 + i, 6 + j);

    ukf::UKFParams uparams;
    uparams.alpha = params.alpha;
    uparams.beta = params.beta;
    uparams.kappa = params.kappa;

    using UKF = ukf::UKFUpdate<3, 2, float>;
    cmath_fx::Vector<3,float> x_err = cmath_fx::Vector<3,float>::Zero();

    // Functor version of observation mapping (captures q_nom and params)
    struct HFunc2D {
        cmath_fx::Vector<4,float> q_nom_local;
        Params params_local;
        HFunc2D(const cmath_fx::Vector<4,float>& q, const Params& p) : q_nom_local(q), params_local(p) {}
        cmath_fx::Vector<2,float> operator()(const cmath_fx::Vector<3,float>& dtheta) const {
            float dx = dtheta(0,0); float dy = dtheta(1,0); float dz = dtheta(2,0);
            float angle = std::sqrt(dx*dx + dy*dy + dz*dz);
            cmath_fx::Vector<4,float> dq;
            if (angle < 1e-9f) {
                dq(0,0) = 1.0f; dq(1,0) = 0.0f; dq(2,0) = 0.0f; dq(3,0) = 0.0f;
            } else {
                float s = std::sin(angle * 0.5f);
                dq(0,0) = std::cos(angle * 0.5f);
                dq(1,0) = (dx/angle) * s;
                dq(2,0) = (dy/angle) * s;
                dq(3,0) = (dz/angle) * s;
            }
            cmath_fx::Vector<4,float> q_i; cquat::multiply_quat(q_nom_local, dq, q_i); cquat::normalize_quat(q_i);
            cmath_fx::Matrix<3,3,float> Rm; cquat::quat_to_rotm(q_i, Rm);
            cmath_fx::Vector<3,float> g_vec; g_vec(0,0) = params_local.g[0]; g_vec(1,0) = params_local.g[1]; g_vec(2,0) = params_local.g[2];
            cmath_fx::Vector<3,float> a_pred = (Rm.transpose() * g_vec) * -1.0f;
            cmath_fx::Vector<2,float> z2; z2(0,0) = a_pred(0,0); z2(1,0) = a_pred(1,0);
            return z2;
        }
    };
    HFunc2D h_func_2d(q_nom, params);

    float a_norm = std::sqrt(a_meas(0,0)*a_meas(0,0) + a_meas(1,0)*a_meas(1,0) + a_meas(2,0)*a_meas(2,0));
    float gravity_deviation = std::abs(a_norm - std::sqrt(params.g[0]*params.g[0] + params.g[1]*params.g[1] + params.g[2]*params.g[2]));
    float R_scale = 1.0f + (gravity_deviation / 0.7f);
    float R_floor = 0.25f;

    cmath_fx::Matrix<2,2,float> R2 = cmath_fx::Matrix<2,2,float>::Zero();
    for (int i=0;i<2;i++) {
        float R_est = params.noise_accel[i];
        R2(i,i) = std::max(R_est, R_floor) * R_scale;
    }

    cmath_fx::Matrix<2,2,float> S_out;
    cmath_fx::Matrix<3,2,float> K_out;
    cmath_fx::Vector<2,float> y_out;

    cmath_fx::Vector<2,float> z_meas2;
    z_meas2(0,0) = a_meas(0,0);
    z_meas2(1,0) = a_meas(1,0);

    cmath_fx::Vector<3,float> x_tmp = x_err;
    cmath_fx::Matrix<3,3,float> P_att_tmp;
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) P_att_tmp(i,j) = P_att(i,j);

    bool ok = UKF::update(x_tmp, P_att_tmp, z_meas2, h_func_2d, R2, uparams, &K_out, &S_out, &y_out);
    if (!ok) {
        output.status = 1;
        return;
    }

    cmath_fx::Matrix<2,2,float> S2_inv;
    if (!S_out.inverse(S2_inv)) {
        output.status = 1;
        return;
    }

    Matrix3x3 Rmat;
    cquat::quat_to_rotm(q_nom, Rmat);
    Vector3 tmp_g;
    tmp_g(0,0) = params.g[0]; tmp_g(1,0) = params.g[1]; tmp_g(2,0) = params.g[2];
    Vector3 g_body = Rmat.transpose() * tmp_g;
    Matrix3x3 g_skew;
    g_skew(0, 0) = 0; g_skew(0, 1) = -g_body(2,0); g_skew(0, 2) = g_body(1,0);
    g_skew(1, 0) = g_body(2,0); g_skew(1, 1) = 0; g_skew(1, 2) = -g_body(0,0);
    g_skew(2, 0) = -g_body(1,0); g_skew(2, 1) = g_body(0,0); g_skew(2, 2) = 0;
    Matrix3x3 H_att = g_skew * -1.0f;
    Matrix2x3 H_sub;
    for(int r=0; r<2; ++r) for(int c=0; c<3; ++c) H_sub(r, c) = H_att(r, c);

    Matrix15x3 P_cross;
    for(int r=0; r<15; ++r) for(int c=0; c<3; ++c) P_cross(r, c) = P_full(r, 6 + c);

    Matrix15x2 tmp = P_cross * H_sub.transpose();
    Matrix15x2 K_full = tmp * S2_inv;

    cmath_fx::Vector<2,float> y2 = y_out;

    float max_innovation = 0.05f;
    float innov_norm = std::sqrt(y2(0,0)*y2(0,0) + y2(1,0)*y2(1,0));
    if (innov_norm > max_innovation) {
        float scale = max_innovation / innov_norm;
        y2(0,0) *= scale; y2(1,0) *= scale;
    }
    cmath_fx::Vector<2,float> S2_inv_y = S2_inv * y2;
    float mahal_sq = y2(0,0)*S2_inv_y(0,0) + y2(1,0)*S2_inv_y(1,0);
    float mahal = std::sqrt(mahal_sq);
    if (mahal > 5.0f) { output.status = 1; return; }
    if (mahal > 2.5f) {
        float att = 2.5f / mahal; y2(0,0) *= att; y2(1,0) *= att;
    }

    cmath_fx::Matrix<3,2,float> K_small = K_out;
    cmath_fx::Vector<3,float> dx_small = K_small * y2;

    float dtheta_norm = std::sqrt(dx_small(0,0)*dx_small(0,0) + dx_small(1,0)*dx_small(1,0));
    float max_dtheta = 0.6f * 3.14159265f / 180.0f;
    if (dtheta_norm > max_dtheta) {
        float sc = max_dtheta / dtheta_norm; dx_small(0,0) *= sc; dx_small(1,0) *= sc;
    }
    dx_small(2,0) = 0.0f;

    cmath_fx::Matrix<3,3,float> P_att_upd = P_att;
    // Compute P_att_upd = P_att - K_small * S_out * K_small'
    cmath_fx::Matrix<3,2,float> KS2 = K_small * S_out;
    cmath_fx::Matrix<3,3,float> KSKt = KS2 * K_small.transpose();
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) P_att_upd(i,j) = P_att(i,j) - KSKt(i,j);
    for (int i=0;i<3;++i) for (int j=i+1;j<3;++j) {
        float avg = (P_att_upd(i,j) + P_att_upd(j,i)) * 0.5f; P_att_upd(i,j)=avg; P_att_upd(j,i)=avg;
    }
    cmath_fx::utils::ensure_positive_definite<3, float>(P_att_upd);

    Vector15 dx_full;
    for(int r=0; r<15; ++r) dx_full(r,0) = K_full(r,0) * y2(0,0) + K_full(r,1) * y2(1,0);

    Vector3 dpf; dpf(0,0)=dx_full(0,0); dpf(1,0)=dx_full(1,0); dpf(2,0)=dx_full(2,0);
    Vector3 dvf; dvf(0,0)=dx_full(3,0); dvf(1,0)=dx_full(4,0); dvf(2,0)=dx_full(5,0);
    Vector3 dbaf; dbaf(0,0)=dx_full(9,0); dbaf(1,0)=dx_full(10,0); dbaf(2,0)=dx_full(11,0);
    Vector3 dbgf; dbgf(0,0)=dx_full(12,0); dbgf(1,0)=dx_full(13,0); dbgf(2,0)=dx_full(14,0);
    p = p + dpf;
    v = v + dvf;
    ba = ba + dbaf;
    bg = bg + dbgf;

    Matrix15x15 KH_full = Matrix15x15::Zero();
    for(int i=0;i<15;++i) for(int j=0;j<15;++j) {
        float sum=0.0f; for(int k=0;k<3;++k) { if (j>=6 && j<9) sum += K_full(i,k) * H_sub(k, j-6); } KH_full(i,j)=sum;
    }
    Matrix15x15 I = Matrix15x15::Identity();
    Matrix15x15 I_KH = I - KH_full;
    Matrix15x15 P_tmp = I_KH * P_full * I_KH.transpose();
    Matrix15x15 KRKt = Matrix15x15::Zero();
    for(int i=0;i<15;++i) for(int j=0;j<15;++j) {
        float sum2=0.0f; for(int k_idx=0;k_idx<2;++k_idx) for(int l_idx=0;l_idx<2;++l_idx) sum2 += K_full(i,k_idx) * R2(k_idx,l_idx) * K_full(j,l_idx);
        KRKt(i,j)=sum2;
    }
    P_full = P_tmp + KRKt;
    P_full = (P_full + P_full.transpose()) * 0.5f;

    float dxs = dx_small(0,0), dys = dx_small(1,0), dzs = dx_small(2,0);
    float ang = std::sqrt(dxs*dxs + dys*dys + dzs*dzs);
    cmath_fx::Vector<4,float> dqv;
    if (ang < 1e-9f) { dqv(0,0)=1.0f; dqv(1,0)=dqv(2,0)=dqv(3,0)=0.0f; }
    else { float s=std::sin(ang*0.5f); dqv(0,0)=std::cos(ang*0.5f); dqv(1,0)=(dxs/ang)*s; dqv(2,0)=(dys/ang)*s; dqv(3,0)=(dzs/ang)*s; }
    cmath_fx::Vector<4,float> q_updated_v; cquat::multiply_quat(q_nom, dqv, q_updated_v); cquat::normalize_quat(q_updated_v);

    for (int i=0;i<3;++i) for (int j=0;j<3;++j) P_full(6+i,6+j) = P_att_upd(i,j);

    for (int r=0;r<15;++r) for (int c=0;c<15;++c) output.pred_P[r*15 + c] = P_full_pre(r,c);
    for (int r=0;r<15;++r) {
        output.last_K[r*3 + 0] = K_full(r,0);
        output.last_K[r*3 + 1] = K_full(r,1);
        output.last_K[r*3 + 2] = 0.0f;
    }
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) output.last_S[i*3 + j] = 0.0f;
    output.last_S[0*3 + 0] = S_out(0,0);
    output.last_S[0*3 + 1] = S_out(0,1);
    output.last_S[1*3 + 0] = S_out(1,0);
    output.last_S[1*3 + 1] = S_out(1,1);
    for (int i=0;i<3;i++) output.last_y[i]=0.0f;
    output.last_y[0]=y2(0,0); output.last_y[1]=y2(1,0); output.last_y_len=2; output.last_sensor_type=1;

    vars_to_state(p, v, q_updated_v, ba, bg, P_full, state);
}

void MEUKFCore::update_mag_meukf_ukf_version(State& state, const Vector3& m_meas, const Params& params, MEUKFOutput& output) {
    Vector3 p, v, ba, bg;
    Vector4 q_nom;
    Matrix15x15 P_full;
    state_to_vars(state, p, v, q_nom, ba, bg, P_full);

    Matrix15x15 P_full_pre = P_full;

    Matrix3x3 P_att;
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) P_att(i,j) = P_full(6+i, 6+j);

    ukf::UKFParams uparams;
    uparams.alpha = params.alpha;
    uparams.beta = params.beta;
    uparams.kappa = params.kappa;

    using UKF3 = ukf::UKFUpdate<3,3,float>;

    cmath_fx::Vector<3,float> x_err = cmath_fx::Vector<3,float>::Zero();

    // Functor version for 3D observation mapping (captures q_nom and params)
    struct HFunc3D {
        cmath_fx::Vector<4,float> q_nom_local;
        Params params_local;
        HFunc3D(const cmath_fx::Vector<4,float>& q, const Params& p) : q_nom_local(q), params_local(p) {}
        cmath_fx::Vector<3,float> operator()(const cmath_fx::Vector<3,float>& dtheta) const {
            float dx = dtheta(0,0), dy = dtheta(1,0), dz = dtheta(2,0);
            float angle = std::sqrt(dx*dx + dy*dy + dz*dz);
            cmath_fx::Vector<4,float> dq;
            if (angle < 1e-9f) { dq(0,0)=1; dq(1,0)=dq(2,0)=dq(3,0)=0; }
            else { float s = std::sin(angle*0.5f); dq(0,0)=std::cos(angle*0.5f); dq(1,0)=(dx/angle)*s; dq(2,0)=(dy/angle)*s; dq(3,0)=(dz/angle)*s; }
            cmath_fx::Vector<4,float> q_i; cquat::multiply_quat(q_nom_local, dq, q_i); cquat::normalize_quat(q_i);
            cmath_fx::Matrix<3,3,float> Rm; cquat::quat_to_rotm(q_i, Rm);
            cmath_fx::Vector<3,float> mag_ref; mag_ref(0,0) = params_local.mag_ref[0]; mag_ref(1,0) = params_local.mag_ref[1]; mag_ref(2,0) = params_local.mag_ref[2];
            cmath_fx::Vector<3,float> m_pred = Rm.transpose() * mag_ref;
            return m_pred;
        }
    };
    HFunc3D h_func_3d(q_nom, params);

    cmath_fx::Matrix<3,3,float> R3 = cmath_fx::Matrix<3,3,float>::Zero();
    for (int i=0;i<3;++i) R3(i,i) = std::max(params.noise_mag[i], 1e-6f);

    cmath_fx::Matrix<3,3,float> S_out;
    cmath_fx::Matrix<3,3,float> K_out;
    cmath_fx::Vector<3,float> y_out;

    cmath_fx::Vector<3,float> z_meas;
    z_meas(0,0) = m_meas(0,0); z_meas(1,0) = m_meas(1,0); z_meas(2,0) = m_meas(2,0);

    cmath_fx::Vector<3,float> x_tmp = x_err;
    cmath_fx::Matrix<3,3,float> P_att_tmp;
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) P_att_tmp(i,j) = P_att(i,j);

    bool ok = UKF3::update(x_tmp, P_att_tmp, z_meas, h_func_3d, R3, uparams, &K_out, &S_out, &y_out);
    if (!ok) { output.status = 1; return; }

    cmath_fx::Matrix<3,3,float> S_inv;
    if (!S_out.inverse(S_inv)) { output.status = 1; return; }

    Matrix3x3 Rmat; cquat::quat_to_rotm(q_nom, Rmat);
    Vector3 mag_ref_vec; mag_ref_vec(0,0)=params.mag_ref[0]; mag_ref_vec(1,0)=params.mag_ref[1]; mag_ref_vec(2,0)=params.mag_ref[2];
    Vector3 m_body = Rmat.transpose() * mag_ref_vec;
    Matrix3x3 m_skew;
    m_skew(0, 0) = 0; m_skew(0, 1) = -m_body(2,0); m_skew(0, 2) = m_body(1,0);
    m_skew(1, 0) = m_body(2,0); m_skew(1, 1) = 0; m_skew(1, 2) = -m_body(0,0);
    m_skew(2, 0) = -m_body(1,0); m_skew(2, 1) = m_body(0,0); m_skew(2, 2) = 0;
    Matrix3x3 H_sub = m_skew;

    Matrix15x3 P_cross; for(int r=0;r<15;++r) for(int c=0;c<3;++c) P_cross(r,c) = P_full(r,6+c);
    Matrix15x3 tmp = P_cross * H_sub.transpose();
    Matrix15x3 K_full = tmp * S_inv;

    cmath_fx::Vector<3,float> y3 = y_out;
    cmath_fx::Vector<3,float> S_inv_y = S_inv * y3;
    float mahal_sq = y3(0,0)*S_inv_y(0,0) + y3(1,0)*S_inv_y(1,0) + y3(2,0)*S_inv_y(2,0);
    float mahal = std::sqrt(mahal_sq);
    if (mahal > 5.0f) { output.status = 1; return; }
    if (mahal > 2.5f) { float att = 2.5f/mahal; y3(0,0)*=att; y3(1,0)*=att; y3(2,0)*=att; }

    cmath_fx::Matrix<3,3,float> K_small = K_out;
    cmath_fx::Vector<3,float> dx_small = K_small * y3;

    cmath_fx::Matrix<3,3,float> KS = K_small * S_out;
    cmath_fx::Matrix<3,3,float> KSKt = KS * K_small.transpose();
    cmath_fx::Matrix<3,3,float> P_att_upd = P_att;
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) P_att_upd(i,j) = P_att(i,j) - KSKt(i,j);
    for(int i=0;i<3;++i) for(int j=i+1;j<3;++j) { float avg = 0.5f*(P_att_upd(i,j)+P_att_upd(j,i)); P_att_upd(i,j)=avg; P_att_upd(j,i)=avg; }
    cmath_fx::utils::ensure_positive_definite<3, float>(P_att_upd);

    Vector15 dx_full; for(int r=0;r<15;++r) dx_full(r,0) = K_full(r,0)*y3(0,0) + K_full(r,1)*y3(1,0) + K_full(r,2)*y3(2,0);
    Vector3 dpf2; dpf2(0,0)=dx_full(0,0); dpf2(1,0)=dx_full(1,0); dpf2(2,0)=dx_full(2,0);
    Vector3 dvf2; dvf2(0,0)=dx_full(3,0); dvf2(1,0)=dx_full(4,0); dvf2(2,0)=dx_full(5,0);
    Vector3 dbaf2; dbaf2(0,0)=dx_full(9,0); dbaf2(1,0)=dx_full(10,0); dbaf2(2,0)=dx_full(11,0);
    Vector3 dbgf2; dbgf2(0,0)=dx_full(12,0); dbgf2(1,0)=dx_full(13,0); dbgf2(2,0)=dx_full(14,0);
    p = p + dpf2;
    v = v + dvf2;
    ba = ba + dbaf2;
    bg = bg + dbgf2;

    Matrix15x15 KH_full = Matrix15x15::Zero();
    for(int i=0;i<15;++i) for(int j=0;j<15;++j) { float sum=0.0f; if (j>=6 && j<9) { for(int k=0;k<3;++k) sum += K_full(i,k) * H_sub(k, j-6); } KH_full(i,j)=sum; }
    Matrix15x15 I = Matrix15x15::Identity();
    Matrix15x15 I_KH = I - KH_full;
    Matrix15x15 P_tmp = I_KH * P_full * I_KH.transpose();
    Matrix15x15 KRKt = Matrix15x15::Zero();
    for(int i=0;i<15;++i) for(int j=0;j<15;++j) { float sum2=0.0f; for(int k_idx=0;k_idx<3;++k_idx) for(int l_idx=0;l_idx<3;++l_idx) sum2 += K_full(i,k_idx) * R3(k_idx,l_idx) * K_full(j,l_idx); KRKt(i,j)=sum2; }
    P_full = P_tmp + KRKt;
    P_full = (P_full + P_full.transpose()) * 0.5f;

    float dxs=dx_small(0,0), dys=dx_small(1,0), dzs=dx_small(2,0);
    float ang = std::sqrt(dxs*dxs + dys*dys + dzs*dzs);
    Vector4 dqv;
    if (ang < 1e-9f) {
        dqv(0,0)=1.0f; dqv(1,0)=0.0f; dqv(2,0)=0.0f; dqv(3,0)=0.0f;
    } else {
        float s = std::sin(ang*0.5f);
        dqv(0,0) = std::cos(ang*0.5f);
        dqv(1,0) = (dxs/ang) * s;
        dqv(2,0) = (dys/ang) * s;
        dqv(3,0) = (dzs/ang) * s;
    }
    Vector4 q_updated; cquat::multiply_quat(q_nom, dqv, q_updated); cquat::normalize_quat(q_updated);

    for(int i=0;i<3;++i) for(int j=0;j<3;++j) P_full(6+i,6+j) = P_att_upd(i,j);

    for(int r=0;r<15;++r) for(int c=0;c<15;++c) output.pred_P[r*15 + c] = P_full_pre(r,c);
    for(int r=0;r<15;++r) { output.last_K[r*3 + 0] = K_full(r,0); output.last_K[r*3 + 1] = K_full(r,1); output.last_K[r*3 + 2] = K_full(r,2); }
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) output.last_S[i*3 + j] = 0.0f;
    output.last_S[0*3 + 0] = S_out(0,0); output.last_S[0*3 + 1] = S_out(0,1); output.last_S[0*3 + 2] = S_out(0,2);
    output.last_S[1*3 + 0] = S_out(1,0); output.last_S[1*3 + 1] = S_out(1,1); output.last_S[1*3 + 2] = S_out(1,2);
    output.last_S[2*3 + 0] = S_out(2,0); output.last_S[2*3 + 1] = S_out(2,1); output.last_S[2*3 + 2] = S_out(2,2);
    output.last_y[0]=y3(0,0); output.last_y[1]=y3(1,0); output.last_y[2]=y3(2,0); output.last_y_len=3; output.last_sensor_type=2;

    vars_to_state(p, v, q_updated, ba, bg, P_full, state);
}

} // namespace meukf

#include "kalman_filter_core.hpp"
#include "../../Common/Math/fixed_matrix.hpp"
#include "../../Common/Math/quaternion.hpp"
#include <cmath>
#include <algorithm>

namespace kf {

using cm = cmath_fx::FixedMatrix;

cm KalmanFilterCore::compute_jacobian(const cm& q, const cm& a_meas, const cm& ba, float dt) {
    // rotation matrix
    cm R; quat_to_rotm(q, R);
    // a_nom = R * (a_meas - ba)
    cm diff; diff.resize(3,1);
    for (int i=0;i<3;++i) diff(i,0) = a_meas(i,0) - ba(i,0);
    cm a_nom; a_nom.resize(3,1);
    cmath_fx::multiply(R, diff, a_nom);

    // S_a = skew(a_nom)
    cm S_a; S_a.resize(3,3);
    S_a(0,0)=0; S_a(0,1)=-a_nom(2,0); S_a(0,2)=a_nom(1,0);
    S_a(1,0)=a_nom(2,0); S_a(1,1)=0; S_a(1,2)=-a_nom(0,0);
    S_a(2,0)=-a_nom(1,0); S_a(2,1)=a_nom(0,0); S_a(2,2)=0;

    cm F; F.resize(15,15);
    // set identity
    for(int i=0;i<15;++i) for(int j=0;j<15;++j) F(i,j) = (i==j?1.0:0.0);
    // F(1:3,4:6) = I3 * dt
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) F(i,3+j) = (i==j?dt:0.0);
    // F(4:6,7:9) = -S_a * dt
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) F(3+i,6+j) = - S_a(i,j) * dt;
    // F(4:6,10:12) = -R * dt
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) F(3+i,9+j) = - R(i,j) * dt;
    // F(7:9,13:15) = -I*dt
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) F(6+i,12+j) = (i==j? -dt:0.0);
    return F;
}

cm KalmanFilterCore::predict_step(const cm& P, const cm& q, const cm& a_meas, const cm& ba, const cm& w_meas, const cm& bg, const cm& Q, float dt) {
    cm F = compute_jacobian(q, a_meas, ba, dt);
    // P_out = F * P * F' + Q * dt
    cm temp; cm temp2; cm Ft;
    cmath_fx::multiply(F, P, temp);
    cmath_fx::transpose(F, Ft);
    cmath_fx::multiply(temp, Ft, temp2);
    cm P_out; P_out.resize(temp2.rows, temp2.cols);
    for(int i=0;i<temp2.rows;++i) for(int j=0;j<temp2.cols;++j) P_out(i,j) = temp2(i,j) + Q(i,j) * dt;
    return regularize_covariance(P_out);
}

cm KalmanFilterCore::compute_kalman_gain(const cm& P_pred, const cm& H, const cm& S) {
    cm Ht; cmath_fx::transpose(H, Ht);
    cm PHt; cmath_fx::multiply(P_pred, Ht, PHt);
    cm X;
    bool ok = cmath_fx::solve_linear_system(S, PHt, X);
    if (!ok) return cm();
    return X;
}

bool KalmanFilterCore::compute_innovation_and_S(cm& y_out, cm& S_out, cm& R_out, const cm& z, const cm& h, const cm& H, const cm& P_pred, const cm& R) {
    // y = z - h
    y_out.resize(z.rows, z.cols);
    for (int i=0;i<z.rows;++i) y_out(i,0) = z(i,0) - h(i,0);
    float zero_thresh = 1e-8f;
    for (int i=0;i<y_out.rows;++i) if (std::abs(y_out(i,0)) < zero_thresh) y_out(i,0) = 0.0;
    // S = H * P_pred * H' + R
    cm temp; cm temp2; cm Ht;
    cmath_fx::transpose(H, Ht);
    cmath_fx::multiply(H, P_pred, temp);
    cmath_fx::multiply(temp, Ht, temp2);
    S_out.resize(temp2.rows, temp2.cols);
    for (int i=0;i<temp2.rows;++i) for (int j=0;j<temp2.cols;++j) S_out(i,j) = temp2(i,j) + R(i,j);
    R_out = R;
    for (int i=0;i<y_out.rows;++i) if (!std::isfinite(y_out(i,0))) { y_out(i,0)=0.0; S_out(i,i) = 1e6; }
    return true;
}

bool KalmanFilterCore::update_state_covariance(cm& x_upd, cm& P_upd, const cm& x_pred, const cm& P_pred, const cm& K, const cm& H, const cm& y, const cm& R) {
    cm dx; cmath_fx::multiply(K, y, dx);
    if (dx.rows == 15) {
        float max_delta[15] = {10.f,10.f,10.f,5.f,5.f,5.f,0.5f,0.5f,0.5f,1.f,1.f,1.f,0.1f,0.1f,0.1f};
        for (int i=0;i<15;++i) {
            if (dx(i,0) > max_delta[i]) dx(i,0) = max_delta[i];
            if (dx(i,0) < -max_delta[i]) dx(i,0) = -max_delta[i];
        }
    }
    x_upd.resize(x_pred.rows, x_pred.cols);
    for (int i=0;i<x_pred.rows;++i) x_upd(i,0) = x_pred(i,0) + dx(i,0);
    cm I; I = cm::Identity(P_pred.rows);
    cm KH; cm temp;
    cmath_fx::multiply(K, H, KH);
    cm IKH; IKH.resize(I.rows, I.cols);
    for (int i=0;i<I.rows;++i) for (int j=0;j<I.cols;++j) IKH(i,j) = I(i,j) - KH(i,j);
    cm temp1; cmath_fx::multiply(IKH, P_pred, temp1);
    cm IKHt; cmath_fx::transpose(IKH, IKHt);
    cm term1; cmath_fx::multiply(temp1, IKHt, term1);
    cm KR; cmath_fx::multiply(K, R, temp);
    cm KRt; cmath_fx::transpose(K, KRt);
    cm term2; cmath_fx::multiply(temp, KRt, term2);
    P_upd.resize(term1.rows, term1.cols);
    for (int i=0;i<term1.rows;++i) for (int j=0;j<term1.cols;++j) P_upd(i,j) = term1(i,j) + term2(i,j);
    for (int i=0;i<P_upd.rows;++i) for (int j=0;j<P_upd.cols;++j) P_upd(i,j) = 0.5*(P_upd(i,j) + P_upd(j,i));
    P_upd = regularize_covariance(P_upd);
    return true;
}

cm KalmanFilterCore::regularize_covariance(const cm& P) {
    cm P_reg = P;
    int n = P_reg.rows;
    if (n == 0) return P_reg;
    for (int i=0;i<n;++i) for (int j=0;j<n;++j) if (!std::isfinite(P_reg(i,j))) { if (i==j) P_reg(i,j)=1e-12; else P_reg(i,j)=0.0; }
    float base = 0.0f; for (int i=0;i<n;++i) base += P_reg(i,i); base = std::max(base / n, 1e-12f);
    for (int iter=0; iter<12; ++iter) {
        if (cmath_fx::is_positive_definite(P_reg)) break;
        float jitter = std::pow(10.0f, static_cast<float>(iter)) * 1e-6f * base + 1e-12f;
        for (int i=0;i<n;++i) P_reg(i,i) += jitter;
    }
    if (n==15) {
        float caps[15] = {1e6f,1e6f,1e6f,1e4f,1e4f,1e4f,100.f,100.f,100.f,1e2f,1e2f,1e2f,1e-1f,1e-1f,1e-1f};
        for (int i=0;i<15;++i) if (P_reg(i,i) > caps[i]) P_reg(i,i) = caps[i];
    }
    for (int i=0;i<n;++i) for (int j=0;j<n;++j) P_reg(i,j) = 0.5*(P_reg(i,j) + P_reg(j,i));
    return P_reg;
}

cm KalmanFilterCore::skew_symmetric(const cm& v) {
    cm S; S.resize(3,3);
    S(0,0)=0; S(0,1)=-v(2,0); S(0,2)=v(1,0);
    S(1,0)=v(2,0); S(1,1)=0; S(1,2)=-v(0,0);
    S(2,0)=-v(1,0); S(2,1)=v(0,0); S(2,2)=0;
    return S;
}

void KalmanFilterCore::quat_to_rotm(const cm& q, cm& R_out) {
    // q: 4x1 (qw,qx,qy,qz)
    float qw = q(0,0), qx = q(1,0), qy = q(2,0), qz = q(3,0);
    R_out.resize(3,3);
    R_out(0,0) = 1 - 2*(qy*qy + qz*qz);
    R_out(0,1) = 2*(qx*qy - qz*qw);
    R_out(0,2) = 2*(qx*qz + qy*qw);
    R_out(1,0) = 2*(qx*qy + qz*qw);
    R_out(1,1) = 1 - 2*(qx*qx + qz*qz);
    R_out(1,2) = 2*(qy*qz - qx*qw);
    R_out(2,0) = 2*(qx*qz - qy*qw);
    R_out(2,1) = 2*(qy*qz + qx*qw);
    R_out(2,2) = 1 - 2*(qx*qx + qy*qy);
}

} // namespace kf

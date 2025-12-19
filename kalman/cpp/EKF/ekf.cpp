#include "ekf.hpp"

namespace ekf {

void EKF::init(const Vector& x0, const Matrix& P0) {
    x_ = x0;
    P_ = P0;
}

void EKF::predict(StateTransitionFunc f, const Matrix& Q) {
    if (x_.size() == 0 || P_.rows() == 0) {
        throw std::runtime_error("EKF not initialized");
    }
    
    x_ = f(x_);
    P_ = P_ + Q;
    
    // 対称化
    P_ = (P_ + P_.transpose()) / 2.0;
}

void EKF::update(
    const Vector& z,
    ObservationFunc h,
    const Matrix& H,
    const Matrix& R
) {
    if (x_.size() == 0 || P_.rows() == 0) {
        throw std::runtime_error("EKF not initialized");
    }
    
    // イノベーション
    Vector h_pred = h(x_);
    Vector y = z - h_pred;
    
    // イノベーション共分散
    Matrix S = H * P_ * H.transpose() + R;
    
    // カルマンゲイン
    Matrix K = P_ * H.transpose() * S.inverse();
    
    // 状態更新
    x_ = x_ + K * y;
    
    // 共分散更新 (Joseph形式)
    Matrix I = Matrix::Identity(P_.rows(), P_.cols());
    Matrix IKH = I - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    
    // 対称化
    P_ = (P_ + P_.transpose()) / 2.0;
}

} // namespace ekf

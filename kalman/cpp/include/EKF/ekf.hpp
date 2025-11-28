#pragma once

#include <Eigen/Dense>
#include <functional>

namespace ekf {

using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;

// 状態遷移関数型
using StateTransitionFunc = std::function<Vector(const Vector&)>;
// 観測関数型
using ObservationFunc = std::function<Vector(const Vector&)>;

class EKF {
public:
    EKF() = default;
    
    // 初期化
    void init(const Vector& x0, const Matrix& P0);
    
    // 予測ステップ
    void predict(StateTransitionFunc f, const Matrix& Q);
    
    // 更新ステップ
    void update(
        const Vector& z,
        ObservationFunc h,
        const Matrix& H,
        const Matrix& R
    );
    
    // 状態取得
    Vector get_state() const { return x_; }
    Matrix get_covariance() const { return P_; }
    
private:
    Vector x_;  // 状態ベクトル
    Matrix P_;  // 共分散行列
};

} // namespace ekf

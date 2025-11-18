#pragma once

#include "../../Common/Math/fixed_matrix.hpp"
#include "../../Common/Math/quaternion.hpp"

namespace kf {

using cm = cmath_fx::FixedMatrix;

// カルマンフィルタコア機能（nomalloc）
class KalmanFilterCore {
public:
    // 予測ステップ: P_out = F*P*F' + Q*dt
    static cm predict_step(const cm& P, const cm& q, const cm& a_meas, const cm& ba, const cm& w_meas, const cm& bg, const cm& Q, float dt);

    // カルマンゲイン計算: returns K (empty matrix on failure)
    static cm compute_kalman_gain(const cm& P_pred, const cm& H, const cm& S);

    // 状態・共分散更新: returns true on success
    static bool update_state_covariance(cm& x_upd, cm& P_upd, const cm& x_pred, const cm& P_pred, const cm& K, const cm& H, const cm& y, const cm& R);

    // イノベーション計算
    static bool compute_innovation_and_S(cm& y_out, cm& S_out, cm& R_out, const cm& z, const cm& h, const cm& H, const cm& P_pred, const cm& R);

    // ヤコビアン計算
    static cm compute_jacobian(const cm& q, const cm& a_meas, const cm& ba, float dt);

    // 共分散正則化
    static cm regularize_covariance(const cm& P);

    // 歪対称行列
    static cm skew_symmetric(const cm& v);

private:
    static void quat_to_rotm(const cm& q, cm& R_out);
};

} // namespace kf

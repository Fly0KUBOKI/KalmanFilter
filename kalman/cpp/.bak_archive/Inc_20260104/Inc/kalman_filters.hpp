#pragma once

// KF/EKF/UKF統合ヘッダー

#include "../Lib/KF/inc/kalman_filter_core.hpp"
// EKF/ekf.hpp removed - Eigen依存で未使用
// UKF/ukf_sigma_points.hpp removed - Eigen依存、ukf_core.hppを使用
// UKF/ukf_update.hpp removed - Eigen依存、ukf_core.hppを使用
#include "../Lib/UKF/inc/ukf_core.hpp"

namespace kalman_filters {

// 名前空間エイリアス
namespace kf_core = kf;
// namespace ekf_ns = ekf;  // Removed - Eigen依存
namespace ukf_ns = ukf;

// 使用例:
// kalman_filters::kf_core::KalmanFilterCore::compute_kalman_gain(...)
// kalman_filters::ukf_ns::UKFCore<N, M>::update(...)

} // namespace kalman_filters

#pragma once

// KF/EKF/UKF統合ヘッダー

#include "KF/Core/kalman_filter_core.hpp"
#include "EKF/ekf.hpp"
#include "UKF/Core/ukf_sigma_points.hpp"
#include "UKF/Core/ukf_update.hpp"

namespace kalman_filters {

// 名前空間エイリアス
namespace kf_core = kf;
namespace ekf_ns = ekf;
namespace ukf_ns = ukf;

// 使用例:
// kalman_filters::kf_core::KalmanFilterCore::predict_step(...)
// kalman_filters::ekf_ns::EKF ekf;
// kalman_filters::ukf_ns::UKFUpdate::update(...)

} // namespace kalman_filters

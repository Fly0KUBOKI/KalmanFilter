#pragma once

#include "ukf_sigma_points.hpp"
#include <functional>

namespace ukf {

// 観測関数型
using ObservationFunc = std::function<Vector(const Vector&)>;

class UKFUpdate {
public:
    // UKF更新ステップ
    static void update(
        Vector& x_upd,
        Matrix& P_upd,
        Matrix& K,
        Matrix& S,
        Vector& y,
        const Vector& x,
        const Matrix& P,
        const Vector& z,
        ObservationFunc h_func,
        const Matrix& R,
        float alpha = 1e-3f,
        float beta = 2.0f,
        float kappa = 0.0f
    );
};

} // namespace ukf

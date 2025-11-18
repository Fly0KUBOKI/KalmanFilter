#pragma once

#include <Eigen/Dense>
#include <vector>

namespace ukf {

using Matrix = Eigen::MatrixXf;
using Vector = Eigen::VectorXf;

// シグマポイント生成
class SigmaPoints {
public:
    static void generate(
        std::vector<Vector>& sigma_points,
        Vector& wm,
        Vector& wc,
        const Vector& x,
        const Matrix& P,
        float alpha = 1e-3f,
        float beta = 2.0f,
        float kappa = 0.0f
    );
};

} // namespace ukf

#include "ukf_sigma_points.hpp"
#include <cmath>

namespace ukf {

void SigmaPoints::generate(
    std::vector<Vector>& sigma_points,
    Vector& wm,
    Vector& wc,
    const Vector& x,
    const Matrix& P,
    float alpha,
    float beta,
    float kappa
) {
    int n = x.size();
    float lambda = alpha * alpha * (n + kappa) - n;
    
    // 重み計算
    int n_sigma = 2 * n + 1;
    wm = Vector::Zero(n_sigma);
    wc = Vector::Zero(n_sigma);

    wm(0) = lambda / (n + lambda);
    wc(0) = wm(0) + (1.0f - alpha * alpha + beta);

    for (int i = 1; i < n_sigma; ++i) {
        wm(i) = 1.0f / (2.0f * (n + lambda));
        wc(i) = wm(i);
    }
    
    // シグマポイント生成
    sigma_points.clear();
    sigma_points.reserve(n_sigma);
    
    // Cholesky分解
    Matrix P_scaled = (n + lambda) * P;
    Eigen::LLT<Matrix> llt(P_scaled);
    
    Matrix sqrtP;
    if (llt.info() == Eigen::Success) {
        sqrtP = llt.matrixL();
    } else {
        // 正則化して再試行
        Matrix P_reg = P_scaled + 1e-9f * Matrix::Identity(n, n);
        Eigen::LLT<Matrix> llt_reg(P_reg);
        if (llt_reg.info() == Eigen::Success) {
            sqrtP = llt_reg.matrixL();
        } else {
            // 固有値分解にフォールバック
            Eigen::SelfAdjointEigenSolver<Matrix> es(P_scaled);
            Matrix D = es.eigenvalues().cwiseMax(0.0f).asDiagonal();
            sqrtP = es.eigenvectors() * D.cwiseSqrt();
        }
    }
    
    // 中心点
    sigma_points.push_back(x);
    
    // +方向
    for (int i = 0; i < n; ++i) {
        sigma_points.push_back(x + sqrtP.col(i));
    }
    
    // -方向
    for (int i = 0; i < n; ++i) {
        sigma_points.push_back(x - sqrtP.col(i));
    }
}

} // namespace ukf

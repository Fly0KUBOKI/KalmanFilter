#include "ukf_update.hpp"

namespace ukf {

void UKFUpdate::update(
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
    float alpha,
    float beta,
    float kappa
) {
    // シグマポイント生成
    std::vector<Vector> sigma_points;
    Vector wm, wc;
    SigmaPoints::generate(sigma_points, wm, wc, x, P, alpha, beta, kappa);
    
    int n_sigma = sigma_points.size();
    int m = z.size();
    int n = x.size();
    
    // 各シグマポイントを観測関数で変換
    std::vector<Vector> z_sigma(n_sigma);
    for (int i = 0; i < n_sigma; ++i) {
        z_sigma[i] = h_func(sigma_points[i]);
    }
    
    // 予測観測値の平均
    Vector z_mean = Vector::Zero(m);
    for (int i = 0; i < n_sigma; ++i) {
        z_mean += wm(i) * z_sigma[i];
    }
    
    // イノベーション共分散 S
    S = Matrix::Zero(m, m);
    for (int i = 0; i < n_sigma; ++i) {
        Vector dz = z_sigma[i] - z_mean;
        S += wc(i) * dz * dz.transpose();
    }
    S += R;
    
    // クロス共分散 Pxz
    Matrix Pxz = Matrix::Zero(n, m);
    for (int i = 0; i < n_sigma; ++i) {
        Vector dx = sigma_points[i] - x;
        Vector dz = z_sigma[i] - z_mean;
        Pxz += wc(i) * dx * dz.transpose();
    }
    
    // カルマンゲイン
    K = Pxz * S.inverse();
    
    // イノベーション
    y = z - z_mean;
    
    // 状態更新
    x_upd = x + K * y;
    
    // 共分散更新
    P_upd = P - K * S * K.transpose();
    
    // 対称化
    P_upd = (P_upd + P_upd.transpose()) / 2.0f;
}

} // namespace ukf

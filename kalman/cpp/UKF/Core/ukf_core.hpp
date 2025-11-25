#pragma once

#include "../../Common/Math/fixed_matrix.hpp"
#include <cmath>

namespace ukf {

template<int N, int M, typename T = float>
class UKFCore {
public:
    using VectorN = cmath_fx::Vector<N, T>;
    using VectorM = cmath_fx::Vector<M, T>;
    using MatrixNN = cmath_fx::Matrix<N, N, T>;
    using MatrixMM = cmath_fx::Matrix<M, M, T>;
    using MatrixNM = cmath_fx::Matrix<N, M, T>;
    using SigmaPoints = cmath_fx::Matrix<N, 2*N+1, T>;
    using Weights = cmath_fx::Vector<2*N+1, T>;
    
    // UKF観測更新
    // h_func: 観測関数 (sigma point -> predicted observation)
    template<typename HFunc>
    static void update(
        VectorN& x,           // 状態ベクトル
        MatrixNN& P,          // 共分散行列
        const VectorM& z,     // 観測値
        HFunc h_func,         // 観測関数
        const MatrixMM& R,    // 観測ノイズ共分散
        T alpha = 1e-3,
        T beta = 2.0,
        T kappa = 0.0,
        MatrixNM* K_out = nullptr,
        MatrixMM* S_out = nullptr,
        VectorM* y_out = nullptr
    ) {
        // シグマポイント生成
        SigmaPoints sig;
        Weights wm, wc;
        generate_sigma_points(x, P, alpha, beta, kappa, sig, wm, wc);
        
        // 各シグマポイントを観測モデルで変換
        constexpr int n_sig = 2 * N + 1;
        cmath_fx::Matrix<M, n_sig, T> z_pred;
        
        for (int i = 0; i < n_sig; ++i) {
            VectorN x_sig;
            for (int j = 0; j < N; ++j) x_sig(j, 0) = sig(j, i);
            
            VectorM z_i = h_func(x_sig);
            for (int j = 0; j < M; ++j) z_pred(j, i) = z_i(j, 0);
        }
        
        // 予測観測値の平均
        VectorM z_mean;
        for (int i = 0; i < M; ++i) {
            z_mean(i, 0) = 0.0;
            for (int j = 0; j < n_sig; ++j) {
                z_mean(i, 0) += z_pred(i, j) * wm(j, 0);
            }
        }
        
        // イノベーション共分散 S とクロス共分散 Pxz
        MatrixMM S;
        MatrixNM Pxz;
        for (int i = 0; i < M; ++i) {
            for (int j = 0; j < M; ++j) {
                S(i, j) = 0.0;
            }
        }
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < M; ++j) {
                Pxz(i, j) = 0.0;
            }
        }
        
        for (int k = 0; k < n_sig; ++k) {
            // dz = z_pred[:, k] - z_mean
            VectorM dz;
            for (int i = 0; i < M; ++i) {
                dz(i, 0) = z_pred(i, k) - z_mean(i, 0);
            }
            
            // S += wc[k] * dz * dz'
            T wc_k = wc(k, 0);
            for (int i = 0; i < M; ++i) {
                for (int j = 0; j < M; ++j) {
                    S(i, j) += wc_k * dz(i, 0) * dz(j, 0);
                }
            }
            
            // dx = sig[:, k] - x
            VectorN dx;
            for (int i = 0; i < N; ++i) {
                dx(i, 0) = sig(i, k) - x(i, 0);
            }
            
            // Pxz += wc[k] * dx * dz'
            for (int i = 0; i < N; ++i) {
                for (int j = 0; j < M; ++j) {
                    Pxz(i, j) += wc_k * dx(i, 0) * dz(j, 0);
                }
            }
        }
        
        // S += R
        for (int i = 0; i < M; ++i) {
            for (int j = 0; j < M; ++j) {
                S(i, j) += R(i, j);
            }
        }
        
        // カルマンゲイン K = Pxz / S = Pxz * inv(S)
        MatrixNM K;
        if (!compute_gain(Pxz, S, K)) {
            // 失敗時は更新しない
            return;
        }
        
        // イノベーション y = z - z_mean
        VectorM y = z - z_mean;
        
        // 状態更新 x = x + K * y
        VectorN Ky = K * y;
        x = x + Ky;
        
        // 共分散更新 P = P - K * S * K'
        auto KS = K * S;
        MatrixNN KSKt = KS * K.transpose();
        P = P - KSKt;
        
        // 対称化
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                P(i, j) = 0.5 * (P(i, j) + P(j, i));
            }
        }
        
        // 出力パラメータ
        if (K_out) *K_out = K;
        if (S_out) *S_out = S;
        if (y_out) *y_out = y;
    }
    
private:
    // シグマポイント生成
    static void generate_sigma_points(
        const VectorN& x,
        const MatrixNN& P,
        T alpha,
        T beta,
        T kappa,
        SigmaPoints& sig,
        Weights& wm,
        Weights& wc
    ) {
        T n = static_cast<T>(N);
        T lambda = alpha * alpha * (n + kappa) - n;
        T c = n + lambda;
        
        // 重みの計算
        wm(0, 0) = lambda / c;
        wc(0, 0) = lambda / c + (1.0 - alpha * alpha + beta);
        
        for (int i = 1; i < 2 * N + 1; ++i) {
            wm(i, 0) = 0.5 / c;
            wc(i, 0) = 0.5 / c;
        }
        
        // sqrt((n+lambda)*P) の計算（Cholesky分解）
        MatrixNN L;
        if (!cholesky(P, L)) {
            // フォールバック: 単位行列
            for (int i = 0; i < N; ++i) {
                for (int j = 0; j < N; ++j) {
                    L(i, j) = (i == j) ? 1.0 : 0.0;
                }
            }
        }
        
        T scale = std::sqrt(c);
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                L(i, j) *= scale;
            }
        }
        
        // シグマポイントの生成
        // sig[:,0] = x
        for (int i = 0; i < N; ++i) {
            sig(i, 0) = x(i, 0);
        }
        
        // sig[:,1:N] = x + L
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                sig(j, i + 1) = x(j, 0) + L(j, i);
            }
        }
        
        // sig[:,N+1:2N] = x - L
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                sig(j, N + i + 1) = x(j, 0) - L(j, i);
            }
        }
    }
    
    // Cholesky分解（下三角行列）
    static bool cholesky(const MatrixNN& A, MatrixNN& L) {
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                L(i, j) = 0.0;
            }
        }
        
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j <= i; ++j) {
                T sum = 0.0;
                
                if (j == i) {
                    for (int k = 0; k < j; ++k) {
                        sum += L(j, k) * L(j, k);
                    }
                    T val = A(j, j) - sum;
                    if (val <= 0.0) return false;
                    L(j, j) = std::sqrt(val);
                } else {
                    for (int k = 0; k < j; ++k) {
                        sum += L(i, k) * L(j, k);
                    }
                    L(i, j) = (A(i, j) - sum) / L(j, j);
                }
            }
        }
        
        return true;
    }
    
    // カルマンゲイン計算 K = Pxz * inv(S)
    static bool compute_gain(const MatrixNM& Pxz, const MatrixMM& S, MatrixNM& K) {
        // Gauss-Jordan消去法で inv(S) を計算し、K = Pxz * inv(S)
        // 簡易実装: S が小さい(M<=3)ことを想定
        
        MatrixMM S_inv;
        if (!invert_matrix(S, S_inv)) {
            return false;
        }
        
        K = Pxz * S_inv;
        return true;
    }
    
    // 行列の逆行列計算（ガウス消去法）
    static bool invert_matrix(const MatrixMM& A, MatrixMM& A_inv) {
        constexpr T eps = 1e-12;
        
        // 拡大行列 [A | I]
        T aug[M][2*M];
        for (int i = 0; i < M; ++i) {
            for (int j = 0; j < M; ++j) {
                aug[i][j] = A(i, j);
                aug[i][M + j] = (i == j) ? 1.0 : 0.0;
            }
        }
        
        // 前進消去
        for (int k = 0; k < M; ++k) {
            // ピボット選択
            int pivot = k;
            T max_val = std::abs(aug[k][k]);
            for (int i = k + 1; i < M; ++i) {
                if (std::abs(aug[i][k]) > max_val) {
                    max_val = std::abs(aug[i][k]);
                    pivot = i;
                }
            }
            
            if (max_val < eps) return false;
            
            // 行交換
            if (pivot != k) {
                for (int j = 0; j < 2 * M; ++j) {
                    T tmp = aug[k][j];
                    aug[k][j] = aug[pivot][j];
                    aug[pivot][j] = tmp;
                }
            }
            
            // 正規化
            T diag = aug[k][k];
            for (int j = 0; j < 2 * M; ++j) {
                aug[k][j] /= diag;
            }
            
            // 消去
            for (int i = 0; i < M; ++i) {
                if (i != k) {
                    T factor = aug[i][k];
                    for (int j = 0; j < 2 * M; ++j) {
                        aug[i][j] -= factor * aug[k][j];
                    }
                }
            }
        }
        
        // 逆行列を抽出
        for (int i = 0; i < M; ++i) {
            for (int j = 0; j < M; ++j) {
                A_inv(i, j) = aug[i][M + j];
            }
        }
        
        return true;
    }
};

} // namespace ukf

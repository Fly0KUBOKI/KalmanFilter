#pragma once

#include "../Common/Math/fixed_matrix.hpp"
#include "../Common/Sensor/sensor_filter.hpp"
#include <cmath>

namespace kf {

using cm = cmath_fx::FixedMatrix;

// カルマンフィルタコア
class KFCore {
private:
    cm x_;          // 状態ベクトル
    cm P_;          // 共分散行列
    cm Q_;          // プロセスノイズ
    int n_;         // 状態次元
    bool initialized_;
    bool has_result_;  // getData用フラグ
    
    // 最新の結果
    cm K_;          // カルマンゲイン
    cm S_;          // イノベーション共分散
    cm y_;          // イノベーション
    
    common::sensor::SensorFilterLib sensor_filter_;
    
public:
    KFCore() : n_(0), initialized_(false), has_result_(false) {}
    
    // 初期化
    void initialize(const cm& x0, const cm& P0, const cm& Q) {
        x_ = x0;
        P_ = P0;
        Q_ = Q;
        n_ = x0.rows;
        initialized_ = true;
        has_result_ = false;
    }
    
    // Predict ステップ
    void predict(const cm& F, const cm& u) {
        if (!initialized_) return;
        
        // x = F*x + u
        cm x_new; x_new.resize(n_, 1);
        for (int i = 0; i < n_; i++) {
            float sum = u(i, 0);
            for (int j = 0; j < n_; j++) {
                sum += F(i, j) * x_(j, 0);
            }
            x_new(i, 0) = sum;
        }
        x_ = x_new;
        
        // P = F*P*F' + Q
        cm FP; FP.resize(n_, n_);
        for (int i = 0; i < n_; i++) {
            for (int j = 0; j < n_; j++) {
                float sum = 0.0f;
                for (int k = 0; k < n_; k++) {
                    sum += F(i, k) * P_(k, j);
                }
                FP(i, j) = sum;
            }
        }
        
        cm P_new; P_new.resize(n_, n_);
        for (int i = 0; i < n_; i++) {
            for (int j = 0; j < n_; j++) {
                float sum = Q_(i, j);
                for (int k = 0; k < n_; k++) {
                    sum += FP(i, k) * F(j, k);
                }
                P_new(i, j) = sum;
            }
        }
        P_ = P_new;
        
        // 対称化
        for (int i = 0; i < n_; i++) {
            for (int j = i+1; j < n_; j++) {
                float avg = (P_(i,j) + P_(j,i)) * 0.5f;
                P_(i,j) = avg;
                P_(j,i) = avg;
            }
        }
    }
    
    // Update ステップ
    void update(const cm& z, const cm& H, const cm& R) {
        if (!initialized_) return;
        
        int m = z.rows;
        
        // h = H*x
        cm h; h.resize(m, 1);
        for (int i = 0; i < m; i++) {
            float sum = 0.0f;
            for (int j = 0; j < n_; j++) {
                sum += H(i, j) * x_(j, 0);
            }
            h(i, 0) = sum;
        }
        
        // y = z - h
        y_.resize(m, 1);
        for (int i = 0; i < m; i++) {
            y_(i, 0) = z(i, 0) - h(i, 0);
        }
        
        // S = H*P*H' + R
        cm HP; HP.resize(m, n_);
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n_; j++) {
                float sum = 0.0f;
                for (int k = 0; k < n_; k++) {
                    sum += H(i, k) * P_(k, j);
                }
                HP(i, j) = sum;
            }
        }
        
        S_.resize(m, m);
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < m; j++) {
                float sum = R(i, j);
                for (int k = 0; k < n_; k++) {
                    sum += HP(i, k) * H(j, k);
                }
                S_(i, j) = sum;
            }
        }
        
        // K = P*H' / S (簡易実装: 直接逆行列)
        // K = P*H' * inv(S)
        K_.resize(n_, m);
        cm PHt; PHt.resize(n_, m);
        for (int i = 0; i < n_; i++) {
            for (int j = 0; j < m; j++) {
                float sum = 0.0f;
                for (int k = 0; k < n_; k++) {
                    sum += P_(i, k) * H(j, k);
                }
                PHt(i, j) = sum;
            }
        }
        
        // S の逆行列（簡易版：対角のみ）
        cm S_inv; S_inv.resize(m, m);
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < m; j++) {
                if (i == j) {
                    S_inv(i, j) = (S_(i, i) > 1e-12f) ? (1.0f / S_(i, i)) : 0.0f;
                } else {
                    S_inv(i, j) = 0.0f;
                }
            }
        }
        
        for (int i = 0; i < n_; i++) {
            for (int j = 0; j < m; j++) {
                float sum = 0.0f;
                for (int k = 0; k < m; k++) {
                    sum += PHt(i, k) * S_inv(k, j);
                }
                K_(i, j) = sum;
            }
        }
        
        // x = x + K*y
        for (int i = 0; i < n_; i++) {
            float sum = 0.0f;
            for (int j = 0; j < m; j++) {
                sum += K_(i, j) * y_(j, 0);
            }
            x_(i, 0) += sum;
        }
        
        // P = (I - K*H)*P (Joseph形式は省略)
        cm KH; KH.resize(n_, n_);
        for (int i = 0; i < n_; i++) {
            for (int j = 0; j < n_; j++) {
                float sum = 0.0f;
                for (int k = 0; k < m; k++) {
                    sum += K_(i, k) * H(k, j);
                }
                KH(i, j) = sum;
            }
        }
        
        cm I_KH; I_KH.resize(n_, n_);
        for (int i = 0; i < n_; i++) {
            for (int j = 0; j < n_; j++) {
                I_KH(i, j) = (i == j ? 1.0f : 0.0f) - KH(i, j);
            }
        }
        
        cm P_new; P_new.resize(n_, n_);
        for (int i = 0; i < n_; i++) {
            for (int j = 0; j < n_; j++) {
                float sum = 0.0f;
                for (int k = 0; k < n_; k++) {
                    sum += I_KH(i, k) * P_(k, j);
                }
                P_new(i, j) = sum;
            }
        }
        P_ = P_new;
        
        // 対称化
        for (int i = 0; i < n_; i++) {
            for (int j = i+1; j < n_; j++) {
                float avg = (P_(i,j) + P_(j,i)) * 0.5f;
                P_(i,j) = avg;
                P_(j,i) = avg;
            }
        }
        
        // 正則化
        sensor_filter_.divergence_guard.regularize_covariance(P_);
        
        has_result_ = true;
    }
    
    // getData用: 結果を取得
    int get_data(cm& x_out, cm& P_out, cm& K_out, cm& S_out, cm& y_out) {
        if (!has_result_) return 1;  // データなし
        
        x_out = x_;
        P_out = P_;
        K_out = K_;
        S_out = S_;
        y_out = y_;
        
        has_result_ = false;  // 取得後はフラグクリア
        return 0;  // 成功
    }
    
    // 状態取得（簡易版）
    void get_state(cm& x_out, cm& P_out) {
        x_out = x_;
        P_out = P_;
    }
    
    void set_state(const cm& x, const cm& P) {
        x_ = x;
        P_ = P;
    }
};

} // namespace kf

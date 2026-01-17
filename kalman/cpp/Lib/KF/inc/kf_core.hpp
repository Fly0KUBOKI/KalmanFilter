#pragma once
#ifndef LIB_KF_INC_KF_CORE_HPP
#define LIB_KF_INC_KF_CORE_HPP

#include "../../Matrix/fixed_matrix.hpp"
#include "../../Sensor/sensor_filters.hpp"
#include "kalman_filter_core.hpp"
#include "kf_operations.hpp"
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
        // x = F * x + u
        x_ = F * x_ + u;

        // P = F * P * F' + Q
        P_ = F * P_ * F.transpose() + Q_;

        // Symmetrize
        cmath_fx::utils::symmetrize(P_);
    }
    
    // Update ステップ
    void update(const cm& z, const cm& H, const cm& R) {
        if (!initialized_) return;
        
        int m = z.rows;
        // h = H * x
        cm h = H * x_;

        // y = z - h
        y_.resize(m, 1);
        y_ = z - h;

        // S = H*P*H' + R
        cm S = H * P_ * H.transpose() + R;
        cmath_fx::utils::symmetrize(S);

        // Compute K = P * H' * S_inv (fallback to zero gain on singular S)
        cm S_inv; S_inv.resize(m, m);
        K_.resize(n_, m);
        if (!S.inverse(S_inv)) {
            // singular S -> zero gain
            for (int i = 0; i < n_; ++i) for (int j = 0; j < m; ++j) K_(i, j) = 0.0f;
        } else {
            K_ = P_ * H.transpose() * S_inv;
        }

        // State update
        x_ = x_ + K_ * y_;

        // Joseph-form covariance update using runtime helper
        cm P_upd; P_upd.resize(n_, n_);
        kf::ops::joseph_form_update(P_, K_, H, R, P_upd);
        P_ = P_upd;

        // Symmetrize & regularize
        cmath_fx::utils::symmetrize(P_);
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

#endif // LIB_KF_INC_KF_CORE_HPP

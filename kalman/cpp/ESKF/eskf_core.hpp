#pragma once

#include "../Common/Math/fixed_matrix.hpp"
#include "../Common/Math/quaternion.hpp"

namespace eskf {

using cm = cmath_fx::FixedMatrix;

class ESKFCore {
public:
    // ノミナル状態積分（RK2/台形則）
    static void integrate_nominal(
        cm& p, cm& v, cm& q, cm& ba, cm& bg,
        const cm& a_meas, const cm& w_meas,
        float dt, const cm& g,
        const cm& gyro_noise_threshold,
        const cm& accel_noise_threshold
    );
    
    // 加速度更新（Roll/Pitch）
    static void update_accel(
        cm& q,
        const cm& a_meas,
        float scale_factor = 1.0f
    );
    
    // 磁気計更新
    static void update_mag(
        cm& q, cm& P,
        const cm& m_meas,
        const cm& m_world,
        const cm& R_mag,
        cm& K_out,
        cm& dx_out
    );
    
    // GPS更新
    static void update_gps(
        cm& p, cm& v, cm& P,
        const cm& gps_pos,
        const cm& gps_origin,
        const cm& R_gps,
        cm& K_out,
        cm& dx_out
    );
    
    // 気圧計更新
    static void update_baro(
        cm& p, cm& P,
        float pressure,
        const cm& gps_origin,
        float R_baro,
        cm& K_out,
        cm& dx_out
    );
    
    // 共分散予測
    static void predict_covariance(const cm& P, const cm& q, const cm& a_meas, const cm& ba,
                                   const cm& w_meas, const cm& bg, const cm& Q, float dt,
                                   cm& P_new);
    
    // 状態遷移行列計算
    static void compute_F_matrix(const cm& q, const cm& a_meas, const cm& ba,
                                 const cm& w_meas, const cm& bg, float dt,
                                 cm& F);
    
    // 誤差状態注入
    static void inject_error_state(cm& p, cm& v, cm& q, cm& ba, cm& bg, const cm& dx);

private:
    // AB2積分用の静的バッファ
    static cm prev_a_world;
    static cm prev_v;
    static bool prev_initialized;
    
    // ヘルパー関数
    static float pressure_to_altitude(float pressure);
    static void gps_to_local(const cm& gps_pos, const cm& origin, cm& local_pos);
};

} // namespace eskf

#pragma once

#include "../Common/Math/fixed_matrix.hpp"
#include "../Common/Math/quaternion.hpp"

namespace eskf {

using Vector3 = cmath_fx::Vector<3, float>;
using Vector4 = cmath_fx::Vector<4, float>;
using Matrix3x3 = cmath_fx::Matrix<3, 3, float>;
using Matrix15x15 = cmath_fx::Matrix<15, 15, float>;
using Vector15 = cmath_fx::Vector<15, float>;

class ESKFCore {
public:
    // ノミナル状態積分（RK2/台形則）
    static void integrate_nominal(
        Vector3& p, Vector3& v, Vector4& q, Vector3& ba, Vector3& bg,
        const Vector3& a_meas, const Vector3& w_meas,
        float dt, const Vector3& g,
        const Vector3& gyro_noise_threshold,
        const Vector3& accel_noise_threshold
    );
    
    // 加速度更新（Roll/Pitch）
    static void update_accel(
        Vector4& q,
        const Vector3& a_meas,
        float scale_factor = 1.0f
    );
    
    // 磁気計更新
    static void update_mag(
        Vector4& q, Matrix15x15& P,
        const Vector3& m_meas,
        const Vector3& m_world,
        const Matrix3x3& R_mag,
        cmath_fx::Matrix<15, 3, float>& K_out,
        Vector15& dx_out
    );
    
    // GPS更新
    static void update_gps(
        Vector3& p, Vector3& v, Matrix15x15& P,
        const Vector3& gps_pos,
        const Vector3& gps_origin,
        const Matrix3x3& R_gps,
        cmath_fx::Matrix<15, 3, float>& K_out,
        Vector15& dx_out
    );
    
    // 気圧計更新
    static void update_baro(
        Vector3& p, Matrix15x15& P,
        float pressure,
        const Vector3& gps_origin,
        float R_baro,
        cmath_fx::Matrix<15, 1, float>& K_out,
        Vector15& dx_out
    );
    
    // 共分散予測
    static void predict_covariance(const Matrix15x15& P, const Vector4& q, const Vector3& a_meas, const Vector3& ba,
                                   const Vector3& w_meas, const Vector3& bg, const Matrix15x15& Q, float dt,
                                   Matrix15x15& P_new);
    
    // 状態遷移行列計算
    static void compute_F_matrix(const Vector4& q, const Vector3& a_meas, const Vector3& ba,
                                 const Vector3& w_meas, const Vector3& bg, float dt,
                                 Matrix15x15& F);
    
    // 誤差状態注入
    static void inject_error_state(Vector3& p, Vector3& v, Vector4& q, Vector3& ba, Vector3& bg, const Vector15& dx);

private:
    // AB2積分用の静的バッファ
    static Vector3 prev_a_world;
    static Vector3 prev_v;
    static bool prev_initialized;
    
    // ヘルパー関数
    static float pressure_to_altitude(float pressure);
    static void gps_to_local(const Vector3& gps_pos, const Vector3& origin, Vector3& local_pos);
};

} // namespace eskf

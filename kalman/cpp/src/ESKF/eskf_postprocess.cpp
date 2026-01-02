#include "../../Inc/ESKF/eskf_postprocess.hpp"
#include "../../Lib/Quaternion/quaternion_lib.hpp"
#include "Common/Math/vector_utils.hpp"
#include "Common/filter_management.hpp"
#include <cmath>

namespace eskf {

void predict_postprocess(
    cmath_fx::Vector<3, float>& v,
    const cmath_fx::Vector<4, float>& q,
    cmath_fx::Matrix<15, 15, float>& P,
    const cmath_fx::Vector<3, float>& a_for_vel,
    float dt,
    const cmath_fx::Vector<3, float>& g,
    const PredictPostprocessParams& params
) {
    using namespace common::math;
    using namespace common::filter;
    using Quat = quat_lib::Quaternion<float>;
    
    // 1. accel_z_integration
    // 注意: この部分はMATLAB呼び出しを含むため、MEXファイル内に実装を残す
    // ここでは純粋なC++ロジックのみを実装
    
    // 2. velocity_damping
    if (params.velocity_damping > 0.0f) {
        v(0, 0) = v(0, 0) * (1.0f - params.velocity_damping * dt);
        v(1, 0) = v(1, 0) * (1.0f - params.velocity_damping * dt);
    }
    
    // 3. P normalization (max_var check)
    normalize_covariance(P);
    
    // 4. Velocity norm check (clip to 10.0 m/s)
    clip_vector_norm(v, 10.0f);
}

UpdatePostprocessResult update_state_from_dx(
    const cmath_fx::Vector<15, float>& dx,
    const cmath_fx::Vector<3, float>& state_p,
    const cmath_fx::Vector<3, float>& state_v,
    const cmath_fx::Vector<4, float>& state_q,
    const cmath_fx::Vector<3, float>& state_ba,
    const cmath_fx::Vector<3, float>& state_bg,
    const cmath_fx::Matrix<15, 15, float>& new_state_P
) {
    using Quat = quat_lib::Quaternion<float>;
    
    UpdatePostprocessResult result;
    
    // 位置、速度、バイアスの更新
    for (int i = 0; i < 3; ++i) {
        result.p(i, 0) = state_p(i, 0) + dx(i, 0);
        result.v(i, 0) = state_v(i, 0) + dx(i + 3, 0);
        result.ba(i, 0) = state_ba(i, 0) + dx(i + 9, 0);
        result.bg(i, 0) = state_bg(i, 0) + dx(i + 12, 0);
    }
    
    // クォータニオン更新: dq = [1; 0.5 * phi], q_new = dq * q
    float phi[3] = {dx(6, 0), dx(7, 0), dx(8, 0)};
    float dq[4] = {1.0f, 0.5f * phi[0], 0.5f * phi[1], 0.5f * phi[2]};
    float q_state[4] = {state_q(0, 0), state_q(1, 0), state_q(2, 0), state_q(3, 0)};
    
    Quat quat_dq(dq[0], dq[1], dq[2], dq[3]);
    Quat quat_q(q_state[0], q_state[1], q_state[2], q_state[3]);
    Quat quat_new = Quat::multiply(quat_dq, quat_q);
    quat_new.normalize();
    
    result.q(0, 0) = quat_new.w;
    result.q(1, 0) = quat_new.x;
    result.q(2, 0) = quat_new.y;
    result.q(3, 0) = quat_new.z;
    
    // P行列をコピーして対称化
    result.P = new_state_P;
    symmetrize_covariance(result.P);
    
    result.should_skip = false;
    return result;
}

void symmetrize_covariance(cmath_fx::Matrix<15, 15, float>& P) {
    for (int i = 0; i < 15; ++i) {
        for (int j = i + 1; j < 15; ++j) {
            float avg = 0.5f * (P(i, j) + P(j, i));
            P(i, j) = avg;
            P(j, i) = avg;
        }
    }
}

} // namespace eskf


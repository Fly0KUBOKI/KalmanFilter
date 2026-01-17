#include "../inc/eskf_postprocess.hpp"
#include "../inc/eskf_includes.hpp"
// filter_mgmt.hpp is provided by eskf_includes.hpp

namespace eskf {

void predict_postprocess(cmath_fx::Vector<3, float>& v, const cmath_fx::Vector<4, float>& q, cmath_fx::Matrix<15, 15, float>& P, const cmath_fx::Vector<3, float>& a_for_vel, float dt, const cmath_fx::Vector<3, float>& g, const PredictPostprocessParams& params) {
    using namespace common::math; 
    using namespace cquat;
    if (params.velocity_damping > 0.0f) { v(0,0) = v(0,0) * (1.0f - params.velocity_damping * dt); v(1,0) = v(1,0) * (1.0f - params.velocity_damping * dt); }
    common::covariance::ensure_positive_definite(P); 
    common::math::clip_vector_norm(v, 10.0f);
}

UpdatePostprocessResult update_state_from_dx(const cmath_fx::Vector<15, float>& dx, const cmath_fx::Vector<3, float>& state_p, const cmath_fx::Vector<3, float>& state_v, const cmath_fx::Vector<4, float>& state_q, const cmath_fx::Vector<3, float>& state_ba, const cmath_fx::Vector<3, float>& state_bg, const cmath_fx::Matrix<15, 15, float>& new_state_P) {
    using namespace cquat; UpdatePostprocessResult result; for (int i=0;i<3;++i){ result.p(i,0)=state_p(i,0)+dx(i,0); result.v(i,0)=state_v(i,0)+dx(i+3,0); result.ba(i,0)=state_ba(i,0)+dx(i+9,0); result.bg(i,0)=state_bg(i,0)+dx(i+12,0); }
    cmath_fx::Vector<4,float> dq; dq(0,0)=1.0f; dq(1,0)=0.5f*dx(6,0); dq(2,0)=0.5f*dx(7,0); dq(3,0)=0.5f*dx(8,0); cmath_fx::Vector<4,float> quat_new; multiply_quat(dq, state_q, quat_new); cquat::normalize_quat(quat_new); result.q(0,0)=quat_new(0,0); result.q(1,0)=quat_new(1,0); result.q(2,0)=quat_new(2,0); result.q(3,0)=quat_new(3,0); result.P = new_state_P; cmath_fx::utils::symmetrize(result.P); result.should_skip = false; return result;
}

} // namespace eskf

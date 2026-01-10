#pragma once

#ifndef MEX_MEX_RUN_ESKF_FILTER_OPS_HPP_IMPL
#define MEX_MEX_RUN_ESKF_FILTER_OPS_HPP_IMPL

/**
 * mex_run_eskf.cpp用のフィルター操作関数群
 */

#include "mex_eskf_common.hpp"

namespace mex_run_eskf_impl {

inline void check_and_reset(ESKFState* s, int k) {
    Matrix<15, 15, float> P_float;
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            P_float(i, j) = s->P[i + j*15];
        }
    }
    
    Vector<3, float> p_float, v_float, ba_float, bg_float;
    Vector<4, float> q_float;
    for (int i = 0; i < 3; ++i) {
        p_float(i, 0) = s->p[i];
        v_float(i, 0) = s->v[i];
        ba_float(i, 0) = s->ba[i];
        bg_float(i, 0) = s->bg[i];
    }
    for (int i = 0; i < 4; ++i) q_float(i, 0) = s->q[i];

    bool need_reset = check_state_divergence(p_float, v_float, q_float, ba_float, bg_float, P_float);
    if (need_reset) {
        s->last_reset_step = k;
        using namespace common::filter;
        Matrix<15, 15, float> P2;
        setIdentityScaled(P2, 0.01f);
        reset_state_on_divergence(v_float, ba_float, bg_float, q_float, P2);
        for (int i = 0; i < 3; ++i) {
            s->v[i] = v_float(i, 0);
            s->ba[i] = ba_float(i, 0);
            s->bg[i] = bg_float(i, 0);
        }
        for (int i = 0; i < 4; ++i) s->q[i] = q_float(i, 0);
        for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) s->P[i + j*15] = P2(i, j);
    }
}

inline void zupt_check_and_update(ESKFState* s, const double* a_meas, const double* w_meas) {
    Vector<3, float> a_float, w_float;
    for (int i = 0; i < 3; ++i) { a_float(i,0) = static_cast<float>(a_meas[i]); w_float(i,0) = static_cast<float>(w_meas[i]); }
    bool stationary = check_zupt_condition(a_float, w_float, s->zupt_threshold_accel, s->zupt_threshold_gyro);
    if (stationary) s->zupt_counter++; else s->zupt_counter = 0;
    s->is_stationary = (s->zupt_counter >= s->zupt_min_duration);
    if (s->is_stationary) {
        Vector<3, float> v_in, v_out;
        Matrix<15, 15, float> P_in, P_out;
        for (int i = 0; i < 3; ++i) v_in(i,0) = s->v[i];
        for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) P_in(i,j) = s->P[i + j*15];
        ESKFCore::update_zupt(v_in, P_in, v_out, P_out);
        for (int i = 0; i < 3; ++i) s->v[i] = v_out(i,0);
        for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) s->P[i + j*15] = P_out(i,j);
    }
}

} // namespace mex_run_eskf_impl

#endif // MEX_MEX_RUN_ESKF_FILTER_OPS_HPP_IMPL


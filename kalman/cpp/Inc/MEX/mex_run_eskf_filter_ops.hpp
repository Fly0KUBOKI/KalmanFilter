#pragma once

#ifndef MEX_MEX_RUN_ESKF_FILTER_OPS_HPP
#define MEX_MEX_RUN_ESKF_FILTER_OPS_HPP

/**
 * mex_run_eskf.cpp用のフィルター操作関数群
 * 
 * リセットチェック、ZUPTチェックなどの実装を含みます。
 */

#include "mex_eskf_common.hpp"

namespace mex_run_eskf_impl {

/**
 * リセットチェックと処理
 */
inline void check_and_reset(ESKFState* s, int k) {
    // Check for divergence (implementation moved to Src/Common/filter_management.cpp)
    // Convert P matrix to Matrix type
    Matrix<15, 15, float> P_float;
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) {
            P_float(i, j) = static_cast<float>(s->P[i + j*15]);
        }
    }
    
    Vector<3, float> p_float, v_float, ba_float, bg_float;
    Vector<4, float> q_float;
    for (int i = 0; i < 3; ++i) {
        p_float(i, 0) = static_cast<float>(s->p[i]);
        v_float(i, 0) = static_cast<float>(s->v[i]);
        ba_float(i, 0) = static_cast<float>(s->ba[i]);
        bg_float(i, 0) = static_cast<float>(s->bg[i]);
    }
    for (int i = 0; i < 4; ++i) {
        q_float(i, 0) = static_cast<float>(s->q[i]);
    }
    
    bool need_reset = check_state_divergence(p_float, v_float, q_float, ba_float, bg_float, P_float);
    
    if (need_reset) {
        s->last_reset_step = k;
        
        // Reset state using filter_management directly (mex_filter_management を統合)
        using namespace common::filter;
        Vector<3, float> v_float, ba_float, bg_float;
        Vector<4, float> q_float;
        Matrix<15, 15, float> P_float;
        
        // Reset P matrix using setIdentityScaled (reset_scale = 0.01)
        float reset_scale = 0.01f;
        setIdentityScaled(P_float, reset_scale);
        
        // Convert current state to float type
        for (int i = 0; i < 3; ++i) {
            v_float(i, 0) = static_cast<float>(s->v[i]);
            ba_float(i, 0) = static_cast<float>(s->ba[i]);
            bg_float(i, 0) = static_cast<float>(s->bg[i]);
        }
        for (int i = 0; i < 4; ++i) {
            q_float(i, 0) = static_cast<float>(s->q[i]);
        }
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                P_float(i, j) = static_cast<float>(s->P[i + j*15]);
            }
        }
        
        // Reset processing
        reset_state_on_divergence(v_float, ba_float, bg_float, q_float, P_float);
        
        // Convert results back to double type
        for (int i = 0; i < 3; ++i) {
            s->v[i] = static_cast<double>(v_float(i, 0));
            s->ba[i] = static_cast<double>(ba_float(i, 0));
            s->bg[i] = static_cast<double>(bg_float(i, 0));
        }
        for (int i = 0; i < 4; ++i) {
            s->q[i] = static_cast<double>(q_float(i, 0));
        }
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                s->P[i + j*15] = static_cast<double>(P_float(i, j));
            }
        }
    }
}

/**
 * ZUPTチェックと更新処理
 */
inline void zupt_check_and_update(ESKFState* s, const double* a_meas, const double* w_meas) {
    // ZUPT check (implementation moved to Src/Common/filter_management.cpp)
    Vector<3, float> a_float, w_float;
    for (int i = 0; i < 3; ++i) {
        a_float(i, 0) = static_cast<float>(a_meas[i]);
        w_float(i, 0) = static_cast<float>(w_meas[i]);
    }
    
    bool stationary = check_zupt_condition(a_float, w_float, 
                                           static_cast<float>(s->zupt_threshold_accel),
                                           static_cast<float>(s->zupt_threshold_gyro));
    
    if (stationary) {
        s->zupt_counter++;
    } else {
        s->zupt_counter = 0;
    }
    
    s->is_stationary = (s->zupt_counter >= s->zupt_min_duration);
    
    if (s->is_stationary) {
        // ZUPT update using ESKFCore directly (mex_eskf_zupt を統合)
        using namespace eskf;
        Vector<3, float> v_in, v_out;
        Matrix<15, 15, float> P_in, P_out;
        
        // Convert double to float
        for (int i = 0; i < 3; ++i) {
            v_in(i, 0) = static_cast<float>(s->v[i]);
        }
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                P_in(i, j) = static_cast<float>(s->P[i + j*15]);
            }
        }
        
        // Call ESKFCore::update_zupt
        ESKFCore::update_zupt(v_in, P_in, v_out, P_out);
        
        // Convert back to double
        for (int i = 0; i < 3; ++i) {
            s->v[i] = static_cast<double>(v_out(i, 0));
        }
        for (int i = 0; i < 15; ++i) {
            for (int j = 0; j < 15; ++j) {
                s->P[i + j*15] = static_cast<double>(P_out(i, j));
            }
        }
    }
}

} // namespace mex_run_eskf_impl

#endif // MEX_MEX_RUN_ESKF_FILTER_OPS_HPP


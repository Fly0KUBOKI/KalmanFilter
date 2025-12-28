#ifndef UNIFIED_FILTER_HPP
#define UNIFIED_FILTER_HPP

// Implementation: Src/MEUKF/unified_filter.cpp

#include "unified_types.hpp"  // FilterInput, FilterOutput, FilterState, Vec3, Vec4, Mat3, Mat15を定義

namespace meukf {

// ========== 統合フィルタクラス ==========
class UnifiedFilter {
public:
    UnifiedFilter();
    ~UnifiedFilter() = default;
    
    // メインインターフェース
    FilterOutput update(FilterState& state, const FilterInput& input);
    
    // リセット機能
    void reset();
    
private:
    // ヘルパー関数
    void predict_step(FilterState& state, const FilterInput& input);
    bool update_accel(FilterState& state, const FilterInput& input, FilterOutput& output);
    bool update_mag(FilterState& state, const FilterInput& input, FilterOutput& output);
    bool update_gps(FilterState& state, const FilterInput& input, FilterOutput& output);
    bool update_baro(FilterState& state, const FilterInput& input, FilterOutput& output);
    bool check_zupt(const Vec3& accel, const Vec3& gyro);
    void update_zupt(FilterState& state);
    void check_divergence(const FilterState& state, FilterOutput& output);
    void pack_output(const FilterState& state, FilterOutput& output);
    bool sensor_changed(const Vec3& current, const Vec3& prev) const;
    
    // Quaternion helpers
    Vec4 quaternion_multiply(const Vec4& q1, const Vec4& q2) const;
    Vec4 normalize_quaternion(const Vec4& q) const;
    Mat3 quaternion_to_rotation_matrix(const Vec4& q) const;
    Vec3 quaternion_to_euler(const Vec4& q) const;
    
    // 内部状態
    Vec3 prev_accel_;
    Vec3 prev_gyro_;
    Vec3 prev_mag_;
    Vec3 prev_gps_pos_;
    float prev_baro_alt_;
    float tolerance_;
    int zupt_counter_;
    int zupt_min_duration_;
    float zupt_threshold_accel_;
    float zupt_threshold_gyro_;
};

} // namespace meukf

#endif // UNIFIED_FILTER_HPP

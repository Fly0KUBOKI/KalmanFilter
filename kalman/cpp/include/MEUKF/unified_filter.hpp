#ifndef UNIFIED_FILTER_HPP
#define UNIFIED_FILTER_HPP

#include "unified_types.hpp"

namespace meukf {

class UnifiedFilter {
public:
    UnifiedFilter();
    ~UnifiedFilter() = default;

    // メインインターフェース
    void initialize(const FilterState& initial_state);
    FilterOutput update(FilterState& state, const FilterInput& input);

    // リセット機能
    void reset();

private:
    // 内部状態を保持するための型は FilterState 等を使用
    FilterState current_state_;
    
    // 内部パラメータ（実装で定義）
    double tolerance_;
    int zupt_counter_;
    int zupt_min_duration_;
    double zupt_threshold_accel_;
    double zupt_threshold_gyro_;
    
    // 前回センサー値（変更検知用）
    Vec3 prev_accel_;
    Vec3 prev_gyro_;
    Vec3 prev_mag_;
    Vec3 prev_gps_pos_;
    double prev_baro_alt_;
};

} // namespace meukf

#endif // UNIFIED_FILTER_HPP

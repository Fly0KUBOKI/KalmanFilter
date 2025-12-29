#pragma once

#include <cstdint>

namespace lib {

// 浮動小数点型 (float固定)
using Scalar = float;

// 整数型 (200以下のインデックス・カウンタ)
using Index = uint8_t;

// 固定サイズ制限
static constexpr Index MAX_STATE_DIM = 20;
static constexpr Index MAX_MEAS_DIM = 10;

// ステータスコード
enum Status : uint8_t {
    STATUS_OK = 0,
    STATUS_ERROR = 1,
    STATUS_SINGULAR = 2,
    STATUS_NOT_INITIALIZED = 3
};

} // namespace lib





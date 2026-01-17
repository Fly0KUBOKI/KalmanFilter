#pragma once
#ifndef MEX_IMPL_MEX_HYBRID_FILTER_COMMON_HPP
#define MEX_IMPL_MEX_HYBRID_FILTER_COMMON_HPP


// mex_hybrid_filter.cpp用の共通インクルードと定義

// 標準ライブラリ
#include <mex.h>
#include <cmath>
#include <cstring>

// 定数定義
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// レイヤー1: 基本型（最初に配置）
#include "../../Lib/Matrix/fixed_matrix.hpp"

#include "../../Lib/Matrix/fixed_matrix.hpp"
#include "../../Lib/Quaternion/quaternion_functions.hpp"
#include "../../Lib/Matrix/Math/statistics.hpp"

// レイヤー3: ESKF コア
#include "../../Lib/ESKF/inc/eskf_core.hpp"
#include "../../Lib/ESKF/inc/eskf_postprocess.hpp"
#include "../../Lib/ESKF/inc/eskf_state.hpp"
#include "../../Lib/ESKF/inc/eskf_runner.hpp"
#include "mex_hybrid_filter_initializer.hpp"

// レイヤー4: 統合層
#include "../../Lib/ESKF/inc/filter_mgmt.hpp"
#include "../../Lib/Sensor/sensor_filters.hpp"
#include "../../Lib/Sensor/sensor_preprocessor.hpp"
#include "../../Lib/ESKF/inc/eskf_sensor_updates.hpp"
#include "mex_type_conversion.hpp"
#include "mex_helpers.hpp"

// using宣言
using namespace common::math;
using namespace common::filter;
using namespace common::sensor;
// Quaternion types using cquat:: functions
using Quat = cmath_fx::Vector<4, float>;
using QuatF = cmath_fx::Vector<4, float>;
using namespace cmath_fx;
using namespace eskf;
using namespace mex_conv;
using namespace mex_helpers;
using cm = cmath_fx::FixedMatrix;  // Alias for sensor filter

// マクロ定義（センサーデータ取得用）
#define getAccel(obs, idx, out) getVec3(obs, "ax", "ay", "az", idx, out)
#define getGyro(obs, idx, out)  getVec3(obs, "wx", "wy", "wz", idx, out)
#define getMag(obs, idx, out)   getVec3(obs, "mx", "my", "mz", idx, out)

// グローバル変数の前方宣言（mex_hybrid_filter_impl名前空間用）
namespace mex_hybrid_filter_impl {
    extern SensorFilterLib g_filter_lib;
}

#endif // MEX_IMPL_MEX_HYBRID_FILTER_COMMON_HPP


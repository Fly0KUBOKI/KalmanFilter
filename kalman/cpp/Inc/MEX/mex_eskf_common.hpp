#pragma once

#ifndef MEX_MEX_ESKF_COMMON_HPP
#define MEX_MEX_ESKF_COMMON_HPP

/**
 * mex_run_eskf.cpp用の共通インクルードと定義
 * 
 * このヘッダーは、ESKF関連のMEXファイルで共通して使用される
 * インクルード、定義、using宣言をまとめています。
 */

// 標準ライブラリ
#include <mex.h>
#include <cmath>
#include <cstring>
#include <string>
#include <map>
#include <vector>

// 定数定義
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// レイヤー1: 基本型（最初に配置）
#include "../Common/Math/fixed_matrix.hpp"

// レイヤー2: ユーティリティ
#include "../Common/Math/vector_utils.hpp"
#include "../Common/Math/quaternion_lib.hpp"
#include "../Common/Math/statistics.hpp"

// レイヤー3: ESKF コア
#include "../ESKF/eskf_core.hpp"
#include "../ESKF/eskf_postprocess.hpp"
#include "../ESKF/eskf_state.hpp"
#include "../ESKF/eskf_runner.hpp"
#include "../ESKF/eskf_initializer.hpp"

// レイヤー4: 統合層
#include "../Common/filter_management.hpp"
#include "../Common/Sensor/sensor_filter.hpp"
#include "../Common/Sensor/sensor_preprocessor.hpp"
#include "../ESKF/eskf_sensor_updates.hpp"
#include "mex_type_conversion.hpp"
#include "mex_helpers.hpp"

// using宣言
using namespace common::math;
using namespace common::filter;
using namespace common::sensor;
using Quat = quat_lib::Quaternion<double>;
using QuatF = quat_lib::Quaternion<float>;
using namespace cmath_fx;
using namespace eskf;
using namespace mex_conv;
using namespace mex_helpers;
using cm = cmath_fx::FixedMatrix;  // Alias for sensor filter

// マクロ定義（センサーデータ取得用）
#define getAccel(obs, idx, out) getVec3(obs, "ax", "ay", "az", idx, out)
#define getGyro(obs, idx, out)  getVec3(obs, "wx", "wy", "wz", idx, out)
#define getMag(obs, idx, out)   getVec3(obs, "mx", "my", "mz", idx, out)

#endif // MEX_MEX_ESKF_COMMON_HPP


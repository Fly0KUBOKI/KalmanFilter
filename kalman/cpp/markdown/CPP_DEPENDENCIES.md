# C++実装依存関係

## 依存関係グラフ

### MEX層

```
mex_run_eskf.cpp
├─ mex_eskf_common.hpp
│   ├─ eskf_state.hpp
│   ├─ eskf_runner.hpp
│   └─ sensor_filter.hpp
├─ mex_run_eskf_impl.hpp
│   ├─ mex_run_eskf_sensor_updates.hpp
│   │   ├─ eskf_sensor_updates.hpp
│   │   └─ sensor_preprocessor.hpp
│   ├─ mex_run_eskf_filter_ops.hpp
│   │   ├─ filter_management.hpp
│   │   └─ eskf_postprocess.hpp
│   ├─ meukf_core.hpp
│   │   ├─ unified_types.hpp
│   │   ├─ meukf_types.hpp
│   │   ├─ fixed_matrix.hpp
│   │   └─ quaternion.hpp
│   └─ mex_type_conversion.hpp
│       └─ mex_type_conv.hpp
└─ mex_helpers.hpp
```

### ESKF層

```
eskf_runner.cpp
├─ eskf_core.hpp
│   ├─ fixed_matrix.hpp
│   ├─ quaternion.hpp
│   └─ math_utils.hpp
├─ eskf_postprocess.hpp
│   ├─ quaternion_lib.hpp
│   ├─ vector_utils.hpp
│   └─ filter_management.hpp
└─ sensor_filter.hpp

eskf_core.cpp
├─ eskf_core.hpp
├─ eskf_math.hpp
├─ kalman_filter_core.hpp
├─ quaternion.hpp
└─ math_utils.hpp

eskf_sensor_updates.cpp
├─ eskf_sensor_updates.hpp
├─ eskf_state.hpp
└─ sensor_preprocessor.hpp

eskf_initializer.cpp
├─ eskf_initializer.hpp
├─ eskf_state.hpp
├─ quaternion_lib.hpp
└─ statistics.hpp
```

### MEUKF層

```
meukf_core.cpp
├─ meukf_core.hpp
│   ├─ unified_types.hpp
│   ├─ meukf_types.hpp
│   ├─ fixed_matrix.hpp
│   └─ quaternion.hpp
├─ math_utils.hpp
└─ quaternion.hpp

unified_filter.cpp
├─ unified_filter.hpp
│   ├─ unified_types.hpp
│   └─ filter_interface.hpp
├─ sensor_filter.hpp
├─ math_utils.hpp
└─ meukf_core.hpp
```

### 共通ライブラリ層

```
filter_management.cpp
└─ filter_management.hpp

sensor_preprocessor.cpp
└─ sensor_preprocessor.hpp
```

## 名前空間構造

```
namespace eskf {
    class ESKFCore;
    class ESKFRunner;
    struct ESKFState;
    // ...
}

namespace meukf {
    class MEUKFCore;
    class UnifiedFilter;
    struct State;
    struct MEUKFInput;
    struct MEUKFOutput;
    // ...
}

namespace common {
    namespace filter {
        // フィルター管理関数
    }
    namespace sensor {
        // センサー前処理関数
    }
    namespace math {
        // 数学ユーティリティ
    }
}

namespace mex_conv {
    // MATLAB型変換関数
}

namespace mex_run_eskf_impl {
    // MEX実装関数
}
```

## 主要な型定義

### ESKF型

```cpp
namespace eskf {
    using Scalar = float;
    using Vector3 = cmath_fx::Vector<3, Scalar>;
    using Vector4 = cmath_fx::Vector<4, Scalar>;
    using Matrix3x3 = cmath_fx::Matrix<3, 3, Scalar>;
    using Matrix15x15 = cmath_fx::Matrix<15, 15, Scalar>;
    using Vector15 = cmath_fx::Vector<15, Scalar>;
}
```

### MEUKF型

```cpp
namespace meukf {
    using Vector3 = cmath_fx::Matrix<3, 1, float>;
    using Vector2 = cmath_fx::Matrix<2, 1, float>;
    using Vector4 = cmath_fx::Matrix<4, 1, float>;
    using Matrix3x3 = cmath_fx::Matrix<3, 3, float>;
    using Matrix2x2 = cmath_fx::Matrix<2, 2, float>;
    using Matrix15x15 = cmath_fx::Matrix<15, 15, float>;
    using Vector15 = cmath_fx::Matrix<15, 1, float>;
}
```

## データフロー

### ESKF実行フロー

```
MATLAB
  ↓
mex_run_eskf('init', ...)
  ↓
initialize_eskf_from_matlab()
  ↓
ESKFState* (初期化)
  ↓
mex_run_eskf('step', ...)
  ↓
do_step()
  ↓
ESKFRunner::predict()
  ├─ ESKFCore::integrate_nominal()
  ├─ ESKFCore::predict_covariance()
  └─ predict_postprocess()
  ↓
センサー更新
  ├─ update_accel_sensor()
  ├─ update_mag_sensor()
  ├─ update_baro_sensor()
  └─ update_gps_sensor()
  ↓
mex_run_eskf('get_state', ...)
  ↓
MATLAB (状態出力)
```

### MEUKF実行フロー

```
MATLAB
  ↓
mex_run_eskf('meukf_step', ...)
  ↓
do_meukf_step()
  ↓
MEUKFCore::step()
  ├─ predict()
  │   ├─ クォータニオン積分
  │   ├─ 位置・速度積分
  │   └─ 共分散予測
  └─ 更新
      ├─ update_accel_meukf() (UKF)
      ├─ update_mag_meukf() (UKF)
      ├─ update_gps() (線形KF)
      ├─ update_baro() (線形KF)
      └─ update_zupt() (線形KF)
  ↓
MATLAB (状態出力)
```

## 外部依存

### 必須ライブラリ
- **MATLAB MEX API**: `mex.h`
- **標準C++ライブラリ**: `<cmath>`, `<cstring>`, `<vector>`, etc.

### 内部ライブラリ
- **cmath_fx**: 固定サイズ行列・ベクトルライブラリ
- **quaternion_lib**: クォータニオン演算ライブラリ
- **sensor_filter**: センサーフィルターライブラリ

## コンパイル依存

### ビルド対象ファイル

**MEXファイル**:
- `MEX/mex_run_eskf.cpp`
- `MEX/mex_meukf_step.cpp` (非推奨)

**ESKFソース**:
- `src/ESKF/eskf_core.cpp`
- `src/ESKF/eskf_runner.cpp`
- `src/ESKF/eskf_initializer.cpp`
- `src/ESKF/eskf_sensor_updates.cpp`
- `src/ESKF/eskf_postprocess.cpp`
- `src/ESKF/eskf_math.cpp`

**MEUKFソース**:
- `src/MEUKF/meukf_core.cpp`
- `src/MEUKF/unified_filter.cpp`

**共通ソース**:
- `src/Common/filter_management.cpp`
- `src/Common/Sensor/sensor_preprocessor.cpp`

**その他**:
- `src/EKF/ekf_linear_update.cpp`
- `src/UKF/ukf_sigma_points.cpp`

## 循環依存の回避

以下の設計により循環依存を回避:

1. **前方宣言**: ヘッダーでは可能な限り前方宣言を使用
2. **実装分離**: 実装は`.cpp`ファイルに分離
3. **インターフェース分離**: 共通インターフェースを`filter_interface.hpp`に定義
4. **名前空間分離**: 各モジュールを名前空間で分離


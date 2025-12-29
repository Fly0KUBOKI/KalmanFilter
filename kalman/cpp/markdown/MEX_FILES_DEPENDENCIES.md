# MEXファイル依存関係マップ

## 依存関係の種類

1. **C++実装への依存**: `Inc/`, `Src/`, `Lib/`のヘッダー/実装ファイル
2. **MEX間の依存**: `mexCallMATLAB`による他のMEX関数の呼び出し
3. **外部依存**: なし（すべて自己完結）

## 主要な依存チェーン

### ESKF実行フロー

```
mex_run_eskf (ESKF.m完全置き換え)
│
├─ 初期化
│   └─ mex_eskf_constructor
│       └─ (C++実装なし、MATLABロジック)
│
├─ 予測ステップ
│   ├─ mex_adaptive_predict
│   │   └─ Src/ESKF/eskf_core.cpp
│   └─ mex_eskf_predict_postprocess
│       └─ (後処理ロジック)
│
├─ センサー更新
│   └─ mex_eskf_sensor_updates_full
│       ├─ mex_sensor_preprocessor
│       └─ mex_eskf_do_update
│           ├─ mex_sensor_filter
│           │   └─ Inc/Common/Sensor/sensor_filter.hpp
│           ├─ mex_meukf_step_v2
│           │   └─ Src/MEUKF/meukf_core.cpp
│           └─ mex_eskf_update_postprocess
│
├─ ZUPT更新
│   └─ mex_eskf_zupt
│
└─ フィルタ管理
    └─ mex_filter_management
        └─ Inc/Common/Math/fixed_matrix.hpp
```

### 統合フィルタフロー

```
mex_eskf_step (統合ESKFステップ)
│
└─ Inc/MEUKF/unified_filter.hpp
    └─ Src/MEUKF/unified_filter.cpp
        └─ Src/MEUKF/meukf_core.cpp
```

### ステップハンドルフロー

```
mex_eskf_step_handle
│
└─ mex_unified_filter
    └─ Src/MEUKF/unified_filter.cpp
        └─ Src/MEUKF/meukf_core.cpp
```

## 詳細依存関係表

### mex_run_eskf.cpp
**呼び出すMEX関数:**
- `mex_adaptive_predict` (予測)
- `mex_eskf_predict_postprocess` (予測後処理)
- `mex_eskf_sensor_updates_full` (センサー更新)
- `mex_eskf_zupt` (ZUPT)
- `mex_filter_management` (発散検出、リセット)
- `mex_eskf_constructor` (初期化)

**C++依存:**
- なし（すべてMEX関数経由）

### mex_eskf_sensor_updates_full.cpp
**呼び出すMEX関数:**
- `mex_sensor_preprocessor` (前処理)
- `mex_eskf_do_update` (更新実行)

**C++依存:**
- なし

### mex_eskf_do_update.cpp
**呼び出すMEX関数:**
- `mex_sensor_filter` (R行列取得、ノイズ推定)
- `mex_meukf_step_v2` (MEUKF更新)
- `mex_eskf_update_postprocess` (後処理)

**C++依存:**
- なし

### mex_eskf_step.cpp
**呼び出すMEX関数:**
- なし（直接C++実装を呼び出し）

**C++依存:**
- `Inc/MEUKF/unified_types.hpp`
- `Inc/MEUKF/unified_filter.hpp`
- `Src/MEUKF/unified_filter.cpp`
- `Src/MEUKF/meukf_core.cpp`

### mex_adaptive_predict.cpp
**呼び出すMEX関数:**
- なし

**C++依存:**
- `Inc/ESKF/eskf_core.hpp`
- `Src/ESKF/eskf_core.cpp`

### mex_eskf_math.cpp
**呼び出すMEX関数:**
- なし

**C++依存:**
- `Inc/ESKF/eskf_math.hpp`
- `Src/ESKF/eskf_math.cpp`

### mex_sensor_filter.cpp
**呼び出すMEX関数:**
- なし

**C++依存:**
- `Inc/Common/Sensor/sensor_filter.hpp`
- `Inc/Common/Math/fixed_matrix.hpp`

### mex_meukf_step.cpp
**呼び出すMEX関数:**
- なし

**C++依存:**
- `Src/MEUKF/meukf_core.cpp`

### mex_unified_filter.cpp
**呼び出すMEX関数:**
- なし

**C++依存:**
- `Inc/MEUKF/unified_types.hpp`
- `Inc/MEUKF/unified_filter.hpp`
- `Src/MEUKF/unified_filter.cpp`
- `Src/MEUKF/meukf_core.cpp`

### mex_ekf.cpp
**呼び出すMEX関数:**
- なし

**C++依存:**
- `Src/EKF/ekf_linear_update.cpp`

### mex_ukf_sigma_points.cpp
**呼び出すMEX関数:**
- なし

**C++依存:**
- `Src/UKF/ukf_sigma_points.cpp`

## MEX間呼び出しマトリックス

| 呼び出し元 | 呼び出し先 |
|-----------|-----------|
| `mex_run_eskf` | `mex_adaptive_predict`, `mex_eskf_predict_postprocess`, `mex_eskf_sensor_updates_full`, `mex_eskf_zupt`, `mex_filter_management`, `mex_eskf_constructor` |
| `mex_eskf_sensor_updates_full` | `mex_sensor_preprocessor`, `mex_eskf_do_update` |
| `mex_eskf_do_update` | `mex_sensor_filter`, `mex_meukf_step_v2`, `mex_eskf_update_postprocess` |
| `mex_eskf_sensor_update` | `mex_sensor_preprocessor`, `mex_sensor_filter`, `mex_meukf_step_v2`, `mex_eskf_update_postprocess` |
| `mex_eskf_sensor_update_full` | `mex_sensor_preprocessor` |
| `mex_eskf_do_cpp_update` | `mex_sensor_filter`, `mex_meukf_step_v2`, `mex_eskf_update_postprocess` |
| `mex_eskf_step_handle` | `mex_unified_filter` |
| `mex_eskf_full` | `mex_adaptive_predict`, `mex_eskf_predict_postprocess` |

## 循環依存

現在のところ、MEX間の循環依存は確認されていません。

## 推奨事項

1. **段階的統合**: MEX間呼び出しを減らし、直接C++実装を呼び出す方向へ
2. **依存関係の明確化**: 各ファイルの依存関係をドキュメント化
3. **不要ファイルの削除**: 使用されていないMEXファイルの特定と削除





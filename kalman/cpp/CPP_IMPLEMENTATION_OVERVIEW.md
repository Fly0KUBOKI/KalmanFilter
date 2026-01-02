# C++実装概要

## ディレクトリ構造

```
kalman/cpp/
├── Inc/              # ヘッダーファイル
│   ├── Common/       # 共通ライブラリ
│   ├── ESKF/         # Error State Kalman Filter
│   ├── MEUKF/        # Modified Error Unscented Kalman Filter
│   ├── EKF/          # Extended Kalman Filter
│   ├── UKF/          # Unscented Kalman Filter
│   ├── KF/           # Kalman Filter
│   └── MEX/          # MATLAB MEXインターフェース
├── src/              # ソースファイル
│   ├── Common/
│   ├── ESKF/
│   ├── MEUKF/
│   ├── EKF/
│   └── UKF/
├── MEX/              # MEXファイル実装
├── MEUKF/            # MEUKF関連（重複ファイルあり）
├── Lib/              # ライブラリ
└── tests/            # テストファイル
```

## 主要コンポーネント

### 1. MEXインターフェース
- **mex_run_eskf.cpp**: ESKFのメインMEX関数（init, step, get_state, free, meukf_step）
- **mex_meukf_step.cpp**: MEUKFステップ関数（非推奨、mex_run_eskfに統合済み）

### 2. ESKF (Error State Kalman Filter)
- **eskf_core.cpp**: ESKFコア実装（予測、更新）
- **eskf_runner.cpp**: ESKF実行エンジン
- **eskf_initializer.cpp**: ESKF初期化
- **eskf_sensor_updates.cpp**: センサー更新処理
- **eskf_postprocess.cpp**: 後処理
- **eskf_math.cpp**: 数学ユーティリティ

### 3. MEUKF (Modified Error Unscented Kalman Filter)
- **meukf_core.cpp**: MEUKFコア実装（予測、UKF更新）
- **unified_filter.cpp**: 統一フィルターインターフェース

### 4. 共通ライブラリ
- **filter_management.cpp**: フィルター管理（発散検知、リセット）
- **sensor_preprocessor.cpp**: センサー前処理
- **Math/**: 数学ライブラリ（クォータニオン、行列演算）

### 5. その他のフィルター
- **EKF**: Extended Kalman Filter（線形更新のみ実装）
- **UKF**: Unscented Kalman Filter（シグマポイント生成）

## 重複ファイル

以下のファイルは重複しており、実際に使用されているのは`src/`配下のファイルです：

1. **MEUKF/meukf_core.cpp** vs **src/MEUKF/meukf_core.cpp**
   - 使用: `src/MEUKF/meukf_core.cpp`
   - 削除推奨: `MEUKF/meukf_core.cpp`

2. **MEUKF/unified_filter.cpp** vs **src/MEUKF/unified_filter.cpp**
   - 使用: `src/MEUKF/unified_filter.cpp`
   - 削除推奨: `MEUKF/unified_filter.cpp`

3. **MEUKF/meukf_types.hpp** vs **Inc/MEUKF/meukf_types.hpp**
   - 使用: `Inc/MEUKF/meukf_types.hpp`
   - 削除推奨: `MEUKF/meukf_types.hpp`

4. ~~**MEX/mex_type_conv.hpp** vs **Inc/MEX/mex_type_conversion.hpp**~~ ✅ 統合完了
   - `mex_type_conv.hpp`の内容を`mex_type_conversion.hpp`に統合
   - `MEX/mex_type_conv.hpp`を削除
   - 現在は`Inc/MEX/mex_type_conversion.hpp`のみ使用

## データ型

- **float**: 内部計算は主にfloat型を使用（パフォーマンス重視）
- **double**: MATLABインターフェースではdouble型も使用（GPSデータはdoubleのみ）
- **Matrix型**: `cmath_fx::Matrix<N, M, float>`を使用（固定サイズ行列）
- **Vector型**: `cmath_fx::Vector<N, float>`を使用（固定サイズベクトル）

## 状態ベクトル

15次元状態ベクトル:
- 0-2: 位置 (p)
- 3-5: 速度 (v)
- 6-8: 姿勢誤差 (dtheta)
- 9-11: 加速度バイアス (ba)
- 12-14: ジャイロバイアス (bg)

## 主要な依存関係

```
mex_run_eskf.cpp
  ├─ mex_run_eskf_impl.hpp
  │   ├─ mex_run_eskf_sensor_updates.hpp
  │   ├─ mex_run_eskf_filter_ops.hpp
  │   └─ meukf_core.hpp
  │       └─ meukf_core.cpp
  └─ eskf_runner.hpp
      └─ eskf_runner.cpp
          ├─ eskf_core.cpp
          ├─ eskf_postprocess.cpp
          └─ sensor_filter.hpp
```


# MEXファイルの役割と実装状況

## 概要

このドキュメントは、MEXフォルダ内の全MEXファイルの役割と実装コードの移行状況をまとめたものです。

## MEXファイル一覧

### 1. `mex_sensor_preprocessor.cpp`

**役割**: センサー前処理（加速度、磁気、気圧、GPS）
- `preprocess_accel`: 加速度前処理（外れ値検出、変化検知）
- `preprocess_mag`: 磁気前処理（外れ値検出、変化検知）
- `preprocess_baro`: 気圧前処理（高度変換）
- `preprocess_gps`: GPS前処理（座標変換、外れ値検出）

**実装コードの移行状況**: ✅ 完了
- 実装: `Src/Common/Sensor/sensor_preprocessor.cpp`
- ヘッダー: `Inc/Common/Sensor/sensor_preprocessor.hpp`
- MEXファイル: MATLAB-C++型変換とコア関数呼び出しのみ

---

### 2. `mex_filter_management.cpp`

**役割**: フィルタ管理ユーティリティ
- `check_divergence`: 共分散行列の発散チェック
- `reset_state`: 状態リセット（共分散行列をスケール付き単位行列に設定）
- `apply_zupt`: ZUPT適用（速度をゼロにして共分散を減らす）

**実装コードの移行状況**: ✅ 完了
- 実装: `Src/Common/filter_management.cpp`
- ヘッダー: `Inc/Common/filter_management.hpp`
- MEXファイル: MATLAB-C++型変換とコア関数呼び出しのみ

---

### 3. `mex_adaptive_predict.cpp`

**役割**: 適応予測（Adaptive Q scaling）
- `predict`: 状態予測と適応的プロセスノイズ共分散の計算

**実装コードの移行状況**: ✅ 完了
- 実装: `Src/ESKF/eskf_core.cpp` (`ESKFCore::compute_adaptive_Q`, `ESKFCore::integrate_nominal`, `ESKFCore::predict_covariance`)
- ヘッダー: `Inc/ESKF/eskf_core.hpp`
- MEXファイル: MATLAB-C++型変換とコア関数呼び出しのみ

---

### 4. `mex_eskf_zupt.cpp`

**役割**: ZUPT更新（Zero Velocity Update）
- `update`: 速度をゼロとして観測するKalman filter update

**実装コードの移行状況**: ✅ 完了（最新）
- 実装: `Src/ESKF/eskf_core.cpp` (`ESKFCore::update_zupt`)
- ヘッダー: `Inc/ESKF/eskf_core.hpp`
- MEXファイル: MATLAB-C++型変換とコア関数呼び出しのみ

---

### 5. `mex_quaternion_lib.cpp`

**役割**: クォータニオン演算ライブラリ
- `to_rotation_matrix`: クォータニオンから回転行列へ変換
- `to_euler`: クォータニオンからオイラー角へ変換
- `from_euler`: オイラー角からクォータニオンへ変換
- `normalize`: クォータニオンの正規化
- `multiply`: クォータニオンの乗算
- `small_angle_quat`: 小角度クォータニオン生成
- `integrate`: クォータニオンの積分
- `conjugate`: 共役
- `inverse`: 逆元
- `from_two_vectors`: 2つのベクトルからクォータニオン生成
- `distance`: クォータニオン間の距離
- `slerp`: 球面線形補間
- `skew`: スキュー対称行列

**実装コードの移行状況**: ✅ 完了
- 実装: `Inc/Common/Math/quaternion_lib.hpp` (ヘッダーオンリー実装)
- MEXファイル: MATLAB-C++型変換とコア関数呼び出しのみ

---

### 6. `mex_eskf_predict_postprocess.cpp`

**役割**: 予測後処理
- `postprocess`: 予測後の処理（accel_z_integration, velocity_damping, P normalization, divergence_guard, velocity clipping）

**実装コードの移行状況**: ✅ 完了
- 実装: `Src/ESKF/eskf_postprocess.cpp` (`eskf::predict_postprocess`)
- ヘッダー: `Inc/ESKF/eskf_postprocess.hpp`
- MEXファイル: MATLAB-C++型変換とコア関数呼び出しのみ（一部MATLAB呼び出しを含む）

**注意**: `accel_z_integration`はMATLAB呼び出しを含むため、MEXファイル内に実装が残っています。

---

### 7. `mex_eskf_update_postprocess.cpp`

**役割**: 更新後処理
- `postprocess`: 更新後の処理（divergence_guard.check_and_attenuate_update, 状態更新, クォータニオン更新）

**実装コードの移行状況**: ✅ 完了
- 実装: `Src/ESKF/eskf_postprocess.cpp` (`eskf::update_state_from_dx`)
- ヘッダー: `Inc/ESKF/eskf_postprocess.hpp`
- MEXファイル: MATLAB-C++型変換とコア関数呼び出しのみ（一部MATLAB呼び出しを含む）

**注意**: `divergence_guard.check_and_attenuate_update`はMATLAB呼び出しを含むため、MEXファイル内に実装が残っています。

---

### 8. `mex_eskf_do_update.cpp`

**役割**: ESKF更新処理
- `update`: センサー更新処理（accel, mag, gps, baro）

**実装コードの移行状況**: ✅ 完了
- 実装: 他のMEX関数を呼び出すラッパー（`mex_eskf_sensor_updates_full`, `mex_eskf_update_postprocess`）
- MEXファイル: MATLAB-C++型変換と他のMEX関数呼び出しのみ

---

### 9. `mex_eskf_sensor_updates_full.cpp`

**役割**: センサー更新処理（前処理 + 更新統合）
- `accel`: 加速度更新
- `mag`: 磁気更新
- `baro`: 気圧更新
- `gps`: GPS更新

**実装コードの移行状況**: ✅ 完了
- 実装: 他のMEX関数を呼び出すラッパー（`mex_sensor_preprocessor`, `mex_eskf_do_update`）
- MEXファイル: MATLAB-C++型変換と他のMEX関数呼び出しのみ

**注意**: `is_nan_any_matlab`はMATLAB用のNaNチェック関数で、MEXファイルに残すべきです。

---

### 10. `mex_sensor_filter.cpp`

**役割**: センサーフィルタライブラリ
- `reset`: 全フィルタリセット
- `reset_zero`: 全フィルタをゼロにリセット
- `log`: ログ有効/無効
- `get_R`: ノイズ共分散行列取得
- `noise_estimate`: ノイズ推定
- `accel_config`: 加速度フィルタ設定
- `accel`: 加速度フィルタ
- `mag`: 磁気フィルタ
- `gps`: GPSフィルタ
- `baro`: 気圧フィルタ
- `divergence_check`: 発散チェック
- `divergence_regularize`: 共分散正則化
- `divergence_clip_velocity`: 速度クリップ

**実装コードの移行状況**: ✅ 完了
- 実装: `Inc/Common/Sensor/sensor_filter.hpp` (ヘッダーオンリー実装)
- MEXファイル: MATLAB-C++型変換とコア関数呼び出しのみ

---

### 11. `mex_meukf_step.cpp`

**役割**: MEUKF（Multi-State Extended Unscented Kalman Filter）ステップ処理
- `step`: MEUKFの1ステップ処理

**実装コードの移行状況**: ✅ 完了
- 実装: `Inc/MEUKF/meukf_core.hpp` (ヘッダーオンリー実装)
- MEXファイル: MATLAB-C++型変換とコア関数呼び出しのみ

**注意**: MEUKFはESKFとは別のアルゴリズムです。

---

### 12. `mex_eskf_constructor.cpp`

**役割**: ESKFコンストラクタ（初期化処理）
- `init`: ESKF状態の初期化（静止データから初期状態を推定）

**実装コードの移行状況**: ⚠️ 一部残存
- 移行済み:
  - 統計関数: `Inc/Common/Math/statistics.hpp`
  - クォータニオン関数: `Inc/Common/Math/quaternion_lib.hpp`
- 残存:
  - 初期化ロジック（MATLABデータの処理が必要なため、MEXファイルに残すべき）
  - `quaternion_from_euler`, `quaternion_to_rotation_matrix`（MEXラッパーとして適切）

**評価**: 初期化処理はMATLABデータの処理が必要なため、MEXファイルに残すのが適切です。ただし、統計関数やクォータニオン関数は既に移行済みです。

---

### 13. `mex_run_eskf.cpp`

**役割**: ESKF実行（全体制御）
- `init`: ESKF状態の初期化
- `step`: ESKFの1ステップ処理
- `get_state`: 状態取得
- `free`: 状態解放

**実装コードの移行状況**: ✅ 完了
- 実装: 他のMEX関数を呼び出すラッパー（`mex_eskf_constructor`, `mex_adaptive_predict`, `mex_eskf_predict_postprocess`, `mex_eskf_sensor_updates_full`, `mex_filter_management`, `mex_eskf_zupt`）
- MEXファイル: MATLAB-C++型変換と他のMEX関数呼び出しのみ

**注意**: 
- `quat_to_euler`: クォータニオンからオイラー角への変換（MEXラッパーとして適切）
- `check_and_reset`, `zupt_check_and_update`: 既に移行済みの関数を呼び出すラッパー（MEXラッパーとして適切）

---

## 実装コードの移行状況まとめ

### ✅ 完全に移行済み（MEXラッパーのみ）

1. `mex_sensor_preprocessor.cpp`
2. `mex_filter_management.cpp`
3. `mex_adaptive_predict.cpp`
4. `mex_eskf_zupt.cpp` ⭐ 最新移行
5. `mex_quaternion_lib.cpp`
6. `mex_eskf_predict_postprocess.cpp`
7. `mex_eskf_update_postprocess.cpp`
8. `mex_eskf_do_update.cpp`
9. `mex_eskf_sensor_updates_full.cpp`
10. `mex_sensor_filter.cpp`
11. `mex_meukf_step.cpp`
12. `mex_run_eskf.cpp`

### ⚠️ 一部残存（MEXラッパーとして適切）

13. `mex_eskf_constructor.cpp`
- 初期化ロジックはMATLABデータの処理が必要なため、MEXファイルに残すのが適切
- 統計関数やクォータニオン関数は既に移行済み

---

## 移行先ディレクトリ構造

```
kalman/cpp/
├── Inc/
│   ├── Common/
│   │   ├── Math/
│   │   │   ├── statistics.hpp
│   │   │   ├── quaternion_lib.hpp
│   │   │   ├── vector_utils.hpp
│   │   │   └── math_utils.hpp
│   │   └── Sensor/
│   │       ├── sensor_preprocessor.hpp
│   │       └── sensor_filter.hpp
│   └── ESKF/
│       ├── eskf_core.hpp
│       └── eskf_postprocess.hpp
└── Src/
    ├── Common/
    │   ├── Math/
    │   │   └── (statistics, vector_utils はヘッダーオンリー)
    │   ├── Sensor/
    │   │   └── sensor_preprocessor.cpp
    │   └── filter_management.cpp
    └── ESKF/
        ├── eskf_core.cpp
        └── eskf_postprocess.cpp
```

---

## 注意事項

1. **MATLAB呼び出しを含む実装**: 一部の実装はMATLAB関数（`mexCallMATLAB`）を呼び出すため、MEXファイル内に残す必要があります。
   - `mex_eskf_predict_postprocess.cpp`: `accel_z_integration`（MATLAB呼び出し）
   - `mex_eskf_update_postprocess.cpp`: `divergence_guard.check_and_attenuate_update`（MATLAB呼び出し）

2. **MEXラッパーとしての役割**: MEXファイルは、MATLAB-C++の型変換とコアC++関数の呼び出しのみを行うべきです。現在の実装は、この役割を適切に果たしています。

3. **初期化処理**: `mex_eskf_constructor.cpp`の初期化処理は、MATLABデータの処理が必要なため、MEXファイルに残すのが適切です。

---

## 更新履歴

- 2024-12-29: `mex_eskf_zupt.cpp`のZUPT更新実装を`Src/ESKF/eskf_core.cpp`に移行


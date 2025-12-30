# MEX インターフェース仕様

## 概要

このドキュメントは、`mex_run_eskf`と`mex_meukf_step_v2`のMEXインターフェース仕様を定義します。

## mex_run_eskf

### API概要

```matlab
% 初期化
handle = mex_run_eskf('init', obs, static_time, dt)

% ステップ実行
mex_run_eskf('step', handle, obs, k)

% 状態取得
state = mex_run_eskf('get_state', handle)

% メモリ解放
mex_run_eskf('free', handle)
```

### 初期化: `'init'`

**入力:**
- `obs`: 観測データ構造体（MATLAB構造体）
- `static_time`: 静止時間（秒、double）
- `dt`: タイムステップ（秒、double）

**出力:**
- `handle`: ハンドル（uint64）

**説明:**
- ESKF状態を初期化し、ハンドルを返す
- `obs`から初期状態を推定

### ステップ実行: `'step'`

**入力:**
- `handle`: ハンドル（uint64）
- `obs`: 観測データ構造体（MATLAB構造体）
- `k`: タイムステップインデックス（整数）

**出力:**
- なし（状態は内部で更新される）

**説明:**
- 予測ステップとセンサー更新を実行
- 加速度計、ジャイロ、磁力計、気圧計、GPSの更新を処理

### 状態取得: `'get_state'`

**入力:**
- `handle`: ハンドル（uint64）

**出力:**
- `state`: 状態構造体
  - `p`: 位置 [3×1] (double, メートル)
  - `v`: 速度 [3×1] (double, m/s)
  - `q`: クォータニオン [4×1] (double)
  - `euler`: オイラー角 [3×1] (double, 度)
  - `ba`: 加速度計バイアス [3×1] (double)
  - `bg`: ジャイロバイアス [3×1] (double)
  - `P`: 共分散行列 [15×15] (double, MATLAB column-major)

### メモリ解放: `'free'`

**入力:**
- `handle`: ハンドル（uint64）

**出力:**
- なし

**説明:**
- ハンドルに対応するESKF状態のメモリを解放

---

## mex_meukf_step_v2

### API概要

```matlab
[new_state, dbg_info, dbg_output] = mex_meukf_step_v2(prev_state, sensor_data, params)
```

### 入力

#### `prev_state` (構造体)
- `p`: 位置 [3×1] (double, メートル)
- `v`: 速度 [3×1] (double, m/s)
- `q`: クォータニオン [4×1] (double)
- `ba`: 加速度計バイアス [3×1] (double)
- `bg`: ジャイロバイアス [3×1] (double)
- `P`: 共分散行列 [15×15] (double, MATLAB column-major)

#### `sensor_data` (構造体)
- `accel`: 加速度計 [3×1] (double, m/s²)
- `gyro`: ジャイロ [3×1] (double, rad/s)
- `mag`: 磁力計 [3×1] (double)
- `gps_pos`: GPS位置 [3×1] (double, メートル、ECEF座標)
- `alt_baro`: 気圧高度 [1×1] (double, メートル)
- `dt`: タイムステップ [1×1] (double, 秒)
- `prev_mag`: 前回の磁力計値 [3×1] (double) **重要: 変化検出用**
- `prev_gps_pos`: 前回のGPS位置 [3×1] (double) **重要: 変化検出用**
- `prev_baro_alt`: 前回の気圧高度 [1×1] (double) **重要: 変化検出用**
- `update_accel`: 加速度計更新フラグ [1×1] (logical)
- `update_gyro`: ジャイロ更新フラグ [1×1] (logical)
- `update_mag`: 磁力計更新フラグ [1×1] (logical)
- `update_gps`: GPS更新フラグ [1×1] (logical)
- `update_baro`: 気圧計更新フラグ [1×1] (logical)
- `update_zupt`: ZUPT更新フラグ [1×1] (logical)

#### `params` (構造体)
- `g`: 重力ベクトル [3×1] (double, m/s²)
- `mag_ref`: 磁場参照ベクトル [3×1] (double)
- `noise_accel`: 加速度計ノイズ [3×1] (double, 分散)
- `noise_gyro`: ジャイロノイズ [3×1] (double, 分散)
- `noise_ba`: 加速度計バイアスノイズ [3×1] (double, 分散)
- `noise_bg`: ジャイロバイアスノイズ [3×1] (double, 分散)
- `noise_mag`: 磁力計ノイズ [3×1] (double, 分散)
- `noise_gps`: GPSノイズ [3×1] (double, 分散、メートル²)
- `noise_baro`: 気圧計ノイズ [1×1] (double, 分散)
- `noise_zupt`: ZUPTノイズ [3×1] (double, 分散)
- `alpha`: UKFパラメータ [1×1] (double, 通常 1e-3)
- `beta`: UKFパラメータ [1×1] (double, 通常 2)
- `kappa`: UKFパラメータ [1×1] (double, 通常 0)

### 出力

#### `new_state` (構造体)
- `prev_state`と同じ構造
- 更新後の状態値

#### `dbg_info` (オプション、[1×10] double)
- デバッグ情報配列

#### `dbg_output` (オプション、構造体)
- `pred_P`: 予測共分散 [15×15] (double, MATLAB column-major)
- `last_K`: カルマンゲイン [15×3] (double, MATLAB column-major)
- `last_S`: イノベーション共分散 [3×3] (double, MATLAB column-major)
- `last_S_inv`: イノベーション共分散の逆行列 [3×3] (double, MATLAB column-major)
- `last_H`: 観測行列 [3×15] (double, MATLAB column-major)
- `last_y`: イノベーション [3×1] (double)
- `last_y_len`: イノベーション長 [1×1] (double)
- `last_sensor_type`: 最後のセンサータイプ [1×1] (double)
- `input_update_gps`: GPS更新フラグ [1×1] (double)
- `input_noise_gps`: GPSノイズ [3×1] (double)

---

## 重要事項

### dt の扱い

- **dt は常に > 0 であること**
- dt = 0 の場合は予測ステップがスキップされる
- デフォルト: 0.01 秒
- `sensor_data.dt`に正しい値を設定すること

### 前フレーム値の扱い

**重要**: `prev_mag`, `prev_gps_pos`, `prev_baro_alt`は変化検出に使用されます。

- 初回呼び出し時は0で初期化可能
- 2回目以降は前回の値を渡すこと
- 値が変化しない場合、更新がスキップされる可能性がある

### 座標系

- **位置**: ECEF座標系（メートル）
- **速度**: ECEF座標系（m/s）
- **クォータニオン**: ボディ→ECEF変換
- **共分散行列**: MATLAB column-major形式

### 出力の対称化

共分散行列`P`は対称行列である必要があります。MEX関数内で対称化が行われます：

```cpp
// 対称化処理（mex_run_eskf_sensor_updates.hpp内）
for (int i = 0; i < 15; ++i) {
    for (int j = 0; j < 15; ++j) {
        out_P[i + j*15] = 0.5 * (new_P[i + j*15] + new_P[j + i*15]);
    }
}
```

### エラーハンドリング

- 無効なハンドル: `mexErrMsgIdAndTxt`でエラー
- 無効な入力: 検証後にエラー
- NaN/Inf検出: 警告またはスキップ

## 実装ファイル

- `kalman/cpp/MEX/mex_run_eskf.cpp`: メイン実装
- `kalman/cpp/Inc/MEX/mex_run_eskf_impl.hpp`: 実装関数群
- `kalman/cpp/Inc/MEX/mex_run_eskf_sensor_updates.hpp`: センサー更新処理
- `kalman/cpp/MEX/mex_meukf_step.cpp`: MEUKFステップ実装

## 参考

- [座標系変換仕様](COORDINATE_SYSTEM_SPEC.md)
- [失敗分析レポート](../../FAILURE_ANALYSIS_REPORT.md)
- [再統合チェックリスト](../../REINTEGRATION_CHECKLIST.md)


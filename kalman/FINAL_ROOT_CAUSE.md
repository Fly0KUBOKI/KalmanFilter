# 根本原因: Velocity RMSE 5.67 m/s の問題

## 発見された事実

### 1. 問題の症状
- **MATLAB実装**: Velocity RMSE = `0.5773 m/s`
- **MEX実装**: Velocity RMSE = `5.67 m/s` (約10倍)
- **差分**: 主にZ成分（垂直方向）の速度が大きく異なる

### 2. 詳細な分析結果

#### Step 100での比較
- **MATLAB**: `v = [0.010849, -0.005118, -0.009534]`
- **MEX**: `v = [0.010849, -0.005118, -1.958418]`
- **差分**: X, Y成分は完全に一致、Z成分のみ約2 m/s異なる

#### `mex_adaptive_predict`の出力検証
- `debug_mex_adaptive_predict.m`の実行結果:
  - `Expected dv`と実際の`dv`に差分がある
  - 特にZ成分（垂直方向）の速度更新が正しくない

### 3. 根本原因の仮説

#### 仮説1: `max_accel`の値が小さすぎる
- `kalman/cpp/src/ESKF/eskf_core.cpp`: `max_accel = 2.0`
- 重力加速度（約9.8 m/s²）を含む加速度が`max_accel * dt = 2.0 * 0.0025 = 0.005 m/s`に制限される
- これにより、速度更新が正しく行われない可能性

#### 仮説2: 座標系の違い
- NED座標系とENU座標系で重力の符号が異なる可能性
- `a_world + g`の計算が正しいかどうか確認が必要

#### 仮説3: `a_world`の計算が正しくない
- `a_world = R * a`で計算されているが、`a`は比力（specific force）
- 重力を加える必要があるが、座標系によって符号が異なる可能性

## 推奨される修正

### 修正1: `max_accel`の値を確認
- `kalman/cpp/src/ESKF/eskf_core.cpp`の`max_accel = 2.0`を確認
- 必要に応じて、`max_accel = 200.0`に変更（`kalman/cpp/ESKF/eskf_core.cpp`と同じ値）

### 修正2: 座標系の確認
- NED座標系では、重力は`[0, 0, 9.80665]`（下向きが正）
- `a_world + g`の計算が正しいかどうか確認

### 修正3: `a_world`の計算の確認
- `a_world = R * a`で計算されているが、`a`は比力（specific force）
- 重力を加える必要があるが、座標系によって符号が異なる可能性

## 次のステップ

1. `kalman/cpp/src/ESKF/eskf_core.cpp`の`max_accel`の値を確認
2. 座標系の確認（NED vs ENU）
3. `a_world + g`の計算が正しいかどうか確認


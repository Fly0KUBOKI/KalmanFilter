# MEUKF リアルタイム推定用 C++化計画

## 1. 概要
MATLABで実装されているMEUKF (Manifold Error UKF) アルゴリズムを、リアルタイムシステム（マイコン等）への移植を見据えてC++化する。
目標は更新周期1ms以内での動作と、センサー更新フラグに基づいた効率的な計算である。

## 2. 設計方針
- **ステートレス設計**: 計算クラスは状態を持たず、入力として「前回の状態」を受け取り、「今回の状態」を出力する。
- **入出力の構造体化**: 関数引数を整理し、`Input` 構造体と `Output` 構造体でやり取りする。
- **条件付き更新**: 各センサー（加速度、ジャイロ、磁気、GPS）の更新フラグを確認し、新しいデータがある場合のみフィルタ更新を行う。
- **データ型**: 基本的に `float` を使用。フラグ類は `uint8_t`。
- **ライブラリ**: 行列演算には軽量なテンプレートライブラリ（`fixed_matrix.hpp`）を使用し、動的メモリ確保を回避する。

## 3. ファイル構成
```text
kalman/cpp/
  ├── MEUKF/
  │   ├── meukf_types.hpp      // データ構造体定義 (State, SensorData, Params, Input, Output)
  │   ├── meukf_core.hpp       // 計算クラス宣言 (MEUKFCore)
  │   └── meukf_core.cpp       // 計算ロジック実装 (Predict, Update)
  └── mex_meukf_step.cpp       // MATLAB検証用MEXインターフェース
```

## 4. データ構造 (`meukf_types.hpp`)
### State (状態量)
- 位置 `p[3]`, 速度 `v[3]`, 姿勢 `q[4]`, バイアス `ba[3]`, `bg[3]`
- 誤差共分散 `P[15*15]`

### SensorData (入力)
- センサー値: `accel[3]`, `gyro[3]`, `mag[3]`, `gps[3]`
- 更新フラグ: `update_accel`, `update_mag`, `update_gps` (uint8_t)
- 経過時間: `dt`

### Params (パラメータ)
- ノイズパラメータ、重力加速度、基準磁気ベクトル、UKFパラメータ

## 5. 実装フェーズ

### Phase 1: 基盤実装
- ディレクトリ作成とヘッダーファイルの定義。
- `MEUKFCore` クラスの枠組み作成。
- 予測ステップ (`predict`) の移植（ESKFCoreから流用・統合）。

### Phase 2: 更新ロジック実装
- MEUKF姿勢更新 (`update_attitude_meukf`) のC++実装。
  - 誤差空間でのシグマ点生成
  - 多様体上での観測予測
  - 共分散更新
- GPS/気圧等のその他更新処理の実装。
- `step` 関数の実装（フラグ分岐ロジック）。

### Phase 3: MATLAB統合と検証
- `mex_meukf_step.cpp` の実装。
- ビルドスクリプト (`build_mex.m`) の更新。
- 比較検証スクリプト (`test_cpp_meukf.m`) の作成。
- `run_batch_10sets.m` を用いた性能評価と解析。

## 6. 検証項目
- **一致性**: MATLAB実装とC++実装の出力が数値的に一致すること（許容誤差範囲内）。
- **速度**: 1ステップあたりの計算時間が1ms以下であること。
- **安定性**: 長時間動作（バッチテスト）で発散しないこと。

# GitHub Copilot 指示 — KalmanFilter リポジトリ

このファイルは、このリポジトリで効率よく作業するためのAIエージェント向け具体的指示をまとめます。目的は、MATLAB + C++ MEX ハイブリッドの ESKF/UKF/MEUKF 実装を素早く理解し、安全に変更できるようにすることです。

- **ゴール**: ビルド（MEX）、シミュレーションの実行、C++コアの修正、MATLABラッパーの保守が主な作業領域。

## 主要コンポーネント（ビッグピクチャ）
- MATLAB orchestration: `kalman/run_simulation.m`, `kalman/GenerateData/sim_generate.m`, `kalman/ESKF/@ESKF/ESKF.m` がエントリーポイント。
- C++ MEX core: `kalman/cpp/` 以下に C++ 実装と `build/` スクリプトがある。ビルド成果物は `kalman/cpp/bin/*.mexw64`。
- データ: `kalman/GenerateData/` はシミュレーション入力と生成スクリプトを保持（`config_params.m`, `truth_data.csv`, `sensor_data.csv`）。

## 重要なワークフロー（必須コマンド例）
- MEX ビルド（Windows の例）:

```powershell
cd kalman/cpp/build
build_mex()    % MATLAB 内から呼ぶ（ビルドスクリプトを実行）
clear mex     % ビルド後は必ず実行して MEX を再読込み
```

- シミュレーション実行:

```matlab
run_simulation()           % フル実行（データ生成含む）
run_simulation(42, true)   % 乱数固定、データ生成をスキップ
```

## プロジェクト特有の慣習・注意点
- 状態ベクトルは 15 次元: `[p(3), v(3), q(4), ba(3), bg(3)]`（クォータニオンはスカラー先頭 `qw,qx,qy,qz`）。参照: `kalman/ESKF/@ESKF/ESKF.m`。
- 全 MEX 更新後は MATLAB 内で `clear mex` を必ず行う。古いバイナリが残ると推論結果が欠落する。
- 共分散は対称性を保つ（更新後に `P = (P + P')/2` を適用するパターンあり）。
- C++ 側でクォータニオン正規化を行う実装があるので、MATLAB 側で二重正規化しないよう注意。

## 典型的な編集パターン
- 小さなアルゴリズム変更: `kalman/cpp/MEUKF/` 内のコアファイルを編集 → `build_mex()` → `clear mex` → `run_simulation()`。
- MATLAB API 変更: `kalman/ESKF/@ESKF/*.m` を編集 → ユニット的に `test_phase1.m` または `run_simulation` で確認。

## 参照すべきファイル（素早く理解するため）
- 初見: [kalman/run_simulation.m](run_simulation.m)
- MEX ビルド: [kalman/cpp/build/build_mex.m](cpp/build/build_mex.m)
- C++ コアの例: [kalman/cpp/MEUKF/meukf_core.cpp](cpp/MEUKF/meukf_core.cpp) （コアアルゴリズム）
- ESKF MATLAB クラス: [kalman/ESKF/@ESKF/ESKF.m](ESKF/@ESKF/ESKF.m)
- データ生成: [kalman/GenerateData/config_params.m](GenerateData/config_params.m)

## デバッグと検証のヒント
- MEX 実行時の欠落推定が発生したら `run_mex_missing_estimation.txt`（`Results/`）を確認。
- 出力比較は `Results/estimation_matlab.csv` と `Results/estimation_mex.csv` を用いる。差分スクリプトや `plot_csv_file` を活用。
- C++ ビルドログは `kalman/cpp/build/build_log.txt` にあり、コンパイルオプションや警告が記録される。

## 禁止・慎重事項
- バイナリ（`cpp/bin/*.mexw64`）を直接編集しない。常にソースを修正してビルドする。 
- MATLAB セッションの MEX キャッシュを無視しない（`clear mex` を省くとデバッグが誤導される）。

---
もしこのファイルに追加してほしい具体的なチューニング項目（例: よく壊れるテスト、よく使うパラメータ）や、既存の社内ドキュメントを反映させたい箇所があれば教えてください。
# Kalman Filter Implementation - AI Coding Agent Guide

## Architecture Overview

Hybrid MATLAB + C++ MEX system for ESKF-based IMU/GPS/Mag/Baro sensor fusion.

- **MATLAB**: Orchestration, state management, data I/O (`run_simulation.m`, `@ESKF/`)
- **C++ MEX**: Core math - MEUKF updates, quaternion ops, UKF sigma points (`cpp/MEUKF/`, `cpp/MEX/`)
- **State**: 15-dim `[p(3), v(3), q(4), ba(3), bg(3)]` with 15×15 covariance P
- **Quaternion**: `[qw, qx, qy, qz]` scalar-first, body-to-NED frame

## Execution Flow

```
run_simulation.m → sim_generate.m → ESKF.m(init) → Main loop:
  predict(a,w) → sensor_updates('accel'|'mag'|'gps'|'baro') → reset('check')
```

All sensor updates go through `mex_meukf_step_v2` (C++ core).

## Key Workflows

### Build MEX
```matlab
cd kalman/cpp/build
build_mex()  % → cpp/bin/*.mexw64
```
**Critical**: Run `clear mex` after rebuild to reload binaries.

### Run Simulation
```matlab
run_simulation()           % Full run with data generation
run_simulation(42, true)   % Seed=42, skip data generation
run_batch_10sets()         % Batch testing (10 seeds)
```

### Sensor Update Frequencies
Controlled via modulo in main loop (not data rate):
- Accel: every 5 samples | Mag: every 25 | GPS: every 40 | Baro: every 50

## Project Patterns

### MATLAB `@ClassName` Folder
ESKF is one class split across files: `@ESKF/ESKF.m`, `@ESKF/predict.m`, `@ESKF/sensor_updates.m`, etc.

### MEX Interface (struct-based)
```matlab
state_in = struct('p', obj.p, 'v', obj.v, 'q', obj.q, 'ba', obj.ba, 'bg', obj.bg, 'P', obj.P);
sensor = struct('dt', dt, 'accel', a, ...);
[state_out, debug] = mex_meukf_step_v2(state_in, sensor, params);
```

### Change Detection
MATLAB buffers (`prev_accel`, `prev_mag`, etc.) skip updates when data unchanged.

## Key Files

| File | Purpose |
|------|---------|
| `kalman/run_simulation.m` | Entry point |
| `kalman/ESKF/@ESKF/ESKF.m` | State initialization from static period |
| `kalman/ESKF/@ESKF/sensor_updates.m` | Sensor dispatch + change detection |
| `kalman/cpp/MEUKF/meukf_core.cpp` | Core MEUKF algorithms (~1100 lines) |
| `kalman/cpp/MEX/mex_meukf_step.cpp` | MEX wrapper |
| `kalman/GenerateData/config_params.m` | Simulation parameters |

## Common Pitfalls

1. **MEX cache**: Always `clear mex` after C++ rebuild
2. **Quaternion normalization**: Enforce after operations (C++ does this automatically)
3. **Covariance symmetry**: Use `P = (P + P')/2` after updates
4. **Config regeneration**: Re-run `sim_generate()` after changing `config_params.m`

## Testing & Accuracy

- Output: `Results/estimation.csv` (compare with `GenerateData/truth_data.csv`)
- Targets: Position RMSE < 5m, Attitude < 1-2°
- Visualization: `plot_csv_file('Results/estimation.csv', 'GenerateData/truth_data.csv')`

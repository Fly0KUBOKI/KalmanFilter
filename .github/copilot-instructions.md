# GitHub Copilot 指示 — KalmanFilter

MATLAB実験フロントエンド + C++ MEXで計算ホットパスを高速化するハイブリッド実装です。

## 【最優先ルール】状態・型・同期

- **状態ベクトル順序（変更厳禁）**: `[p(3), v(3), q(4), ba(3), bg(3)]` 計15次元  
  - `q = [w, x, y, z]` スカラー先頭（ここがbugの出どころになりやすい）
  - 全C++コードで統一。別の順序を発見したら横断検索で要確認

- **型の厳密性**: MEXを通した変換で混在しやすい  
  - GPS座標系データのみ `double`（`lat`, `lon`, `alt`）  
  - 他の全センサー出力は `float32` で統一
  - 共分散行列 `P[15x15]` は `float32` column-major で返却
  - [CPP_INPUT_OUTPUT_SPEC.md](kalman/cpp/markdown/CPP_INPUT_OUTPUT_SPEC.md)で型マッピング確認必須

- **MEX-MATLAB同期ルール**:  
  - MEXバイナリ置換後は必ず `clear mex` してから再実行
  - 共分散行列は出力前に `P = (P+P')/2` で強制対称化

## アーキテクチャ【数分で理解する】

```
MATLAB frontend (実験・可視化)
├─ run_simulation.m        ← 単体テスト入口（observationデータ読込→初期化→ループ実行）
├─ run_batch_10sets.m      ← 回帰テスト（10seed並列実行→CSV比較）
└─ GenerateData/           ← syntheticセンサーデータ生成（ノイズ/外れ値付き）

C++ MEX実装層（計算エンジン）
├─ MEX/mex_run_eskf.cpp          ← メインエントリーポイント（init/step/get_state）
├─ Lib/ESKF/                     ← ESKF filter実装（15x15共分散更新）
├─ Lib/Common/Sensor/            ← センサー外れ値検出・ロバスト統計
├─ Lib/Quaternion/               ← 四元数演算（正規化・乗算）
└─ Lib/Matrix/                   ← 固定サイズ行列（Cholesky分解など）

データフロー：
obs.struct → mex_init() → handle → loop: mex_step() → mex_get_state() → state.struct → Results/
```

## 主要インタフェース【即座に使えるコマンド】

### MEX初期化 & ステップ実行
```cpp
// MATLAB側での呼び出しパターン（run_simulation.m参照）
handle = mex_run_eskf('init', obs, static_time, dt);
mex_run_eskf('step', handle, obs, k);
state = mex_run_eskf('get_state', handle);
mex_run_eskf('free', handle);
```

### 入出力構造体仕様
```matlab
% 入力: obs = struct
%   ax,ay,az: single[n]  加速度 (m/s²)
%   wx,wy,wz: single[n]  ジャイロ (deg/s)
%   mx,my,mz: single[n]  磁気 (nT)
%   pressure: single[n]  気圧 (Pa)
%   lat,lon,alt: double[n] GPS (rad/m)

% 出力: state = struct
%   p,v,ba,bg: single[3]   位置/速度/加速度bias/ジャイロbias
%   q: single[4]           四元数 [w,x,y,z]
%   euler: single[3]       オイラー角 (deg)
%   P: single[15x15]       共分散（column-major、対称化済み）
```

## ビルド・テストワークフロー

### MEX再ビルド（C++修正後）
```matlab
cd kalman/cpp/build
build_mex();                    % 全MEX再ビルド
% または
build_mex({'mex_run_eskf'});   % 特定ターゲットのみ

clear mex  % 重要: MATLAB内キャッシュをクリア
cd ../..
```

### 単体・回帰テスト
```matlab
% 1. 単体確認（差分が発生するか素早くチェック）
run_simulation(42, true);
% → Results/estimation_01.csv 出力
% innov_norm, maha_dist などのメトリクスをvisual inspection

% 2. 回帰テスト（10seedで統計的安定性確認）
run_batch_10sets();
% → Results/batch_10sets_results.mat, *.csv に結果集約
% → Results/log/ に詳細ログ保存（ログは上書きされない）

% 3. MEX vs MATLAB 差分分析
compare_mex_matlab_detailed();
% → RMSE, max_error, type_conflicts の詳細レポート出力
```

## 重要ファイルと役割

| ファイル | 役割 | 確認時機 |
|---------|------|--------|
| [kalman/cpp/build/build_mex.m](kalman/cpp/build/build_mex.m) | MEXコンパイルオプション・ソース統合 | C++ファイル追加時 |
| [kalman/run_simulation.m](kalman/run_simulation.m) | 単体テストループ | 初期化・ステップ挙動の確認 |
| [kalman/run_batch_10sets.m](kalman/run_batch_10sets.m) | 10seed並列実行・統計分析 | 回帰テスト/パリティ確認 |
| [kalman/cpp/MEX/mex_run_eskf.cpp](kalman/cpp/MEX/mex_run_eskf.cpp) | MEX entry point（init/step/get_state dispatcher） | MEX I/O仕様確認 |
| [kalman/cpp/Lib/ESKF/src/*.cpp](kalman/cpp/Lib/ESKF/src/) | ESKF状態更新・予測・リセット実装 | フィルタアルゴリズム修正 |
| [kalman/cpp/Lib/Common/Sensor/sensor_filter.hpp](kalman/cpp/Lib/Common/Sensor/sensor_filter.hpp) | 外れ値検出・ロバスト統計 | センサー異常値処理 |
| [kalman/cpp/markdown/CPP_INPUT_OUTPUT_SPEC.md](kalman/cpp/markdown/CPP_INPUT_OUTPUT_SPEC.md) | 型マッピング・配列レイアウト | MATLAB←→C++型変換debug時 |

## よくある落とし穴【10分で整理】

### ❌ 四元数・状態インデックスの不一致
```cpp
// OK: [w, x, y, z] with w leading
float q[4] = {state.q[0], state.q[1], state.q[2], state.q[3]};

// NG: [x, y, z, w] や混在
float q[4] = {state.q[1], state.q[2], state.q[3], state.q[0]}; // 致命的なバグ
```
対策：`normalize()` 関数内や `quat_*` 関数内でも順序を確認。

### ❌ float32/float64混在による累積誤差
```cpp
// NG: double(センサー) → float(内部) → double(出力) → 型変換ロス
double accel = obs.accel[0];  // double入力
float accel_f = (float)accel; // ←ここでprecision loss
// ...
double out = (double)accel_f; // さらにprecision loss

// OK: 入出力→内部処理まで型を統一
float accel = (float)obs.accel[0];  // 一度だけキャスト
```

### ❌ 共分散行列の非対称化
```cpp
// NG: 丸め誤差で P != P^T になる
P(i,j) = value1;
P(j,i) = value2; // 端数誤差で一致しないことが多い

// OK: 更新後に強制対称化
P = (P + P.transpose()) / 2.0f;
```

### ❌ MEXキャッシュ残存
```matlab
% NG: 古いMEX実行状態が残る
build_mex({'mex_run_eskf'});
run_simulation(42, true);  % ←新バイナリ使われない可能性

% OK: 明示的にクリア
build_mex({'mex_run_eskf'});
clear mex  % ←重要
run_simulation(42, true);
```

## 数値差の主原因チェックリスト

1. **型混在** → [TYPE_MIX_REPORT.md](kalman/cpp/TYPE_MIX_REPORT.md) で検索
2. **二重正規化** → `normalize(normalize(q))` 検出（grep: `normalize.*normalize`）
3. **クォータニオン順序** → [w,x,y,z] で統一確認（grep: `q\[0\]`, `q\[1\]`）
4. **共分散非対称** → `P = (P+P')/2` が実行されているか確認
5. **センサー外れ値フラグ** → 更新しているセンサー数が一致しているか
6. **行列メモリレイアウト** → column-major vs row-major の変換漏れ

## 探索キーワード【すぐに問題を特定】

```bash
# MEX関連
mex_run_eskf, mex_meukf_step_v2, mex_eskf_init

# フィルタ更新
eskf_predict, eskf_update, normalize, reset

# センサー処理  
OutlierDetector, sensor_filter, robust_statistics

# 型チェック
float32, double, mxSINGLE_CLASS, mxDOUBLE_CLASS

# 共分散
covariance, symmetric, Cholesky, P = (P+P')/2
```

## 【参考資料】
- PLAN.md — 進捗・成功指標  
- ROADMAP_TO_PHASE_13.md — 全体ロードマップ  
- kalman/cpp/FILE_DUPLICATION_REPORT.md — ファイル重複解析  
- kalman/cpp/Lib/README.md — ライブラリモジュール説明

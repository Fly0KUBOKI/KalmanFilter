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
// OK: [w, x, y, z] with w leading（標準実装）
float q[4] = {state.q[0], state.q[1], state.q[2], state.q[3]};
cquat::normalize_quat<float>(q);  // ← 必ずこの関数を使う

// NG: [x, y, z, w] や混在、または別の正規化関数
float q[4] = {state.q[1], state.q[2], state.q[3], state.q[0]}; // 致命的なバグ
utils::normalizeQuat(q);  // ❌ deprecated - cquat::normalize_quat 使用
```
対策：全ファイルで `cquat::normalize_quat<float/double>()` に統一（Phase 3で完成済み）。

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

---

## コード検索の実践パターン

### パターン1: 新しい正規化関数を追加したい
```bash
# 既存の正規化実装を一覧化
grep -rn "normalize\|symmetrize\|regularize" kalman/cpp/Lib --include="*.hpp" --include="*.cpp"
# → filter_mgmt.hpp, validation.hpp, utils.hpp に既に類似関数が存在の可能性 高

# 追加前に確認：Filter mgmt.hpp::normalize_covariance() で十分か？
grep -A 10 "normalize_covariance" kalman/cpp/Lib/Common/inc/filter_mgmt.hpp
```

### パターン2: センサー外れ値判定の流れを追跡
```bash
# 1. OutlierDetector の検出結果
grep -rn "OutlierDetector::is_outlier" kalman/cpp --include="*.hpp"

# 2. 検出結果が sensor_updates に渡されているか
grep -rn "is_outlier\|skip_update" kalman/cpp/MEX --include="*.hpp"

# 3. フィルタが実際に該当センサーをスキップしているか
grep -rn "if.*is_outlier\|should_skip" kalman/cpp/Lib --include="*.hpp"
```

### パターン3: float/double 境界の追跡
```bash
# MEX入力側（mxArrayToFloatArray 呼び出し）
grep -rn "mxArrayToFloat\|mxGetPr\|mxGetScalar" kalman/cpp/MEX --include="*.hpp"

# runner 内での変換
grep -rn "static_cast<float>\|double\>" kalman/cpp/Lib/ESKF/inc/eskf_runner.hpp

# 出力側（float→MATLAB 変換）
grep -rn "mxCreateNumericMatrix.*SINGLE\|setMxArray" kalman/cpp/MEX --include="*.hpp"
```

### パターン4: 状態ベクトル順序を確認
```bash
# 15次元状態ベクトル [p,v,q,ba,bg] のインデックスアクセス
grep -rn "state\\.p\\|state\\.v\\|state\\.q\\|state\\.ba\\|state\\.bg" kalman/cpp/Lib --include="*.hpp" | head -20

# 別の順序で実装されている可能性を検査
grep -rn "state\\.q\\[0\\].*state\\.p\\|state\\.ba\\[0\\].*state\\.v" kalman/cpp --include="*.cpp"
```

---

## デバッグ時の確認リスト（ステップバイステップ）

### 数値差が発生した場合
1. **MEXバイナリが最新か** → `clear mex` → `build_mex()` → 再実行
2. **型混在がないか** → `CPP_INPUT_OUTPUT_SPEC.md` で型確認，GPS以外 float32?
3. **四元数正規化は1度か** → 二重正規化（`normalize(normalize(q))`）検出
4. **共分散対称化は出力時か** → `P = (P+P')/2` が `mex_get_state` 内か確認
5. **メモリレイアウト** → row-major vs column-major の P[i*15+j] vs P[i+j*15] 確認
6. **初期化パラメータ** → GPS origin, Q_nominal, 静止時間が一致しているか

### 新しいセンサー更新関数を追加した場合
1. `mex_run_eskf_sensor_updates.hpp` に関数シグネチャを記述
2. `mex_run_eskf_impl.hpp` の `mex_step_impl()` 内で呼び出し追加
3. `sensor_preprocessor.hpp` で前処理（外れ値検出）を実施
4. `OutlierDetector::is_outlier()` で検出，フラグを返却
5. 更新関数内で `if (!is_outlier) { update(...) }`
6. ビルド & テスト：`run_simulation(42, true)` → `Results/estimation_01.csv` で検証

### MEX インターフェース変更時
1. `mex_run_eskf.cpp` の入出力構造体更新
2. `MEX/Inc/mex_eskf_common.hpp` の state/obs struct 定義更新
3. `mex_type_conversion.hpp` の変換関数更新
4. `mex_run_eskf_impl.hpp` の dispatcher 更新
5. `kalman/run_simulation.m` で obs struct 定義も更新
6. ビルド & テスト：`run_batch_10sets()` で回帰確認

## 【参考資料】
- PLAN.md — 進捗・成功指標  
- ROADMAP_TO_PHASE_13.md — 全体ロードマップ  
- kalman/cpp/FILE_DUPLICATION_REPORT.md — ファイル重複解析  
- kalman/cpp/Lib/README.md — ライブラリモジュール説明

---

## ライブラリ層の依存関係と設計パターン

### Lib/ 構造（7層アーキテクチャ）
```
LAYER 4: ESKF/MEUKF実装（ホットパス）
  ├─ eskf_runner.hpp — double↔float変換、state積分、センサー更新委譲
  ├─ eskf_core.hpp — 予測/更新のアルゴリズム実装
  └─ meukf_core.cpp — 代替フィルタ（1346行、要分割候補）

LAYER 3: フィルタ更新・統計（センサー処理）
  ├─ sensor_filter.hpp — 外れ値検出、Mahalanobis距離（831行）
  ├─ sensor_preprocessor.hpp — IMUノイズ，GPS→ENU変換
  └─ Common/filter_mgmt.hpp — ZUPT，発散検出，共分散保証

LAYER 2: 基本フィルタテンプレート
  ├─ KF/kalman_filter_core.hpp — 汎用KFコア
  ├─ EKF/ekf_core.hpp — 非線形観測モデル
  └─ UKF/ukf_core.hpp — Unscented変換（実装途上）

LAYER 1: ユーティリティ（型変換、数学）
  ├─ Matrix/fixed_matrix.hpp — 固定サイズ行列テンプレート
  ├─ Quaternion/quaternion_functions.hpp — 四元数演算
  ├─ Math/math_utils.hpp — ベクトル・スキュー対称化
  └─ interface.hpp — MATLAB⟷C++のstruct定義（Version 2準拠）
```

### 重要な設計パターン

#### 1. Lib内の二重変換を最小化
**Problem**: `eskf_runner.cpp` で毎フレーム `double[15x15] → float[15x15]` 変換が発生  
**Solution**: 
- 初期化時のみ変換（`mex_init`で `ESKFState` → `float` コピー）
- ホットパス（`mex_step`）では `float`のみ操作
- 出力時のみ `float → double` 変換（column-major→row-major 考慮）

#### 2. 共分散行列の対称化は出力時に一度だけ
```cpp
// 内部計算: P は計算誤差で P != P^T
// 出力時（mex_get_state()内）:
state.P_float[i,j] = (calc_P[i,j] + calc_P[j,i]) / 2.0f;
```
複数の正規化関数が存在（`symmetrize_covariance`, `normalize_covariance`）だが、  
**出力直前に1度だけ実施**が原則。

#### 3. インクルードパス統一（Phase 3完了）
```cpp
// OK: 相対パス ../../ に統一
#include "../../Matrix/fixed_matrix.hpp"
#include "../../Quaternion/quaternion_functions.hpp"
#include "../../Common/inc/Math/math_utils.hpp"

// NG: ../Lib/ は重複、避けよ
#include "../Lib/Quaternion/quaternion_functions.hpp"  ❌
```

### Lib の既知の問題と対応

| 問題 | ファイル | 状態 | 対応 |
|------|---------|------|------|
| メモリレイアウト混在 | MEX層 | ⚠️ | row-major(MATLAB) ↔ column-major(C++) の変換を `mex_type_conversion.hpp` 参照 |
| float/double 境界 | eskf_runner, MEX層 | ✅ Phase 3済み | GPS→frame変換時のみ，他は float統一 |
| quaternion normalize 重複 | utils.hpp, meukf, ESKF | ✅ Phase 3済み | 全て `cquat::normalize_quat<T>()` に統一 |
| sensor_filter 巨大 | sensor_filter.hpp (831行) | ⚠️ 将来分割予定 | EMA/Biquad/Outlier/Robust を個別ヘッダに分割予定 |
| meukf_core 超長 | meukf_core.cpp (1346行) | ⚠️ 将来分割予定 | predict/update関数を .cpp ファイルに分割予定 |

---

## MEX層の入出力設計パターン

### mex_run_eskf() dispatcher の3つのモード

#### init モード
```cpp
// 入力: obs (struct with sensor arrays), static_time, dt
// 1. ESKFState 初期化 (double精度)
// 2. 最初の static_time秒のセンサーで Roll/Pitch/Yaw推定
// 3. GPS originを設定，共分散P,Q_nominalを初期化
// 出力: handle (状態ポインタをuint64で返却)
```

#### step モード
```cpp
// 入力: handle, obs (k番目のセンサー), k (step number)
// 1. obs[k] を float にキャスト（GPSのみ double→ENU変換）
// 2. ESKFCore::integrate_nominal() で RK2積分
// 3. センサー更新（加速度→Roll/Pitch, 磁気→Yaw, GPS/気圧→Position）
// 4. 状態注入（誤差状態dx→名義状態修正）
// 出力: handle のみ（状態は内部保持）
```

#### get_state モード
```cpp
// 入力: handle
// 出力: state (struct)
// 1. p,v を float[3] で返却（相対ENU座標系）
// 2. q を float[4] [w,x,y,z] で返却
// 3. euler を deg で返却
// 4. P[15x15] を float で返却（強制対称化：P = (P+P')/2）
// ⚠️ column-major (C++) → row-major (MATLAB) 変換が隠れている
```

### 型マッピング（要暗記）
```
Input obs struct:
  ax,ay,az, wx,wy,wz, mx,my,mz, pressure → float[k]
  lat,lon,alt → double[k]  （GPS専用double）

Output state struct:
  p,v,ba,bg,q,euler → single (float32)
  P → single[15x15]，column-major，対称化済み
```

---

## C++ エラーの出所トップ 10

1. **状態ベクトル順序混乱** (`p,v,q,ba,bg` 以外のセッション) → grep `state.q[0]`, `state.p[0]`
2. **Float/Double混在** → [CPP_INPUT_OUTPUT_SPEC.md](kalman/cpp/markdown/CPP_INPUT_OUTPUT_SPEC.md) で型確認
3. **四元数正規化忘れ** → `cquat::normalize_quat()` は全積分後に必須
4. **共分散非対称** → 出力前に `P = (P+P')/2`
5. **メモリレイアウト** → `P[i,j]` が row-major と column-major で反転
6. **インクルードパス** → `#include "../Lib/...` は禁止，`../../...` 使用
7. **センサー外れ値フラグ** → `OutlierDetector` の判定結果が `sensor_updates` に反映されているか確認
8. **ZUPT条件の閾値** → `filter_mgmt.hpp::check_zupt_condition()` の引数確認
9. **MEXキャッシュ残存** → ビルド後は必ず `clear mex`
10. **未初期化エラー** → 初期化済み ( `is_initialized == true`) の確認

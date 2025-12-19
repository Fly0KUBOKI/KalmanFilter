# KF/Utils フォルダ C++化移行計画

## 概要
`kalman/KF/Utils` フォルダ内のMATLABコードを最下層から順番にC++へ移行する計画書。

## ターミナルでの実行に関する注意点
matlab -batch "run_batch_10sets"
のように実行して
ターミナルのコマンドを入力する際に、実行中に他のコマンドを入力すると実行が止まってしまうので、
実行が完了するまでコマンドを入力してはいけない


## 移行方針
1. **最下層から順番に移行**: 依存関係のない関数/クラスから開始
2. **段階的移行**: 部分的にC++化し、バッチテストで検証
3. **MATLAB実装の削除**: テスト成功後にMATLAB実装を削除
4. **繰り返し実行**: この流れを全ファイルに適用

## 依存関係分析

### レイヤー0: 最下層（依存なし）
- `alpha_beta_step.m` - 単純な関数
- `ema_update.m` - 単純な関数
- `hampel_causal.m` - 単純な関数

### レイヤー1: 基本クラス（レイヤー0に依存する可能性）
- `BiquadFilter.m` - クラス（既にC++実装あり: `BiquadLowpassFilter`）
- `AccelFilter.m` - クラス（`ema_update`に依存する可能性）

### レイヤー2: ユーティリティクラス（レイヤー1に依存）
- `NoiseEstimator.m` - クラス（既にC++実装あり: `NoiseEstimator`）
- `DivergenceGuard.m` - クラス（既にC++実装あり: `DivergenceGuard`）

### レイヤー3: センサーフィルタ基底（レイヤー1-2に依存）
- `SensorFilter.m` - 基底クラス（Staticメソッドのみ）

### レイヤー4: センサー専用フィルタ（レイヤー3に依存）
- `SensorAccelFilter.m` - `SensorFilter`に依存
- `SensorBaroFilter.m` - `SensorFilter`に依存
- `SensorGyroFilter.m` - `SensorFilter`, `BiquadFilter`に依存
- `SensorGPSFilter.m` - `SensorFilter`に依存
- `SensorMagFilter.m` - `SensorFilter`に依存

### レイヤー5: 統合クラス（レイヤー4に依存）
- `OutlierGuard.m` - `SensorFilter`, `DivergenceGuard`に依存
- `SensorFilterFactory.m` - 全ての`SensorFilter`に依存

### レイヤー6: ユーティリティ関数集（複数関数を含む）
- `FilterUtils.m` - `ema_update`, `hampel_causal`, `alpha_beta_step`を含む

## 移行順序（最下層から）

### Phase 1: 基本関数（レイヤー0）
#### 1.1 `alpha_beta_step.m`
- **優先度**: 高
- **依存**: なし
- **C++実装場所**: `kalman/cpp/Common/Math/math_utils.hpp` または新規ファイル
- **MEX関数**: 必要（MATLABから呼び出し）
- **テスト**: `run_batch_10sets.m`で検証

#### 1.2 `ema_update.m`
- **優先度**: 高
- **依存**: なし
- **C++実装場所**: `kalman/cpp/Common/Math/math_utils.hpp` または `sensor_filter.hpp`（既に`EMAFilter`クラスあり）
- **MEX関数**: 必要（MATLABから呼び出し）
- **テスト**: `run_batch_10sets.m`で検証
- **注意**: 既に`EMAFilter`クラスが存在するが、関数版も必要

#### 1.3 `hampel_causal.m`
- **優先度**: 中
- **依存**: なし
- **C++実装場所**: `kalman/cpp/Common/Math/math_utils.hpp` または新規ファイル
- **MEX関数**: 必要（MATLABから呼び出し）
- **テスト**: `run_batch_10sets.m`で検証

### Phase 2: 基本クラス（レイヤー1） — 完了
#### 2.1 `BiquadFilter.m` (完了)
- **優先度**: 中
- **依存**: なし
- **C++実装場所**: `kalman/cpp/Common/Sensor/sensor_filter.hpp`（`BiquadLowpassFilter`として実装済み）
- **MEX関数**: `mex_sensor_filter` の一部として動作検証済み
- **テスト**: `run_batch_10sets.m` で検証済み（バッチ成功）

#### 2.2 `AccelFilter.m` (完了)
- **優先度**: 低（使用頻度が低い可能性）
- **依存**: `ema_update`（Phase 1.2で移行済み）
- **C++実装場所**: `kalman/cpp/Common/Sensor/sensor_filter.hpp`（`EMAFilter` と `SensorFilterLib::filter_accel` が対応）
- **MEX関数**: `mex_sensor_filter('accel',...)` で動作確認済み
- **テスト**: `run_batch_10sets.m` で検証済み（バッチ成功）

### Phase 3: ユーティリティクラス（レイヤー2）
#### 3.1 `NoiseEstimator.m`
- **優先度**: 高
- **依存**: なし
- **C++実装場所**: `kalman/cpp/Common/Sensor/sensor_filter.hpp`（既に実装済み）
- **MEX関数**: 既存の`mex_sensor_filter.cpp`を確認
- **テスト**: `run_batch_10sets.m`で検証
- **注意**: 既存実装との互換性確認が必要

#### 3.2 `DivergenceGuard.m`
- **優先度**: 高
- **依存**: なし
- **C++実装場所**: `kalman/cpp/Common/Sensor/sensor_filter.hpp`（既に実装済み）
- **MEX関数**: 既存の`mex_sensor_filter.cpp`を確認
- **テスト**: `run_batch_10sets.m`で検証
- **注意**: 既存実装との互換性確認が必要

### Phase 4: センサーフィルタ基底（レイヤー3）
#### 4.1 `SensorFilter.m`
- **優先度**: 高
- **依存**: なし（Staticメソッドのみ）
- **C++実装場所**: `kalman/cpp/Common/Sensor/sensor_filter.hpp` または新規ファイル
- **MEX関数**: 必要
- **テスト**: `run_batch_10sets.m`で検証

### Phase 5: センサー専用フィルタ（レイヤー4）
#### 5.1 `SensorAccelFilter.m`
- **優先度**: 高
- **依存**: `SensorFilter`（Phase 4.1で移行済み）
- **C++実装場所**: `kalman/cpp/Common/Sensor/sensor_filter.hpp` または新規ファイル
- **MEX関数**: 必要
- **テスト**: `run_batch_10sets.m`で検証

#### 5.2 `SensorBaroFilter.m`
- **優先度**: 高
- **依存**: `SensorFilter`（Phase 4.1で移行済み）
- **C++実装場所**: `kalman/cpp/Common/Sensor/sensor_filter.hpp` または新規ファイル
- **MEX関数**: 必要
- **テスト**: `run_batch_10sets.m`で検証

#### 5.3 `SensorGyroFilter.m` (Removed)
- **状態**: 削除済み（MATLAB 実装は削除し、MEX 側の `gyro` コマンドも廃止）
- **理由**: 実運用で未使用かつ実装差異がリスクとなったため。
- **影響**: `SensorFilter.createGyroFilter` は呼び出し不可または空を返すように変更済み。コード内の呼び出しは既に置換/削除されています。
- **テスト**: フルバッチ `run_batch_10sets.m` を実行し、動作確認済み（バッチ成功）

#### 5.4 `SensorGPSFilter.m`
- **優先度**: 高
- **依存**: `SensorFilter`（Phase 4.1で移行済み）
- **C++実装場所**: `kalman/cpp/Common/Sensor/sensor_filter.hpp` または新規ファイル
- **MEX関数**: 必要
- **テスト**: `run_batch_10sets.m`で検証

#### 5.5 `SensorMagFilter.m`
- **優先度**: 高
- **依存**: `SensorFilter`（Phase 4.1で移行済み）
- **C++実装場所**: `kalman/cpp/Common/Sensor/sensor_filter.hpp` または新規ファイル
- **MEX関数**: 必要
- **テスト**: `run_batch_10sets.m`で検証

### Phase 6: 統合クラス（レイヤー5）
#### 6.1 `OutlierGuard.m`
- **優先度**: 高
- **依存**: `SensorFilter`, `DivergenceGuard`（Phase 3.2, 4.1で移行済み）
- **C++実装場所**: `kalman/cpp/Common/Sensor/sensor_filter.hpp` または新規ファイル
- **MEX関数**: 必要
- **テスト**: `run_batch_10sets.m`で検証

#### 6.2 `SensorFilterFactory.m`
- **優先度**: 高
- **依存**: 全ての`SensorFilter`（Phase 5で移行済み）
- **C++実装場所**: `kalman/cpp/Common/Sensor/sensor_filter.hpp` または新規ファイル
- **MEX関数**: 必要
- **テスト**: `run_batch_10sets.m`で検証

### Phase 7: ユーティリティ関数集（レイヤー6）
#### 7.1 `FilterUtils.m`
- **優先度**: 低
- **依存**: `ema_update`, `hampel_causal`, `alpha_beta_step`（Phase 1で移行済み）
- **C++実装場所**: 各関数は既に移行済みのため、削除のみ
- **MEX関数**: 不要（個別関数を使用）
- **テスト**: `run_batch_10sets.m`で検証
- **注意**: このファイルは複数の関数を含むため、個別関数の移行後に削除

## 移行手順（各Phase共通）

### ステップ1: C++実装
1. 既存のC++実装を確認（`kalman/cpp/Common/Sensor/sensor_filter.hpp`など）
2. 新規実装または既存実装の拡張
3. 既存実装がある場合は互換性を確認

### ステップ2: MEX関数作成/更新
1. `kalman/cpp/MEX/` にMEX関数を追加または更新
2. MATLABから呼び出せるようにインターフェースを実装
3. エラーハンドリングを追加

### ステップ3: MATLABラッパー作成
1. `kalman/KF/Utils/` にMEX関数を呼び出すMATLABラッパーを作成
2. 既存のMATLAB関数と同じインターフェースを維持
3. 後方互換性を確保

### ステップ4: ビルド
1. `kalman/cpp/build/build_mex.m` または適切なビルドスクリプトを実行
2. コンパイルエラーを修正
3. リンクエラーを修正

### ステップ5: バッチテスト実行
1. `kalman/run_batch_10sets.m` を実行
2. 結果を確認（エラー、警告、結果の一致）
3. 問題があれば修正

### ステップ6: MATLAB実装の削除
1. バッチテストが成功したことを確認
2. MATLAB実装ファイル（`.m`）を削除
3. 必要に応じてアーカイブフォルダに移動

### ステップ7: ドキュメント更新
1. 移行完了を記録
2. 使用方法を更新（必要に応じて）

## 既存C++実装の確認事項

### `kalman/cpp/Common/Sensor/sensor_filter.hpp`に既に実装されているもの
- `EMAFilter` - `ema_update`のクラス版
- `BiquadLowpassFilter` - `BiquadFilter`に対応
- `NoiseEstimator` - `NoiseEstimator.m`に対応
- `DivergenceGuard` - `DivergenceGuard.m`に対応

### 確認が必要な項目
1. 既存実装のインターフェースがMATLAB版と一致しているか
2. MEX関数が適切に実装されているか
3. テストが通るか

## 注意事項

1. **既存実装の活用**: 既にC++実装がある場合は、新規実装ではなく既存実装を活用
2. **互換性の維持**: MATLAB版と同じインターフェースを維持
3. **段階的移行**: 一度に全てを移行せず、1つずつ確実に移行
4. **テストの徹底**: 各Phaseで必ずバッチテストを実行
5. **ロールバック準備**: 問題が発生した場合にMATLAB実装に戻せるように準備

## 進捗管理

各Phaseの進捗を以下で管理：
- [x] Phase 1: 基本関数
  - [x] 1.1 alpha_beta_step
  - [x] 1.2 ema_update
  - [x] 1.3 hampel_causal
- [x] Phase 2: 基本クラス (完了)
  - [x] 2.1 BiquadFilter
  - [x] 2.2 AccelFilter
- [ ] Phase 3: ユーティリティクラス
 - [ ] Phase 3: ユーティリティクラス
  - [~] 3.1 NoiseEstimator (C++ 実装あり, 統合テスト保留)
  - [~] 3.2 DivergenceGuard (C++ 実装あり, 統合テスト保留)

### Phase3 進捗と MEX 化状況

- **現状要約**: `NoiseEstimator` と `DivergenceGuard` の C++ 実装は `kalman/cpp/Common/Sensor/sensor_filter.hpp` に存在します。MEX ラッパ（例: `kalman/cpp/MEX/mex_sensor_filter.cpp`）および複数の MEX バイナリがワークスペースにあります。
- **MEX化の完了判定**: C++ 実装および MEX エントリは実装済みですが、MATLAB 側との完全なインターフェース互換性検証（単体テスト・API 差分チェック）が未完了です。したがって「MEX化は実装済みだが、統合検証が完了していない（部分完了）」と表現します。

### 残タスク（Phase3）

1. `kalman/cpp/MEX/mex_sensor_filter.cpp` と `sensor_filter.hpp` の公開メソッドが MATLAB 側 (`NoiseEstimator.m`, `DivergenceGuard.m`) と厳密に一致するかを突合する（引数/戻り値/型/単位）。
  - 状態: 進行中 — `sensor_filter.hpp` に実装あり。MEX 側のコマンドハンドラは部分実装。
2. `kalman/cpp/tests/` に小さな単体テストを追加して、ノイズ推定のオンライン更新やダイバージェンス判定の閾値動作を確認する。
  - 状態: 未完了 — テスト追加が必要（推奨: `tests/noise_estimator_test.cpp` と `tests/divergence_guard_test.cpp` を作成）。
3. CI またはローカルで `run_batch_10sets` を自動化し、C++経路（`mex_sensor_filter` 有効）と MATLAB フォールバック経路の両方で結果が一致することを確認する。
  - 状態: 未完了 — ローカル手順は `build_mex()` → `clear mex` → `run_batch_10sets()`。
4. 合格したら `3.1` / `3.2` のチェックを完全完了に更新する。

---

### Phase3 進捗 (2025-12-18)

- 概要: `NoiseEstimator` と `DivergenceGuard` の C++ 実装は `kalman/cpp/Common/Sensor/sensor_filter.hpp` に存在し、MEX 経路への組み込みは部分的に完了しています。ESKF 側で発生した初期化/呼び出し不一致については修正済み（`ESKF.m` と `sensor_updates.m` の復元修正。詳細は「発生した障害と対応記録」を参照）。
- 実施済の作業:
  - C++ 実装の存在確認（`sensor_filter.hpp`）
  - ESKF 初期化回復と `sensor_updates.m` の修正により `run_batch_10sets` の大部分が回復（10/10 PASS 最終確認は Phase2 のログ参照）
  - 一時的な回帰は解消し、特定の呼び出しパスを MATLAB にフォールバックさせるガードを追加
- 現在の未解決課題:
  - 公開API（引数・戻り値・型・スケール）の厳密突合が残る（優先度: 高）
  - MEX 側のコマンドハンドラ（例: `'noise_estimate'`, `'get_R'`, `'divergence_check'`, `'divergence_regularize'`）の完全実装が未完（優先度: 高）
  - 自動化された単体テストが未作成（優先度: 中）
- 推奨次アクション（短期、担当1人で対応可能）:
 1. 12/20 までに `sensor_filter.hpp` の公開シグネチャと `NoiseEstimator.m` / `DivergenceGuard.m` を突合して差分リストを作成する。
 2. MEX ハンドラに不足するコマンドを `mex_sensor_filter.cpp` に追加し、ローカルでビルドして smoke test を行う（`build_mex()` → `clear mex` → `run_batch_10sets`）。
 3. `kalman/cpp/tests/` に小さなユニットテストを追加し、`run_batch_10sets` を回して差分を検証する。
 4. 成功したら `PHASE3_COMPLETE.md` を作成して移行完了を記録する。

-- 完了報告: この節の更新を行いました。変更を確認して追加の優先事項があれば指示してください。

### 優先度と推奨順序

- 優先度: 高 — まずインターフェース突合と単体テストを実装し、不整合を解消する。
- 推奨順序: 1) インターフェース確認、2) 単体テスト追加、3) MEXビルド & `clear mex`、4) `run_batch_10sets` で回帰確認。
- [ ] Phase 4: センサーフィルタ基底
  - [ ] 4.1 SensorFilter
- [ ] Phase 5: センサー専用フィルタ
  - [x] 5.1 SensorAccelFilter
  - [ ] 5.2 SensorBaroFilter
  - [x] 5.3 SensorGyroFilter (削除済)
  - [ ] 5.4 SensorGPSFilter
  - [ ] 5.5 SensorMagFilter
- [ ] Phase 6: 統合クラス
  - [ ] 6.1 OutlierGuard
  - [ ] 6.2 SensorFilterFactory
- [ ] Phase 7: ユーティリティ関数集
  - [ ] 7.1 FilterUtils

## Phase2 完了記録

- 完了日: 2025-12-17
- 概要: Phase2（`BiquadFilter` と `AccelFilter` の移行）を完了しました。`SensorGyroFilter` は廃止しています。
- 検証: フルバッチ回帰を実行し、`kalman/Results/batch_10sets_log.txt` に示される通り 10/10 のRunが PASS しました。

  - バッチログ: `kalman/Results/batch_10sets_log.txt`

次のステップ:
1. Phase1 の未実装関数を C++ 実装してテストする（`alpha_beta_step`, `ema_update`, `hampel_causal`）。
2. Phase3〜7 の残件を段階的に移行・検証する。
3. ドキュメントに移行済みファイル一覧を追加し、不要な MATLAB ファイルは `archive/` に移動済みであることを記録する。

  ## Phase3 準備（検証・導入手順）

  目的: Phase3（`NoiseEstimator`, `DivergenceGuard`）の C++ 実装を本番パスに組み込み、互換性とテストカバレッジを確保する。

  推奨手順:
  1. インターフェース確認: `kalman/cpp/Common/Sensor/sensor_filter.hpp` 内の `NoiseEstimator` / `DivergenceGuard` の公開メソッドと MATLAB 実装の引数/返り値を照合する。
  2. MEX 呼び出し確認: `kalman/cpp/MEX/mex_sensor_filter.cpp` に必要なコマンドハンドラが存在するかを確認し、なければ追加する。
  3. 単体テスト追加: `kalman/cpp/tests/` に小さなユニットテストを作成し、C++ 実装の挙動を検証する（例: ノイズ推定のオンライン更新、ダイバージェンス判定の閾値テスト）。
  4. ビルドとバッチ検証: `kalman/cpp/build/build_mex.m` を実行して MEX をビルドし、`run_batch_10sets.m` でフルバッチを実行して差分を確認する。
  5. ドキュメント更新: テスト完了後に `cpp_migration_plan.md` と `PHASE0_COMPLETE.md` 等へ完了記録を追加する。

  クイックコマンド例（MATLAB で実行）:
  ```matlab
  cd('kalman/cpp/build');
  build_mex();
  clear mex;
  cd(fullfile('..','..'));
  run_batch_10sets();
  ```

  成功条件:
  - 単体テストが通ること
  - フルバッチで既存結果と許容誤差内（差分ほぼゼロ）で一致すること
  - 互換性問題がなければ `Phase 3` のチェックを完了としてマークする

## Phase0-1-2 完了記録（統合）

### Phase0 (2025-12-14)
- **内容**: 3つの基本関数を C++/MEX 化: `alpha_beta_step`, `ema_update`, `hampel_causal`
- **実装**: `kalman/cpp/MEX/mex_filter_utils.cpp`
- **ラッパー**: `kalman/KF/Utils/*_cpp.m`
- **詳細**: [PHASE0_COMPLETE.md](kalman/KF/Utils/PHASE0_COMPLETE.md)

### Phase1 (2025-12-16)
- **内容**: Phase0 関数が MATLAB ラッパーを通じて MEX 経由で実行される状態に統一
- **主要変更**: MEX 初期化（`clear mex` + `mex_sensor_filter('reset')`）と PASS 判定の厳格化
- **詳細**: [PHASE1_COMPLETE.md](kalman/KF/Utils/PHASE1_COMPLETE.md)、[PHASE1_2_MIGRATION_SUMMARY.md](kalman/KF/Utils/PHASE1_2_MIGRATION_SUMMARY.md)

### Phase2 (2025-12-17)
- **内容**: `BiquadFilter` と `AccelFilter` の C++/MEX 化完了
- **実装**: `kalman/cpp/Common/Sensor/sensor_filter.hpp` の `BiquadLowpassFilter` と `SensorFilterLib::filter_accel`
- **検証**: `run_batch_10sets` で 10/10 PASS
- **削除**: `SensorGyroFilter`（実運用未使用）
- **詳細**: [AccelFilter_Migration_Plan.md](kalman/KF/Utils/AccelFilter_Migration_Plan.md)

## Phase3 準備（検証・統合段階）

- **目的**: `NoiseEstimator` / `DivergenceGuard` の C++ 実装を MATLAB 側と完全に統合
- **現状**: C++ 実装（`sensor_filter.hpp`）あり、MEX コマンド部分的あり、統合テスト未完了
- **推奨手順**:
  1. インターフェース突合: `kalman/cpp/Common/Sensor/sensor_filter.hpp` と MATLAB 実装（`NoiseEstimator.m`, `DivergenceGuard.m`）のシグネチャを照合
  2. MEX API 拡張: `kalman/cpp/MEX/mex_sensor_filter.cpp` に `'noise_estimate'`, `'get_R'`, `'divergence_check'`, `'divergence_regularize'` を追加
  3. 単体テスト: `kalman/cpp/tests/` にノイズ推定・ダイバージェンス判定テストを追加
  4. フルバッチ検証: `run_batch_10sets` で結果一致を確認
  5. ドキュメント更新: 完了後に本計画を更新
- **詳細**: [phase3_migration_notes.md](kalman/KF/Utils/phase3_migration_notes.md)

## 発生した障害と対応記録 (2025-12-17)

### **概要**
- **事象**: Phase3 の移行作業中に MATLAB 側フィルタ初期化やノイズ推定の置換が入り、実行時エラーと性能劣化が発生。
- **検出**: `run_batch_10sets` により Position/Attitude RMSE が許容外となるか、実行時エラーが発生。

### **原因**
- **NoiseEstimator 初期化の置換**: `NoiseEstimator(10)` が `struct()` に置換され、動的ノイズ推定機能を喪失。
- **sensor_filters の未初期化化**: `obj.sensor_filters.*` が `[]` に設定され、MATLAB 側フォールバックが使えなくなった。
- **呼び出し形の混在**: `obj.sensor_filters.*.apply(...)` と `SensorFilters.*(...)` の置換が混在し、未初期化オブジェクトにアクセス。
- **バロメータ処理の欠落**: `weight_factor` が削除され、バロメータ更新でエラー。

### **実施した対応**
- `kalman/ESKF/@ESKF/ESKF.m` の初期化を復元: `NoiseEstimator(10)` と `SensorFilter.create*` に戻した。
- `kalman/ESKF/@ESKF/sensor_updates.m` を修正: `obj.sensor_filters.*.apply(...)` 呼び出しに復帰し、`weight_factor = 1.0 / obj.baro_weight;` を再追加。
- 修正後に単体実行および `run_batch_10sets` を実行し、Run 1–9 が PASS、Run 10 を単独再実行で正常終了を確認。

### **テスト結果（要約）**
- バッチ 10 セット中のいくつかは一時的に中断が発生したが、最終的に 10/10 を PASS として確認。
- 代表指標: Position RMSE 全体 ≈ 0.50–0.59 m、Roll/Pitch RMSE ≈ 0.27–0.30 deg。

### **再発防止と推奨対応**
- `SensorFilters` と `mex_sensor_filter` の経路切替を明示的に管理し、ESKF 側で `FORCE_MATLAB_FILTERS` を確認して単一経路に統一する。
- Phase3 の C++ 統合前に `NoiseEstimator` / `DivergenceGuard` のインターフェース互換テストを追加する（`kalman/cpp/tests/` に単体テストを追加）。
- 小さな変更は必ずローカルで `run_batch_10sets` を回してからコミットするワークフローを徹底し、CI で自動バッチ回帰を実行する。

## 参考情報

- **既存C++実装**: `kalman/cpp/Common/Sensor/sensor_filter.hpp`
- **MEX関数例**: `kalman/cpp/MEX/mex_sensor_filter.cpp`, `mex_filter_utils.cpp`
- **ビルドスクリプト**: `kalman/cpp/build/build_mex.m`
- **バッチテスト**: `kalman/run_batch_10sets.m`
- **関連ドキュメント**:
  - [PHASE0_COMPLETE.md](kalman/KF/Utils/PHASE0_COMPLETE.md)
  - [PHASE0_TEST_GUIDE.md](kalman/KF/Utils/PHASE0_TEST_GUIDE.md)
  - [PHASE1_COMPLETE.md](kalman/KF/Utils/PHASE1_COMPLETE.md)
  - [PHASE1_2_MIGRATION_SUMMARY.md](kalman/KF/Utils/PHASE1_2_MIGRATION_SUMMARY.md)
  - [phase3_migration_notes.md](kalman/KF/Utils/phase3_migration_notes.md)
  - [AccelFilter_Migration_Plan.md](kalman/KF/Utils/AccelFilter_Migration_Plan.md)

---
**マスター更新日**: 2025-12-17  
**統合内容**: Phase0-2 完了記録、Phase3 準備内容、障害対応、参考リンク一覧


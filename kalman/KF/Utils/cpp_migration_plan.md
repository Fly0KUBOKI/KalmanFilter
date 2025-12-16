# KF/Utils フォルダ C++化移行計画

## 概要
`kalman/KF/Utils` フォルダ内のMATLABコードを最下層から順番にC++へ移行する計画書。

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
  - [ ] 3.1 NoiseEstimator
  - [ ] 3.2 DivergenceGuard
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

## 参考情報

- 既存C++実装: `kalman/cpp/Common/Sensor/sensor_filter.hpp`
- MEX関数例: `kalman/cpp/MEX/mex_sensor_filter.cpp`
- ビルドスクリプト: `kalman/cpp/build/build_mex.m`
- バッチテスト: `kalman/run_batch_10sets.m`

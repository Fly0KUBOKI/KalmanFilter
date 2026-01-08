# UKF ライブラリ抽出の段階的実装計画

## 現状の整理

### ✅ 完了したこと
1. Git復元により、MEXビルドを正常な状態に戻した（2 MEX成功）
2. 新しいUKFライブラリファイルを作成
   - `kalman/cpp/Lib/UKF/inc/ukf_generic.hpp` - 汎用UKFテンプレート
   - `kalman/cpp/Lib/UKF/inc/ukf_utils.hpp` - Cholesky、シグマポイント生成
   - `kalman/cpp/Lib/MEUKF/inc/meukf_observation_models.hpp` - 観測モデル
3. UKFライブラリの独立ビルド検証完了（`test_ukf_library.cpp`で全テストPass）

### ⚠️ 未完了の課題
1. MEUKFがまだ新しいUKFライブラリを使用していない（独自実装のまま）
2. 他の変更ファイル（math_utils.hpp, sensor_filter.hpp等）が未統合
3. 回帰テストが未実行（10 sets並列テスト）

## 段階的統合計画（5ステップ）

---

### Phase 1: UKF観測モデルの実装完成
**目的**: `meukf_observation_models.hpp`の観測関数を実際のMEUKFと等価にする

**タスク**:
1. `update_accel_meukf()`のシグマポイント変換ロジックを抽出
   - axis-angle → quaternion 変換
   - quaternion 合成 (`q_nom * dq`)
   - rotation matrix 変換
   - 重力ベクトル回転

2. `AccelObservationModel::h_accel()`を完全実装
   - 現在: 簡易版（式が不完全）
   - 目標: MEUKF実装と等価（3D → 3D観測）

3. `MagObservationModel::h_mag()`を実装
   - 磁気ベクトル回転（quaternion → rotation matrix）
   - 正規化処理

**検証**:
- 単体テスト作成（`test_observation_models.cpp`）
- 既知の入力（quaternion + 重力）で予測値を確認
- MEUKF内部のベクトルと一致するか検証

**成功条件**:
- ✅ `h_accel(x_15)` が `update_accel_meukf()` のz_pred_sigmaと±1e-6以内で一致
- ✅ ビルド成功（MEX 2ファイル）

---

### Phase 2: MEUKF内に1つの更新関数でUKF delegationを実装
**目的**: `update_accel_meukf()`の一部をukf_generic.hppに委譲

**タスク**:
1. `update_accel_meukf_ukf_version()` 新規関数作成
   - ukf_generic.hpp の `UKFUpdate<3, 2, float>::update()` を呼び出し
   - 観測モデル: `AccelObservationModel::h_accel`
   - 2D観測（x, y成分のみ）をラップ

2. MEUKF::step()内でフラグ切り替え
   ```cpp
   #ifdef USE_UKF_LIBRARY
       update_accel_meukf_ukf_version(state, a_meas, params, output);
   #else
       update_accel_meukf(state, a_meas, params, output);
   #endif
   ```

3. 両方のパスで結果比較（差分ログ出力）

**検証**:
- `run_simulation(42, true)` で両方のパスを実行
- 状態ベクトル（p,v,q,ba,bg）の差を `Results/ukf_delegation_diff.csv` に出力
- RMSE < 1e-5 を確認

**成功条件**:
- ✅ ビルド成功（MEX 2ファイル）
- ✅ 両パスでRMSE < 1e-5（ほぼ等価）
- ✅ イノベーション、Mahalanobis距離が±0.1%以内

---

### Phase 3: ukf_generic.hpp の2D観測対応
**目的**: 現在のMEUKFは2D観測（加速度x,y成分のみ）を使用している → ukf_generic.hppでサポート

**タスク**:
1. `UKFUpdate` に観測次元射影機能を追加
   - テンプレートパラメータ `M_obs`（実際の観測次元） vs `M_state`（状態次元）
   - 例: `UKFUpdate<3, 3, float>` → 3D観測予測 → 2D観測 (x,y) に射影

2. または、ラッパー関数で3D→2D変換
   ```cpp
   auto h_accel_2d = [](const Vector<3,float>& x) -> Vector<2,float> {
       Vector<3,float> z3d = AccelObservationModel::h_accel(x_15);
       return Vector<2,float>{z3d(0), z3d(1)};
   };
   ```

**検証**:
- test_ukf_library.cppに2D観測ケース追加
- MEUKF実装と完全一致確認

**成功条件**:
- ✅ 2D観測でのUKF更新が正常動作
- ✅ MEUKF 2D観測パスと数値的に等価

---

### Phase 4: 全センサー更新関数をUKF委譲に置き換え
**目的**: accel, mag, GPS, barometer の全更新をukf_generic.hppに委譲

**タスク**:
1. `update_mag_meukf()` → `update_mag_meukf_ukf_version()`
   - `MagObservationModel::h_mag` を使用
   - 3D観測（全成分）

2. `update_gps_meukf()` → `update_gps_meukf_ukf_version()`（存在する場合）
   - `GPSObservationModel::h_gps` を使用
   - 3D位置観測

3. `update_alt_meukf()` → `update_alt_meukf_ukf_version()`（存在する場合）
   - `AltObservationModel::h_alt` を使用
   - 1D高度観測

4. 全関数でフラグ切り替え可能にする
   ```cpp
   #define USE_UKF_LIBRARY 1  // build時に切り替え
   ```

**検証**:
- `run_batch_10sets()` で10seed回帰テスト
- 新旧実装の統計比較（RMSE, max_error, std_dev）
- 全seedで差分 < 1e-4 確認

**成功条件**:
- ✅ 10/10テストPass
- ✅ 平均RMSE差 < 1e-4
- ✅ ビルド時間が5%以上増加していない

---

### Phase 5: 旧UKF実装の削除とクリーンアップ
**目的**: meukf_core.cpp から独自UKF実装を削除、ukf_generic.hpp に完全移行

**タスク**:
1. `#ifdef USE_UKF_LIBRARY` を削除（常にライブラリ使用）
2. 旧関数（`update_accel_meukf`, `update_mag_meukf`）を削除
3. 内部ヘルパー関数（`cholesky3x3`, シグマポイント生成）を削除
4. `meukf_core.cpp` のファイルサイズ削減確認（1346行 → ~800行目標）

**検証**:
- `run_batch_10sets()` で最終回帰テスト
- コミット前にバックアップ取得（Gitタグ: `before-ukf-cleanup`）
- 削除後ビルド成功確認

**成功条件**:
- ✅ ビルド成功（MEX 2ファイル）
- ✅ 10/10テストPass（baseline比較でRMSE差 < 1e-4）
- ✅ meukf_core.cpp のLOC削減（~40%減）

---

## リスクと対策

### リスク1: 数値精度の低下
**対策**:
- 各Phaseで差分ログ出力
- RMSE閾値（1e-4）厳守
- 回帰時は即座にGit revert

### リスク2: ビルド失敗
**対策**:
- 各Phase完了時に `build_mex()` 実行
- エラー時は該当Phaseのみ巻き戻し
- 依存ファイル変更は最小限に（ukf_generic.hpp, meukf_core.cpp のみ）

### リスク3: 観測モデルの実装ミス
**対策**:
- Phase 1で単体テスト徹底
- 既知の入力（identity quaternion + 標準重力）で手計算と比較
- MATLABスクリプトで同じ計算を実施し、C++と比較

---

## 次のアクション

**即座に実行**:
1. Phase 1開始: `AccelObservationModel::h_accel()` の完全実装
2. `test_observation_models.cpp` 作成
3. MEUKF内のシグマポイント変換ロジックを抽出

**完了目標**:
- Phase 1: 2時間以内
- Phase 2-3: 各4時間
- Phase 4-5: 各6時間
- **合計**: ~22時間（3営業日想定）

---

## 成功の定義

1. ✅ 全MEXビルド成功
2. ✅ 回帰テスト10/10 Pass
3. ✅ 数値精度保持（RMSE < 1e-4）
4. ✅ コードの重複削除（meukf_core.cpp 40%削減）
5. ✅ UKFライブラリ独立性確保（他フィルタでも再利用可能）

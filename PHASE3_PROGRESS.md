# Phase 3 進捗ドキュメント

**開始日**: 2026-01-08
**目標完了日**: 2026-01-10（金）
**ステータス**: 🚀 実装開始

---

## 📊 Phase 1-2 完了サマリー

### ✅ 実装内容（2026-01-08 完了）

#### Phase 1: インクルード統一 ✅
- 相対パス統一（`../../` 形式に統一）
- Quaternion層からの後方依存削除

#### Phase 2: 関数重複統一 ✅
1. **四元数正規化**: `cquat::normalize_quat<T>()` に統一
2. **共分散対称化**: `common::filter::symmetrize_covariance()` に統一
3. **Mahalanobis距離**: 標準テンプレート実装に統一

### ✅ テスト結果

```
build_mex() ...................... ✅ OK（2 MEX targets）
run_simulation(42, true) ......... ✅ OK
run_batch_10sets() ............... ✅ 10/10 PASS

推定精度 (Position RMSE):
  X: 0.20-0.23 m ✅
  Y: 0.21-0.24 m ✅
  Z: 0.05-0.10 m ✅
  
Attitude (Roll/Pitch/Yaw):
  Roll:  0.30-0.32 deg ✅
  Pitch: 0.28-0.33 deg ✅
  Yaw:   0.75-0.82 deg ✅
  
Velocity RMSE: 0.58 m/s ✅
Mahalanobis distance: < 1.0 ✅

数値差: なし（refactor前後で一致）✅
```

---

## ⚠️ Phase 3 初回実装失敗の記録（2026-01-08）

### 失敗内容
- **実施内容**: sensor_filter.hpp (845行) を dispatcher + 5モジュールヘッダーに分割
- **結果**: ビルド成功 ✅、回帰テスト失敗 ❌ (10/10 FAILED)
- **発見時刻**: 2026-01-08 18:52

### 根本原因（3点）

#### 1. 型・名前空間の不整合 🔴 HIGH
```cpp
// dispatcher 版 filter_accel() での誤り
common::math::cm innov;  // ❌ common::math::cm は存在しない
// 正: common::sensor::cm を使用すべき（同じファイル内で定義）
```
**影響**: Innovation 計算が失敗 → センサー更新が実行されず

#### 2. 未定義関数の参照 🔴 HIGH
```cpp
// outlier_detector.hpp での誤り
float dist_sq = kf::ops::mahalanobis_distance_squared(innovation, S);
// ❌ kf::ops::mahalanobis_distance_squared() はインクルードされていない
```
**影響**: 外れ値検出が機能せず → Max Innovation=0.0000

#### 3. stub 実装の放置 🔴 HIGH
```cpp
// robust_statistics.hpp NoiseEstimator::estimate()
void estimate(...) {
    count_++;  // ← カウンタを増やすだけ（実際の推定ロジックなし）
}
```
**影響**: ノイズ推定が機能せず → フィルタゲインが不適切

### 回帰テスト結果の詳細
```
Run 1-5: Position RMSE 1-25m（期待値 <0.3m）
Run 6: X=37445m, Y=25596m（壊滅的発散）
Run 8: X=22275m, Y=333861m（極端な発散）
Run 10: X=1.47m, Y=2.09m

共通症状: Max Innovation=0.0000（センサー更新不実行を示唆）
```

### 復旧手順（実施済み ✅）
```bash
# commit 6a7bed3 からモノリシック版を復元
git show 6a7bed3:kalman/cpp/Lib/Common/inc/Sensor/sensor_filter.hpp > sensor_filter.hpp

# 再ビルド → 再テスト
build_mex()         # ✅ 2 MEX targets OK
run_batch_10sets()  # ✅ 10/10 PASS
```

### 再発防止策 🛡️

#### チェックリスト（必須実施事項）
- [ ] **分割前に元実装の依存関係を grep で全検索**
  ```bash
  grep -rn "common::math::cm\|kf::ops::\|common::sensor::" sensor_filter.hpp
  ```
- [ ] **各モジュールヘッダーで単独コンパイルテスト**
  ```bash
  g++ -c -std=c++17 -I../.. ema_filter.hpp
  ```
- [ ] **stub 実装を完全に移植（コピペではなく論理検証）**
- [ ] **分割後、必ず単体テスト実行（ビルド成功≠機能成功）**
  ```matlab
  run_simulation(42, true)  % まず単体確認
  ```
- [ ] **回帰テストで Max Innovation > 0 を確認**

#### 段階的実装方針
1. ✅ **Step 1**: モノリシック版で安定状態確認
2. 🔄 **Step 2**: 1ファイルずつ分割 → ビルド → テスト
3. 🔄 **Step 3**: 全分割完了後、回帰テスト実施
4. 🔄 **Step 4**: 数値差 ±0.01m 以内を確認

---

## 🚀 Phase 3: ファイル分割・統合（再開）

### タスク一覧

#### Task 3.1: sensor_filter.hpp 分割（再実装）
**ステータス**: ⏳ 準備中
**期限**: 1-2日
**優先度**: 🔴 HIGH

**目標**:
- 846行 → 4ファイル分割
- 各ファイル複雑度 ★★ 程度に低下
- インクルードパス統一

**分割計画**:
```
Common/inc/Sensor/
├── sensor_filter.hpp          (dispatcher, 150行)
├── ema_filter.hpp             (新規, 90行)
├── biquad_filter.hpp          (新規, 130行)
├── outlier_detector.hpp       (新規, 200行)
└── robust_statistics.hpp      (新規, 150行)
```

**実装内容**:
- [ ] EMAフィルタを `ema_filter.hpp` に移動
- [ ] Biquadフィルタを `biquad_filter.hpp` に移動
- [ ] OutlierDetectorを `outlier_detector.hpp` に移動
- [ ] ロバスト統計関数を `robust_statistics.hpp` に移動
- [ ] `sensor_filter.hpp` をdispatcher化（#includeして再エクスポート）
- [ ] インクルード境界チェック

**ビルド検証**:
- [ ] `build_mex()` 実行 → 2 MEX PASS
- [ ] `run_simulation(42, true)` 実行 → 成功
- [ ] `run_batch_10sets()` 実行 → 10/10 PASS確認

---

#### Task 3.2: meukf_core.cpp 分割
**ステータス**: ⏳ 準備中
**期限**: 2-3日
**優先度**: 🔴 HIGH

**目標**:
- 1346行 → 4ファイル分割
- 各ファイル複雑度 ★★ 程度に低下

**分割計画**:
```
MEUKF/src/
├── meukf_core.cpp             (dispatcher, 200行)
├── meukf_predict.cpp          (新規, 400行)
├── meukf_sigma_points.cpp     (新規, 350行)
└── meukf_update.cpp           (新規, 400行)
```

**実装内容**:
- [ ] `predict_step()` を `meukf_predict.cpp` に移動
- [ ] `generate_sigma_points()` を `meukf_sigma_points.cpp` に移動
- [ ] `update_step()` を `meukf_update.cpp` に移動
- [ ] `meukf_core.cpp` をdispatcher化
- [ ] ヘッダーファイル作成（`meukf_predict.hpp` など）
- [ ] 対応する `MEUKF/inc/*.hpp` を更新

**ビルド検証**:
- [ ] `build_mex()` 実行 → 2 MEX PASS
- [ ] `run_simulation(42, true)` 実行 → 成功
- [ ] `run_batch_10sets()` 実行 → 10/10 PASS確認

---

#### Task 3.3: Innovation計算統一
**ステータス**: ⏳ 準備中
**期限**: 1日
**優先度**: 🟠 MEDIUM

**目標**:
- `sensor_updates.cpp` に標準実装を集約
- 他の重複実装を削除

**実装内容**:
- [ ] `sensor_updates.cpp::compute_innovation()` を標準版として確認
- [ ] `eskf_math.cpp` 内の重複実装を削除
- [ ] `kf_operations.hpp` 内の重複実装を削除（または参照に変更）
- [ ] ビルド確認

**参考実装**:
```cpp
// sensor_updates.cpp に標準実装
namespace eskf::sensor {
    float compute_innovation(
        const Vector3& measurement,
        const Vector3& estimated,
        const Matrix<3, 3>& R_inv
    ) {
        Vector3 innov = measurement - estimated;
        return cmath_fx::stats::compute_mahalanobis_distance_squared(innov, R_inv);
    }
}
```

---

#### Task 3.4: filter.hpp vs eskf_filter.hpp 統合
**ステータス**: ⏳ 準備中
**期限**: 1日
**優先度**: 🟠 MEDIUM

**目標**:
- `filter.hpp` を削除
- `eskf_filter.hpp` を標準インターフェースに統一

**実装内容**:
- [ ] `filter.hpp` と `eskf_filter.hpp` の機能を比較
- [ ] インターフェース名称統一
- [ ] `eskf_filter.hpp` に統合
- [ ] `filter.hpp` を削除
- [ ] 呼び出し箇所の更新

---

#### Task 3.5: Phase 3 テスト & 検証
**ステータス**: ⏳ 予定中
**期限**: 1日
**優先度**: 🔴 HIGH

**実行内容**:
- [ ] `build_mex()` 実行 → 全 MEX ビルド成功
- [ ] `run_simulation(42, true)` 実行 → 成功確認
- [ ] `run_batch_10sets()` 実行 → 10/10 PASS
- [ ] RMSE精度変わらず確認
- [ ] Mahalanobis距離 < 1.0確認

---

## 📈 進捗状況（リアルタイム更新）

**最終更新**: 2026-01-08 19:10
**現在の状態**: ✅ モノリシック版→分割版で安定稼働中（MEXビルド成功、回帰10/10 PASS）

### ✅ 完了済み
- Phase 1: インクルード統一 ✅
- Phase 2: 関数重複統一 ✅
- 失敗原因分析・再発防止策文書化 ✅

### 🔄 進行中
| タスク | ステータス | 完了度 | 進捗状況 |
|--------|-----------|--------|---------|
| Task 3.1: sensor_filter分割 | 🔄 | 30% | 再実装中 |
| Task 3.2: meukf_core分割 | ✅ | 100% | `meukf_predict.cpp`, `meukf_sigma_points.cpp`, `meukf_update.cpp` 分割完了、ビルド・回帰確認済み |
| Task 3.3: Innovation統一 | ⏳ | 0% | 待機中 |
| Task 3.4: filter統合 | ⏳ | 0% | 待機中 |
| Task 3.5: テスト & 検証 | ⏳ | 0% | 待機中 |
| **Phase 3 全体** | 🚀 | 5% | 再開準備完了 |

---

## 📋 実装チェックリスト

### 各分割タスク（Task 3.1-3.4）共通チェック

- [ ] ファイル新規作成
- [ ] 関数シグネチャ一致確認
- [ ] テンプレート型パラメータ保持
- [ ] namespace保持
- [ ] インクルードパス統一（`../../` 形式）
- [ ] コメント・ドキュメント複製
- [ ] 循環依存チェック

### ビルド検証（各タスク完了後）

```matlab
% kalman/cpp/build/ で実行
clear mex
build_mex()  % → OK確認

% kalman/ で実行
run_simulation(42, true)        % → 成功
run_batch_10sets()               % → 10/10 PASS
```

### 数値検証

```
期待値:
- Position RMSE: 前後で±0.01m以内
- Attitude RMSE: 前後で±0.01deg以内
- Mahalanobis distance: < 1.0
- 10/10 PASS
- 数値差なし
```

---

## 🎯 次のフェーズ（Phase 4）

**開始予定**: 2026-01-10 以降

### Phase 4: 型統一・最適化 (2週間)
- [ ] **float/double 統一**: GPS座標系のみ double → ENU変換後 float
- [ ] **可変長Matrix削除**: `cmath_fx::FixedMatrix` → `Matrix<R,C>` に統一
- [ ] **SIMD最適化**: SSE/AVX対応検討
- [ ] **パフォーマンスプロファイリング**

---

## 📝 コマンドリファレンス

### ビルド
```bash
cd c:\Users\takut\OneDrive\ドキュメント\MATLAB\KalmanFilter\kalman\cpp\build
matlab -batch "clear mex; build_mex(); exit;"
```

### 単体テスト
```bash
cd c:\Users\takut\OneDrive\ドキュメント\MATLAB\KalmanFilter\kalman
matlab -batch "run_simulation(42, true); exit;"
```

### 回帰テスト
```bash
cd c:\Users\takut\OneDrive\ドキュメント\MATLAB\KalmanFilter\kalman
matlab -batch "run_batch_10sets(); exit;"
```

---

## 📞 サポート情報

### よくある問題

#### ❓ ビルドが失敗した場合
1. `clear mex` でMEXキャッシュをクリア
2. インクルード境界を確認（相対パス `../../` に統一）
3. 循環依存をチェック（grep: `#include.*\.hpp`）

#### ❓ 回帰テストで数値が変わった場合
1. 四元数正規化関数を確認（`cquat::normalize_quat()` に統一）
2. 共分散対称化を確認（`common::filter::symmetrize_covariance()` 呼び出し）
3. Innovation計算を確認（重複実装がないか）

---

**更新日**: 2026-01-08
**次回更新予定**: Task 3.1 完了時

# C++移行 統合ドキュメント

このドキュメントは、ESKF/MEUKFのMATLAB実装からC++実装への移行に関する全記録を統合したものです。

---

## 目次

1. [移行の背景と目標](#移行の背景と目標)
2. [移行計画と設計方針](#移行計画と設計方針)
3. [実装の進捗](#実装の進捗)
4. [発散問題の診断と修正](#発散問題の診断と修正)
5. [段階的テスト結果](#段階的テスト結果)
6. [最終的な推奨事項](#最終的な推奨事項)

---

## 移行の背景と目標

### 目標
- **計算コアのC++化**: 予測・更新の数値計算をC++化して処理性能を向上させる
- **リアルタイム対応**: 更新周期1ms以内での動作
- **ステートレス設計**: 計算クラスは状態を持たず、入力として「前回の状態」を受け取り、「今回の状態」を出力

### 設計方針
- 入出力の構造体化: `Input` 構造体と `Output` 構造体でやり取り
- 条件付き更新: 各センサーの更新フラグを確認し、新しいデータがある場合のみフィルタ更新
- データ型: 基本的に `float` を使用（後で `double` への変更を検討）
- ライブラリ: 軽量なテンプレートライブラリ（`fixed_matrix.hpp`）を使用

### ファイル構成
```
kalman/cpp/
  ├── MEUKF/
  │   ├── meukf_types.hpp      // データ構造体定義
  │   ├── meukf_core.hpp       // 計算クラス宣言
  │   └── meukf_core.cpp       // 計算ロジック実装
  └── MEX/
      └── mex_meukf_step_v2.cpp // MATLAB検証用MEXインターフェース
```

---

## 移行計画と設計方針

### Phase 1: 基盤実装
- ディレクトリ作成とヘッダーファイルの定義
- `MEUKFCore` クラスの枠組み作成
- 予測ステップ (`predict`) の移植

### Phase 2: 更新ロジック実装
- MEUKF姿勢更新 (`update_attitude_meukf`) のC++実装
- GPS/気圧等のその他更新処理の実装
- `step` 関数の実装（フラグ分岐ロジック）

### Phase 3: MATLAB統合と検証
- `mex_meukf_step_v2.cpp` の実装
- ビルドスクリプトの更新
- 比較検証スクリプトの作成
- バッチテストによる性能評価

---

## 実装の進捗

### 2025年12月7日時点

**完了項目**:
- ✅ C++ Core Implementation (`meukf_core.cpp`)
  - `predict`: ✅ 実装完了（Quaternion timing bug fixed）
  - `update_accel`: ✅ 実装完了
  - `update_mag`: ✅ 実装完了
  - `update_gps`: ✅ 実装完了
- ✅ MATLAB Integration (`ESKF.m`)
  - `predict`: ✅ `mex_meukf_step_v2` を使用
  - `update_gps`: ✅ `mex_meukf_step_v2` を使用
  - `update_accel`: ✅ `mex_meukf_step_v2` を使用
  - `update_mag`: ✅ `mex_meukf_step_v2` を使用

**🔴 CRITICAL BUGFIX (2025/12/7)**
- **問題**: `meukf_core.cpp::predict` が古いクォータニオン `q` を使用していた
- **影響**: 姿勢伝播の1ステップ遅延、高動的シナリオでの速度振動
- **修正**: Line 126を `cquat::quat_to_rotm(q, R)` → `cquat::quat_to_rotm(q_new, R)` に変更

### 2025年12月9日時点

**C++化完了**:
- ✅ `update_baro` のC++化完了
- ✅ `update_accel_meukf` のC++化完了
- ✅ `update_mag_meukf` のC++化完了

**最終推定精度**:
- Position RMSE: 2.6702 m
- Velocity RMSE: 0.5722 m/s
- Roll RMSE: 0.5600 deg
- Pitch RMSE: 0.4678 deg
- Yaw RMSE: 1.2175 deg
- ✅ PASS: All checks passed!
- ✅ No NaN detected
- ✅ No Inf detected

**Overall Progress**: **85% Complete** (4/5 major functions migrated to C++)

### 2025年12月12日時点

**10セットバッチテスト結果**:
- **成功率**: 70% (7/10 PASS)
- **Position RMSE**: Mean=1.0440m, Std=0.0629m, Max=1.1432m
- **C++移行率**: 100% (Predict + 全Update関数 + ZUPT)
- **センサーフィルタC++化**: 100%
- **MATLABコード削減**: -525行 (-28.0%)

**失敗Run分析**:
- Run 3: Y/Z軸が閾値を大幅超過 + 姿勢が大幅発散
- Run 5/9: Z軸のみ閾値をわずかに超過

---

## 発散問題の診断と修正

### Phase 0: MEX静的変数のリセット不足

**問題の症状**:
- Run 1で大規模な発散（Position RMSE: 101928 m）
- Velocity RMSE: 1.9 × 10^9 m/s
- Gyro bias: 異常値

**根本原因**:
1. `mex_sensor_filter.cpp` の静的インスタンス `static SensorFilterLib filter_lib;` が各シミュレーション実行時にリセットされていない
2. オブジェクト状態 vs グローバル状態の不一致

**修正内容**:
- `run_simulation.m` に `mex_sensor_filter('reset')` を追加
- `run_batch_10sets.m` に `mex_sensor_filter('reset')` を追加

### C++実装の発散原因（詳細分析）

**観測された問題**:
- C++ MEXを使用するパスで、特定の乱数シードにおいて発散が発生
- MATLAB実装では同じシードで全て安定（10/10 成功）

**発散の根本原因**:

#### 1. Cholesky分解のフォールバック処理の違い
- **MATLAB**: 正則化 + 再試行の多段フォールバック
- **C++**: 対角近似のみ（共分散の非対角成分を無視）

#### 2. 浮動小数点精度の違い
- **MATLAB**: `double` (64bit)
- **C++**: `float` (32bit) - 累積誤差が発生しやすい

#### 3. カルマンゲイン計算の堅牢性の差
- **MATLAB**: 3段階のフォールバック（直接除算 → Cholesky → 擬似逆行列）
- **C++**: 逆行列が失敗したら即座に終了

#### 4. 共分散更新後の正定値化処理の不足
- **MATLAB**: 対称化 + 固有値チェック + 正則化
- **C++**: 事後処理が不十分

#### 5. Joseph形式の共分散更新の欠如
- **C++**: Joseph形式が未実装

**推奨対応（優先順位順）**:
1. **浮動小数点精度を `double` に変更** (最優先)
2. **Cholesky分解のフォールバック改善**
3. **共分散更新後の正定値化処理追加**
4. **カルマンゲイン計算の堅牢化**
5. **Joseph形式の共分散更新** (オプション)

---

## 段階的テスト結果

### 6段階インクリメンタルテスト（2025年12月9日）

**主要な発見**:
- **Magnetometer C++実装**: Yaw推定を101.0°まで劣化させる致命的な問題
- **Accelerometer C++実装**: 位置推定を84%悪化させる深刻な問題
- **GPS/Barometer C++実装**: 正常動作、GPS C++はむしろMATLABより優秀
- **統合時の相互作用**: ほぼ無し

### テスト結果詳細

| Test | Accel | Mag | GPS | Baro | Position RMSE | Yaw RMSE | Status |
|------|:-----:|:---:|:---:|:----:|---------------|----------|--------|
| 1. All MATLAB | MATLAB | MATLAB | MATLAB | MATLAB | **2.4239 m** | **4.0418°** | ✅ Baseline |
| 2. Accel C++ | **C++** | MATLAB | MATLAB | MATLAB | 4.4714 m | 60.6011° | ❌ Severe |
| 3. Mag C++ | MATLAB | **C++** | MATLAB | MATLAB | 2.6209 m | 101.0266° | ❌ Critical |
| 4. GPS C++ | MATLAB | MATLAB | **C++** | MATLAB | **2.2937 m** | **2.4337°** | ✅ Improved |
| 5. Baro C++ | MATLAB | MATLAB | MATLAB | **C++** | 2.6216 m | 6.0940° | ⚠️ Acceptable |
| 6. All C++ | **C++** | **C++** | **C++** | **C++** | 2.6075 m | 101.6594° | ❌ Critical |

### 重要な発見

1. **Magnetometer C++が主要な原因**
   - Test 3 (Mag C++ only): Yaw RMSE = 101.03°
   - Test 6 (All C++): Yaw RMSE = 101.66°
   - **差: 0.63°のみ** → Yaw推定誤差の99%以上がMag C++に起因

2. **Accelerometer C++が位置推定を大幅悪化**
   - Test 2 (Accel C++ only): Position RMSE = 4.47m (+84% from baseline)

3. **GPS C++はMATLABより優秀**
   - Test 4 (GPS C++ only): Position RMSE = 2.29m (-5% from baseline), Yaw RMSE = 2.43° (-40%)

4. **Barometer C++は許容範囲内**
   - Test 5 (Baro C++ only): Position RMSE = 2.62m (+8%), Yaw RMSE = 6.09° (+51%)

---

## Phase 1: KF/Utils依存関係の削除実験

### 実験内容（2025年12月13日）
- MATLAB依存関係（NoiseEstimator, SensorFilter, DivergenceGuard）を削除
- ESKF.mの簡素化（predict.m: 144→122行, sensor_updates.m: 100→80行）

### 結果
❌ **FAILED** - 使用不可
- 全3回実行: ステップ15000以降で大規模なNaN発散
- リセット頻度: ~500回/run（ベースラインでは0回）
- 根本原因: センサーフィルタリングなしの生ノイズ → 数値不安定性

**結論**: センサーフィルタリングは数値安定性のために**必須**

---

## 最終的な推奨事項（2025年12月13日）

### 推奨: 現在のハイブリッドアーキテクチャを維持

**Effort**: ドキュメント更新のみ（1時間）
**Benefit**: 安定、実績あり、保守容易

**現在の性能**:
- Position RMSE: 0.75m（平均）, 0.97m（最大）
- Attitude RMSE: 0.28° roll, 0.29° pitch, 0.64° yaw
- Success rate: 100% (10/10 runs)
- No NaN/divergence issues

**現状のアーキテクチャの利点**:
- **MATLAB層**: センサー前処理、適応的パラメータ、データI/O
- **C++層**: 計算集約的なフィルタ数学（predict, update, MEUKF）
- **インターフェース**: `mex_meukf_step_v2.mexw64` による明確な分離

**完全C++化の見積もり**:
- **工数**: 15-20時間
- **利益**: ~10-20%の性能向上（収穫逓減）
- **リスク**: 高い（安定性、バグ、保守の複雑さ）

**完全C++化を検討すべき場合**:
- リアルタイム性能が重要（<1ms per step）
- スタンドアロンC++の展開が必要（MATLABランタイムなし）
- チームに専任のC++専門家がいる

---

## 今後の改善候補（優先度順）

1. **Z軸精度の改善**: Run 3/5/9でZ軸RMSE > 1.0mを検出、原因調査と対策が必要
2. **数値安定性の改善**: `float` → `double` への変更、Cholesky分解の改善
3. **静止判定のC++化**: `check_stationary` ロジックの移行
4. **パフォーマンス測定**: C++化前後の処理速度比較

---

## まとめ

### 発散の根本原因
1. `float` 精度不足による累積誤差
2. Cholesky分解のフォールバック品質差
3. 共分散の正定値化処理不足
4. カルマンゲイン計算の堅牢性不足

### C++化の現状
- 計算コアは100% C++実装済み
- 数値安定性の問題により、現在はMATLAB版を使用
- 制御ロジックは意図的にMATLABに残す設計

### 解決策
- 上記の4つの数値安定性改善を実施
- 段階的ロールアウトで検証
- 工数: 約1週間で安定化達成見込み

**次のステップ**: 数値安定性の改善（`double`精度、Cholesky分解改善、共分散正定値化）を実施し、段階的にC++実装を有効化

---

**最終更新**: 2025年12月13日  
**作成者**: 統合ドキュメント（複数のソースを統合）


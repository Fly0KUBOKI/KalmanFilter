# 05: 現在の状況と失敗原因の分析

## 作成日時
2025年12月30日

## 概要
`mex_run_eskf`の統合とソース分離を試みたが、直接C++実装への置き換えにより精度が大幅に低下したため、元の`mexCallMATLAB`実装に戻した。本ドキュメントでは、現在の状況と失敗の原因を分析する。

---

## 1. 現在の状況

### 1.1 実施した作業

1. **初期統合の試み**
   - `mex_run_eskf.cpp`の`call_sensor_update`と`call_gps_update`を直接C++実装に置き換え
   - `ESKFCore::update_accel`, `update_mag`, `update_baro`, `update_gps`を実装
   - `Src/ESKF/eskf_sensor_updates.cpp`にセンサー更新処理を分離

2. **共分散行列Pの更新を追加**
   - `ESKFCore::update_*`関数にJoseph formによる共分散行列Pの更新を追加
   - `update_mag`, `update_gps`, `update_baro`すべてに実装

3. **元の実装への復帰**
   - 精度が大幅に低下したため、`mexCallMATLAB`を使用する元の実装に戻した
   - 前処理（`mex_sensor_preprocessor`）を含む完全な実装に復帰

### 1.2 現在の実装状態

- **予測ステップ**: C++直接実装（`call_predict`関数）
  - `ESKFCore::integrate_nominal`
  - `ESKFCore::predict_covariance`
  - 後処理（`predict_postprocess`）

- **センサー更新**: `mexCallMATLAB`経由（元の実装）
  - `mex_sensor_preprocessor` → 前処理
  - `mex_eskf_do_update` → 更新処理
  - `mex_meukf_step_v2` → 実際のKalman filter更新（内部で呼び出される）

- **初期化**: C++直接実装（`do_init`関数）
  - `ESKFInitializer`を使用

### 1.3 テスト結果

#### 統合前（正常動作）
```
Run 1 Summary: PASS
  Position RMSE: Overall=0.9506 m, X=0.1815 m, Y=0.1395 m, Z=0.9226 m
  Velocity RMSE: 0.5729 m/s
  Roll/Pitch/Yaw RMSE: 0.2646 / 0.2735 / 0.5890 deg
  Gyro bias (final): [-0.2313, 0.0350, 0.0055] deg/s
```

#### 統合後（直接C++実装使用時）
```
Run 1: エラー検出 - Axis RMSE too high
  Position RMSE: Overall=12.5815 m, X=12.4020 m, Y=1.4951 m, Z=1.4997 m
  Velocity RMSE: 2.2476 m/s
  Roll/Pitch/Yaw RMSE: 105.5618 / 45.1174 / 117.5618 deg
  Gyro bias (final): [0.0000, 0.0000, 0.0000] deg/s
```

#### 元の実装に戻した後（2025-12-30 11:37:58）
```
Run 1 Summary: PASS
  Position RMSE: Overall=0.8420 m, X=0.1789 m, Y=0.1389 m, Z=0.8110 m
  Velocity RMSE: 0.5720 m/s
  Roll/Pitch/Yaw RMSE: 0.2697 / 0.2802 / 0.6042 deg
  Gyro bias (final): [-0.2320, 0.0315, 0.0068] deg/s
```
- **結果**: 精度が正常に回復（統合前と同等またはそれ以上）
- **結論**: 元の`mexCallMATLAB`実装に戻すことで、問題が解決した

---

## 2. 失敗の原因分析

### 2.1 主な問題点

#### 問題1: アルゴリズムの不一致
- **現象**: ジャイロバイアスが0のまま更新されない
- **原因**: 
  - 元の実装では`mex_meukf_step_v2`（MEUKF）を呼び出していた
  - 直接C++実装では`ESKFCore::update_*`（ESKF）を使用
  - MEUKFとESKFでは、バイアス更新の方法が異なる可能性

#### 問題2: 共分散行列Pの更新タイミング
- **現象**: 共分散行列Pが正しく更新されていない
- **原因**:
  - `ESKFCore::update_*`関数内でPを更新したが、`update_state_from_dx`に渡すPが更新前の値だった可能性
  - または、Joseph formの実装に誤りがあった可能性

#### 問題3: 前処理・後処理の不完全な統合
- **現象**: センサーデータの前処理が正しく行われていない
- **原因**:
  - `mex_sensor_preprocessor`の呼び出しを削除した
  - 直接C++実装（`preprocess_accel`等）を使用したが、動作が異なる可能性

#### 問題4: イノベーション計算と発散チェックの欠如
- **現象**: イノベーションが0のまま（Max Innovation: 0.0000）
- **原因**:
  - 元の実装では`mex_sensor_filter`で発散チェックを行っていた
  - 直接C++実装では、この処理が抜けていた

### 2.2 技術的な詳細

#### 元の実装フロー
```
mex_run_eskf::call_sensor_update
  → mex_sensor_preprocessor (前処理)
  → mex_eskf_do_update
    → mex_meukf_step_v2 (MEUKFアルゴリズム)
      → 状態更新 + 共分散更新
    → mex_eskf_update_postprocess
      → mex_sensor_filter (発散チェック)
```

#### 直接C++実装フロー（失敗）
```
mex_run_eskf::call_sensor_update
  → preprocess_accel/mag/baro (C++直接実装)
  → ESKFCore::update_* (ESKFアルゴリズム)
    → dx, K計算
    → P更新（Joseph form）
  → update_state_from_dx
    → 状態更新
```

#### 問題点の比較

| 項目 | 元の実装 | 直接C++実装 | 問題 |
|------|----------|-------------|------|
| アルゴリズム | MEUKF | ESKF | 異なるアルゴリズム |
| 前処理 | `mex_sensor_preprocessor` | `preprocess_*` (C++) | 実装の違い |
| 共分散更新 | `mex_meukf_step_v2`内 | `ESKFCore::update_*`内 | タイミングの違い |
| 発散チェック | `mex_sensor_filter` | なし | 処理の欠如 |
| 後処理 | `mex_eskf_update_postprocess` | `update_state_from_dx` | 処理の違い |

### 2.3 根本原因

1. **MEUKFとESKFの違いを理解していなかった**
   - 元の実装はMEUKF（Multiplicative Extended Unscented Kalman Filter）を使用
   - 直接C++実装ではESKF（Error State Kalman Filter）を使用
   - これらは異なるアルゴリズムであり、単純に置き換えることはできない

2. **統合の範囲が広すぎた**
   - 予測、更新、前処理、後処理を一度に統合しようとした
   - 段階的な統合が必要だった

3. **テストが不十分だった**
   - ビルドが成功しただけで、実際の動作確認を十分に行わなかった
   - 中間結果（共分散行列、バイアス値など）の検証が不足していた

---

## 3. 学んだ教訓

### 3.1 統合の原則

1. **段階的な統合**
   - 一度にすべてを統合せず、小さな単位で統合し、各段階でテストする
   - 予測 → 更新 → 前処理 → 後処理の順に統合

2. **アルゴリズムの理解**
   - 置き換え前に、元のアルゴリズムと新しいアルゴリズムの違いを理解する
   - MEUKFとESKFは異なるアルゴリズムであることを認識する

3. **テストの重要性**
   - ビルド成功だけでなく、実際の動作確認と精度検証を行う
   - 中間結果（共分散行列、バイアス値など）を検証する

### 3.2 今後の方針

1. **段階的な統合計画**
   - Phase 1: 予測ステップの統合（完了）
   - Phase 2: 初期化の統合（完了）
   - Phase 3: センサー更新の統合（失敗 → 元に戻した）
   - Phase 4: 前処理の統合（未実施）
   - Phase 5: 後処理の統合（未実施）

2. **MEUKF実装の確認**
   - `mex_meukf_step_v2`の実装を確認し、ESKFとの違いを理解する
   - MEUKFを直接C++実装に置き換えるか、ESKFに完全に移行するかを決定する

3. **テスト戦略**
   - 各統合段階で、精度テストを実行する
   - 中間結果を検証する仕組みを構築する

---

## 4. 現在のファイル状態

### 4.1 変更されたファイル

1. **`kalman/cpp/MEX/mex_run_eskf.cpp`**
   - 予測ステップ: C++直接実装（`call_predict`）
   - センサー更新: `mexCallMATLAB`経由（元の実装に戻した）
   - 初期化: C++直接実装（`do_init`）

2. **`kalman/cpp/src/ESKF/eskf_core.cpp`**
   - `ESKFCore::update_*`関数を実装（現在は使用されていない）
   - Joseph formによる共分散行列Pの更新を実装

3. **`kalman/cpp/src/ESKF/eskf_sensor_updates.cpp`**
   - センサー更新処理を分離（現在は使用されていない）

4. **`kalman/cpp/build/build_mex.m`**
   - `eskf_sensor_updates.cpp`と`eskf_math.cpp`をビルドに追加

### 4.2 未使用のコード

以下のコードは実装されているが、現在は使用されていない：

- `Src/ESKF/eskf_sensor_updates.cpp`の関数群
- `ESKFCore::update_mag`, `update_gps`, `update_baro`（Joseph form実装を含む）

これらは将来の統合の参考として残している。

---

## 5. 推奨される次のステップ

### 5.1 短期（即座に実施）

1. **元の実装に戻した後のテスト** ✅ 完了
   - ビルド後のテストを実行し、精度が元に戻っていることを確認
   - 統合前と同等の精度が得られることを確認（実際には若干改善）

2. **変更のコミット**
   - 現在の状態（元の実装に戻した状態）をコミット
   - 統合の試みは別ブランチに保存

### 5.2 中期（統合を再開する場合）

1. **MEUKF実装の理解**
   - `mex_meukf_step_v2`の実装を詳細に分析
   - MEUKFとESKFの違いを文書化

2. **段階的な統合計画の再検討**
   - より小さな単位での統合を計画
   - 各段階でのテスト計画を策定

3. **テストフレームワークの構築**
   - 中間結果を検証するテストを追加
   - 精度テストを自動化

### 5.3 長期（完全な統合を目指す場合）

1. **MEUKFからESKFへの完全移行**
   - MEUKFの機能をESKFで再現
   - または、MEUKFを直接C++実装に置き換える

2. **パフォーマンス最適化**
   - `mexCallMATLAB`のオーバーヘッドを削減
   - 完全なC++実装による高速化

---

## 6. 結論

`mex_run_eskf`の統合とソース分離を試みたが、直接C++実装への置き換えにより精度が大幅に低下した。主な原因は、MEUKFとESKFの違いを理解せずに置き換えようとしたこと、および統合の範囲が広すぎたことである。

現在は元の`mexCallMATLAB`実装に戻し、予測ステップと初期化のみがC++直接実装となっている。今後の統合を再開する場合は、段階的なアプローチと十分なテストが必要である。

---

## 付録: 関連ファイル

- `01_COMMIT_CHANGES_SUMMARY.md`: 初期の統合試みの概要
- `02_FAILURE_ROOT_CAUSE_ANALYSIS.md`: 以前の失敗原因分析
- `03_PREVENTION_STRATEGIES.md`: 失敗防止戦略
- `04_INTEGRATION_REFACTORING_PLAN.md`: 統合計画


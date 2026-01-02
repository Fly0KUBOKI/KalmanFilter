# MEXファイル関係と削除可能ファイルまとめ

## 現在の状態（2025年12月31日）

### ✅ 実際に存在するMEXファイル（3つ）

**MEXフォルダ（ソース）:**
1. `mex_run_eskf.cpp` - メインエントリーポイント
2. `mex_sensor_filter.cpp` - 初期化用
3. `mex_meukf_step.cpp` - `mex_meukf_step_v2`としてビルド

**binフォルダ（バイナリ）:**
1. `mex_run_eskf.mexw64` ⭐ **必須**
2. `mex_sensor_filter.mexw64` ⭐ **必須**
3. `mex_meukf_step_v2.mexw64` ⭐ **必須**（現在の実装では）

---

## 呼び出し関係

```
MATLABコード
│
├─ mex_run_eskf (直接呼び出し)
│  │
│  ├─ C++直接呼び出し（統合済み）
│  │  ├─ ESKFRunner::predict() - 予測処理
│  │  ├─ ESKFCore::update_zupt() - ZUPT更新
│  │  ├─ check_state_divergence() - 発散チェック
│  │  ├─ reset_state_on_divergence() - リセット処理
│  │  ├─ preprocess_accel/mag/baro/gps() - 前処理
│  │  ├─ g_filter_lib.noise_estimator.estimate() - ノイズ推定
│  │  └─ g_filter_lib.divergence_guard.check_and_attenuate() - 発散ガード
│  │
│  └─ mexCallMATLAB経由
│     └─ mex_meukf_step_v2 - センサー更新処理（将来はC++直接呼び出しに移行予定）
│
└─ mex_sensor_filter (直接呼び出し - 初期化のみ)
```

---

## 削除可能なファイル

### 1. build_mex.m内の不要なエントリ

以下のエントリは、対応するソースファイルが存在しないため、**削除またはコメントアウト可能**です：

```matlab
% 以下のエントリは、ソースファイルが存在しないため削除可能
- mex_matlab_helpers.cpp (行111)
- mex_kalman_filter_core.cpp (行117)
- mex_ekf.cpp (行124)
- mex_ukf_sigma_points.cpp (行131)
- mex_eskf_math.cpp (行138)
- mex_eskf_init.cpp (行144)
- mex_eskf_get_state.cpp (行149)
- mex_eskf_free.cpp (行152)
- mex_eskf_set_state.cpp (行155)
- mex_eskf_step_handle.cpp (行158)
- mex_ukf.cpp (行163)
- mex_ukf_update.cpp (行169)
- mex_unified_filter.cpp (行187)
- mex_eskf_step.cpp (行193)
- mex_eskf_predict_postprocess.cpp (行201)
- mex_eskf_full.cpp (行207)
- mex_eskf_sensor_updates.cpp (行213)
- mex_eskf_sensor_update.cpp (行219)
```

**注意**: `build_mex.m`には`exist()`チェックがあるため、ファイルが存在しない場合は自動的にスキップされます。しかし、コードの整理のため、これらのエントリを削除またはコメントアウトすることを推奨します。

### 2. 既に削除済み（確認不要）

以下のファイルは既にMEXフォルダから削除されているか、存在しません：
- `mex_adaptive_predict.cpp` - C++直接呼び出しに統合済み
- `mex_eskf_predict_postprocess.cpp` - C++直接呼び出しに統合済み
- `mex_eskf_sensor_updates_full.cpp` - C++直接呼び出しに統合済み
- `mex_eskf_do_update.cpp` - C++直接呼び出しに統合済み
- `mex_sensor_preprocessor.cpp` - C++直接呼び出しに統合済み
- `mex_filter_management.cpp` - C++直接呼び出しに統合済み
- `mex_eskf_zupt.cpp` - C++直接呼び出しに統合済み
- `mex_eskf_constructor.cpp` - C++直接呼び出しに統合済み
- `mex_eskf_update_postprocess.cpp` - C++直接呼び出しに統合済み

---

## 推奨される整理手順

### Phase 1: build_mex.mの整理（安全）

1. 存在しないソースファイルに対応するエントリをコメントアウト
2. ビルドテスト実行
3. 問題がなければ次へ

### Phase 2: ドキュメントの更新

1. `MEX_FILES_CURRENT_STATUS.md`を参照
2. 不要なエントリの説明を追加

---

## まとめ

### 現在必要なMEXファイル（3つ）

1. ✅ `mex_run_eskf.mexw64` - メインエントリーポイント
2. ✅ `mex_sensor_filter.mexw64` - 初期化用
3. ✅ `mex_meukf_step_v2.mexw64` - センサー更新処理（将来はC++直接呼び出しに移行予定）

### 削除可能なもの

- **build_mex.m内の不要なエントリ**（約18個）- ソースファイルが存在しないため

### 既に整理済み

- 多くのMEXファイルは既にC++直接呼び出しに統合され、ソースファイルは削除済み
- MEXフォルダには必要な3つのファイルのみが存在

---

## 参考資料

- [MEX_FILES_CURRENT_STATUS.md](kalman/cpp/markdown/MEX_FILES_CURRENT_STATUS.md) - 詳細な現在の状態
- [MEX_FILES_USAGE_ANALYSIS.md](kalman/cpp/markdown/MEX_FILES_USAGE_ANALYSIS.md) - 使用状況分析
- [MEX_FILES_DEPENDENCIES.md](kalman/cpp/markdown/MEX_FILES_DEPENDENCIES.md) - 依存関係



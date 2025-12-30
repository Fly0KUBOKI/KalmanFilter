# MEXファイル現在の状態（2025年12月31日）

## 現在のMEXファイル構成

### binフォルダに存在するMEXファイル（3つ）

1. **`mex_run_eskf.mexw64`** ⭐ **メインエントリーポイント**
   - MATLABコードから直接呼び出される
   - 使用箇所: `run_simulation.m`, `run_batch_10sets.m`
   - **必須**

2. **`mex_sensor_filter.mexw64`** ⭐ **初期化用**
   - MATLABコードから直接呼び出される（初期化のみ）
   - 使用箇所: `run_batch_10sets.m`（`mex_sensor_filter('reset_zero')`）
   - **必須**

3. **`mex_meukf_step_v2.mexw64`** ⭐ **内部呼び出し**
   - `mex_run_eskf`から`mexCallMATLAB`経由で呼び出される
   - 使用箇所: `mex_run_eskf_sensor_updates.hpp`（行410）
   - **必須**（現在の実装では）

---

## 呼び出し関係図

```
MATLABコード
│
├─ mex_run_eskf (直接呼び出し)
│  │
│  ├─ C++直接呼び出し（統合済み）
│  │  ├─ ESKFRunner::predict() - 予測処理
│  │  ├─ ESKFCore::update_zupt() - ZUPT更新
│  │  ├─ check_state_divergence() - 発散チェック
│  │  └─ reset_state_on_divergence() - リセット処理
│  │
│  └─ mexCallMATLAB経由
│     └─ mex_meukf_step_v2 - センサー更新処理
│
└─ mex_sensor_filter (直接呼び出し - 初期化のみ)
```

---

## 実装の移行状況

### ✅ C++直接呼び出しに移行済み（MEX関数不要）

以下の機能は既にC++直接呼び出しに統合されています：

1. **予測処理**
   - 旧: `mex_adaptive_predict`
   - 新: `ESKFRunner::predict()`（C++直接呼び出し）

2. **予測後処理**
   - 旧: `mex_eskf_predict_postprocess`
   - 新: `mex_run_eskf_impl.hpp`内で直接実装

3. **ZUPT更新**
   - 旧: `mex_eskf_zupt`
   - 新: `ESKFCore::update_zupt()`（C++直接呼び出し）

4. **フィルター管理（発散チェック・リセット）**
   - 旧: `mex_filter_management`
   - 新: `check_state_divergence()`, `reset_state_on_divergence()`（C++直接呼び出し）

5. **センサー前処理**
   - 旧: `mex_sensor_preprocessor`
   - 新: `preprocess_accel()`, `preprocess_mag()`, `preprocess_baro()`, `preprocess_gps()`（C++直接呼び出し）

6. **ノイズ推定**
   - 旧: `mex_sensor_filter`（更新処理）
   - 新: `g_filter_lib.noise_estimator.estimate()`（C++直接呼び出し）

7. **更新後処理**
   - 旧: `mex_eskf_update_postprocess`
   - 新: `update_state_from_dx()`, `g_filter_lib.divergence_guard.check_and_attenuate()`（C++直接呼び出し）

### ⚠️ まだMEX関数を使用（移行予定）

1. **MEUKFステップ**
   - 現在: `mex_meukf_step_v2`（`mexCallMATLAB`経由）
   - 将来: `MEUKFCore::step()`（C++直接呼び出し）に移行予定
   - コメント: `mex_run_eskf_sensor_updates.hpp`行388に「後でMEUKFCore::stepに置き換え予定」と記載

---

## 削除可能なファイル

### 1. ソースファイル（MEXフォルダ内）

以下のMEXソースファイルは既にC++直接呼び出しに統合されているため、**削除可能**です：

#### 予測関連
- `mex_adaptive_predict.cpp` - `ESKFRunner::predict()`に統合
- `mex_eskf_predict_postprocess.cpp` - `mex_run_eskf_impl.hpp`に統合

#### センサー更新関連
- `mex_eskf_sensor_updates_full.cpp` - `mex_run_eskf_sensor_updates.hpp`に統合
- `mex_eskf_do_update.cpp` - `mex_run_eskf_sensor_updates.hpp`に統合
- `mex_sensor_preprocessor.cpp` - C++直接呼び出しに統合

#### フィルター管理関連
- `mex_filter_management.cpp` - `mex_run_eskf_filter_ops.hpp`に統合
- `mex_eskf_zupt.cpp` - `mex_run_eskf_filter_ops.hpp`に統合

#### 初期化関連
- `mex_eskf_constructor.cpp` - `mex_run_eskf_impl.hpp`に統合

#### 更新後処理関連
- `mex_eskf_update_postprocess.cpp` - `mex_run_eskf_sensor_updates.hpp`に統合

### 2. バイナリファイル（binフォルダ内）

以下のバイナリファイルは、対応するソースが削除可能なため、**削除可能**です：

- `mex_adaptive_predict.mexw64`
- `mex_eskf_predict_postprocess.mexw64`
- `mex_eskf_sensor_updates_full.mexw64`
- `mex_eskf_do_update.mexw64`
- `mex_sensor_preprocessor.mexw64`
- `mex_filter_management.mexw64`
- `mex_eskf_zupt.mexw64`
- `mex_eskf_constructor.mexw64`
- `mex_eskf_update_postprocess.mexw64`

**注意**: これらのファイルがbinフォルダに存在するかどうかは、実際のファイルシステムを確認してください。

### 3. レガシーファイル（使用されていない）

以下のファイルは、現在のコードから呼び出されていません：

- `mex_eskf_core.cpp` - レガシー（ビルドスクリプトでスキップ済み）
- `mex_eskf_math.cpp` - 未使用
- `mex_eskf_init.cpp` - 未使用
- `mex_eskf_get_state.cpp` - `mex_run_eskf`内で直接実装
- `mex_eskf_free.cpp` - `mex_run_eskf`内で直接実装
- `mex_eskf_set_state.cpp` - 未使用
- `mex_eskf_step.cpp` - 未使用
- `mex_eskf_step_handle.cpp` - 未使用
- `mex_eskf_full.cpp` - `mex_run_eskf`に統合
- `mex_eskf_sensor_update.cpp` - `mex_eskf_sensor_updates_full`に統合（さらに`mex_run_eskf`に統合）
- `mex_ukf.cpp` - 未使用
- `mex_ukf_update.cpp` - 未使用
- `mex_ukf_sigma_points.cpp` - 未使用
- `mex_ekf.cpp` - 未使用
- `mex_kalman_filter_core.cpp` - 未使用
- `mex_unified_filter.cpp` - 未使用

---

## 削除前の確認事項

### 1. ビルドスクリプトの確認

`build_mex.m`で以下のファイルがビルド対象に含まれていないか確認：

```matlab
% 削除可能なファイルがビルド対象に含まれていないか確認
% build_mex.mを確認して、該当する行を削除またはコメントアウト
```

### 2. 依存関係の確認

削除前に、以下のコマンドで使用状況を確認：

```bash
# 削除予定のMEX関数が呼び出されていないか確認
grep -r "mex_adaptive_predict" kalman/
grep -r "mex_eskf_predict_postprocess" kalman/
grep -r "mex_eskf_sensor_updates_full" kalman/
# ... など
```

### 3. テストの実行

削除後、必ずテストを実行：

```matlab
cd kalman
clear mex
run_batch_10sets()
```

---

## 推奨される削除手順

### Phase 1: バイナリファイルの削除（安全）

1. binフォルダ内の不要なバイナリを削除
2. テスト実行（`run_batch_10sets()`）
3. 問題がなければ次へ

### Phase 2: ソースファイルの削除（慎重に）

1. 削除予定のソースファイルを`archive/`フォルダに移動
2. ビルドスクリプトから該当行を削除
3. テスト実行
4. 問題がなければ完全削除

### Phase 3: ビルドスクリプトの整理

1. 不要なビルドエントリを削除
2. コメントを整理
3. ドキュメントを更新

---

## 将来の移行計画

### `mex_meukf_step_v2`のC++直接呼び出し化

現在、`mex_run_eskf_sensor_updates.hpp`の行410で`mexCallMATLAB`を使用していますが、将来的には`MEUKFCore::step()`を直接呼び出す予定です。

**移行時の注意事項**:
- 座標系変換の正確性を確認
- 出力形式の互換性を確認
- 十分なテストを実施

---

## まとめ

### 現在必要なMEXファイル（3つ）

1. ✅ `mex_run_eskf.mexw64` - メインエントリーポイント
2. ✅ `mex_sensor_filter.mexw64` - 初期化用
3. ✅ `mex_meukf_step_v2.mexw64` - センサー更新処理（将来はC++直接呼び出しに移行予定）

### 削除可能なファイル

- **約20-30個のMEXソースファイル**（既にC++直接呼び出しに統合済み）
- **対応するバイナリファイル**（binフォルダ内）
- **レガシーファイル**（使用されていない）

### 削除による効果

- ビルド時間の短縮
- バイナリサイズの削減
- コードベースの整理
- 保守性の向上

---

## 参考資料

- [MEX_FILES_OVERVIEW.md](./MEX_FILES_OVERVIEW.md)
- [MEX_FILES_USAGE_ANALYSIS.md](./MEX_FILES_USAGE_ANALYSIS.md)
- [MEX_FILES_DEPENDENCIES.md](./MEX_FILES_DEPENDENCIES.md)
- [MEX_FILES_CLEANUP_RECOMMENDATIONS.md](./MEX_FILES_CLEANUP_RECOMMENDATIONS.md)


# kalman/cpp ファイル構成と統合計画

## 現状分析

### 1. ディレクトリ構成

```
kalman/cpp/
├── bin/                         # 実行 MEX バイナリ（13個）✅ 保持
├── build/                       # ビルドスクリプト（5個の重複）⚠️ 統合対象
├── MEX/                         # ソース .cpp + 古いビルドスクリプト（混在）⚠️ 再構成
├── src/                         # MATLAB 用ソース実装（レガシー）
│   ├── Common/Math/
│   ├── EKF/
│   ├── ESKF/
│   └── UKF/
├── MEUKF/                       # 本体実装（meukf_core.cpp）
├── EKF/, ESKF/, UKF/            # フィルタ実装（古い / 重複）
├── include/                     # ヘッダー定義
├── tests/                       # テストスクリプト（3個 + ログ）⚠️ 削除対象
└── tools/                       # ツール
```

### 2. ビルドスクリプトの現状（重複・混在）

| ファイル | 場所 | 目的 | 状態 |
|---------|------|------|------|
| `build_mex.m` | `build/` | メインビルド（全 MEX） | ✅ 本体 |
| `build_meukf_only.m` | `build/` | MEUKF のみビルド | ⚠️ 特化 |
| `build_sensor_filter.m` | `build/` | センサーフィルタビルド | ⚠️ 特化 |
| `run_build_and_sim.m` | `build/` | ビルド+シミュレーション | ⚠️ 便利スクリプト |
| `run_selective_build.m` | `build/` | 選択的ビルド | ⚠️ 便利スクリプト |
| `build_all.m` | `MEX/` | 簡易ビルド（古い） | ❌ 古い |
| `build_common_lib.m` | `MEX/` | 共通ライブラリビルド | ⚠️ 未使用? |
| `build_filter_utils.m` | `MEX/` | フィルタユーティリティビルド | ⚠️ 未使用? |
| `build_meukf.m` | `MEX/` | MEUKF ビルド（古い） | ⚠️ 古い |

**問題**:
- `build_mex.m` が主要なのに、`MEX/` にも古いビルドスクリプトが残っている。
- `run_build_and_sim.m` と `run_selective_build.m` は便利だが、本体 `build_mex.m` でも対応可能。
- 重複・混在により、ユーザーが何を使うべきか不明確。

### 3. テストスクリプト（使用済み削除対象）

| ファイル | 用途 | 状態 |
|---------|------|------|
| `compare_phase1.m` | PHASE 1 比較（古い） | ❌ 削除対象 |
| `compare_biquad_implementations.m` | Biquad 比較（PHASE 2） | ❌ 削除対象 |
| `dump_sensor_filter_outputs.m` | センサーダンプ（デバッグ） | ❌ 削除対象 |
| `*.txt` ログ | デバッグ出力 | ❌ 削除対象 |

### 4. ソースコード（.cpp）の配置（重複・混在）

- `MEX/*.cpp`: MEX 用ラッパー + 実装源（40+ファイル）
- `src/*.cpp`: 元々の MATLAB 用ソース（レガシー）
- `MEUKF/meukf_core.cpp`: 本体実装（最新・使用中）
- `EKF/`, `ESKF/`, `UKF/` ディレクトリ: 古い実装

**問題**: `MEX/` にすべて集約されており、`src/` との関係が不明確。

---

## 統合提案

### Phase 1: ビルドスクリプト統合

**案**: `build/build_mex.m` を唯一のメインビルドスクリプトとし、他をサポートスクリプトか参照資料に変更。

**実行内容**:
1. `build/build_mex.m` がすでに汎用的 → 保持（最新）。
2. `build_meukf_only.m`, `build_sensor_filter.m` → 不要ファイルとして削除（`build_mex.m` で `build_mex({'mex_meukf_step_v2'})` 等で対応可能）。
3. `run_build_and_sim.m`, `run_selective_build.m` → 便利スクリプトとして保持（ユーザーが使う可能性あり）。
4. `MEX/build_all.m`, `MEX/build_common_lib.m`, `MEX/build_filter_utils.m`, `MEX/build_meukf.m` → すべて削除（古い・未使用）。

**結果**:
- `build/` に統一されたメインスクリプト `build_mex.m` を集約。
- 便利スクリプトは残す（後方互換性）。
- `MEX/` の古いビルドスクリプトはすべて削除。

### Phase 2: テストスクリプト削除

**実行内容**:
- `tests/compare_phase1.m`
- `tests/compare_biquad_implementations.m`
- `tests/dump_sensor_filter_outputs.m`
- `tests/*.txt` ログファイル

すべて削除（PHASE 4 クリーンアップで既に削除済みと同等）。

### Phase 3: ソースコード構成の整理

**推奨**:
- `MEX/` にはメインの `.cpp` ソースのみ保持。
- `src/` は古い MATLAB 実装のため、参考資料程度にしておく（または削除）。
- `MEUKF/`, `EKF/`, `ESKF/`, `UKF/` に古い実装が分散 → `MEX/` 集約済みならこれらは参考程度。

**実行内容**:
- `src/` ディレクトリは保持（参考値）。
- `EKF/`, `ESKF/`, `UKF/` は保持（参考値）。
- 今後は `MEX/*.cpp` が正規実装。

---

## 統合実行計画

### Step 1: ビルドスクリプト削除（build/）

削除対象:
- `build/build_meukf_only.m`
- `build/build_sensor_filter.m`

保持:
- `build/build_mex.m` ✅ メインスクリプト
- `build/run_build_and_sim.m` ✅ 便利スクリプト
- `build/run_selective_build.m` ✅ 便利スクリプト

### Step 2: MEX/ 内の古いビルドスクリプト削除

削除対象:
- `MEX/build_all.m`
- `MEX/build_common_lib.m`
- `MEX/build_filter_utils.m`
- `MEX/build_meukf.m`

### Step 3: テストスクリプト削除（tests/）

削除対象:
- `tests/compare_phase1.m`
- `tests/compare_biquad_implementations.m`
- `tests/dump_sensor_filter_outputs.m`
- `tests/*.txt` ログ

### Step 4: READMEドキュメント作成

新規作成:
- `kalman/cpp/BUILD_GUIDE.md` — ビルド手順（統合後の単一スクリプト指示）
- `kalman/cpp/DIRECTORY_STRUCTURE.md` — ディレクトリ説明

---

## 期待される効果

✅ **ビルドプロセス简素化**:
- ユーザーは `build_mex.m` 一つだけを理解すればよい。
- 特殊なビルドは引数で対応（`build_mex({'mex_meukf_step_v2'})`）。

✅ **テストスクリプト削除**:
- 不要なテストで混乱がなくなる。
- ファイル数減少 → メンテナンス負担軽減。

✅ **構成の明確化**:
- `bin/` → 実行 MEX
- `build/` → ビルドスクリプト
- `MEX/` → ソースコード（.cpp）
- `src/`, `EKF/`, 他 → 参考資料（古い実装）

---

## 危険性・確認事項

⚠️ **削除前確認**:
- `build_meukf_only.m` が専門的な用途で使用されていないか確認。
- `run_build_and_sim.m` の実行者がいないか確認。
- テストスクリプトが有効なテストケースでないか確認（→ 既に PHASE 4 クリーンアップで削除済みと同等）。

✅ **今後のビルドプロセス**:
- `build_mex.m` で全 MEX ビルド可能（確認済み）。
- 各メイン実行スクリプト（`run_simulation`, `run_batch_10sets`）がビルド依存性なし（確認予定）。


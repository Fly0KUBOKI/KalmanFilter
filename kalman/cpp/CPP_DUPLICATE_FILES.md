# 重複・不要ファイル分析

## 重複ファイル一覧

### 1. MEUKF/meukf_core.cpp vs src/MEUKF/meukf_core.cpp

**状況**: 両方のファイルが存在し、ほぼ同じ内容

**確認結果**:
- `Inc/MEUKF/meukf_core.hpp`のコメントに「Implementation: Src/MEUKF/meukf_core.cpp」と記載
- 実際のビルドでは`src/MEUKF/meukf_core.cpp`が使用されている
- `MEUKF/meukf_core.cpp`は参照されていない

**推奨**: `MEUKF/meukf_core.cpp`を削除

**理由**:
- ビルドシステムで使用されていない
- メンテナンスの混乱を招く
- `src/`配下が正規の配置場所

---

### 2. MEUKF/unified_filter.cpp vs src/MEUKF/unified_filter.cpp

**状況**: 両方のファイルが存在するが、実装が異なる

**確認結果**:
- `Inc/MEUKF/unified_filter.hpp`のコメントに「Implementation: Src/MEUKF/unified_filter.cpp」と記載
- `src/MEUKF/unified_filter.cpp`: 完全な実装（210行）
- `MEUKF/unified_filter.cpp`: プロトタイプ実装（164行、未完成）

**推奨**: `MEUKF/unified_filter.cpp`を削除

**理由**:
- `src/MEUKF/unified_filter.cpp`が正規の実装
- `MEUKF/unified_filter.cpp`は古いプロトタイプ
- ビルドシステムで使用されていない

---

### 3. MEUKF/meukf_types.hpp vs Inc/MEUKF/meukf_types.hpp

**状況**: 両方のファイルが存在し、内容は同一

**確認結果**:
- 両ファイルの内容を比較した結果、完全に同一
- `Inc/MEUKF/meukf_types.hpp`が正規の配置場所
- `MEUKF/meukf_types.hpp`は参照されていない

**推奨**: `MEUKF/meukf_types.hpp`を削除

**理由**:
- ヘッダーファイルは`Inc/`配下が正規の配置場所
- 重複により混乱を招く
- ビルドシステムで使用されていない

---

### 4. MEX/mex_type_conv.hpp vs Inc/MEX/mex_type_conversion.hpp

**状況**: 両方のファイルが存在するが、異なる用途

**確認結果**:
- `MEX/mex_type_conv.hpp`: 基本的な型変換関数（inline関数）
- `Inc/MEX/mex_type_conversion.hpp`: テンプレート関数（`mex_type_conv.hpp`をインクルード）

**推奨**: **両方必要**（削除しない）

**理由**:
- `mex_type_conversion.hpp`は`mex_type_conv.hpp`をインクルードしている
- 異なる用途（基本関数 vs テンプレート関数）
- 依存関係が明確

---

## 不要ファイル候補

### 1. MEX/mex_meukf_step.cpp

**状況**: 非推奨、`mex_run_eskf`に統合済み

**確認結果**:
- ファイル内のコメントに「統合完了」「後方互換性のために残しているが、実際には使用されていない」と記載
- `mexFunction`内でエラーを返す実装
- `mex_run_eskf`の`meukf_step`コマンドで代替

**推奨**: 削除を検討（ただし、後方互奨性が必要な場合は残す）

**理由**:
- 実際には使用されていない
- 統合済みの機能で代替可能
- メンテナンスの負担

---

### 2. tests/compare_cholesky.cpp

**状況**: テストファイル

**確認結果**:
- テスト用のファイル
- 本番ビルドには含まれない可能性が高い

**推奨**: テストファイルとして保持（削除不要）

**理由**:
- テストファイルは保持すべき
- ビルド対象外であれば問題なし

---

## 削除推奨ファイルまとめ

### 即座に削除可能

1. ✅ `MEUKF/meukf_core.cpp`
2. ✅ `MEUKF/unified_filter.cpp`
3. ✅ `MEUKF/meukf_types.hpp`

### 検討が必要

4. ⚠️ `MEX/mex_meukf_step.cpp`（後方互換性を確認）

### 保持すべき

5. ✅ `MEX/mex_type_conv.hpp`（必要）
6. ✅ `Inc/MEX/mex_type_conversion.hpp`（必要）
7. ✅ `tests/compare_cholesky.cpp`（テストファイル）

---

## 削除スクリプト例

```bash
# 削除推奨ファイル
rm kalman/cpp/MEUKF/meukf_core.cpp
rm kalman/cpp/MEUKF/unified_filter.cpp
rm kalman/cpp/MEUKF/meukf_types.hpp

# 検討が必要なファイル（後方互換性確認後）
# rm kalman/cpp/MEX/mex_meukf_step.cpp
```

---

## 削除前の確認事項

1. **ビルドシステム**: これらのファイルがビルドスクリプトに含まれていないか確認
2. **バージョン管理**: Git履歴を確認し、削除による影響を評価
3. **ドキュメント**: 関連ドキュメントを更新
4. **テスト**: 削除後、ビルドとテストを実行して問題がないか確認

---

## ファイル配置の推奨規則

### ヘッダーファイル
- **配置**: `Inc/`配下
- **命名**: `*.hpp`

### ソースファイル
- **配置**: `src/`配下
- **命名**: `*.cpp`

### MEXファイル
- **配置**: `MEX/`配下
- **命名**: `mex_*.cpp`

### テストファイル
- **配置**: `tests/`配下
- **命名**: `test_*.cpp` または `*_test.cpp`

### ライブラリファイル
- **配置**: `Lib/`配下
- **命名**: `*.hpp`


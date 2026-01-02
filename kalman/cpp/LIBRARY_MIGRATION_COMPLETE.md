# ライブラリ移行完了報告

最終更新: 2025-01-02

## 移行概要

`Inc/Common/Math/`にあった行列計算とクォータニオン計算のライブラリを`Lib/`フォルダに移行しました。

## 移行したファイル

### 1. 行列計算ライブラリ
- **移行元**: `Inc/Common/Math/fixed_matrix.hpp`
- **移行先**: `Lib/Matrix/fixed_matrix.hpp`
- **名前空間**: `cmath_fx`（変更なし）

### 2. クォータニオン関数ライブラリ
- **移行元**: `Inc/Common/Math/quaternion.hpp`
- **移行先**: `Lib/Quaternion/quaternion_functions.hpp`
- **名前空間**: `cquat`（変更なし）

### 3. クォータニオンクラスライブラリ
- **移行元**: `Inc/Common/Math/quaternion_lib.hpp`
- **移行先**: `Lib/Quaternion/quaternion_lib.hpp`
- **名前空間**: `quat_lib`（変更なし）

## 実施した作業

### Phase 1: Libフォルダへのファイルコピー ✅
- `Lib/Matrix/fixed_matrix.hpp`を作成
- `Lib/Quaternion/quaternion_functions.hpp`を作成
- `Lib/Quaternion/quaternion_lib.hpp`を作成

### Phase 2: includeパスの更新 ✅
すべてのファイルで以下のパスを更新：
- `#include "Common/Math/fixed_matrix.hpp"` → `#include "Lib/Matrix/fixed_matrix.hpp"`
- `#include "Common/Math/quaternion.hpp"` → `#include "Lib/Quaternion/quaternion_functions.hpp"`
- `#include "Common/Math/quaternion_lib.hpp"` → `#include "Lib/Quaternion/quaternion_lib.hpp"`
- 相対パスの場合も同様に更新（`../../Inc/Common/Math/` → `Lib/`）

**更新したファイル**:
- `Inc/`フォルダ内のすべてのヘッダーファイル（約20ファイル）
- `Src/`フォルダ内のすべてのソースファイル（約10ファイル）
- `MEX/Inc/`フォルダ内のヘッダーファイル（数ファイル）

### Phase 3: ビルド設定の確認 ✅
- `build/build_mex.m`で`Lib/`が既にインクルードパスに追加されていることを確認
- 追加の変更は不要

## 次のステップ

### Phase 4: 旧ファイルの削除（保留）
移行が正常に動作することを確認後、以下のファイルを削除：
- `Inc/Common/Math/fixed_matrix.hpp`
- `Inc/Common/Math/quaternion.hpp`
- `Inc/Common/Math/quaternion_lib.hpp`

**注意**: コンパイル確認が完了するまで削除しないでください。

### Phase 5: ドキュメントの更新（保留）
- `STRUCTURE_AND_LIBRARIES.md`を更新
- `Lib/README.md`を更新（非推奨マークを削除）
- `README.md`を更新（必要に応じて）

## 移行後の構造

```
Lib/
├── Matrix/
│   ├── fixed_matrix.hpp      # ✅ 移行済み（主要行列ライブラリ）
│   ├── matrix.hpp            # ⚠️ 未使用（旧ライブラリ）
│   └── decomposition.hpp     # コレスキー分解（既存）
├── Quaternion/
│   ├── quaternion_functions.hpp  # ✅ 移行済み（関数ベース）
│   ├── quaternion_lib.hpp        # ✅ 移行済み（クラスベース）
│   └── quaternion.hpp        # ⚠️ 未使用（旧ライブラリ）
└── ...
```

## 互換性

- **名前空間**: すべて変更なし（`cmath_fx`、`cquat`、`quat_lib`）
- **API**: 変更なし
- **ビルドシステム**: 変更なし（`Lib/`は既にインクルードパスに含まれていた）

## 確認事項

1. ✅ すべてのincludeパスを更新
2. ⏳ コンパイル確認（実施が必要）
3. ⏳ 旧ファイルの削除（コンパイル確認後）
4. ⏳ ドキュメントの更新（コンパイル確認後）


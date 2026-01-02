# ライブラリ移行完了サマリー

最終更新: 2025-01-02

## ✅ 移行完了

`Inc/Common/Math/`にあった行列計算とクォータニオン計算のライブラリを`Lib/`フォルダに移行しました。

## 移行したファイル

1. **`Lib/Matrix/fixed_matrix.hpp`** - `cmath_fx::Matrix`（行列計算ライブラリ）
2. **`Lib/Quaternion/quaternion_functions.hpp`** - `cquat::`（クォータニオン関数ライブラリ）
3. **`Lib/Quaternion/quaternion_lib.hpp`** - `quat_lib::Quaternion`（クォータニオンクラスライブラリ）

## 実施した作業

### ✅ Phase 1: Libフォルダへのファイルコピー
- `Lib/Matrix/fixed_matrix.hpp`を作成
- `Lib/Quaternion/quaternion_functions.hpp`を作成
- `Lib/Quaternion/quaternion_lib.hpp`を作成

### ✅ Phase 2: includeパスの更新
すべてのファイル（約30ファイル）でincludeパスを更新：
- `#include "Common/Math/fixed_matrix.hpp"` → `#include "Lib/Matrix/fixed_matrix.hpp"`
- `#include "Common/Math/quaternion.hpp"` → `#include "Lib/Quaternion/quaternion_functions.hpp"`
- `#include "Common/Math/quaternion_lib.hpp"` → `#include "Lib/Quaternion/quaternion_lib.hpp"`

**更新したファイル**:
- `Inc/`フォルダ内: 15ファイル
- `Src/`フォルダ内: 8ファイル
- `MEX/Inc/`フォルダ内: 3ファイル

### ✅ Phase 3: ビルド設定の確認
- `build/build_mex.m`で`Lib/`が既にインクルードパスに追加されていることを確認
- 追加の変更は不要

## 次のステップ

### ⏳ Phase 4: コンパイル確認
移行が正常に動作することを確認してください。

```matlab
cd kalman/cpp/build
build_mex()
```

### ⏳ Phase 5: 旧ファイルの削除（コンパイル確認後）
コンパイルが成功したら、以下のファイルを削除：
- `Inc/Common/Math/fixed_matrix.hpp`
- `Inc/Common/Math/quaternion.hpp`
- `Inc/Common/Math/quaternion_lib.hpp`

### ⏳ Phase 6: ドキュメントの更新（コンパイル確認後）
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
└── Quaternion/
    ├── quaternion_functions.hpp  # ✅ 移行済み（関数ベース）
    ├── quaternion_lib.hpp        # ✅ 移行済み（クラスベース）
    └── quaternion.hpp        # ⚠️ 未使用（旧ライブラリ）
```

## 互換性

- **名前空間**: すべて変更なし（`cmath_fx`、`cquat`、`quat_lib`）
- **API**: 変更なし
- **ビルドシステム**: 変更なし（`Lib/`は既にインクルードパスに含まれていた）


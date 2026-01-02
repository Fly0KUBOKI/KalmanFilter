# ライブラリ移行計画：Inc/Common/Math → Lib/

最終更新: 2025-01-02

## 移行の目的

`Inc/Common/Math/`にある行列計算とクォータニオン計算のライブラリを`Lib/`フォルダに移行し、独立ライブラリとして整理します。

## 移行対象

### 1. 行列計算ライブラリ
- **移行元**: `Inc/Common/Math/fixed_matrix.hpp`
- **移行先**: `Lib/Matrix/fixed_matrix.hpp`
- **名前空間**: `cmath_fx`（変更なし）
- **理由**: 既存の`Lib/Matrix/matrix.hpp`（`lib::matrix::Mat`）は使用されていないため、`fixed_matrix.hpp`を`Lib/`に移行

### 2. クォータニオン関数ライブラリ
- **移行元**: `Inc/Common/Math/quaternion.hpp`
- **移行先**: `Lib/Quaternion/quaternion_functions.hpp`（または既存の`quaternion.hpp`と統合）
- **名前空間**: `cquat`（変更なし）
- **理由**: 関数ベースのクォータニオン演算

### 3. クォータニオンクラスライブラリ
- **移行元**: `Inc/Common/Math/quaternion_lib.hpp`
- **移行先**: `Lib/Quaternion/quaternion_lib.hpp`
- **名前空間**: `quat_lib`（変更なし）
- **理由**: クラスベースのクォータニオン演算（高機能版）

## 移行手順

### Phase 1: Libフォルダへのファイルコピー

1. `Inc/Common/Math/fixed_matrix.hpp` → `Lib/Matrix/fixed_matrix.hpp`
2. `Inc/Common/Math/quaternion.hpp` → `Lib/Quaternion/quaternion_functions.hpp`
3. `Inc/Common/Math/quaternion_lib.hpp` → `Lib/Quaternion/quaternion_lib.hpp`

### Phase 2: includeパスの更新（段階的）

すべてのファイルで`#include`パスを更新：
- `#include "Common/Math/fixed_matrix.hpp"` → `#include "Lib/Matrix/fixed_matrix.hpp"`
- `#include "Common/Math/quaternion.hpp"` → `#include "Lib/Quaternion/quaternion_functions.hpp"`
- `#include "Common/Math/quaternion_lib.hpp"` → `#include "Lib/Quaternion/quaternion_lib.hpp"`

**影響範囲**:
- `Inc/`フォルダ内のヘッダー（約30ファイル）
- `Src/`フォルダ内の実装ファイル（約10ファイル）
- `MEX/`フォルダ内のファイル（数ファイル）

### Phase 3: ビルド設定の更新

- `build/build_mex.m`のincludeパスを更新（`-I Lib/`を追加）

### Phase 4: 旧ファイルの削除

移行が完了し、すべての参照が更新されたら：
- `Inc/Common/Math/fixed_matrix.hpp` を削除
- `Inc/Common/Math/quaternion.hpp` を削除
- `Inc/Common/Math/quaternion_lib.hpp` を削除

### Phase 5: ドキュメントの更新

- `STRUCTURE_AND_LIBRARIES.md`を更新
- `Lib/README.md`を更新
- `README.md`を更新（必要に応じて）

## 注意事項

1. **名前空間の互換性**: 名前空間（`cmath_fx`、`cquat`、`quat_lib`）は変更しないため、コードの変更は最小限になります

2. **段階的な移行**: 一度にすべてを移行せず、ファイルごとに移行してコンパイル確認を行います

3. **既存のLibライブラリ**: 
   - `Lib/Matrix/matrix.hpp`（`lib::matrix::Mat`）は未使用のため、保持または削除を検討
   - `Lib/Quaternion/quaternion.hpp`（`lib::quat::`）は未使用のため、保持または削除を検討

4. **依存関係**: `quaternion.hpp`は`fixed_matrix.hpp`に依存しているため、`fixed_matrix.hpp`を先に移行する必要があります

5. **ビルド確認**: 各段階でビルドが成功することを確認します

## 移行後の構造

```
Lib/
├── Common/
│   └── types.hpp          # 型定義（既存）
├── Matrix/
│   ├── fixed_matrix.hpp   # ✅ 移行（主要行列ライブラリ）
│   ├── matrix.hpp         # ⚠️ 未使用（削除検討）
│   └── decomposition.hpp  # コレスキー分解（既存）
├── Quaternion/
│   ├── quaternion_functions.hpp  # ✅ 移行（関数ベース）
│   ├── quaternion_lib.hpp        # ✅ 移行（クラスベース）
│   └── quaternion.hpp     # ⚠️ 未使用（削除検討）
└── KalmanCore/
    └── gain.hpp           # カルマンゲイン（既存）
```


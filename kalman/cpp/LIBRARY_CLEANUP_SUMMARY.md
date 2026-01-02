# ライブラリ整理サマリー

最終更新: 2025-01-02

## 実施した整理内容

### 1. 実装ファイルが存在しないヘッダーの削除

以下のヘッダーファイルは実装ファイル（.cpp）が存在しないため、使用不可能な状態でした。削除しました。

- ❌ `Inc/Common/Math/quaternion_compute.hpp` - 削除済み
- ❌ `Inc/Common/Math/rotation_compute.hpp` - 削除済み
- ❌ `Inc/Common/Math/compute_types.hpp` - 削除済み（未使用）
- ❌ `Inc/ESKF/eskf_compute.hpp` - 削除済み（実装ファイルなし、未使用）

### 2. 未使用includeの削除

- ✅ `src/UKF/ukf_sigma_points.cpp`
  - `#include "../../Lib/Matrix/decomposition.hpp"` を削除
  - `#include "../../Lib/Common/types.hpp"` を削除
  - 実際には使用されていなかった（独自実装を使用）

### 3. Libフォルダの非推奨化

`Lib/`フォルダ全体は現在使用されていませんが、歴史的経緯を考慮し、削除せず非推奨として保持しました。

- ✅ `Lib/DEPRECATED.md` を作成（非推奨理由を説明）
- ✅ `Lib/README.md` を更新（非推奨であることを明記）

**Libフォルダの現状**:
- `Lib/Matrix/matrix.hpp` - 未使用
- `Lib/Matrix/decomposition.hpp` - 未使用
- `Lib/Quaternion/quaternion.hpp` - 未使用
- `Lib/Common/types.hpp` - 未使用
- `Lib/KalmanCore/gain.hpp` - 未使用

実際には、すべてのフィルタ実装は`Inc/Common/Math/`のライブラリを使用しています。

### 4. ドキュメントの更新

- ✅ `STRUCTURE_AND_LIBRARIES.md` を更新（削除済みライブラリを反映）
- ✅ `LIBRARY_CLEANUP_PLAN.md` を作成（整理計画）
- ✅ `LIBRARY_CLEANUP_SUMMARY.md` を作成（このファイル）

## 整理後の使用ライブラリ

### ✅ 使用中のライブラリ

| 用途 | ライブラリ | ファイル | 使用箇所 |
|------|-----------|---------|---------|
| **行列計算** | `cmath_fx::Matrix<R, C, T>` | `Inc/Common/Math/fixed_matrix.hpp` | 全フィルタで統一使用 |
| **クォータニオン（通常）** | `cquat::` | `Inc/Common/Math/quaternion.hpp` | ESKF (core), MEUKF (core) |
| **クォータニオン（高機能）** | `quat_lib::Quaternion<T>` | `Inc/Common/Math/quaternion_lib.hpp` | ESKF (initializer, postprocess) |
| **統計計算** | `common::math::` | `Inc/Common/Math/statistics.hpp` | ESKF (initializer) |

## 整理の効果

1. **コードベースの明確化**: 使用されていないライブラリを削除/非推奨化することで、どのライブラリを使用すべきかが明確になりました

2. **混乱の解消**: 複数の類似ライブラリが存在していた混乱が解消されました

3. **メンテナンス性の向上**: 使用されていないコードが削除され、メンテナンス対象が明確になりました

4. **ドキュメントの整備**: ライブラリの使用状況がドキュメント化され、新規開発者にも理解しやすくなりました

## 今後の方針

1. **新しいコードを書く場合**: `STRUCTURE_AND_LIBRARIES.md`の「推奨使用ガイド」に従う

2. **Libフォルダ**: 将来的に完全に削除することも検討可能（現在は非推奨として保持）

3. **ライブラリの追加**: 新しいライブラリを追加する場合は、既存のライブラリと機能が重複していないか確認する


# C++ ライブラリ移行ステータス

## 移行計画
- `Inc/Common/Math/fixed_matrix.hpp` → `Lib/Matrix/fixed_matrix.hpp`
- `Inc/Common/Math/quaternion.hpp` → `Lib/Quaternion/quaternion_functions.hpp`
- `Inc/Common/Math/quaternion_lib.hpp` → `Lib/Quaternion/quaternion_lib.hpp`

## 移行状況

### フェーズ1: ファイルのコピーと名称変更
- [x] `Lib/Matrix/fixed_matrix.hpp` を作成
- [x] `Lib/Quaternion/quaternion_functions.hpp` を作成
- [x] `Lib/Quaternion/quaternion_lib.hpp` を作成

### フェーズ2: インクルードパスの更新
- [x] `Inc/` フォルダ内のファイル (`Inc/ESKF/`, `Inc/EKF/`, `Inc/KF/`, `Inc/UKF/`, `Inc/MEUKF/`, `Inc/Common/`)
  - `#include "Common/Math/fixed_matrix.hpp"` → `#include "../Lib/Matrix/fixed_matrix.hpp"`
  - `#include "Common/Math/quaternion.hpp"` → `#include "../Lib/Quaternion/quaternion_functions.hpp"`
  - `#include "Common/Math/quaternion_lib.hpp"` → `#include "../Lib/Quaternion/quaternion_lib.hpp"`
- [x] `Src/` フォルダ内のファイル
  - `#include "../../Inc/Common/Math/fixed_matrix.hpp"` → `#include "../../Lib/Matrix/fixed_matrix.hpp"`
  - `#include "../../Inc/Common/Math/quaternion.hpp"` → `#include "../../Lib/Quaternion/quaternion_functions.hpp"`
  - `#include "../../Inc/Common/Math/quaternion_lib.hpp"` → `#include "../../Lib/Quaternion/quaternion_lib.hpp"`
- [x] `MEX/Inc/` フォルダ内のファイル
  - `#include "../../Inc/Common/Math/fixed_matrix.hpp"` → `#include "../../Lib/Matrix/fixed_matrix.hpp"`
  - `#include "../../Inc/Common/Math/quaternion_lib.hpp"` → `#include "../../Lib/Quaternion/quaternion_lib.hpp"`

### フェーズ3: コンパイル確認
- [x] `build_mex()` を実行し、正常にコンパイルされることを確認

### フェーズ4: 旧ファイルの削除
- [x] `Inc/Common/Math/` から元のファイルを削除
  - `fixed_matrix.hpp` 削除済み
  - `quaternion.hpp` 削除済み
  - `quaternion_lib.hpp` 削除済み

### フェーズ5: ドキュメントの更新
- [ ] `STRUCTURE_AND_LIBRARIES.md` を更新
- [ ] `Lib/README.md` を更新

## 移行完了日
2025-01-02

## 備考
- すべてのincludeパスを相対パス (`../Lib/...` または `../../Lib/...`) に更新しました
- ビルドと実行が正常に完了したことを確認しました
- 旧ファイルは削除されました

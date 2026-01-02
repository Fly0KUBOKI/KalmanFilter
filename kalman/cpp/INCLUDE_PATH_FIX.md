# Includeパス修正完了

最終更新: 2025-01-02

## 問題

`Inc/`フォルダ内のファイルから`Lib/`をインクルードする場合、`#include "Lib/..."`という形式では見つからないエラーが発生していました。

## 解決策

`Inc/`フォルダ内のファイルから`Lib/`をインクルードする場合、相対パス`../Lib/...`を使用するように変更しました。

### 修正パターン

- `#include "Lib/Matrix/fixed_matrix.hpp"` → `#include "../Lib/Matrix/fixed_matrix.hpp"`
- `#include "Lib/Quaternion/quaternion_functions.hpp"` → `#include "../Lib/Quaternion/quaternion_functions.hpp"`
- `#include "Lib/Quaternion/quaternion_lib.hpp"` → `#include "../Lib/Quaternion/quaternion_lib.hpp"`

### 修正したファイル

すべての`Inc/`フォルダ内のファイル（16ファイル）を修正しました：

- `Inc/ESKF/` - 7ファイル
- `Inc/EKF/` - 2ファイル
- `Inc/KF/` - 2ファイル
- `Inc/UKF/` - 2ファイル
- `Inc/MEUKF/` - 3ファイル

### その他のフォルダ

- `Src/`フォルダ内のファイル: `#include "Lib/..."`のまま（`Lib/`がインクルードパスに含まれているため問題なし）
- `MEX/Inc/`フォルダ内のファイル: `#include "Lib/..."`のまま（`Lib/`がインクルードパスに含まれているため問題なし）

## 確認

コンパイルが正常に動作することを確認してください。


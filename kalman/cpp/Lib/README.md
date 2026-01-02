# Lib ライブラリ

## ✅ 使用中 - 行列計算・クォータニオン計算ライブラリ

このフォルダには、フィルタ実装で使用される独立したライブラリが含まれています。

## ディレクトリ構造

```
Lib/
├── Matrix/
│   └── fixed_matrix.hpp        # cmath_fx::Matrix, FixedMatrix (全てのフィルタで使用)
│
└── Quaternion/
    └── quaternion_functions.hpp # cquat:: 関数群 (全てのクォータニオン計算で使用)
```

## ライブラリの使用状況

### 行列計算 (`Lib/Matrix/fixed_matrix.hpp`)
- **`cmath_fx::Matrix<R, C, T>`**: 固定サイズの行列/ベクトル
- **`cmath_fx::FixedMatrix`**: 動的サイズの行列（最大容量固定）
- **使用状況**: ESKF、MEUKF、EKF、UKFの**全てのフィルタ実装で統一的に使用**

### クォータニオン計算

#### `Lib/Quaternion/quaternion_functions.hpp` (`cquat::`)
- **特徴**: `cmath_fx::Vector<4, T>` を基盤とした関数群
- **使用状況**: 全てのクォータニオン計算で使用（`float`精度）
- **使用ファイル**: `eskf_core.cpp`, `eskf_math.cpp`, `meukf_core.cpp`, `eskf_initializer.cpp`, `eskf_runner.cpp`, `eskf_postprocess.cpp`

## 推奨使用ガイド

### 新しいコードを書く際
- **行列計算**: `cmath_fx::Matrix<R, C, T>` を常に使用してください
- **クォータニオン計算**: `cquat::` 関数群を使用してください（`float`精度、`cmath_fx::Vector<4, float>`と連携）

## 移行履歴

これらのライブラリは、2025-01-02に `Inc/Common/Math/` から移行されました。
- `fixed_matrix.hpp` → `Lib/Matrix/fixed_matrix.hpp`
- `quaternion.hpp` → `Lib/Quaternion/quaternion_functions.hpp`
- `quaternion_lib.hpp` → `Lib/Quaternion/quaternion_functions.hpp` (統合済み)

詳細は `LIBRARY_MIGRATION_STATUS.md` を参照してください。

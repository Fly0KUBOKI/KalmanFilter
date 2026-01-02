# Libフォルダ - 非推奨（未使用）

**最終更新**: 2025-01-02

## 概要

`Lib/`フォルダは、MEUKF向けに設計された独立ライブラリでしたが、実際には使用されていません。

**現在の状況**: すべてのフィルタ実装（ESKF、MEUKF、EKF、UKF）は`Inc/Common/Math/`のライブラリを使用しています。

## 使用されていない理由

1. **行列ライブラリ**: `Lib/Matrix/matrix.hpp`は使用されず、代わりに`Inc/Common/Math/fixed_matrix.hpp`（`cmath_fx::Matrix`）が統一使用されている

2. **クォータニオンライブラリ**: `Lib/Quaternion/quaternion.hpp`は使用されず、代わりに`Inc/Common/Math/quaternion.hpp`（`cquat::`）が使用されている

3. **コレスキー分解**: `Lib/Matrix/decomposition.hpp`は`ukf_sigma_points.cpp`でincludeされていたが、実際には使用されていない（独自実装を使用）

4. **カルマンゲイン**: `Lib/KalmanCore/gain.hpp`は使用されていない

## 削除の推奨

`Lib/`フォルダ全体は以下の理由で削除を推奨します：

- 実際に使用されていない
- `Inc/Common/Math/`のライブラリが統一使用されている
- コードベースの混乱を招く

ただし、歴史的経緯や将来の拡張を考慮し、削除するか非推奨として保持するかはプロジェクトの方針に従ってください。

## 現在の使用ライブラリ

### 行列計算
- `Inc/Common/Math/fixed_matrix.hpp` - `cmath_fx::Matrix<R, C, T>` ✅

### クォータニオン計算
- `Inc/Common/Math/quaternion.hpp` - `cquat::` ✅
- `Inc/Common/Math/quaternion_lib.hpp` - `quat_lib::Quaternion<T>` ✅

### 統計計算
- `Inc/Common/Math/statistics.hpp` - `common::math::` ✅


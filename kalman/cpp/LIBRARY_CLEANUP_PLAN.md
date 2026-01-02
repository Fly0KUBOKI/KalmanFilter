# ライブラリ整理計画

最終更新: 2025-01-02

## 整理の方針

コードベースに複数のライブラリ実装が存在し、混乱を招いているため、使用状況を正確に把握し、未使用のものを整理します。

## 使用状況の確認結果

### ✅ 使用中のライブラリ

#### 1. `Inc/Common/Math/fixed_matrix.hpp` ✅
- **状態**: 全フィルタで統一使用
- **名前空間**: `cmath_fx::Matrix<R, C, T>`
- **使用箇所**: ESKF, EKF, UKF, MEUKFすべて

#### 2. `Inc/Common/Math/quaternion.hpp` ✅
- **状態**: 主要なクォータニオンライブラリ
- **名前空間**: `cquat::`
- **使用箇所**: ESKF (core), MEUKF (core)

#### 3. `Inc/Common/Math/quaternion_lib.hpp` ✅
- **状態**: 高機能クォータニオンライブラリ
- **名前空間**: `quat_lib::Quaternion<T>`
- **使用箇所**: ESKF (initializer, postprocess)

#### 4. `Inc/Common/Math/statistics.hpp` ✅
- **状態**: 統計計算
- **名前空間**: `common::math::`
- **使用箇所**: ESKF (initializer)

---

### ⚠️ 未使用または未確認のライブラリ

#### 1. `Inc/Common/Math/quaternion_compute.hpp` ⚠️
- **状態**: 実装ファイル（.cpp）が存在しない
- **名前空間**: `kalman_compute::QuaternionCompute`
- **使用箇所**: なし
- **対応**: 削除を推奨（実装ファイルが存在しないため使用不可能）

#### 2. `Inc/Common/Math/rotation_compute.hpp` ⚠️
- **状態**: 実装ファイル（.cpp）が存在しない
- **名前空間**: `kalman_compute::RotationCompute`
- **使用箇所**: 未確認
- **対応**: 使用状況を確認し、未使用であれば削除

#### 3. `Lib/Matrix/matrix.hpp` ⚠️
- **状態**: 使用されていない
- **名前空間**: `lib::matrix::Mat<R, C, T>`
- **使用箇所**: なし
- **対応**: 削除を推奨（`cmath_fx::Matrix`が統一使用されている）

#### 4. `Lib/Matrix/decomposition.hpp` ⚠️
- **状態**: `ukf_sigma_points.cpp`でincludeされているが、実際には使用されていない
- **名前空間**: `lib::matrix::`
- **使用箇所**: `src/UKF/ukf_sigma_points.cpp`（includeのみ）
- **対応**: includeを削除し、実際の使用がないことを確認後、削除を検討

#### 5. `Lib/Quaternion/quaternion.hpp` ⚠️
- **状態**: 使用されていない
- **名前空間**: `lib::quat::`
- **使用箇所**: なし
- **対応**: 削除を推奨（`cquat::`と`quat_lib::`が使用されている）

#### 6. `Lib/Common/types.hpp` ⚠️
- **状態**: `ukf_sigma_points.cpp`でincludeされているが、実際には使用されていない
- **使用箇所**: `src/UKF/ukf_sigma_points.cpp`（includeのみ）
- **対応**: includeを削除し、実際の使用がないことを確認後、削除を検討

#### 7. `Lib/KalmanCore/gain.hpp` ⚠️
- **状態**: 使用されていない
- **名前空間**: `lib::kalman::`
- **使用箇所**: なし
- **対応**: 削除を推奨

---

## 整理の手順

### Phase 1: 実装ファイルが存在しないヘッダーの削除

1. **`Inc/Common/Math/quaternion_compute.hpp`**
   - 理由: 実装ファイル（`Src/Common/Math/quaternion_compute.cpp`）が存在しない
   - 動作: 削除

2. **`Inc/Common/Math/rotation_compute.hpp`**
   - 理由: 実装ファイル（`Src/Common/Math/rotation_compute.cpp`）が存在しない
   - 動作: 使用状況を確認後、未使用であれば削除

### Phase 2: 未使用includeの削除

3. **`src/UKF/ukf_sigma_points.cpp`**
   - `#include "../../Lib/Matrix/decomposition.hpp"` を削除（実際には使用されていない）
   - `#include "../../Lib/Common/types.hpp"` を削除（実際には使用されていない）

### Phase 3: 未使用Libライブラリの削除

4. **`Lib/`フォルダ全体**
   - 現状: `Lib/Matrix/decomposition.hpp`がincludeされているが、実際には使用されていない
   - 対応: 使用状況を再確認し、未使用であれば`Lib/`フォルダ全体を削除または非推奨化

   **削除候補**:
   - `Lib/Matrix/matrix.hpp`
   - `Lib/Matrix/decomposition.hpp`
   - `Lib/Quaternion/quaternion.hpp`
   - `Lib/Common/types.hpp`
   - `Lib/KalmanCore/gain.hpp`
   - `Lib/README.md`

---

## 注意事項

1. **`Lib/`フォルダの設計思想**
   - `Lib/README.md`によると、MEUKF向けに設計された独立ライブラリ
   - しかし、実際には`Inc/Common/Math/`のライブラリが使用されている
   - 歴史的経緯を考慮し、削除前に確認が必要

2. **実装ファイルの存在確認**
   - `quaternion_compute.cpp`と`rotation_compute.hpp`の実装ファイルが存在しない
   - これらのヘッダーファイルは使用不可能な状態

3. **段階的な整理**
   - まず実装ファイルが存在しないヘッダーを削除
   - 次に未使用includeを削除
   - 最後にLibフォルダ全体の整理を検討

---

## 実行計画

### Step 1: 実装ファイルが存在しないヘッダーの確認と削除
- [ ] `quaternion_compute.hpp`の使用状況を最終確認
- [ ] `rotation_compute.hpp`の使用状況を確認
- [ ] 未使用を確認したら削除

### Step 2: 未使用includeの削除
- [ ] `ukf_sigma_points.cpp`から未使用includeを削除
- [ ] コンパイル確認

### Step 3: Libフォルダの整理
- [ ] `Lib/`フォルダ内の全ファイルの使用状況を再確認
- [ ] 未使用を確認したら削除または非推奨化
- [ ] `Lib/README.md`を更新（または削除）

### Step 4: ドキュメントの更新
- [ ] `STRUCTURE_AND_LIBRARIES.md`を更新
- [ ] `README.md`を更新（必要に応じて）


# C++コードベース構造とライブラリ整理

最終更新: 2025-01-02 (ライブラリ移行完了: Inc/Common/Math/ → Lib/)

## 目次

1. [ディレクトリ構造](#ディレクトリ構造)
2. [クォータニオンライブラリ](#クォータニオンライブラリ)
3. [行列計算ライブラリ](#行列計算ライブラリ)
4. [数学ライブラリ一覧](#数学ライブラリ一覧)
5. [フィルタ実装と使用ライブラリ](#フィルタ実装と使用ライブラリ)
6. [推奨使用ガイド](#推奨使用ガイド)

---

## ディレクトリ構造

```
cpp/
├── Inc/              # ヘッダーファイル（アルゴリズム実装）
│   ├── Common/       # 共通ライブラリ
│   │   └── Math/     # 数学ライブラリ（固定サイズ行列、クォータニオン、統計など）
│   ├── ESKF/         # ESKF実装
│   ├── EKF/          # EKF実装
│   ├── UKF/          # UKF実装
│   └── MEUKF/        # MEUKF実装
│
├── Src/              # ソースファイル（実装）
│   ├── Common/
│   ├── ESKF/
│   ├── EKF/
│   ├── UKF/
│   └── MEUKF/
│
├── Lib/              # 独立ライブラリ（✅ 使用中）
│   ├── Matrix/       # 行列ライブラリ（fixed_matrix.hpp - 使用中）
│   ├── Quaternion/   # クォータニオンライブラリ（quaternion_functions.hpp, quaternion_lib.hpp - 使用中）
│   ├── Common/       # 型定義（未使用）
│   ├── KalmanCore/   # カルマンフィルタ基盤（未使用）
│   └── README.md     # ライブラリの説明
│
├── MEX/              # MATLAB MEXラッパー（インターフェースのみ）
│   ├── Inc/          # MEX用ヘッダー
│   └── *.cpp         # MEXエントリーポイント
│
└── build/            # ビルドスクリプト
```

---

## クォータニオンライブラリ

### 概要

コードベースには**4つの異なるクォータニオンライブラリ**が存在しますが、実際に使用されているのは**2つ**です。

### 1. `quat_lib::Quaternion<T>` ✅ **使用中**

**場所**: `Lib/Quaternion/quaternion_lib.hpp`

**特徴**:
- クラスベースの実装
- メンバ変数: `w, x, y, z`
- テンプレート型（`T = float`または`double`）
- 豊富な機能（積分、SLERP、2ベクトル間の回転など）

**使用箇所**:
- `Src/ESKF/eskf_initializer.cpp` - ESKF初期化（`double`型使用）
- `Src/ESKF/eskf_postprocess.cpp` - 後処理
- `Inc/ESKF/eskf_helper.hpp` - ヘルパー関数

**使用例**:
```cpp
#include "Common/Math/quaternion_lib.hpp"

using Quat = quat_lib::Quaternion<double>;

Quat q = Quat::from_euler(roll_deg, pitch_deg, yaw_deg);
q.normalize();

double R[9];
q.to_rotation_matrix(R);
```

---

### 2. `cquat::` ✅ **使用中（主要）**

**場所**: `Lib/Quaternion/quaternion_functions.hpp`

**特徴**:
- インライン関数の集合
- `cmath_fx::Vector<4, T>`をクォータニオンとして扱う
- `cmath_fx::Matrix<3, 3, T>`を回転行列として扱う
- パフォーマンス重視（インライン展開）

**使用箇所**:
- `Src/ESKF/eskf_core.cpp` - ESKFコア実装
- `Src/ESKF/eskf_math.cpp` - ESKF数学関数
- `Src/MEUKF/meukf_core.cpp` - MEUKF実装
- `Inc/ESKF/eskf_core.hpp` - ESKFコアヘッダー
- `Inc/MEUKF/meukf_core.hpp` - MEUKFコアヘッダー

**使用例**:
```cpp
#include "Common/Math/quaternion.hpp"
#include "Common/Math/fixed_matrix.hpp"

using namespace cmath_fx;
using Vector4 = Vector<4, float>;
using Matrix3x3 = Matrix<3, 3, float>;

Vector4 q;  // クォータニオン [w, x, y, z]
cquat::normalize_quat(q);

Vector4 q2;
cquat::multiply_quat(q, dq, q2);

Matrix3x3 R;
cquat::quat_to_rotm(q, R);
```

**主な関数**:
- `normalize_quat()` - 正規化
- `multiply_quat()` - クォータニオン積
- `quat_to_rotm()` - 回転行列変換
- `from_euler_deg()` - オイラー角から生成
- `to_euler_deg()` - オイラー角へ変換

---

### 3. `kalman_compute::QuaternionCompute` ⚠️ **未使用**

**場所**: `Inc/Common/Math/quaternion_compute.hpp`

**特徴**:
- 関数スタイル（静的メンバ関数）
- 配列入出力（`Scalar*`）
- MATLAB側からの呼び出しを想定した設計

**状態**: 現在のコードベースでは使用されていません

---

### 4. `lib::quat::` ⚠️ **未使用**

**場所**: `Lib/Quaternion/quaternion.hpp`

**特徴**:
- MEUKF向けに設計
- `lib::matrix::Mat`を使用
- `lib::matrix::Vec4`をクォータニオンとして使用

**状態**: `Lib/`フォルダは独立ライブラリとして設計されたが、実際には`Inc/Common/Math/`のライブラリが使用されているため、現在は未使用

---

## 行列計算ライブラリ

### 概要

コードベースには**2つの行列ライブラリ**が存在しますが、実際に使用されているのは**1つ**です。

### 1. `cmath_fx::Matrix<R, C, T>` ✅ **使用中（統一使用）**

**場所**: `Lib/Matrix/fixed_matrix.hpp`

**特徴**:
- テンプレートベースの固定サイズ行列
- コンパイル時サイズ決定（`R`行、`C`列）
- 型パラメータ: `T = float`（デフォルト）
- 演算子オーバーロード（`+`, `-`, `*`, `()`など）
- 逆行列計算（Gauss-Jordan法）
- 転置、単位行列生成など

**使用箇所**: **全てのフィルタ実装で使用**
- ESKF (`Inc/ESKF/`, `Src/ESKF/`)
- EKF (`Inc/EKF/`, `Src/EKF/`)
- UKF (`Inc/UKF/`, `Src/UKF/`)
- MEUKF (`Inc/MEUKF/`, `Src/MEUKF/`)

**使用例**:
```cpp
#include "Common/Math/fixed_matrix.hpp"

using namespace cmath_fx;
using Vector3 = Vector<3, float>;      // 3x1ベクトル
using Vector4 = Vector<4, float>;      // 4x1ベクトル
using Matrix3x3 = Matrix<3, 3, float>; // 3x3行列
using Matrix15x15 = Matrix<15, 15, float>;

Vector3 v;
v(0, 0) = 1.0f;
v(1, 0) = 2.0f;
v(2, 0) = 3.0f;

Matrix3x3 A = Matrix3x3::Identity();
Matrix3x3 B = A * A.transpose();

Matrix3x3 A_inv;
if (A.inverse(A_inv)) {
    // 逆行列計算成功
}
```

**主な機能**:
- `Zero()` - ゼロ行列
- `Identity()` - 単位行列
- `operator()(r, c)` - 要素アクセス
- `operator+`, `-`, `*` - 演算子
- `transpose()` - 転置
- `inverse()` - 逆行列

**補足: `FixedMatrix`構造体**
- `FixedMatrix`構造体も同じファイル内に定義
- ランタイムサイズ対応（最大20x20）
- MEXインターフェース用（現在は未使用）

---

### 2. `lib::matrix::Mat<R, C, T>` ⚠️ **未使用**

**場所**: `Lib/Matrix/matrix.hpp`

**特徴**:
- MEUKF向けに設計
- `lib::matrix::Mat`テンプレートクラス
- `lib::matrix::Vec3`, `Vec4`などの型エイリアス

**状態**: `Lib/`フォルダは独立ライブラリとして設計されたが、実際には`cmath_fx::Matrix`が統一使用されているため、現在は未使用

**補足: コレスキー分解**
- `Lib/Matrix/decomposition.hpp`にコレスキー分解が実装されている
- しかし、MEUKFでは`cmath_fx::Matrix`を使用し、独自のコレスキー分解関数を使用している

---

## 数学ライブラリ一覧

### `Inc/Common/Math/`ディレクトリ

| ファイル | 名前空間/クラス | 用途 | 状態 |
|---------|----------------|------|------|
| `fixed_matrix.hpp` | `cmath_fx::Matrix<R, C, T>` | 固定サイズ行列 | ✅ 使用中 |
| `quaternion_lib.hpp` | `quat_lib::Quaternion<T>` | クォータニオン（クラス） | ✅ 使用中（初期化、後処理） |
| `quaternion.hpp` | `cquat::` | クォータニオン（関数） | ✅ 使用中（主要） |
| `statistics.hpp` | `common::math::` | 統計関数（平均、標準偏差） | ✅ 使用中 |
| `vector_utils.hpp` | `common::math::` | ベクトルユーティリティ | ✅ 使用中 |
| `math_utils.hpp` | 各種 | 数学ユーティリティ | ✅ 使用中 |

---

## フィルタ実装と使用ライブラリ

### ESKF

**使用ライブラリ**:
- **行列**: `cmath_fx::Matrix<R, C, float>` （統一使用）
- **クォータニオン**: 
  - `cquat::` （コア実装: `eskf_core.cpp`, `eskf_math.cpp`）
  - `quat_lib::Quaternion<double>` （初期化: `eskf_initializer.cpp`、後処理: `eskf_postprocess.cpp`）
- **統計**: `common::math::` （初期化で使用）

**主要ファイル**:
- `Inc/ESKF/eskf_core.hpp` - コアヘッダー
- `Src/ESKF/eskf_core.cpp` - コア実装
- `Src/ESKF/eskf_initializer.cpp` - 初期化
- `Src/ESKF/eskf_runner.cpp` - 実行制御

---

### MEUKF

**使用ライブラリ**:
- **行列**: `cmath_fx::Matrix<R, C, float>` （統一使用）
- **クォータニオン**: `cquat::` （統一使用）
- **統計**: 使用なし

**主要ファイル**:
- `Inc/MEUKF/meukf_core.hpp` - コアヘッダー
- `Src/MEUKF/meukf_core.cpp` - コア実装

**注意**: MEUKFは`Lib/`フォルダのライブラリを使用

---

### EKF / UKF

**使用ライブラリ**:
- **行列**: `cmath_fx::Matrix<R, C, float>` （統一使用）
- **クォータニオン**: 使用なし（または`cquat::`）

---

## 推奨使用ガイド

### 新しいコードを書く場合

#### 1. 行列計算

**必ず使用**: `cmath_fx::Matrix<R, C, float>`

```cpp
#include "Common/Math/fixed_matrix.hpp"

using namespace cmath_fx;
using Vector3 = Vector<3, float>;
using Matrix3x3 = Matrix<3, 3, float>;
```

#### 2. クォータニオン計算

**通常の計算**: `cquat::`関数を使用

```cpp
#include "Common/Math/quaternion.hpp"
#include "Common/Math/fixed_matrix.hpp"

using namespace cmath_fx;
using Vector4 = Vector<4, float>;
using Matrix3x3 = Matrix<3, 3, float>;

Vector4 q;
cquat::normalize_quat(q);
cquat::multiply_quat(q1, q2, result);
cquat::quat_to_rotm(q, R);
```

**初期化や高機能が必要な場合**: `quat_lib::Quaternion<T>`を使用

```cpp
#include "Common/Math/quaternion_lib.hpp"

using Quat = quat_lib::Quaternion<float>;

Quat q = Quat::from_euler(roll, pitch, yaw);
q.normalize();
Quat q2 = Quat::integrate(q, wx, wy, wz, dt);
```

#### 3. 統計計算

```cpp
#include "Common/Math/statistics.hpp"  // 統計関数はInc/Common/Math/に残存

using namespace common::math;

double mean = compute_mean(data, n);
compute_mean_3d(ax, ay, az, n, &mean_x, &mean_y, &mean_z);
double std = compute_std(data, n, mean);
```

---

## まとめ

### 使用するライブラリ

| 用途 | ライブラリ | ファイル |
|------|-----------|---------|
| **行列計算** | `cmath_fx::Matrix<R, C, T>` | `Lib/Matrix/fixed_matrix.hpp` |
| **クォータニオン（通常）** | `cquat::` | `Lib/Quaternion/quaternion_functions.hpp` |
| **クォータニオン（高機能）** | `quat_lib::Quaternion<T>` | `Lib/Quaternion/quaternion_lib.hpp` |
| **統計計算** | `common::math::` | `Inc/Common/Math/statistics.hpp` |

### 使用しないライブラリ

- `Lib/Matrix/matrix.hpp` - 未使用（`Lib/Matrix/fixed_matrix.hpp`を使用）
- `Lib/Quaternion/quaternion.hpp` - 未使用（`Lib/Quaternion/quaternion_functions.hpp`を使用）
- `Lib/KalmanCore/gain.hpp` - 未使用
- `Inc/Common/Math/quaternion_compute.hpp` - ❌ 削除済み（実装ファイルなし）
- `Inc/Common/Math/rotation_compute.hpp` - ❌ 削除済み（実装ファイルなし）
- `Inc/Common/Math/compute_types.hpp` - ❌ 削除済み（未使用）
- `Inc/ESKF/eskf_compute.hpp` - ❌ 削除済み（実装ファイルなし）

### 設計原則

1. **統一性**: すべてのフィルタ実装で`cmath_fx::Matrix`を使用
2. **パフォーマンス**: インライン関数（`cquat::`）を優先使用
3. **機能性**: 高機能が必要な場合のみクラスベース（`quat_lib::Quaternion`）を使用

---

## 参考資料

- `README.md` - コードベース構造の概要
- `CPP_DEPENDENCIES.md` - 依存関係の詳細
- `Lib/README.md` - Libフォルダの設計思想（ただし現在は未使用）


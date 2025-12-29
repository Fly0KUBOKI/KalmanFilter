# Lib ライブラリ

## 概要

Libフォルダは、MEUKFを含む全てのカルマンフィルタ実装が使用する共通ライブラリを提供します。
各ライブラリは独立して完結し、外部依存を最小化しています。

## ディレクトリ構造

```
Lib/
├── Common/           # 共通定義
│   └── types.hpp     # 型定義 (Scalar=float, Index=uint8_t)
├── Matrix/           # 静的行列ライブラリ
│   ├── matrix.hpp    # 行列クラス定義
│   └── decomposition.hpp  # コレスキー分解等
├── Quaternion/       # クォータニオンライブラリ
│   └── quaternion.hpp
└── KalmanCore/      # カルマンフィルタ基盤
    └── gain.hpp     # カルマンゲイン計算
```

## 使用方法

### 基本型

```cpp
#include <Lib/Common/types.hpp>

using namespace lib;

Scalar x = 1.0f;  // float
Index i = 5;      // uint8_t
```

### 行列操作

```cpp
#include <Lib/Matrix/matrix.hpp>

using namespace lib::matrix;

Mat<3, 3> A = Mat<3, 3>::Identity();
Vec3 v;
v(0, 0) = 1.0f;
v(1, 0) = 2.0f;
v(2, 0) = 3.0f;

Mat<3, 3> B = A * A.transpose();
```

### コレスキー分解

```cpp
#include <Lib/Matrix/decomposition.hpp>

using namespace lib::matrix;

Mat<3, 3> P;
Mat<3, 3> L;
if (cholesky(P, L)) {
    // 成功
} else {
    // 失敗 - 正定値でない
}

// 堅牢版（正則化付き）
cholesky_robust(P, L);
```

### クォータニオン

```cpp
#include <Lib/Quaternion/quaternion.hpp>

using namespace lib::quat;

Quat q = from_euler_deg(0.0f, 0.0f, 0.0f);
Quat q2 = integrate(q, omega, dt);
Mat3 R = to_rotation_matrix(q2);
Vec3 euler = to_euler_deg(q2);
```

### カルマンゲイン

```cpp
#include <Lib/KalmanCore/gain.hpp>

using namespace lib::kalman;

Mat<15, 3> K;
Status status = compute_gain(P, H, S, K);
if (status == STATUS_OK) {
    // ゲイン計算成功
}
```

## 設計方針

1. **型の統一**: 全て`float`型、インデックスは`uint8_t`
2. **テンプレート使用**: コンパイル時サイズ決定
3. **名前空間**: `lib::matrix`, `lib::quat`, `lib::kalman`
4. **エラー処理**: `Status` enumで返す
5. **外部依存なし**: Eigen等の外部ライブラリ不使用

## 移行ガイド

既存コードからLibへの移行:

```cpp
// 旧コード
#include "../Common/Math/fixed_matrix.hpp"
using cm = cmath_fx::FixedMatrix;

// 新コード
#include <Lib/Matrix/matrix.hpp>
using namespace lib::matrix;
using Vec3 = Vec<3>;
```





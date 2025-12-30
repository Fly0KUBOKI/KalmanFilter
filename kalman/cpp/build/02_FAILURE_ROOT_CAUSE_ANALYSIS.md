# ビルド失敗の根本原因分析

**生成日**: 2025-12-30  
**エラー総数**: 104個以上  
**影響範囲**: 2 MEX ファイル (`mex_run_eskf.cpp`, `mex_eskf_update_postprocess.cpp`)

---

## エラー分類

### 種別別エラー統計

| エラー種別 | 発生数 | 優先度 | 対応済み |
|----------|-------|------|--------|
| 型認識エラー | 45個 | 🔴 P1 | ❌ 未対応 |
| スコープエラー | 30個 | 🔴 P1 | ❌ 未対応 |
| 関数シグネチャ不一致 | 20個 | 🟡 P2 | ⚠️ 部分対応 |
| 未実装関数 | 5個 | 🟡 P2 | ✓ 確認済み |
| その他 | 4個 | 🟢 P3 | ? |

---

## 根本原因 #1: 型認識エラー（最優先）

### エラー例

```
C:\...\vector_utils.hpp(14): error C4430: 型指定子がありません - int と仮定しました。
C:\...\mex_type_conversion.hpp(17): error C2061: 構文エラー: 識別子 'Vector'
C:\...\mex_type_conversion.hpp(31): error C2061: 構文エラー: 識別子 'Matrix'
```

### 問題のコード

#### `vector_utils.hpp` の問題

```cpp
// Line 14
template<typename T>
T norm3(const T* v) {  // ← エラー: T が型として認識されない
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}
```

**コンパイラのメッセージ**:
```
error C4430: 型指定子がありません - int と仮定しました。
```

これは `T` が `int` として扱われていることを意味します。

#### `mex_type_conversion.hpp` の問題

```cpp
// 型定義が認識されない
template<int R>
bool matToVector(const mxArray* arr, Vector<R, float>& out) {
    //                                  ^^^^^^
    //                 ← Vector が認識されていない
}
```

### 根本原因の仮説

1. **インクルード順序の問題**
   ```cpp
   // mex_run_eskf.cpp の順序
   #include "../Inc/Common/Math/quaternion_lib.hpp"
   #include "../Inc/Common/Math/vector_utils.hpp"
   #include "../Inc/Common/filter_management.hpp"
   #include "../Inc/Common/Math/fixed_matrix.hpp"  ← fixed_matrix が遅すぎる
   ```
   
   **修正方法**: `fixed_matrix.hpp` を最初に配置
   
   ```cpp
   #include "../Inc/Common/Math/fixed_matrix.hpp"  // 最初
   #include "../Inc/Common/Math/vector_utils.hpp"
   #include "../Inc/Common/Math/quaternion_lib.hpp"
   ```

2. **固有の型定義が見つからない**
   - `fixed_matrix.hpp` で `Vector`, `Matrix` が `cmath_fx` 名前空間で定義されているが、
   - `vector_utils.hpp` では `cmath_fx::` プレフィックスなしで使用されている
   
   **確認すべき点**:
   ```cpp
   // fixed_matrix.hpp
   namespace cmath_fx {
       template<int R, typename T = float>
       using Vector = Matrix<R, 1, T>;
       
       template<int R, int C, typename T = float>
       class Matrix { ... };
   }
   ```
   
   **vector_utils.hpp での使用**:
   ```cpp
   // 明示的な名前空間が必要かもしれない
   template<typename T>
   T norm3(const T* v) {
       using namespace cmath_fx;  // ← この行が必要？
       // ...
   }
   ```

3. **インクルード ガード / 二重定義の問題**
   - どこかで `Vector` や `Matrix` が予期しない場所で定義されている
   - または、インクルードガードが正しく機能していない

### 修正提案

#### 方法1: インクルード順序の修正（確実）

**ファイル**: `kalman/cpp/MEX/mex_run_eskf.cpp`

```cpp
// 修正前
#include "../Inc/Common/Math/quaternion_lib.hpp"
#include "../Inc/Common/Math/vector_utils.hpp"
#include "../Inc/Common/filter_management.hpp"
#include "../Inc/Common/Math/fixed_matrix.hpp"

// 修正後
#include "../Inc/Common/Math/fixed_matrix.hpp"        // 最初に配置
#include "../Inc/Common/Math/vector_utils.hpp"
#include "../Inc/Common/Math/quaternion_lib.hpp"
#include "../Inc/Common/filter_management.hpp"
```

#### 方法2: 明示的な名前空間の使用（確実）

**ファイル**: `kalman/cpp/Lib/Common/vector_utils.hpp`

```cpp
#ifndef VECTOR_UTILS_HPP
#define VECTOR_UTILS_HPP

#include "fixed_matrix.hpp"

namespace cmath_fx {

template<typename T>
T norm3(const T* v) {  // ← T の型が確定している
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// ...その他の関数

}  // namespace cmath_fx

#endif
```

---

## 根本原因 #2: スコープエラー

### エラー例

```
C:\...\mex_run_eskf.cpp(122): error C2065: 'R_row': 定義されていない識別子です。
C:\...\mex_run_eskf.cpp(266): error C2065: 'zeros3': 定義されていない識別子です。
C:\...\mex_run_eskf.cpp(218): error C2065: 'R_noise': 定義されていない識別子です。
```

### 問題のコード

#### `R_row` スコープエラー

```cpp
static void quaternion_to_rotation_matrix(const double* q, double* R) {
    Quat quat(q[0], q[1], q[2], q[3]);
    quat.normalize();

    // quaternion_lib.hppのto_rotation_matrixはrow-majorで返す
    // MATLABはcolumn-majorなので変換が必要
    double R_row[9];  // ← ここで定義（スコープ内）
    quat.to_rotation_matrix(R_row);

    // row-major -> column-major変換
    R[0] = R_row[0];  // OK（同じスコープ内）
    // ...
}

// 他の場所で R_row を使用
R_row[2] = ...;  // ❌ エラー: R_row はこのスコープに存在しない
```

### 根本原因

1. **ローカル変数のスコープ外参照**
   - `R_row` が `quaternion_to_rotation_matrix` 関数内で定義
   - しかし、別の関数やスコープで使用されている

2. **コピー＆ペースト時のスコープミス**
   - 統合時に、関数から値を取り出すコードが正しく配置されていない

### 修正提案

#### 方法1: 変数をスコープの外に出す

```cpp
// 修正前
static void quaternion_to_rotation_matrix(const double* q, double* R) {
    double R_row[9];
    // ...
}
// R_row はここでスコープ外

// 修正後
double R_row[9];  // グローバル または関数外のスコープ

static void quaternion_to_rotation_matrix(const double* q, double* R) {
    // ...
    quat.to_rotation_matrix(R_row);
}
```

#### 方法2: 変数をローカルに保つ（推奨）

```cpp
static void quaternion_to_rotation_matrix(const double* q, double* R) {
    Quat quat(q[0], q[1], q[2], q[3]);
    quat.normalize();

    double R_row[9];
    quat.to_rotation_matrix(R_row);

    // row-major -> column-major変換
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i + j*3] = R_row[j*3 + i];
        }
    }
}

// ← R_row は使用されない
```

---

## 根本原因 #3: 関数シグネチャの不一致

### エラー例

```
C:\...\mex_eskf_update_postprocess.cpp(41): error C2672: 'matToVector': 
    一致するオーバーロードされた関数が見つかりませんでした。
note: 'bool mex_conv::matToVector(const mxArray *)': 
    1 引数が必要です - 2 が指定されました
```

### 問題のコード

#### `mex_type_conversion.hpp` の定義

```cpp
namespace mex_conv {

// テンプレート関数（引数1個）
template<int R>
bool matToVector(const mxArray* arr, Vector<R, float>& out) {
    // ...
    return true;
}

}  // namespace mex_conv
```

#### `mex_eskf_update_postprocess.cpp` での呼び出し

```cpp
// 呼び出し（引数2個）
matToVector<15>(prhs[1], dx);  // ❌ テンプレートパラメータを指定していない
```

### 根本原因

1. **テンプレートパラメータが推論されていない**
   - `Vector<15, float>` の `15` が推論されていない
   - 明示的に指定する必要がある

2. **マクロの定義漏れ**
   - 以前のコードでは、マクロで簡易的に定義されていたが、
   - 新しい `mex_type_conversion.hpp` では完全なテンプレート形式になった

### 修正提案（既に実装済み）

#### 修正内容

```cpp
// 修正前
matToVector(prhs[1], dx);

// 修正後
matToVector<15>(prhs[1], dx);
```

**状態**: FIX_STATUS.md に記載されている通り、一部修正済みですが、
型認識エラーの影響で、修正が反映されていない可能性があります。

---

## 根本原因 #4: 未実装関数

### エラー例

```
C:\...\mex_eskf_update_postprocess.cpp(107): error C3861: 
    'update_state_from_dx': 識別子が見つかりませんでした
```

### 問題のコード

#### ヘッダーでの宣言

```cpp
// eskf_postprocess.hpp
UpdatePostprocessResult update_state_from_dx(
    const cmath_fx::Vector<15, float>& dx,
    const cmath_fx::Vector<3, float>& state_p,
    // ...
);
```

#### 実装の確認

```
Src/ESKF/eskf_postprocess.cpp が存在するか？
→ 存在する場合: ビルドスクリプトに含まれているか？
→ 存在しない場合: 実装が必要
```

### 修正提案

#### 方法1: 実装を確認して build_mex.m に追加

```matlab
% kalman/cpp/build/build_mex.m
mex(..., 'Src/ESKF/eskf_postprocess.cpp', ...)
```

#### 方法2: ヘッダー内にインライン実装

```cpp
// eskf_postprocess.hpp
namespace eskf {

inline UpdatePostprocessResult update_state_from_dx(...) {
    // 実装
}

}
```

---

## エラーの連鎖効果

```
1. fixed_matrix.hpp のインクルード問題
   ↓
2. Vector<> と Matrix<> が認識されない
   ↓
3. mex_type_conversion.hpp の型定義が失敗
   ↓
4. すべての mex_conv::matToVector() 呼び出しがエラー
   ↓
5. コンパイラが以降の行を正しく解析できない
   ↓
6. スコープエラー、シグネチャエラーが多発
```

### 解決の優先度

```
🔴 P1: fixed_matrix.hpp のインクルード順序修正
🔴 P1: vector_utils.hpp の名前空間確認
🟡 P2: スコープエラーの変数定義位置確認
🟡 P2: 関数シグネチャの確認（既に文書化されている）
🟢 P3: 未実装関数の確認・実装
```

---

## 修正の実施手順

### ステップ 1: インクルード順序の修正（15分）

```bash
# mex_run_eskf.cpp の インクルード順序を修正
# fixed_matrix.hpp を最初に配置
```

### ステップ 2: ビルドテスト（5分）

```matlab
cd kalman/cpp/build
build_mex({'mex_run_eskf'})
```

**期待結果**: 型認識エラーの大幅削減（45個 → 5個以下）

### ステップ 3: 残りのエラーを個別修正（30分～1時間）

- スコープエラー（変数定義位置）
- シグネチャエラー（テンプレートパラメータ）
- 未実装関数（実装 or ビルドスクリプト修正）

### ステップ 4: 回帰テスト（10分）

```matlab
clear mex
run_simulation(42, true)
```

---

## 統計情報

| 項目 | 値 |
|-----|-----|
| **期待される修正効果** | 80-90% のエラー削減 |
| **推定対応時間** | 1-2時間 |
| **必要なファイル修正** | 3-5個 |
| **テスト時間** | 15-20分 |

---

## 参考

- [01_COMMIT_CHANGES_SUMMARY.md](01_COMMIT_CHANGES_SUMMARY.md) - コミット変更内容
- [BUILD_ERROR_ANALYSIS.md](BUILD_ERROR_ANALYSIS.md) - ビルドエラーログの詳細
- [FIX_STATUS.md](FIX_STATUS.md) - 修正状況

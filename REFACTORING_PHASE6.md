# Phase 6: コメント・コード規約の統一

**目標**: コメントスタイルの統一、不要なコメントの削除、命名規則の適用  
**所要時間**: 2時間  
**リスク**: 低（コメントのみの変更）  
**前提条件**: Phase 5 完了

---

## 1. コメント規約

### 1.1 採用するスタイル

| 項目 | 規約 |
|-----|------|
| ファイルヘッダ | なし（削除） |
| 関数コメント | 1行の `//` コメント、必要時のみ |
| インラインコメント | 短く簡潔に |
| Doxygenコメント | 廃止 |
| 変更履歴コメント | 廃止（Git使用） |
| TODOコメント | 廃止（Issue使用） |

### 1.2 禁止するスタイル

```cpp
// ❌ 禁止: Doxygenスタイル
/**
 * @brief 四元数を正規化する
 * @param q 入力四元数 [w, x, y, z]
 * @return 正規化された四元数
 */

// ❌ 禁止: 冗長なファイルヘッダ
/*
 * File: quaternion_functions.hpp
 * Author: xxx
 * Date: 2025-01-01
 * Description: 四元数演算関数
 */

// ❌ 禁止: 変更履歴
// 2025-01-01: 初版作成
// 2025-01-10: バグ修正
// 2025-01-15: 機能追加

// ❌ 禁止: 冗長な説明
// この関数は四元数を正規化します。
// 四元数は [w, x, y, z] の形式で入力され、
// ノルムが1になるように正規化されます。
// 入力が零ベクトルの場合は、単位四元数を返します。
```

### 1.3 推奨するスタイル

```cpp
// ✅ 推奨: 簡潔な関数コメント
// 四元数を正規化
template<typename T>
inline void normalize_quat(T q[4]) {
    // ...
}

// ✅ 推奨: 必要な場合のみ補足
// 15次元状態ベクトル: [p(3), v(3), q(4), ba(3), bg(3)]
struct ESKFState {
    float p[3];   // 位置 (m)
    float v[3];   // 速度 (m/s)
    float q[4];   // 四元数 [w,x,y,z]
    float ba[3];  // 加速度バイアス
    float bg[3];  // ジャイロバイアス
};

// ✅ 推奨: アルゴリズムの出典を記載
// Trawny-Roumeliotis quaternion integration
void integrate_quaternion(const float w[3], float dt, float q[4]);
```

---

## 2. 削除対象コメント

### 2.1 検索コマンド

```bash
# Doxygenコメント
grep -rn "@brief\|@param\|@return\|@note\|@warning" kalman/cpp --include="*.hpp" --include="*.cpp"

# ファイルヘッダ
grep -rn "File:\|Author:\|Date:\|Description:\|Copyright" kalman/cpp --include="*.hpp" --include="*.cpp"

# 変更履歴
grep -rn "History:\|Changelog:\|Modified:\|Updated:" kalman/cpp --include="*.hpp" --include="*.cpp"

# TODO/FIXME
grep -rn "TODO\|FIXME\|XXX\|HACK" kalman/cpp --include="*.hpp" --include="*.cpp"

# 冗長なセパレータ
grep -rn "^// =\|^// -\|^/\*\*\*" kalman/cpp --include="*.hpp" --include="*.cpp"
```

### 2.2 削除例

```cpp
// 削除前
/**
 * @brief クォータニオン乗算
 * @param[in] q1 左オペランド四元数 [w, x, y, z]
 * @param[in] q2 右オペランド四元数 [w, x, y, z]
 * @param[out] result 乗算結果 [w, x, y, z]
 * @note Hamilton規約に従う
 */
template<typename T>
inline void multiply_quat(const T q1[4], const T q2[4], T result[4]) {
    // ...
}

// 削除後
// 四元数乗算 (Hamilton規約)
template<typename T>
inline void multiply_quat(const T q1[4], const T q2[4], T result[4]) {
    // ...
}
```

---

## 3. 命名規則

### 3.1 変数名

| 種類 | 規約 | 例 |
|-----|------|-----|
| ローカル変数 | snake_case | `float accel_norm;` |
| メンバー変数 | m_プレフィックス + snake_case | `float m_threshold;` |
| 定数 | ALL_CAPS | `static const int MAX_SIZE = 100;` |
| グローバル変数 | g_プレフィックス（禁止推奨） | `g_log_counter`（Phase 2で削除済み） |

### 3.2 関数名

| 種類 | 規約 | 例 |
|-----|------|-----|
| 通常関数 | snake_case | `void normalize_quat(...)` |
| テンプレート関数 | snake_case | `template<typename T> void clamp(...)` |
| メンバー関数 | snake_case | `bool is_outlier(...)` |

### 3.3 クラス・構造体名

| 種類 | 規約 | 例 |
|-----|------|-----|
| クラス | PascalCase | `class OutlierDetector` |
| 構造体 | PascalCase | `struct ESKFState` |
| 名前空間 | snake_case | `namespace common::math` |

### 3.4 命名の一貫性確認

```bash
# camelCase の検出（修正対象）
grep -rn "[a-z][A-Z]" kalman/cpp --include="*.hpp" --include="*.cpp" | head -50

# 不統一なプレフィックスの検出
grep -rn "m[A-Z]\|_m[a-z]" kalman/cpp --include="*.hpp" --include="*.cpp"
```

---

## 4. 型の使用規則

### 4.1 推奨する型

| 用途 | 型 | 理由 |
|-----|-----|------|
| センサーデータ | `float` | MATLABとの互換性 |
| GPS座標 | `double` | 精度要求 |
| 配列サイズ | `int` | シンプル |
| ループカウンタ | `int` | シンプル |
| ブール | `bool` | C++標準 |

### 4.2 禁止する型

| 型 | 理由 | 代替 |
|-----|------|------|
| `size_t` | 環境依存のサイズ | `int` |
| `uint32_t` | 不要な精度指定 | `unsigned int` |
| `int64_t` | 不要な精度指定 | `long long` |
| `auto` | 可読性低下 | 明示的な型 |

### 4.3 例外

```cpp
// 例外的にautoを許可する場合（イテレータ等）
// ただし本プロジェクトではイテレータを使用しないため、実質的にauto禁止
```

---

## 5. モダンC++の制限

### 5.1 禁止する機能

| 機能 | 理由 | 代替 |
|-----|------|------|
| `auto` | 型が不明確 | 明示的型指定 |
| `std::atomic` | 環境依存 | 削除済み（Phase 2） |
| `std::thread` | 環境依存 | 使用しない |
| `std::chrono` | 環境依存 | 削除済み（Phase 2） |
| `std::unique_ptr` | 複雑化 | 生ポインタ + 明示的delete |
| `std::shared_ptr` | 複雑化 | 使用しない |
| `std::optional` | C++17 | 戻り値 + bool |
| `std::variant` | C++17 | union または別設計 |
| ラムダ式 | 可読性 | 通常の関数 |
| 範囲for | 明示性 | 通常のfor |

### 5.2 許可する機能

| 機能 | 用途 |
|-----|------|
| テンプレート | 型パラメータ化（Matrix, Filter等） |
| `inline` | ヘッダファイルの関数定義 |
| `constexpr` | コンパイル時定数（限定的） |
| 名前空間 | コード整理 |
| `static_cast` | 明示的型変換 |

---

## 6. ヘッダガードの統一

### 6.1 規約

```cpp
// ファイル: Lib/Core/math_utils.hpp
#pragma once

#ifndef LIB_CORE_MATH_UTILS_HPP
#define LIB_CORE_MATH_UTILS_HPP

// ... 内容 ...

#endif // LIB_CORE_MATH_UTILS_HPP
```

### 6.2 命名規則

```
パス: Lib/ESKF/inc/eskf_core.hpp
ガード名: LIB_ESKF_INC_ESKF_CORE_HPP

パス: MEX/mex_eskf_common.hpp
ガード名: MEX_ESKF_COMMON_HPP
```

### 6.3 確認コマンド

```bash
# 現在のヘッダガード名を一覧
grep -rn "#ifndef\|#define" kalman/cpp --include="*.hpp" | grep -v "pragma"
```

---

## 7. インクルード順序

### 7.1 規約

```cpp
// 1. pragma once（必須）
#pragma once

// 2. ヘッダガード（#pragma onceと併用）
#ifndef FILE_NAME_HPP
#define FILE_NAME_HPP

// 3. プロジェクト内ヘッダ（相対パス）
#include "../../Matrix/fixed_matrix.hpp"
#include "../Core/types.hpp"

// 4. 標準ライブラリ（最小限）
#include <cmath>
#include <cstring>
#include <cfloat>

// 5. 内容

#endif
```

### 7.2 禁止するinclude

```cpp
// ❌ 禁止
#include <iostream>    // デバッグ用
#include <fstream>     // ファイルI/O
#include <string>      // std::string（char*使用）
#include <vector>      // std::vector（固定配列使用）
#include <map>         // コンテナ
#include <atomic>      // 環境依存
#include <chrono>      // 環境依存
#include <thread>      // 環境依存
```

---

## 8. 実施手順

### Step 1: コメント削除スクリプト作成

```python
# remove_doxygen.py
import re
import os

def process_file(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Doxygenブロックコメント削除
    content = re.sub(r'/\*\*.*?\*/', '', content, flags=re.DOTALL)
    
    # @brief等の行削除
    content = re.sub(r'^\s*//\s*@\w+.*$', '', content, flags=re.MULTILINE)
    
    # 空行の連続を1行に
    content = re.sub(r'\n{3,}', '\n\n', content)
    
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(content)

for root, dirs, files in os.walk('kalman/cpp'):
    for f in files:
        if f.endswith('.hpp') or f.endswith('.cpp'):
            process_file(os.path.join(root, f))
```

### Step 2: 手動確認

各ファイルを開いて以下を確認:
- 必要なコメントが残っているか
- 不要なコメントが削除されているか
- 命名規則が適用されているか

### Step 3: ヘッダガード統一

各ファイルのヘッダガードをパスベースの命名に変更

### Step 4: ビルド確認

```matlab
cd kalman/cpp/build
clear mex
build_mex()
```

### Step 5: 回帰テスト

```matlab
clear mex
cd ../..
run_batch_10sets()
```

---

## 9. 対象ファイル一覧

| ファイル | 修正内容 |
|---------|---------|
| Lib/Matrix/fixed_matrix.hpp | Doxygenコメント削除、ヘッダガード統一 |
| Lib/Quaternion/quaternion_functions.hpp | 冗長なコメント削除 |
| Lib/ESKF/inc/eskf_core.hpp | 変更履歴削除 |
| Lib/ESKF/inc/eskf_runner.hpp | 命名規則適用 |
| Lib/ESKF/inc/eskf_state.hpp | コメント簡潔化 |
| Lib/MEUKF/inc/meukf_core.hpp | Doxygenコメント削除 |
| Lib/Core/types.hpp | ヘッダガード統一 |
| Lib/Core/interface.hpp | コメント簡潔化 |
| Lib/Sensor/outlier_detector.hpp | 命名規則適用 |
| Lib/Sensor/noise_estimator.hpp | コメント簡潔化 |
| MEX/*.hpp, *.cpp | 全ファイルにルール適用 |

---

## 10. 完了確認チェックリスト

- [ ] Doxygenコメント削除
- [ ] ファイルヘッダ削除
- [ ] 変更履歴コメント削除
- [ ] TODO/FIXMEコメント削除
- [ ] 命名規則の統一
- [ ] ヘッダガードの統一
- [ ] インクルード順序の統一
- [ ] モダンC++機能の制限確認
- [ ] `build_mex()` 成功
- [ ] `run_batch_10sets()` 10/10 PASS
- [ ] Git commit 完了

---

## 11. 最終成果

### 11.1 コード品質の向上

| 項目 | Before | After |
|-----|--------|-------|
| コメント行数 | 約2,000行 | 約500行 |
| Doxygenブロック | 50個以上 | 0個 |
| TODO/FIXME | 15個以上 | 0個 |
| 命名の不統一 | 多数 | 統一済み |

### 11.2 可読性の向上

```cpp
// Before: 冗長
/**
 * @brief 四元数を正規化する関数
 * @param[in,out] q 正規化する四元数。形式は [w, x, y, z] のスカラー先頭。
 * @note ノルムが非常に小さい場合は単位四元数にリセットされます。
 * @warning 入力が NULL の場合の動作は未定義です。
 */
template<typename T>
inline void normalize_quat(T q[4]) {

// After: 簡潔
// 四元数を正規化 [w,x,y,z]
template<typename T>
inline void normalize_quat(T q[4]) {
```

---

## 12. リファクタリング完了

### 12.1 全Phase完了後の構造

```
kalman/cpp/
├── bin/                    # MEXバイナリ
├── build/                  # ビルドスクリプト
├── MEX/                    # MEX実装（7ファイル）
└── Lib/
    ├── Core/              # 共通コア（4ファイル）
    ├── Sensor/            # センサー処理（3ファイル）
    ├── Matrix/            # 行列（1ファイル）
    ├── Quaternion/        # 四元数（1ファイル）
    ├── ESKF/              # ESKFフィルタ（3ファイル）
    └── MEUKF/             # MEUKFフィルタ（2ファイル）
```

### 12.2 削減効果サマリー

| 項目 | Phase 1 | Phase 2 | Phase 3 | Phase 4 | Phase 5 | Phase 6 | 合計 |
|-----|---------|---------|---------|---------|---------|---------|------|
| 削除ファイル数 | 18 | 1 | 7 | 0 | 17 | 0 | 43 |
| 削減行数 | 2,000 | 1,400 | 1,200 | 50 | 500 | 1,500 | 6,650 |

### 12.3 最終確認

```matlab
cd kalman/cpp/build
clear mex
build_mex()
clear mex
cd ../..
run_batch_10sets()
% 10/10 PASS を確認
```

**リファクタリング完了！**

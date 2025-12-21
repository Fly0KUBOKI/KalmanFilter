# C++ MEX コンパイルエラー分析と対策

## 概要
現在のコンパイルエラーは **ヘッダーファイルの破損・重複** と **インクルードパスの混在** が主原因です。

---

## エラー原因の詳細

### 1. **ヘッダーファイル破損（primary cause）**

**問題箇所**: `cpp/include/Common/Math/fixed_matrix.hpp`

**現象**:
```
error C4430: 型指定子がありません - int と仮定しました
error C2146: 構文エラー: ';' が、識別子 'div' の前に必要です
```

**原因**: 
- ファイルの先頭に `#pragma once` と `#include "../../Common/Math/fixed_matrix.hpp"` を置いて**フォワーディング**したはずが、
- その後に **実装コードが混在**していた（例: `T div = aug(i, i);` など）
- これにより、コンパイラが関数スコープ外でコードを解析し、構文エラーが多発

**修正内容**: 
```cpp
#pragma once
// This file intentionally forwards to the canonical fixed_matrix implementation
#include "../../Common/Math/fixed_matrix.hpp"
```
- ファイルの内容を**フォワーディングヘッダーのみ**に限定
- 混在していた実装コードを削除

---

### 2. **インクルードパスの二重化**

**インクルード構造の問題**:
```
sensor_filter.hpp 
  └── #include "../Math/fixed_matrix.hpp"
       └── cpp/include/Common/Math/fixed_matrix.hpp (破損していた)
            └── #include "../../Common/Math/fixed_matrix.hpp"  (フォワード)
                 └── cpp/Common/Math/fixed_matrix.hpp (正規実装)
```

**修正後の流れ**:
```
sensor_filter.hpp 
  └── #include "../Math/fixed_matrix.hpp"
       └── cpp/include/Common/Math/fixed_matrix.hpp (フォワーディングのみ)
            └── #include "../../Common/Math/fixed_matrix.hpp"
                 └── cpp/Common/Math/fixed_matrix.hpp (正規実装 - **ここ1箇所に統一**)
```

---

### 3. **二重インクルード防止メカニズムの不備**

**エラーメッセージの意味**:
```
C2990: '_complex': 非クラス テンプレート は、既にクラス テンプレート として宣言されていました
C2374: 'cmath_fx::MAX_N': 複数回定義されました
```

**原因**: 
- `#pragma once` は存在するが、ファイル破損により実装が二重解析される
- MSVC の標準ライブラリヘッダ（`cmath`, `fstream` など）も巻き込まれ、カスケード的にエラー増幅

**修正**: 
- ヘッダーファイル破損を排除することで根本解決
- `#pragma once` が機能し始める

---

## ファイル構成の整理ルール

### **正規実装の場所**
```
cpp/Common/Math/fixed_matrix.hpp  ← 【正規版】全テンプレート定義・実装
```
- 実装の唯一の源泉
- 全型定義、関数実装を含む

### **インクルード経路**
```
cpp/include/Common/Math/fixed_matrix.hpp  ← 【フォワーディング】
  └── #include "../../Common/Math/fixed_matrix.hpp"  (相対パスで正規版を指す)

cpp/include/Common/Sensor/sensor_filter.hpp
  └── #include "../Math/fixed_matrix.hpp"  (相対パスでフォワーディングを指す)
```

**注意**:
- `cpp/include/` 配下のヘッダは可能な限り **フォワーディング** または **薄ラッパー** に留める
- 実装コードは混在させない

---

## よく起こるコンパイルエラーパターンと対応

| エラーパターン | 原因 | 対策 |
|---|---|---|
| `C4430: 型指定子がありません` | ヘッダ構文エラー、スコープ外コード | ファイル先頭の `#pragma once` から `#include` までのみ確認 |
| `C2146: ';' が識別子の前に必要` | 型名前解決失敗、フォワード宣言不足 | インクルードチェーンを確認、重複排除 |
| `C2990: 非クラス/クラステンプレート二重宣言` | 同じテンプレートが複数箇所で定義 | `#pragma once` が動作している確認、ファイル破損チェック |
| `fatal error C1003: エラーが100個超過` | カスケード失敗 | 最初のエラー行を特定し、そのヘッダから修正 |

---

## 注意点（再発防止）

### ✅ **守るべきルール**

1. **フォワーディングヘッダは実装を含めない**
   ```cpp
   // cpp/include/Common/Math/fixed_matrix.hpp
   #pragma once
   #include "../../Common/Math/fixed_matrix.hpp"
   // 以上。追加のコードなし。
   ```

2. **正規実装は `cpp/Common/` 配下に統一**
   ```
   cpp/Common/Math/fixed_matrix.hpp  ← 実装
   cpp/include/Common/Math/fixed_matrix.hpp  ← フォワード
   ```

3. **ビルド後は MEX キャッシュをクリア**
   ```matlab
   clear mex;  % MATLAB セッション内の古い MEX バイナリを解放
   ```

4. **ファイル修正時は内容を確認**
   - エディタで最初と最後の数行を確認
   - 混在・断片的な内容がないか視認

### ⚠️ **よくある失敗例**

```cpp
// ❌ NG: フォワーディングに実装が混在
#pragma once
#include "../../Common/Math/fixed_matrix.hpp"

// これ以下は余計なコード（ファイル破損）
template <int R, int C> struct Matrix { ... };

// ❌ NG: 同じファイルが `include/` と `Common/` の両方に実装を持つ
cpp/include/Common/Math/fixed_matrix.hpp   ← 実装
cpp/Common/Math/fixed_matrix.hpp           ← 同じ実装（重複）
```

---

## ビルド時の推奨手順

```matlab
% ステップ1: 環境変数設定
setenv('MEX_DEBUG', '0');  % ログ出力なし（デフォルト）

% ステップ2: ビルド実行
cd kalman/cpp/build
build_mex();

% ステップ3: キャッシュクリア（重要）
clear mex

% ステップ4: パス追加
addpath(fullfile(pwd, '..', 'bin'))

% ステップ5: シミュレーション実行
cd ../../..
run_simulation(42, true)
```

---

## 今後のエラー対応フロー

1. **コンパイルエラー発生時**
   - エラー行が指す `*.hpp` ファイルを開く
   - `#pragma once` の直後が正しいインクルード文か確認
   - 実装コードが混在していないか確認

2. **"〜 エラーが 100 個超過" の場合**
   - 最初のエラー行（通常 line 9 前後）に注目
   - そのヘッダから修正を開始

3. **修正後の確認**
   - `build_log.txt` でビルド結果を確認
   - 「Successfully built X MEX file(s)」と表示される

---

## 緊急対応ログ

### エラー回発生: ヘッダーファイルの完全な破損

**日時**: 2回目のビルド試行後

**症状**: 
- `mex_eskf_math` ビルド失敗（error C2187 at line 4）
- `mex_sensor_filter` ビルド失敗（error C2187 at line 4）
- `mex_unified_filter` ビルド失敗（error C2988 at line 40）
- `mex_eskf_step` ビルド失敗（error C2988 at line 40）

**根本原因**:
- `cpp/include/Common/Math/fixed_matrix.hpp` の全内容が破損
- フォワーディング宣言（`#pragma once` + `#include`）の後に**数十行の実装コード**が混在
- 実装コード内で `std::memset`, `std::assert`, `std::abs`, `std::swap` への参照があり、かつ `FixedMatrix` クラス定義が完全に残存

**修正方法**:
```cpp
// ✅ 正しい形状（3行のみ）
#pragma once
// Forward to canonical implementation in cpp/Common/Math
#include "../../Common/Math/fixed_matrix.hpp"
```

実装コードを**完全削除**し、フォワーディングヘッダの純粋性を確保。

---

### エラー3回発生: インクルードパスの相対参照が誤ったまま

**日時**: 3回目のビルド試行後

**症状**:
- `mex_eskf_math` ビルド失敗（error C2653: `cmath_fx` が見つからない）
- `mex_sensor_filter` ビルド失敗（error C2653: `cmath_fx::FixedMatrix` が見つからない）
- `mex_unified_filter`, `mex_eskf_step` ビルド失敗（類似）

**根本原因**:
- `cpp/include/Common/Math/fixed_matrix.hpp` のフォワーディング宣言の**相対パスが誤ったまま**
- `#include "../../Common/Math/fixed_matrix.hpp"` は**ディレクトリアップが1階層不足**
  - 実際のパス構造：
    ```
    cpp/
      ├── include/
      │   └── Common/
      │       └── Math/
      │           └── fixed_matrix.hpp  ← フォワーディング（ここから相対参照）
      └── Common/
          └── Math/
              └── fixed_matrix.hpp  ← 正規実装（ここへ向かう）
    ```
  - `../../Common/Math/` だと `cpp/Common/Sensor/` を指してしまう！
  - **正しい相対パス**: `../../../Common/Math/`（3階層アップ）

**修正方法**:
```cpp
#pragma once
// Forward to canonical implementation in cpp/Common/Math
#include "../../../Common/Math/fixed_matrix.hpp"  // ✅ 3階層アップに修正
```

---

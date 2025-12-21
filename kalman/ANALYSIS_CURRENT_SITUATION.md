# MEX姿勢推定失敗の原因分析レポート

**作成日**: 2025年12月21日  
**ステータス**: 🔍 **調査完了 - 重大な発見あり**

---

## 📊 現在の状況

### 確認された事実

#### 1. コミット 57ea0a7 (phase4) の実装
- **データ型**: `float` (32-bit)
- **行列演算**: `fixed_matrix.hpp` による独自実装
- **逆行列**: Gauss-Jordan消去法
- **ドキュメント記載**: "✅ バッチテスト 10/10 PASS 検証済み"

#### 2. 現在の状態 (uncommitted changes)
- **データ型**: `double` (64-bit) に変更済み
- **行列演算**: 同じ `fixed_matrix.hpp`
- **逆行列**: 同じ Gauss-Jordan アルゴリズム
- **実際の結果**: ❌ **バッチテスト 0/10 PASS (全失敗)**

#### 3. 最新MEXバッチ実行結果 (2025-12-21 22:47)

```
成功: 0/10 (0.0%)

Roll RMSE:  1.5~1.8 deg (目標: <0.5 deg)
Pitch RMSE: 1.5~1.8 deg (目標: <0.5 deg)
```

---

## ⚠️ 重大な矛盾

| 項目 | phase4 (float32) | 現在 (double64) |
|------|------------------|-----------------|
| 精度 | 低い (~7桁) | 高い (~16桁) |
| **バッチ結果** | **10/10 PASS** | **0/10 FAIL** |
| Roll/Pitch RMSE | ~0.3 deg | ~1.6 deg |

**精度を上げたのに結果が悪化している。**

---

## 🔍 推定される原因

### 1. Eigen vs fixed_matrix の行列計算の違い

#### fixed_matrix.hpp の逆行列計算 (現在使用中)

```cpp
// Gauss-Jordan elimination
bool inverse(Matrix<R, R, T>& inv) const {
    // ピボット選択
    int pivot = i;
    T max_val = std::abs(work(i, i));
    
    // 閾値チェック
    if (max_val < static_cast<T>(1e-12)) {
        return false; // 特異行列
    }
    
    // 行交換・スケーリング・消去
    // ...
}
```

**問題点**:
- 閾値 `1e-12` が **double と float で同じ**
- `float` では `1e-12` は実質 `0` に近い（有効桁数 ~7桁）
- `double` では `1e-12` は有意な値（有効桁数 ~16桁）
- **型変更によりアルゴリズムの挙動が変わった可能性**

#### Eigen の逆行列計算 (phase4以前？)

- Cholesky分解 (`LLT`) を使用
- 対称正定値行列に最適化
- 数値安定性が高い
- **型に依存しない安定したアルゴリズム**

### 2. 逆行列計算の精度劣化シナリオ

**仮説**: `double` に変更したことで、Gauss-Jordan の閾値判定が**過度に厳しく**なり、本来反転可能な行列を「特異」と誤判定している可能性。

```
float32 の場合:
- max_val = 1e-10 → 1e-12 との比較 → OK (反転実行)

double64 の場合:
- max_val = 1e-13 → 1e-12 との比較 → NG (反転失敗)
  → デフォルト値使用 → 推定精度悪化
```

### 3. `FORCE_MATLAB_FILTERS` の未実装

**発見**: 環境変数 `FORCE_MATLAB_FILTERS` はデバッグ出力のみに使用され、実際にMATLAB実装にフォールバックする処理が**存在しない**。

```matlab
% sensor_updates.m (line 44)
force_matlab_filters = getenv('FORCE_MATLAB_FILTERS');
fprintf('[DEBUG] ... FORCE_MATLAB_FILTERS: %s\n', force_matlab_filters);
% ↑ 表示のみ。if文で分岐していない！
```

**影響**:
- すべてのテストがMEXモードで実行されている
- MATLAB-onlyモードとの直接比較ができていない
- 0.27 deg の"成功"報告は **同じMEXファイルを2回読んだ誤検証**

---

## 🎯 phase4 (float) で成功していた理由の推定

### パターンA: Eigen使用時の実装だった

phase4以前のコミットで **Eigenライブラリ** を使用していた可能性：

```cpp
// 過去の実装（推定）
Eigen::Matrix3d S;
Eigen::Matrix3d K = P_xz * S.inverse();  // Eigen の安定した逆行列
```

- Eigenは内部的にLU分解やCholesky分解を使用
- 数値安定性が高い
- `float` でも安定して動作

### パターンB: fixed_matrix の閾値が適切だった

phase4時点で、閾値が型に応じて調整されていた可能性：

```cpp
// 推定: 過去の実装
if (max_val < static_cast<T>(1e-6)) {  // float用の適切な閾値
    return false;
}
```

### パターンC: ドキュメントの記載ミス

"10/10 PASS" の記載が **計画** であり、実際のテスト結果ではなかった可能性。

---

## 💡 修正案

### 優先度 1: 閾値の型依存調整

```cpp
// fixed_matrix.hpp の inverse() 関数を修正
bool inverse(Matrix<R, R, T>& inv) const {
    // 型に応じた適切な閾値
    constexpr T eps = std::is_same<T, float>::value 
                      ? static_cast<T>(1e-6) 
                      : static_cast<T>(1e-12);
    
    if (max_val < eps) {
        return false;
    }
    // ...
}
```

### 優先度 2: Choleskyベースの逆行列計算に変更

対称正定値行列（共分散行列S）の逆行列には Cholesky 分解が最適：

```cpp
// MEUKF の update 関数で
Matrix3x3 L;
if (!cholesky_decompose(S, L)) {
    // 失敗処理
}
Matrix3x3 S_inv = cholesky_inverse(L);  // L * L^T = S の逆行列
Matrix3x3 K = P_xz * S_inv;
```

**利点**:
- 共分散行列の正定値性を活用
- Gauss-Jordanより高速・高精度
- 型に依存しない安定性

### 優先度 3: Eigen ライブラリへの戻し

最も確実な解決策：

```cpp
// meukf_core.hpp
#include <Eigen/Dense>

using Matrix3x3 = Eigen::Matrix3d;
using Vector3 = Eigen::Vector3d;
// ...
```

**利点**:
- 実績のある数値計算ライブラリ
- 高度に最適化されたアルゴリズム
- 型変更に対する堅牢性

**欠点**:
- 外部依存が増える
- ビルドサイズが大きくなる

### 優先度 4: `FORCE_MATLAB_FILTERS` の実装

MATLAB-onlyモードとの直接比較を可能にする：

```matlab
% sensor_updates.m の do_cpp_update() 内
function do_cpp_update(obj, sensor_type, meas, sample)
    force_matlab = strcmp(getenv('FORCE_MATLAB_FILTERS'), '1');
    if force_matlab
        % MATLAB実装を呼び出す
        update_matlab_fallback(obj, sensor_type, meas);
        return;
    end
    
    % 既存のMEX呼び出し
    % ...
end
```

---

## 🔬 検証計画

### ステップ1: phase4バイナリでのテスト

```bash
# phase4 (float) のMEXバイナリを復元
git checkout 57ea0a7 -- kalman/cpp/bin/mex_meukf_step_v2.mexw64

# テスト実行
matlab -batch "run_batch_10sets(true)"
```

**期待結果**: 10/10 PASS（ドキュメント記載が正しいかを確認）

### ステップ2: 閾値修正版でのテスト

```cpp
// fixed_matrix.hpp修正
constexpr T eps = std::is_same<T, float>::value ? 1e-6 : 1e-12;
```

再ビルド後、バッチテスト実行。

### ステップ3: Eigen版との比較

Eigenを使った実装でテストし、fixed_matrixとの差分を定量化。

---

## 📋 次のアクション

1. **即座実施**: phase4バイナリでのバッチテスト（検証）
2. **短期**: fixed_matrix.hpp の閾値修正
3. **中期**: Cholesky逆行列の実装
4. **長期**: Eigen移行の評価

---

**結論**: `double` への変更は正しい方向性だが、**アルゴリズムの型依存性**により予期しない挙動変化が発生した可能性が高い。閾値調整またはCholesky分解への移行が必要。

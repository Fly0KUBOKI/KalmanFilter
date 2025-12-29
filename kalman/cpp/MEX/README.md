# MEXフォルダ - MATLABインターフェース

## 役割

このフォルダには**MATLAB MEXラッパーのみ**が含まれます。

## ⚠️ 重要な原則

### 実装コードは含まない

- **禁止**: アルゴリズム実装（カルマンフィルタ計算、行列演算など）
- **禁止**: ビジネスロジック
- **禁止**: 状態管理

### 許可されるコード

- ✅ MATLAB配列 ↔ C++型の変換関数（`get_vec3`, `set_vec3`など）
- ✅ `mexFunction`の実装
- ✅ エラーチェックとバリデーション
- ✅ `Inc/`, `Src/`, `Lib/`の関数呼び出し

## 実装の場所

すべての実装コードは以下の場所にあります：

- **ヘッダー**: `../Inc/` - クラス定義、関数宣言
- **実装**: `../Src/` - 関数実装、アルゴリズム
- **ライブラリ**: `../Lib/` - 独立ライブラリ

## 例

### ✅ 正しい例: `mex_ekf.cpp`

```cpp
// 実装はSrc/EKF/ekf_linear_update.cppにある
#include "../Inc/EKF/ekf_linear_update.hpp"

void mexFunction(...) {
    // 型変換のみ
    matToFixed(prhs[0], x);
    
    // 実装関数を呼び出す
    ekf::linear::ekf_linear_update(x, P, z, H, R, x_upd, P_upd);
    
    // 結果をMATLABに返す
    plhs[0] = fixedToMat(x_upd);
}
```

### ❌ 間違った例

```cpp
// 実装コードをMEXファイルに書いてはいけない
void mexFunction(...) {
    // ❌ アルゴリズム実装をMEXファイルに書く
    Matrix K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    x_upd = x + K * (z - H * x);
    // ...
}
```

## 確認方法

MEXファイルが正しくラッパーのみか確認：

```bash
# 実装関数の呼び出しを確認
grep -r "ekf_linear_update\|integrate_nominal\|update_accel_meukf" MEX/*.cpp

# アルゴリズム実装がないことを確認（行列演算など）
grep -r "\.inverse()\|\.transpose()\|cholesky\|kalman_gain" MEX/*.cpp
```

## 独立性の確保

`Inc/`, `Src/`, `Lib/`は他のコンパイル環境でも使用可能：

```bash
# MEXフォルダなしでコンパイル可能
g++ -I Inc/ -I Lib/ Src/**/*.cpp -o standalone_app
```



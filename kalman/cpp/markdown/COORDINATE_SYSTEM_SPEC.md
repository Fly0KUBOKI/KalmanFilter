# 座標系変換仕様

## 概要

このドキュメントは、MATLABとC++間での行列データの座標系変換方法を定義します。

## MATLAB ↔ C++ の変換方法

### 行列（共分散行列、観測行列など）

#### MATLAB形式（列優先: column-major）
- 要素アクセス: `A[c*rows + r]` (c: 列インデックス, r: 行インデックス)
- メモリレイアウト: 列ごとに連続して格納
- 例: 3×3行列の場合
  ```
  [a00 a01 a02]  →  [a00, a10, a20, a01, a11, a21, a02, a12, a22]
  [a10 a11 a12]
  [a20 a21 a22]
  ```

#### C++形式（行優先: row-major）
- 要素アクセス: `A[r*cols + c]` (r: 行インデックス, c: 列インデックス)
- メモリレイアウト: 行ごとに連続して格納
- 例: 3×3行列の場合
  ```
  [a00 a01 a02]  →  [a00, a01, a02, a10, a11, a12, a20, a21, a22]
  [a10 a11 a12]
  [a20 a21 a22]
  ```

### 変換コード例

#### MATLAB (15×15 列優先) → C++ (15×15 行優先)

```cpp
// 共分散行列Pの変換
mxArray* f_P = mxGetField(m_state, 0, "P");
if (f_P) {
    float P_tmp[15*15];
    mex_conv::mxArrayToFloatArray(f_P, P_tmp, 15*15);
    // MATLAB column-major → C++ row-major
    for (int r = 0; r < 15; ++r) {
        for (int c = 0; c < 15; ++c) {
            c_state.P[r*15 + c] = P_tmp[c*15 + r];
        }
    }
}
```

#### C++ (15×15 行優先) → MATLAB (15×15 列優先)

```cpp
// 共分散行列Pの逆変換
mxArray* f_P = mxGetField(m_state, 0, "P");
if (f_P) {
    double* pr = mxGetPr(f_P);
    // C++ row-major → MATLAB column-major
    for (int c = 0; c < 15; ++c) {
        for (int r = 0; r < 15; ++r) {
            pr[c*15 + r] = static_cast<double>(c_state.P[r*15 + c]);
        }
    }
}
```

### 対称性の確認

共分散行列は対称行列である必要があります。変換後に対称性を確認することを推奨します：

```cpp
// 対称性の確認（デバッグ用）
for (int r = 0; r < 15; ++r) {
    for (int c = r + 1; c < 15; ++c) {
        float diff = std::abs(P_cpp[r*15 + c] - P_cpp[c*15 + r]);
        if (diff > 1e-5f) {
            mexWarnMsgTxt("Covariance matrix not symmetric!");
        }
    }
}
```

## 実装箇所

### mex_meukf_step.cpp
- `matlab_to_state()`: MATLAB → C++ 変換（行26）
- `state_to_matlab()`: C++ → MATLAB 変換（行59-63）

### mex_run_eskf_sensor_updates.hpp
- `mexCallMATLAB`経由で`mex_meukf_step_v2`を呼び出し、変換はMEX関数内で処理

## テスト方法

### MATLAB側での対称性確認

```matlab
% 共分散行列Pが対称であることを確認
P = state.P;
is_symmetric = max(max(abs(P - P'))) < 1e-6;
assert(is_symmetric, "Covariance matrix not symmetric!");
```

### 変換の正確性確認

```matlab
% テスト用の対称行列を作成
P_test = randn(15, 15);
P_test = P_test * P_test';  % 対称行列にする

% MEX関数に渡して、戻り値を確認
state.P = P_test;
state_new = mex_meukf_step_v2(state, sensor_data, params);

% 対称性が保たれていることを確認
P_new = state_new.P;
assert(max(max(abs(P_new - P_new'))) < 1e-6, "Symmetry lost!");
```

## 注意事項

1. **型変換**: MATLABは`double`、C++内部は`float`を使用
2. **対称性の保持**: 共分散行列は常に対称である必要がある
3. **インデックスの順序**: `[r*cols + c]` vs `[c*rows + r]` を混同しないこと
4. **メモリレイアウト**: 変換時は要素ごとにコピーし、単純な`memcpy`は使用不可

## 参考

- `kalman/cpp/MEX/mex_meukf_step.cpp`: 実装例
- `kalman/cpp/Inc/MEX/mex_type_conversion.hpp`: 型変換ヘルパー



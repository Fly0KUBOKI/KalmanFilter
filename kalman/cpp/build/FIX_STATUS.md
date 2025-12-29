# ビルドエラー修正状況
更新日時: 2025-12-30

## 修正完了項目

### ✅ 1. mex_eskf_update_postprocess.cpp の関数呼び出し修正
- **修正内容:** `matToVector` と `matToMatrix` の呼び出しにテンプレートパラメータを明示的に指定
- **修正箇所:**
  - `matToVector(prhs[1], dx)` → `matToVector<15>(prhs[1], dx)`
  - `matToVector(prhs[3], state_p)` → `matToVector<3>(prhs[3], state_p)`
  - その他すべての `matToVector`/`matToMatrix` 呼び出し
  - `vectorToMat`/`matrixToMat` にもテンプレートパラメータを明示

### ✅ 2. mex_run_eskf.cpp の関数呼び出し修正
- **修正内容:** すべての `matToVector`/`matToMatrix` 呼び出しにテンプレートパラメータを明示
- **修正箇所:**
  - `matToVector<3>(mxGetField(dbg_out, 0, "innov"), innov_vec)`
  - `matToVector<15>(mxGetField(dbg_out, 0, "dx"), dx)`
  - `matToVector<3>(p_arr, state_p)` など
  - `matToMatrix<15, 15>(P_arr, state_P)`

### ✅ 3. update_state_from_dx 関数の確認
- **結果:** 実装済み（`Src/ESKF/eskf_postprocess.cpp` に存在）

## 残っている問題

### ❌ 1. vector_utils.hpp の型認識エラー（最優先）
- **エラー:** `template<typename T>` の `T` が型として認識されない
- **原因:** `fixed_matrix.hpp` のインクルード時に何らかの問題が発生している可能性
- **影響:** 連鎖的に多数のエラーが発生

### ❌ 2. mex_type_conversion.hpp での Vector/Matrix 型認識問題
- **エラー:** `Vector` と `Matrix` 型が認識されない
- **原因:** `fixed_matrix.hpp` のインクルードまたは名前空間の問題
- **影響:** すべての型関連のエラー

### ❌ 3. mex_run_eskf.cpp のスコープエラー
- **エラー:** `R_row`, `zeros3`, `R_noise`, `sensor_type` などが未定義
- **原因:** 復元時に変数の定義位置が間違っている可能性
- **注意:** コードを確認したところ、変数のスコープは正しく見えます
- **可能性:** コンパイラが型認識エラーにより、コードを正しく解析できていない可能性

## 次のステップ

### 優先度1: 型認識の問題を解決
1. `fixed_matrix.hpp` のインクルード順序を確認
2. 名前空間の問題を確認
3. `vector_utils.hpp` での `cmath_fx::Vector` の使用を確認

### 優先度2: スコープエラーの確認
1. 型認識問題が解決された後、スコープエラーが残っているか確認
2. 残っている場合は、変数の定義位置を修正

## 修正されたファイル
- `kalman/cpp/MEX/mex_eskf_update_postprocess.cpp`
- `kalman/cpp/MEX/mex_run_eskf.cpp`


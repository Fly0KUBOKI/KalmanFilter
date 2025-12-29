# ビルドエラー分析レポート
生成日時: 2025-12-30

## エラー概要

2つのMEXファイルのビルドで100個以上のエラーが発生：
- `mex_eskf_update_postprocess.cpp` - ビルド失敗
- `mex_run_eskf.cpp` - ビルド失敗

## 根本原因

### 1. 型認識の問題（最優先）

**エラー:**
```
C:\...\mex_type_conversion.hpp(17): error C2061: 構文エラー: 識別子 'Vector'
C:\...\mex_type_conversion.hpp(31): error C2061: 構文エラー: 識別子 'Matrix'
```

**原因:**
- `fixed_matrix.hpp` から `Vector` と `Matrix` 型が正しく認識されていない
- `vector_utils.hpp(14)` で `template<typename T>` の `T` が型として認識されない
- これは `fixed_matrix.hpp` のインクルード順序または名前空間の問題を示唆

**影響:**
- すべての `Vector<...>` と `Matrix<...>` の使用でエラー
- 関数の引数宣言が正しく解析されない
- 連鎖的に多数のエラーが発生

### 2. スコープエラー（mex_run_eskf.cpp）

**エラー:**
```
C:\...\mex_run_eskf.cpp(122): error C2065: 'R_row': 定義されていない識別子です。
C:\...\mex_run_eskf.cpp(266): error C2065: 'zeros3': 定義されていない識別子です。
C:\...\mex_run_eskf.cpp(218): error C2065: 'R_noise': 定義されていない識別子です。
C:\...\mex_run_eskf.cpp(215): error C2065: 'sensor_type': 定義されていない識別子です。
```

**原因:**
- `R_row`: `quaternion_to_rotation_matrix` 関数内で定義（121行目）だが、関数外で参照
- `zeros3`: 複数の関数内で個別に定義されているが、スコープ外で参照
- `R_noise`, `sensor_type`: 関数内のブロックスコープで定義されているが、ブロック外で参照

**影響:**
- 変数が未定義として扱われ、コンパイルエラー

### 3. 関数シグネチャの不一致

**エラー:**
```
C:\...\mex_eskf_update_postprocess.cpp(41): error C2672: 'matToVector': 一致するオーバーロードされた関数が見つかりませんでした。
note: 'bool mex_conv::matToVector(const mxArray *)': 1 引数が必要です - 2 が指定されました
```

**原因:**
- `mex_type_conversion.hpp` の `matToVector` は1引数のみ（`const mxArray* arr`）
- `mex_eskf_update_postprocess.cpp` では2引数で呼び出し（`matToVector(prhs[1], dx)`）

**正しいシグネチャ:**
```cpp
template<int R>
bool matToVector(const mxArray* arr, Vector<R, float>& out)
```

**問題:**
- テンプレート関数の呼び出しで、テンプレートパラメータ `R` が推論できない可能性

### 4. 未実装の関数

**エラー:**
```
C:\...\mex_eskf_update_postprocess.cpp(107): error C3861: 'update_state_from_dx': 識別子が見つかりませんでした
```

**原因:**
- `eskf_postprocess.hpp` で宣言されているが、実装が見つからない
- `Src/ESKF/eskf_postprocess.cpp` が存在するか確認が必要

## ファイル状態

### 正常なファイル
- ✅ `mex_meukf_step_v2` - ビルド成功
- ✅ `mex_sensor_filter` - ビルド成功

### エラーがあるファイル

#### `mex_eskf_update_postprocess.cpp`
- **主要エラー:** 型認識エラー、関数シグネチャ不一致、未実装関数
- **行番号:** 40-131

#### `mex_run_eskf.cpp`
- **主要エラー:** スコープエラー、型認識エラー
- **行番号:** 122, 190, 215-226, 266-271, 287, 303, 309, 310, 337, 342-345

#### `vector_utils.hpp`
- **主要エラー:** テンプレートパラメータ認識エラー
- **行番号:** 14

#### `mex_type_conversion.hpp`
- **主要エラー:** `Vector` と `Matrix` 型が認識されない
- **行番号:** 17, 31, 49, 58

#### `eskf_postprocess.hpp`
- **主要エラー:** 関数宣言の解析エラー（連鎖エラー）
- **行番号:** 64-73

#### `sensor_preprocessor.hpp`
- **主要エラー:** 関数宣言の解析エラー（連鎖エラー）
- **行番号:** 22-36

## 修正優先順位

### 優先度1（最優先）: 型認識の問題を修正
1. `fixed_matrix.hpp` のインクルード順序を確認
2. `Vector` と `Matrix` の typedef が正しく定義されているか確認
3. 名前空間の問題を確認

### 優先度2: 関数シグネチャの修正
1. `mex_eskf_update_postprocess.cpp` での `matToVector`/`matToMatrix` 呼び出しを修正
2. テンプレートパラメータを明示的に指定

### 優先度3: スコープエラーの修正
1. `mex_run_eskf.cpp` で変数のスコープを修正
2. 必要に応じて変数を上位スコープに移動

### 優先度4: 未実装関数の確認
1. `update_state_from_dx` の実装を確認
2. 実装ファイルが存在するか確認

## 推奨される修正手順

1. **`fixed_matrix.hpp` の問題を確認・修正**
   - インクルードガードの確認
   - 名前空間の閉じ方の確認
   - `Vector` typedef の定義位置の確認

2. **`mex_eskf_update_postprocess.cpp` の関数呼び出しを修正**
   - `matToVector<15>(prhs[1], dx)` のようにテンプレートパラメータを明示

3. **`mex_run_eskf.cpp` のスコープ問題を修正**
   - 変数の定義位置を適切なスコープに移動

4. **実装ファイルの存在確認**
   - `Src/ESKF/eskf_postprocess.cpp` が存在し、`update_state_from_dx` が実装されているか確認

## 注意事項

- エラーログには100個以上のエラーが表示されているが、多くは連鎖エラー
- 根本原因（型認識の問題）を修正すれば、多くのエラーが自動的に解決される
- コードページ932（日本語）の警告は無視できるが、Unicode形式での保存を推奨


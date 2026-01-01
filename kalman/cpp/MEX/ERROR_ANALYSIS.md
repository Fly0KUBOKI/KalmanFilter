# エラー原因分析

## エラーメッセージ
```
Expected single (float) array, but got double. GPS以外のセンサーデータはfloatのみを受け取ります。
```

## 原因

### 1. MATLABのsingleとC++のfloatの関係

**MATLABのsingle = C++のfloat（同じ32ビット浮動小数点）**

- MATLAB: `single` = 32ビット浮動小数点（IEEE 754 single precision）
- C++: `float` = 32ビット浮動小数点（IEEE 754 single precision）
- **型自体は同じですが、MEXインターフェースでは型チェックが必要**

### 2. CSVから読み取ったデータの型

**`readtable`は通常double型で読み込む**

1. **CSV読み込み** (`read_csv.m`):
   - `readtable`でCSVを読み込む → **double型**で読み込まれる
   - その後、`single()`で変換 → **single型**に変換

2. **MATLAB構造体**:
   - `obs.ax`, `obs.ay`, `obs.az`などは**single型**
   - `obs.lat`, `obs.lon`, `obs.alt`は**double型**（GPSデータ）

### 3. 問題の箇所

#### 問題1: `getVec3`関数（`mex_helpers.hpp`）

```cpp
inline void getVec3(..., double* out) {
    out[0] = fx ? mxGetPr(fx)[idx] : 0.0;  // ← mxGetPrはdouble専用！
}
```

- `mxGetPr`は**double型の配列に対してのみ使用可能**
- single型の配列には`mxGetData`を使用し、`float*`にキャストする必要がある
- 現在の実装では、single型のデータを読み取ろうとすると**不正なメモリアクセス**が発生

#### 問題2: `initialize_eskf_from_matlab`関数（`eskf_initializer.cpp`）

```cpp
static double* get_data(const mxArray* arr) {
    if (!arr) return nullptr;
    return mxGetPr(arr);  // ← double専用！
}
```

- 同様に`mxGetPr`を使用しているため、single型のデータを読み取れない

#### 問題3: `do_step`関数内でのデータ取得

`do_step`関数内で`getAccel`, `getGyro`, `getMag`が呼ばれ、これらは`getVec3`マクロを使用：
- single型のデータをdoubleとして読み取ろうとする
- その後、`call_sensor_update`内で`handle_sensor_update_internal`が呼ばれ、`mex_meukf_step_v2`に渡される
- `mex_meukf_step_v2`内で`mxArrayToFloatArray`が呼ばれ、single型を期待しているが、実際にはdouble型のデータが渡されている

## データフロー

```
CSV (テキスト)
  ↓ readtable
MATLAB (double) ← デフォルトでdouble
  ↓ single()変換
MATLAB (single) ← read_csv.mで変換済み
  ↓ mex_run_eskf('step', handle, obs, k)
MEX: getVec3 (mxGetPr使用) ← 問題！single型を読み取れない
  ↓ double配列として取得（不正）
call_sensor_update
  ↓ handle_sensor_update_internal
mex_meukf_step_v2 (mxArrayToFloatArray) ← single型を期待
  ↓ エラー発生！
```

## 解決策（実装済み）

### 1. `getVec3`関数の修正（`mex_helpers.hpp`）

**変更前**:
```cpp
out[0] = fx ? mxGetPr(fx)[idx] : 0.0;  // double専用
```

**変更後**:
```cpp
auto get_value = [](const mxArray* arr, mwIndex i) -> double {
    if (!arr) return 0.0;
    if (mxGetClassID(arr) == mxSINGLE_CLASS) {
        const float* pf = (const float*)mxGetData(arr);
        return static_cast<double>(pf[i]);
    } else {
        const double* pr = mxGetPr(arr);
        return pr[i];
    }
};
```

### 2. `initialize_eskf_from_matlab`の修正（`eskf_initializer.cpp`）

**変更前**:
```cpp
static double* get_data(const mxArray* arr) {
    return mxGetPr(arr);  // double専用
}
```

**変更後**:
- `get_value_at`関数を追加（single/double両対応）
- 各データ取得箇所で`get_value_at`を使用し、一時配列に変換

### 3. `do_step`関数内の`baro`データ取得の修正

**変更前**:
```cpp
double baro = mxGetPr(baro_field)[idx];  // double専用
```

**変更後**:
```cpp
double baro;
if (mxGetClassID(baro_field) == mxSINGLE_CLASS) {
    const float* pf = (const float*)mxGetData(baro_field);
    baro = static_cast<double>(pf[idx]);
} else {
    const double* pr = mxGetPr(baro_field);
    baro = pr[idx];
}
```

### 4. `handle_sensor_update_internal`内の構造体作成の修正

- `sensor_data`構造体: GPS以外をsingle型に変更
- `mex_params`構造体: GPS以外をsingle型に変更
- `state_s`構造体: すべてsingle型に変更

## まとめ

### MATLABのsingleとC++のfloat

- **型自体は同じ**（32ビット浮動小数点、IEEE 754 single precision）
- **MEXインターフェースでは型チェックが必要**
- `mxGetPr`: double型専用
- `mxGetData`: すべての型に対応（型に応じてキャストが必要）

### CSVから読み取ったデータの型

1. **CSV読み込み**: `readtable` → **double型**（デフォルト）
2. **型変換**: `single()` → **single型**（`read_csv.m`で変換）
3. **MATLAB構造体**: 
   - GPS以外: **single型**
   - GPSデータ: **double型**

### 修正箇所

1. ✅ `getVec3`: single/double両対応
2. ✅ `initialize_eskf_from_matlab`: single/double両対応
3. ✅ `do_step`内の`baro`取得: single/double両対応
4. ✅ `handle_sensor_update_internal`: 構造体をsingle型で作成


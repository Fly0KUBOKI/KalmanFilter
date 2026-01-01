# MEXファイル内の型変換処理のまとめ

## 概要

MEXファイル内では、MATLAB（double/single）とC++（float）の間で型変換が行われています。このドキュメントでは、現在残っている変換処理とその処理方法を説明します。

## 1. 入力変換（MATLAB → C++）

### 1.1 `mxArrayToFloatArray` (mex_type_conv.hpp)

**目的**: MATLAB配列（double/single）をC++のfloat配列に変換

**処理方法**:
```cpp
inline void mxArrayToFloatArray(const mxArray* arr, float* out, std::size_t n) {
    if (mxGetClassID(arr) == mxSINGLE_CLASS) {
        // 入力がsingle（float）の場合：直接コピー
        const float* pf = (const float*)mxGetData(arr);
        for (std::size_t i = 0; i < n; ++i) out[i] = pf[i];
    } else {
        // 入力がdoubleの場合：floatにキャスト
        const double* pr = mxGetPr(arr);
        for (std::size_t i = 0; i < n; ++i) out[i] = static_cast<float>(pr[i]);
    }
}
```

**使用箇所**:
- `mex_meukf_step.cpp`: 状態、センサーデータ、パラメータの入力時に使用
  - 位置（p）、速度（v）、クォータニオン（q）、バイアス（ba, bg）
  - 加速度計、ジャイロ、磁気計、GPS位置、気圧高度
  - ノイズパラメータ、重力ベクトル、磁場基準値

### 1.2 `mxGetScalarAsFloat` (mex_type_conv.hpp)

**目的**: MATLABスカラー値をfloatに変換

**処理方法**:
```cpp
inline float mxGetScalarAsFloat(const mxArray* a) {
    if (mxGetClassID(a) == mxSINGLE_CLASS) {
        const float* pf = (const float*)mxGetData(a);
        return pf ? pf[0] : 0.0f;
    }
    return static_cast<float>(mxGetScalar(a));  // double → float
}
```

**使用箇所**:
- `mex_meukf_step.cpp`: スカラー値（alt_baro, dt, alpha, beta, kappa等）の入力時に使用

### 1.3 GPSデータの特別処理

**目的**: GPSデータ（lat, lon, alt）はdouble精度を保持

**処理方法**:
- `mex_run_eskf_impl.hpp`の`do_step`関数内:
  ```cpp
  double lat = mxGetPr(gps_lat)[idx];  // doubleのまま取得
  double lon = mxGetPr(gps_lon)[idx];
  double alt = mxGetPr(gps_alt)[idx];
  call_gps_update(s, lat, lon, alt, ...);  // doubleで渡す
  ```
- `preprocess_gps`関数内でdouble演算を行い、最終的にfloatに変換:
  ```cpp
  PreprocessResult preprocess_gps(double lat, double lon, double alt, ...) {
      // double演算でGPS座標をメートル単位に変換
      double y_m = dlat / 9.0e-6;
      double x_m = dlon / (9.0e-6 / std::cos(lat0rad));
      // 最終的にfloat配列に格納
      result.output(0, 0) = y_m;  // floatに自動変換
  }
  ```

## 2. 出力変換（C++ → MATLAB）

### 2.1 `do_get_state` (mex_run_eskf_impl.hpp)

**目的**: ESKF状態をMATLAB構造体として返す（**float出力**）

**処理方法**:
```cpp
inline mxArray* do_get_state(ESKFState* s) {
    // すべてsingle（float）形式で出力
    mxArray* p = mxCreateNumericMatrix(3, 1, mxSINGLE_CLASS, mxREAL);
    float* p_ptr = (float*)mxGetData(p);
    for (int i = 0; i < 3; i++) p_ptr[i] = static_cast<float>(s->p[i]);
    // 同様に v, q, euler, ba, bg, P もsingle形式で出力
}
```

**出力形式**: すべて`mxSINGLE_CLASS`（float）形式

### 2.2 `state_to_matlab` (mex_meukf_step.cpp)

**目的**: MEUKF状態をMATLAB構造体に書き込む（**single/double両対応**）

**処理方法**:
```cpp
void state_to_matlab(const meukf::State& c_state, mxArray* m_state) {
    // 既存の構造体フィールドの型をチェック
    if (mxGetClassID(f) == mxSINGLE_CLASS) {
        // single形式の場合：直接floatを書き込み
        float* pf = (float*)mxGetData(f);
        pf[0] = in[0]; pf[1] = in[1]; pf[2] = in[2];
    } else {
        // double形式の場合：float → doubleに変換
        double* pr = mxGetPr(f);
        pr[0] = static_cast<double>(in[0]);
        pr[1] = static_cast<double>(in[1]);
        pr[2] = static_cast<double>(in[2]);
    }
}
```

**特徴**: レガシー互換のため、既存の構造体がdouble形式の場合はdoubleに変換

### 2.3 `do_meukf_step` (mex_run_eskf_impl.hpp)

**目的**: MEUKFステップ処理の結果を返す（**double形式で出力**）

**処理方法**:
```cpp
inline mxArray* do_meukf_step(...) {
    mxArray* out_state = mxDuplicateArray(m_prev_state);  // 入力構造体をコピー
    // 出力は入力構造体の型に合わせる（通常はdouble）
    double* pr = mxGetPr(f);
    pr[0] = static_cast<double>(in[0]);  // float → double
}
```

**注意**: この関数は`mex_run_eskf`の内部で使用され、通常はdouble形式で出力

### 2.4 `floatArrayToMxArray` (mex_type_conv.hpp)

**目的**: float配列をdouble形式のMATLAB配列に変換（**レガシー互換用**）

**処理方法**:
```cpp
inline void floatArrayToMxArray(const float* in, mxArray* out, ...) {
    double* pr = mxGetPr(out);
    for (...) {
        pr[r + c * rows] = static_cast<double>(in[r + c * rows]);
    }
}
```

**使用状況**: 現在は使用されていない可能性が高い（未使用コード）

### 2.5 `floatArrayToMxArrayFloat` (mex_type_conv.hpp)

**目的**: float配列をsingle形式のMATLAB配列に変換（**新方式**）

**処理方法**:
```cpp
inline void floatArrayToMxArrayFloat(const float* in, mxArray* out, ...) {
    float* pf = (float*)mxGetData(out);
    for (...) {
        pf[r + c * rows] = in[r + c * rows];  // 直接コピー
    }
}
```

**使用状況**: 現在は使用されていない可能性が高い（未使用コード）

### 2.6 `vectorToMat` / `matrixToMat` (mex_type_conversion.hpp)

**目的**: Vector/Matrix型をdouble形式のMATLAB配列に変換

**処理方法**:
```cpp
template<int R>
mxArray* vectorToMat(const Vector<R, float>& v) {
    mxArray* out = mxCreateDoubleMatrix(R, 1, mxREAL);
    double* pr = mxGetPr(out);
    for (int i = 0; i < R; ++i) pr[i] = static_cast<double>(v(i, 0));
    return out;
}
```

**使用状況**: 現在は使用されていない可能性が高い（未使用コード）

## 3. 現在の使用状況

### 3.1 実際に使用されている変換処理

1. **入力変換**:
   - `mxArrayToFloatArray`: 広く使用（すべてのMEX入力で使用）
   - `mxGetScalarAsFloat`: スカラー値の入力で使用
   - GPSデータ: `mxGetPr`でdoubleのまま取得

2. **出力変換**:
   - `do_get_state`: **single（float）形式で出力** ← メインの出力関数
   - `state_to_matlab`: single/double両対応（レガシー互換）
   - `do_meukf_step`: double形式で出力（内部使用）

### 3.2 未使用の可能性がある変換処理

以下の関数は定義されているが、実際のコードでは使用されていない可能性があります：

- `floatArrayToMxArray`: double形式への変換（レガシー互換用）
- `floatArrayToMxArrayFloat`: single形式への変換（新方式）
- `vectorToMat`: Vector型をdouble形式に変換
- `matrixToMat`: Matrix型をdouble形式に変換

## 4. 型変換の流れ

### 4.1 通常のデータフロー

```
MATLAB (single/double)
    ↓ mxArrayToFloatArray
C++ (float) - 計算処理
    ↓ do_get_state (mxSINGLE_CLASS)
MATLAB (single)
```

### 4.2 GPSデータのデータフロー

```
MATLAB (double: lat, lon, alt)
    ↓ mxGetPr (doubleのまま)
C++ (double) - preprocess_gps (double演算)
    ↓ floatに変換
C++ (float: gps_pos[3])
    ↓ do_get_state (mxSINGLE_CLASS)
MATLAB (single: 位置として出力)
```

## 5. まとめ

### 残っているdouble変換処理

1. **入力側**:
   - GPSデータ（lat, lon, alt）: doubleのまま受け取り
   - その他のデータ: double/single → floatに変換

2. **出力側**:
   - `do_get_state`: **すべてsingle（float）形式で出力** ← メイン
   - `state_to_matlab`: レガシー互換のためdouble形式もサポート
   - `do_meukf_step`: double形式で出力（内部使用）

### 推奨事項

- メインの出力関数（`do_get_state`）は既にsingle形式で出力しているため、現在の実装は適切
- レガシー互換のためのdouble変換（`state_to_matlab`のelse節）は残しておく必要がある
- 未使用の可能性がある関数（`vectorToMat`, `matrixToMat`等）は、将来的に削除を検討


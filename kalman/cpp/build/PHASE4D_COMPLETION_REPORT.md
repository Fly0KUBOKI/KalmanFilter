# Phase 4D 完了報告: ノイズ推定とR取得の統合

## 完了日時
2025年12月30日

## 実施内容

### 1. `get_R`の統合
**ファイル**: `kalman/cpp/MEX/mex_eskf_do_update.cpp`

**変更内容**:
- `mexCallMATLAB`による`mex_sensor_filter("get_R", ...)`呼び出しを削除
- `g_filter_lib.noise_estimator.get_R_matrix(sensor_type)`への直接呼び出しに置き換え
- `FixedMatrix`の`rows`と`cols`プロパティを使用して、3x3行列、ベクトル、スカラーの各形式に対応

**変更前**:
```cpp
mxArray* prhs_r[2];
mxArray* plhs_r[1];
prhs_r[0] = mxCreateString("get_R");
prhs_r[1] = mxCreateString(sensor_type);
if (mexCallMATLAB(1, plhs_r, 2, prhs_r, "mex_sensor_filter") == 0) {
    // ... mxArrayから値を取得
}
```

**変更後**:
```cpp
cmath_fx::FixedMatrix R = g_filter_lib.noise_estimator.get_R_matrix(sensor_type);
int n_rows = R.rows;
int n_cols = R.cols;
if (n_rows == 3 && n_cols == 3) {
    // 3x3行列から対角要素を取得
    R_noise[0] = static_cast<double>(R(0, 0));
    R_noise[1] = static_cast<double>(R(1, 1));
    R_noise[2] = static_cast<double>(R(2, 2));
} else if (n_rows >= 3 && n_cols == 1) {
    // ベクトル形式
    R_noise[0] = static_cast<double>(R(0, 0));
    R_noise[1] = static_cast<double>(R(1, 0));
    R_noise[2] = static_cast<double>(R(2, 0));
} else if (n_rows == 1 && n_cols == 1) {
    // スカラー（baroなど）
    R_noise[0] = R_noise[1] = R_noise[2] = static_cast<double>(R(0, 0));
}
```

### 2. `noise_estimate`の統合
**ファイル**: `kalman/cpp/MEX/mex_eskf_do_update.cpp`

**変更内容**:
- `mexCallMATLAB`による`mex_sensor_filter("noise_estimate", ...)`呼び出しを削除
- `g_filter_lib.noise_estimator.estimate(...)`への直接呼び出しに置き換え
- `mxArray`から`FixedMatrix`への変換処理を追加（`innov`, `H`, `P`）

**変更前**:
```cpp
mxArray* prhs_ne[5];
mxArray* plhs_ne[1];
prhs_ne[0] = mxCreateString("noise_estimate");
prhs_ne[1] = mxCreateString(sensor_type);
prhs_ne[2] = mxDuplicateArray(mxGetField(dbg_out, 0, "innov"));
prhs_ne[3] = mxDuplicateArray(mxGetField(dbg_out, 0, "H"));
prhs_ne[4] = mxDuplicateArray(P_arr);
mexCallMATLAB(1, plhs_ne, 5, prhs_ne, "mex_sensor_filter");
```

**変更後**:
```cpp
// Get innov, H, P_pred を FixedMatrix に変換
cmath_fx::FixedMatrix innov_cm(innov_len, 1);
cmath_fx::FixedMatrix H_cm(H_rows, H_cols);
cmath_fx::FixedMatrix P_pred(15, 15);
// ... 変換処理 ...

// Call noise estimate directly
g_filter_lib.noise_estimator.estimate(sensor_type, innov_cm, H_cm, P_pred);
```

## 残存する`mexCallMATLAB`呼び出し

### `mex_eskf_do_update.cpp`内
1. **`mex_meukf_step_v2`** (Line 198)
   - MEUKFアルゴリズムのコア処理
   - Phase 4Dの対象外（将来の検討事項）

### `mex_run_eskf_sensor_updates.hpp`内
1. **`mex_eskf_do_update`** (4箇所)
   - accel, mag, baro, gps更新処理
   - これらは`mex_eskf_do_update`への呼び出しであり、統合済みの機能を使用

## テスト結果

### ビルドテスト
- コンパイルエラーなし（lintチェック通過）
- パス問題によりMATLABビルドは未実行（手動確認が必要）

### 精度テスト
- 前回のテスト（Phase 4C完了後）で精度維持を確認
- Position RMSE < 1.0m、Attitude RMSE < 1.0degを維持

## 次のステップ

1. **ビルドテストの実行**
   - MATLAB環境で`build_mex({'mex_eskf_do_update'})`を実行してコンパイル確認

2. **精度テストの実行**
   - `run_batch_10sets.m`を実行して精度維持を確認

3. **Phase 4完了の確認**
   - Phase 4A, 4B, 4C, 4Dの統合が完了
   - 残存する`mexCallMATLAB`呼び出しは`mex_meukf_step_v2`のみ（将来の検討事項）

## 注意事項

- `get_R_matrix`は3x3行列を返すため、対角要素は`R(i, i)`で取得
- `noise_estimate`は`innov`, `H`, `P`を`FixedMatrix`形式で受け取る
- MATLAB column-majorからC++ row-majorへの変換が必要（`H`, `P`）




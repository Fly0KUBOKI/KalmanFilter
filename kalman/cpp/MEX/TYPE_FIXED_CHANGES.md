# 型変換廃止・型固定の変更内容

## 変更概要

型変換プログラムを廃止し、以下のように型を固定しました：

1. **GPS以外のセンサーデータ**: float（single）のみを受け取る
2. **GPSデータ**: doubleのみを受け取る
3. **出力**: float（single）のみで出力

## 変更されたファイル

### 1. `kalman/cpp/MEX/mex_type_conv.hpp`

#### `mxArrayToFloatArray`
- **変更前**: double/single両方を受け取り、doubleの場合はfloatに変換
- **変更後**: single（float）のみを受け取り、doubleの場合はエラーを出力
- **エラーメッセージ**: "Expected single (float) array, but got %s. GPS以外のセンサーデータはfloatのみを受け取ります。"

#### `mxGetScalarAsFloat`
- **変更前**: double/single両方を受け取り、doubleの場合はfloatに変換
- **変更後**: single（float）のみを受け取り、doubleの場合はエラーを出力
- **エラーメッセージ**: "Expected single (float) scalar, but got %s. GPS以外のスカラー値はfloatのみを受け取ります。"

### 2. `kalman/cpp/MEX/mex_meukf_step.cpp`

#### `state_to_matlab`
- **変更前**: single/double両対応（レガシー互換）
- **変更後**: single（float）のみで出力、doubleの場合はエラーを出力
- **エラーメッセージ**: "Expected single (float) array for field '%s', but got %s. 出力はfloatのみです。"

### 3. `kalman/cpp/Inc/MEX/mex_run_eskf_impl.hpp`

#### `do_meukf_step`
- **変更前**: 入力構造体の型に合わせてdouble形式で出力
- **変更後**: single（float）のみで出力、doubleの場合はエラーを出力
- **エラーメッセージ**: "Expected single (float) array for field '%s', but got %s. 出力はfloatのみです。"

#### `do_step` (GPSデータ処理)
- **変更前**: `mxGetPr`でdoubleを取得（型チェックなし）
- **変更後**: doubleのみを受け取り、それ以外の場合はエラーを出力
- **エラーメッセージ**: "Expected double array for GPS 'lat'/'lon'/'alt', but got %s. GPSデータはdoubleのみを受け取ります。"

## 型の固定ルール

### 入力
- **GPS以外のセンサーデータ** (accel, gyro, mag, baro等): `single` (float) のみ
- **GPSデータ** (lat, lon, alt): `double` のみ
- **状態データ** (p, v, q, ba, bg, P): `single` (float) のみ
- **パラメータ** (g, mag_ref, noise_*等): `single` (float) のみ

### 出力
- **すべての出力データ**: `single` (float) のみ
  - 位置 (p)
  - 速度 (v)
  - 姿勢 (euler, q)
  - バイアス (ba, bg)
  - 共分散行列 (P)

## エラーハンドリング

型が一致しない場合、以下のエラーが発生します：

1. **入力エラー**: `mex_conv:type_error`
   - GPS以外のデータがdoubleの場合
   - GPSデータがdouble以外の場合

2. **出力エラー**: `mex_meukf_step:type_error` または `mex_run_eskf:type_error`
   - 出力先の構造体フィールドがdoubleの場合

## 互換性

- **破壊的変更**: 以前はdouble/single両方を受け付けていましたが、今後は型が厳密に固定されます
- **MATLAB側の対応**: すべてのデータを`single`型で渡し、GPSデータのみ`double`型で渡す必要があります

## 未使用の関数

以下の関数は定義されていますが、現在は使用されていません（将来の削除を検討）：

- `floatArrayToMxArray`: double形式への変換（レガシー互換用）
- `floatArrayToMxArrayFloat`: single形式への変換
- `vectorToMat`: Vector型をdouble形式に変換
- `matrixToMat`: Matrix型をdouble形式に変換


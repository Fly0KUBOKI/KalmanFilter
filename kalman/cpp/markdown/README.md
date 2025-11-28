# C++ / MEX Implementation for Kalman Filters

ESKFシミュレーションの高速化のため、カルマンフィルタのコア計算をC++/MEXで実装しました。

## ディレクトリ構成

```
cpp/
├── src/              # C++実装ソース (.cpp)
│   ├── KF/           # Kalman Filter 実装
│   ├── EKF/          # Extended Kalman Filter 実装
│   ├── ESKF/         # Error State Kalman Filter 実装
│   └── UKF/          # Unscented Kalman Filter 実装
│
├── include/          # C++ヘッダファイル (.hpp)
│   ├── Common/       # 共通ライブラリ
│   │   ├── Math/     # 数学ユーティリティ (quaternion, fixed_matrix)
│   │   ├── Sensor/   # センサーフィルタ
│   │   └── Validation/ # 検証・正則化
│   ├── KF/           # KF ヘッダ
│   ├── EKF/          # EKF ヘッダ
│   ├── ESKF/         # ESKF ヘッダ
│   ├── UKF/          # UKF ヘッダ
│   └── kalman_filters.hpp  # トップレベルヘッダ
│
├── mex/              # mex ラッパーソース (.cpp)
│   ├── mex_kalman_filter_core.cpp
│   ├── mex_eskf_core.cpp
│   ├── mex_quaternion_lib.cpp
│   ├── mex_ukf_sigma_points.cpp
│   └── mex_ukf_update.cpp  (全5個)
│
├── bin/           # ビルド済MEXバイナリ (.mexw64)
│   ├── mex_kalman_filter_core.mexw64
│   ├── mex_eskf_core.mexw64
│   ├── mex_quaternion_lib.mexw64
│   ├── mex_ukf_sigma_points.mexw64
│   └── mex_ukf_update.mexw64  (全5個)
│
├── build/            # ビルドスクリプト
│   └── build_mex.m   # MEXビルドスクリプト
│
│
├── build/            # ビルドスクリプト
│   └── build_mex.m   # MEXビルドスクリプト
│
├── tests/            # テストスクリプト (予約)
│
└── README.md         # このファイル
```

## 概要

`kalman_filter_core.m`の主要な計算部分をC++で再実装し、MEX関数として呼び出すことで、ESKFシミュレーションを高速化します。

### ビルド済みMEXファイル (5個)

1. **mex_kalman_filter_core.mexw64** - カルマンフィルタコア関数（ゲイン計算、状態更新等）
2. **mex_eskf_core.mexw64** - ESKF積分・予測・更新関数
3. **mex_quaternion_lib.mexw64** - クォータニオン演算（積、共役、回転等）
4. **mex_ukf_sigma_points.mexw64** - UKFシグマポイント生成
5. **mex_ukf_update.mexw64** - UKF更新ステップ

## ビルド方法

### 1. C++コンパイラの設定

初回のみ、MATLABでC++コンパイラを設定します：

```matlab
mex -setup C++
```

- Windows: Visual Studio または MinGW-w64が必要
- Mac: Xcode Command Line Toolsが必要
- Linux: gccが必要

### 2. MEXファイルのビルド

```matlab
cd cpp/build
build_mex()
```

ビルドが成功すると、`cpp/mexw64/` に `.mexw64`（Windows）または対応する拡張子のファイルが生成されます。

**注意事項:**
- `mex_kf_core.cpp` と `mex_ekf.cpp` は古いAPIを使用しているため、現在はビルドされません
- 5個のMEXファイルが正常にビルドされます

## 使用方法

### パス設定

MATLAB でC++ MEXライブラリを使用する前に、パスを設定します：

```matlab
addpath('cpp/mexw64')
```

利用可能なMEXファイルを確認：

```matlab
which mex_eskf_core
which mex_quaternion_lib
```

### MEX動作確認

```matlab
cd cpp
test_mex_loading()
```

8つのMEXファイルがすべて読み込み可能か確認します。

### 自動的にMEXを使用

`kalman_filter_core.m`は自動的にMEX実装が利用可能かチェックし、存在すれば使用します：

```matlab
% 通常通りに呼び出すだけ
P = kalman_filter_core('predict_step', P, q, a_meas, ba, w_meas, bg, Q, dt);
```

初回実行時に以下のメッセージが表示されます：
- MEX使用時: `[kalman_filter_core] MEX acceleration enabled`
- MATLAB実装使用時: `[kalman_filter_core] Using MATLAB implementation`

### テスト

MEX実装が正しく動作するか確認：

```matlab
cd cpp
test_mex_loading()  % MEXが読み込み可能か確認
```

### ESKFシミュレーションの実行

通常通りシミュレーションを実行するだけで、MEX高速化が適用されます：

```matlab
cd ..  % kalman/ ルートに戻る
run_simulation()
```

## パフォーマンス

予測されるパフォーマンス向上：
- `predict_step`: 約5-10倍高速化
- `compute_kalman_gain`: 約3-5倍高速化
- `update_state_covariance`: 約3-5倍高速化

実際の高速化率は、MATLABのバージョンやシステム構成によって異なります。

## ファイル構成

```
cpp/
├── src/              # C++実装ソース
├── include/          # C++ヘッダファイル
├── mex/              # MEX ラッパーソース
├── bin/prebuilt/     # プリビルド済MEXバイナリ
├── build/            # ビルドスクリプト
├── tests/            # テストスクリプト
├── setup_cpp_path.m  # パス設定スクリプト
├── test_mex_loading.m # MEX読み込みテスト
└── README.md         # このファイル
```

## 注意事項

- MEXファイルはプラットフォーム固有です（Windows用の.mexw64はMac/Linuxで動作しません）
- MEXファイルが見つからない場合、自動的にMATLAB実装にフォールバックします
- `bin/prebuilt/` にあるプリビルド済MEXバイナリは配布用です（Git管理対象）

## 今後の拡張

- より高度な最適化（SIMD、並列化）
- GPU対応（CUDA MEX）
- クロスプラットフォームビルドの自動化（CMake）

## ライセンス

このプロジェクトと同じライセンスに従います。

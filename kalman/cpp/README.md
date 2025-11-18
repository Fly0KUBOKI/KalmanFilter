# ESKF MEX高速化

ESKFシミュレーションの高速化のため、カルマンフィルタコア計算をC++/MEXで実装しました。

## 概要

`kalman_filter_core.m`の主要な計算部分をC++で再実装し、MEX関数として呼び出すことで、ESKFシミュレーションを高速化します。

### 高速化された関数

**KF/EKF用:**
- `predict_step`: 共分散の予測ステップ (P = F*P*F' + Q*dt)
- `compute_kalman_gain`: カルマンゲインの計算 (K = P*H' / S)
- `update_state_covariance`: 状態と共分散の更新（Joseph形式）
- `compute_jacobian`: ヤコビアン行列の計算

**UKF用:**
- `ukf_sigma_points`: シグマポイントと重みの生成

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
cd cpp
build_mex()
```

ビルドが成功すると、`mex_kalman_filter_core.mexw64`（Windows）または対応する拡張子のファイルが生成されます。

## 使用方法

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
cd ..
check_mex_usage()  % MEXが使われているか確認
cd cpp
test_mex_kalman_filter_core()  % 詳細テスト
```

### ESKFシミュレーションの実行

通常通りシミュレーションを実行するだけで、MEX高速化が適用されます：

```matlab
cd ..
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
├── build_mex.m                    # ビルドスクリプト
├── test_mex_kalman_filter_core.m  # テストスクリプト
├── README.md                      # このファイル
├── mex_kalman_filter_core.mexw64  # コンパイル済みMEXファイル（Windows）
├── KF/
│   └── Core/
│       ├── kalman_filter_core.hpp # C++ヘッダー
│       └── kalman_filter_core.cpp # C++実装
├── Common/
│   └── Math/
│       ├── fixed_matrix.hpp       # 固定サイズ行列ライブラリ
│       └── quaternion.hpp         # クォータニオンユーティリティ
└── MEX/
    └── mex_kalman_filter_core.cpp # MEXラッパー
```

## トラブルシューティング

### コンパイルエラー

1. **コンパイラが見つからない**
   ```matlab
   mex -setup C++
   ```
   を実行し、表示される指示に従ってコンパイラをインストール/選択

2. **ヘッダーファイルが見つからない**
   - `cpp/KF/Core/`と`cpp/Common/Math/`にファイルが存在するか確認

3. **リンクエラー**
   - Visual Studioがインストールされているか確認（Windows）
   - コンパイラのバージョンがMATLABと互換性があるか確認

### 実行時エラー

1. **MEXファイルが見つからない**
   ```matlab
   which mex_kalman_filter_core
   ```
   で場所を確認。パスが通っているか確認。

2. **MEXファイルが動作しない**
   - MATLABを再起動
   - `clear mex`を実行
   - `build_mex()`でリビルド

3. **精度の問題**
   - MEX実装はMATLAB実装と同じアルゴリズムを使用しています
   - 数値誤差の蓄積が大きい場合は共分散の正則化パラメータを調整

## 注意事項

- `compute_innovation_and_S`は、paramsパラメータの複雑な処理があるため、MATLAB実装のままです
- MEXファイルはプラットフォーム固有です（Windows用の.mexw64はMac/Linuxで動作しません）
- MEXファイルが見つからない場合、自動的にMATLAB実装にフォールバックします

## 削除されたファイル

不要なファイルを整理しました：
- `test_ekf.m`, `test_ukf.m`, `test_mex.m`: 古いテストファイル
- `STRUCTURE.md`: 古いドキュメント
- `mex_build_results.mat`, `mex_kf_core.exp`, `mex_kf_core.lib`: 古いビルド成果物
- `build_all_cpp_mex.m`: 使用されていない古いビルドスクリプト

## 今後の拡張

- UKF/EKFの主要計算部分のMEX化
- より高度な最適化（SIMD、並列化）
- GPU対応（CUDA MEX）

## ライセンス

このプロジェクトと同じライセンスに従います。

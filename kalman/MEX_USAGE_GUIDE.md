# MEX高速化 - 使用方法と確認手順

## 概要

ESKF/EKF/UKFの計算集約的な部分をC++/MEXで実装し、シミュレーションを高速化しました。

## 現在実装されているMEX関数

### 1. mex_kalman_filter_core (KF/EKF/ESKF用)
- `predict_step` - 共分散予測
- `compute_kalman_gain` - カルマンゲイン計算
- `update_state_covariance` - 状態・共分散更新
- `compute_jacobian` - ヤコビアン計算

### 2. mex_ukf_sigma_points (UKF用)
- シグマポイントと重みの生成

## ビルド方法

```matlab
cd cpp
build_mex()
```

期待される出力：
```
=== MEX Build for ESKF Acceleration ===
Using C++ compiler: Microsoft Visual C++ 2022

=== [1/2] Building mex_kalman_filter_core ===
Sources: mex_kalman_filter_core.cpp, kalman_filter_core.cpp
✓ Built and copied: mex_kalman_filter_core.mexw64

=== [2/2] Building mex_ukf_sigma_points ===
Sources: mex_ukf_sigma_points.cpp
✓ Built and copied: mex_ukf_sigma_points.mexw64

=== Build Complete ===
Successfully built 2 MEX file(s)
```

## MEX使用確認

### 方法1: 簡単な確認

```matlab
cd ..  % kalmanディレクトリに戻る
check_mex_usage()
```

期待される出力：
```
=== MEX Usage Check ===

1. Checking MEX files...
   ✓ mex_kalman_filter_core found: C:\...\mex_kalman_filter_core.mexw64

2. Testing kalman_filter_core...
[kalman_filter_core] MEX acceleration enabled
   ✓ kalman_filter_core works (0.XXX ms)
   Output size: 15x15

3. Performance test (100 iterations)...
   Average time: 0.XXX ms per call
   Total time: X.XXX seconds

4. Direct MEX call test...
   ✓ Direct MEX call works (0.XXX ms)
   Difference from MATLAB wrapper: X.XXe-XX

=== Check Complete ===
```

### 方法2: シミュレーション実行で確認

```matlab
clear all  % persistentな変数をクリア
run_simulation()
```

**確認ポイント:**
- 実行開始時に以下のメッセージが表示されるか確認：
  - `[kalman_filter_core] MEX acceleration enabled`
  - `[ukf_sigma_points] MEX acceleration enabled` (UKF使用時)

- メッセージが表示されない場合：
  - MEXファイルが見つからない
  - パスが通っていない
  - ビルドが必要

## トラブルシューティング

### MEXメッセージが表示されない

**原因1: persistentな変数が初期化されていない**
```matlab
clear kalman_filter_core
clear ukf_sigma_points
run_simulation()
```

**原因2: MEXファイルが見つからない**
```matlab
which mex_kalman_filter_core
which mex_ukf_sigma_points
```

両方とも `.mexw64` (Windows) または該当する拡張子のファイルが表示されるべきです。

表示されない場合：
```matlab
cd cpp
build_mex()
```

**原因3: パスの問題**
```matlab
addpath('cpp')
rehash
```

### パフォーマンスが改善しない

1. MEXが実際に使われているか確認
   ```matlab
   check_mex_usage()
   ```

2. MATLAB実装と比較
   ```matlab
   cd cpp
   test_mex_kalman_filter_core()
   ```

3. プロファイラで確認
   ```matlab
   profile on
   run_simulation()
   profile viewer
   ```

## 期待される高速化

### 個別関数
- `predict_step`: 5-10倍
- `compute_kalman_gain`: 3-5倍
- `update_state_covariance`: 3-5倍
- `ukf_sigma_points`: 3-5倍

### 全体シミュレーション
- ESKF: 3-5倍高速化
- UKF使用時: さらに10-20%高速化

## 既存のMEXファイル確認

```matlab
% 現在利用可能なMEXファイルを確認
cd cpp
ls *.mex*
```

期待される出力（Windows）:
```
mex_kalman_filter_core.mexw64
mex_ukf_sigma_points.mexw64
```

## 注意事項

1. **初回実行時のメッセージ**
   - MEX使用の確認メッセージは初回の関数呼び出し時のみ表示
   - 2回目以降は表示されないが、MEXは使用されている

2. **clear後の再初期化**
   - `clear kalman_filter_core`を実行するとpersistent変数がクリアされ、次回実行時に再度メッセージが表示される

3. **プラットフォーム依存**
   - Windows: `.mexw64`
   - Linux: `.mexa64`
   - Mac Intel: `.mexmaci64`
   - Mac Apple Silicon: `.mexmaca64`

4. **フォールバック機能**
   - MEXファイルが見つからない場合、自動的にMATLAB実装を使用
   - エラーは発生しない

## 開発用コマンド

```matlab
% MEXファイルをメモリからアンロード
clear mex

% persistent変数をクリア
clear kalman_filter_core ukf_sigma_points

% 再ビルド
cd cpp
build_mex()

% テスト
cd ..
check_mex_usage()
run_simulation()
```

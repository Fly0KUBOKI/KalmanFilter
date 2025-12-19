# MEXビルドとテスト手順

## ビルド手順

MATLABを開いて、以下のコマンドを実行してください：

```matlab
% 1. cppディレクトリに移動
cd('c:\Users\takut\OneDrive\ドキュメント\MATLAB\KalmanFilter\kalman\cpp')

% 2. C++コンパイラの設定（初回のみ）
mex -setup C++

% 3. MEXファイルのビルド
build_mex()

% 4. テスト実行
test_mex_kalman_filter_core()

% 5. 親ディレクトリに移動してシミュレーション実行
cd ..
run_simulation()
```

## 期待される出力

### ビルド時
```
=== MEX Build for ESKF Acceleration ===
Using C++ compiler: Microsoft Visual C++ 2022

--- Building mex_kalman_filter_core ---
Sources:
  - mex_kalman_filter_core.cpp
  - C:\...\kalman_filter_core.cpp
Include paths:
  - C:\...\KF\Core
  - C:\...\Common\Math
  - C:\...\cpp

Executing: mex -O -DNDEBUG ...
Building with 'Microsoft Visual C++ 2022'.
MEX completed successfully.
✓ Successfully built: mex_kalman_filter_core.mexw64
✓ Copied to: C:\...\cpp\mex_kalman_filter_core.mexw64

=== Build Complete ===
```

### テスト時
```
=== Testing mex_kalman_filter_core ===

✓ MEX file found: C:\...\mex_kalman_filter_core.mexw64

Test 1: predict_step
  MEX execution time: 0.XXX ms
  Output size: 15x15
  Output trace: X.XXXXXX
  ✓ Test passed

Test 2: compute_kalman_gain
  MEX execution time: 0.XXX ms
  Gain size: 15x3
  Gain norm: X.XXXXXX
  ✓ Test passed

Test 3: update_state_covariance
  MEX execution time: 0.XXX ms
  State norm: X.XXXXXX
  P trace: X.XXXXXX
  ✓ Test passed

Test 4: compute_jacobian
  MEX execution time: 0.XXX ms
  Jacobian size: 15x15
  Jacobian norm: X.XXXXXX
  ✓ Test passed

=== Performance Comparison ===
Running 100 iterations...
MEX average time: 0.XXX ms

=== All Tests Complete ===
MEX implementation is working correctly!
```

### シミュレーション実行時
```
[kalman_filter_core] MEX acceleration enabled
推定完了
```

## トラブルシューティング

### エラー: コンパイラが見つからない

**原因**: C++コンパイラが設定されていない

**解決方法**:
```matlab
mex -setup C++
```
を実行し、Visual StudioまたはMinGW-w64を選択

### エラー: ヘッダーファイルが見つからない

**原因**: ファイル構造が正しくない

**確認**:
```matlab
exist('cpp\KF\Core\kalman_filter_core.cpp', 'file')
exist('cpp\Common\Math\fixed_matrix.hpp', 'file')
```
両方が `2` を返すことを確認

### エラー: MEXファイルが動作しない

**解決方法**:
```matlab
clear mex
rehash
build_mex()
```

### パフォーマンスが改善しない

**確認事項**:
1. MEXファイルが実際に使われているか確認:
   ```matlab
   which mex_kalman_filter_core
   ```

2. MATLABの出力メッセージを確認:
   - `[kalman_filter_core] MEX acceleration enabled` が表示されればMEX使用中
   - `[kalman_filter_core] Using MATLAB implementation` が表示されればMATLAB実装使用中

## ファイル一覧

### 必須ファイル
- `cpp/build_mex.m` - ビルドスクリプト
- `cpp/test_mex_kalman_filter_core.m` - テストスクリプト
- `cpp/MEX/mex_kalman_filter_core.cpp` - MEXラッパー
- `cpp/KF/Core/kalman_filter_core.cpp` - コア実装
- `cpp/KF/Core/kalman_filter_core.hpp` - ヘッダー
- `cpp/Common/Math/fixed_matrix.hpp` - 行列ライブラリ
- `cpp/Common/Math/quaternion.hpp` - クォータニオン

### 生成されるファイル
- `cpp/mex_kalman_filter_core.mexw64` (Windows)
- `cpp/mex_kalman_filter_core.mexa64` (Linux)
- `cpp/mex_kalman_filter_core.mexmaci64` (Mac Intel)
- `cpp/mex_kalman_filter_core.mexmaca64` (Mac Apple Silicon)

### 削除されたファイル（不要）
- `test_ekf.m`, `test_ukf.m`, `test_mex.m`
- `STRUCTURE.md`, `KF_EKF_UKF_README.md`
- `build_all_mex.m`, `build_all_cpp_mex.m`
- `mex_build_results.mat`
- `mex_kf_core.exp`, `mex_kf_core.lib`

## 使用方法

MEXファイルがビルドされると、`kalman_filter_core.m`が自動的にMEX実装を使用します。
特別な変更は不要で、通常通りESKFシミュレーションを実行するだけです。

```matlab
run_simulation()
```

初回実行時にMEX高速化が有効になったことを示すメッセージが表示されます。

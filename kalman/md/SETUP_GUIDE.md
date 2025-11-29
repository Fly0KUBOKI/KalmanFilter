# C++/MATLAB統合システム - セットアップ & 実行ガイド

## 📋 概要

このガイドでは、カルマンフィルタC++計算エンジンとMATLAB状態管理システムの統合環境をセットアップし、実行する手順を説明します。

## 🎯 アーキテクチャ

```
┌─────────────────────────────────────────┐
│        MATLAB レイヤー                   │
│  ・状態管理 (ESKF, UKF, EKF, KF)        │
│  ・センサーフィルタリング                │
│  ・データ入出力                          │
└─────────────┬───────────────────────────┘
              │ KalmanCompute.m (ラッパー)
              ↓
┌─────────────────────────────────────────┐
│     MEX インターフェース                 │
│  ・mex_kalman_compute.mexw64            │
│  ・mex_kalman_filter_core.mexw64        │
│  ・mex_ukf_*.mexw64                     │
└─────────────┬───────────────────────────┘
              │
              ↓
┌─────────────────────────────────────────┐
│      C++ 計算エンジン                    │
│  ・QuaternionCompute (14関数)           │
│  ・RotationCompute (12関数)             │
│  ・状態非依存の純粋計算                  │
└─────────────────────────────────────────┘
```

**設計原則:**
- **C++**: 状態に依存しない純粋な計算関数（`float`型、入力/出力行列）
- **MATLAB**: 状態管理、センサー処理、フィルタロジック

## 🚀 セットアップ手順

### ステップ1: 環境確認

**必須要件:**
- MATLAB R2018b以降
- C++コンパイラ (MATLAB MEX用)
  - Windows: Visual Studio 2017以降、または MinGW-w64
- Python 3.x (解析用、オプション)

**MEXコンパイラの設定確認:**
```matlab
>> mex -setup C++
```

### ステップ2: C++ MEXファイルのビルド

**方法A: 統合スクリプトで全自動実行**
```matlab
% MATLABコマンドウィンドウで実行
cd('kalman')
run_build_and_test
```

このスクリプトは以下を自動実行します:
1. ✅ C++ MEXファイルのビルド
2. ✅ MATLAB関数呼び出しの確認
3. ✅ シミュレーション実行
4. ✅ Python解析の起動

**方法B: 手動ビルド（デバッグ用）**
```matlab
% カルマンフィルタディレクトリに移動
cd('kalman/cpp')

% MEXビルドを実行
build_mex

% ビルド結果確認
ls bin/*.mexw64
```

**期待される出力:**
```
cpp/bin/
  ├── mex_kalman_compute.mexw64      (新規統合ライブラリ)
  ├── mex_kalman_filter_core.mexw64
  ├── mex_ukf_sigma_points.mexw64
  ├── mex_ukf_update.mexw64
  ├── mex_quaternion_lib.mexw64      (レガシー)
  ├── mex_eskf_math.mexw64
  └── mex_eskf_core.mexw64
```

### ステップ3: MATLAB関数呼び出しの更新（初回のみ）

既存のMATLABコードで`QuaternionLib.*`や`RotationLib.*`を使用している場合、新しい`KalmanCompute.*`に変換します。

**手動変換例:**
```matlab
% 旧コード
q = QuaternionLib.normalize(q);
R = QuaternionLib.to_rotation_matrix(q);
S = RotationLib.skew_symmetric(v);

% 新コード
q = KalmanCompute.quat_normalize(q);
R = KalmanCompute.quat_to_rotation_matrix(q);
S = KalmanCompute.rot_skew_symmetric(v);
```

**自動変換ツール（オプション）:**
```matlab
% 全MATLABファイルを一括変換
update_to_kalman_compute
```

⚠️ **注意**: 自動変換後は、変更されたファイルを確認してください。

## 🧪 動作確認

### クイックテスト

```matlab
% KalmanComputeが正しく動作するか確認
cd('kalman')

% MEXが利用可能か確認
q_in = [1; 0; 0; 0];
q_out = KalmanCompute.quat_normalize(q_in);
disp(q_out);  % [1; 0; 0; 0] が出力されるはず

% 回転行列への変換
R = KalmanCompute.quat_to_rotation_matrix(q_in);
disp(R);  % 3x3単位行列が出力されるはず
```

### フルシミュレーションテスト

```matlab
% データ生成 → ESKF実行 → 解析
cd('kalman')
run_build_and_test
```

**期待される出力:**
```
========================================
 Kalman Filter C++/MATLAB統合ビルド
========================================

[1/4] C++ MEXファイルをビルド中...
  ✓ MEXビルド完了

[2/4] MATLAB関数呼び出しを確認中...
  ✓ KalmanCompute.m が見つかりました
  ✓ KalmanCompute MEX呼び出し成功

[3/4] カルマンフィルタシミュレーションを実行中...
  - ESKFシミュレーションを実行中...
  ✓ シミュレーション完了

[4/4] Python解析スクリプトを起動中...
  ✓ Python解析完了

========================================
 全ての処理が完了しました！
========================================
```

## 📊 出力ファイル

ビルドとシミュレーション実行後、以下のファイルが生成されます:

```
kalman/
├── cpp/bin/                        # MEXバイナリ（全てここに集約）
│   ├── mex_kalman_compute.mexw64
│   └── ...
├── Results/
│   └── estimation.csv              # カルマンフィルタ推定結果
├── GenerateData/
│   ├── truth_data.csv              # 真値データ
│   └── sensor_data.csv             # センサー観測データ
└── analysis_log.txt                # Python解析ログ
```

## 🔍 トラブルシューティング

### MEXビルドエラー

**症状:** `build_mex`実行時にコンパイルエラー

**解決策:**
1. MEXコンパイラが設定されているか確認
   ```matlab
   mex -setup C++
   ```

2. Visual Studioがインストールされているか確認（Windows）

3. 詳細ログを確認
   ```matlab
   cd cpp
   build_mex  % エラーメッセージを確認
   ```

### KalmanCompute呼び出しエラー

**症状:** `Undefined function or variable 'KalmanCompute'`

**解決策:**
1. パスが正しいか確認
   ```matlab
   addpath('Common/Math')
   which KalmanCompute
   ```

2. MEXファイルがビルドされているか確認
   ```matlab
   ls cpp/bin/mex_kalman_compute.mexw64
   ```

3. フォールバック動作を確認
   ```matlab
   % MEXが無くてもMATLAB実装にフォールバック
   KalmanCompute.quat_normalize([1;0;0;0])
   ```

### シミュレーションエラー

**症状:** `run_simulation`実行時にエラー

**解決策:**
1. データファイルが存在するか確認
   ```matlab
   ls GenerateData/*.csv
   ```

2. データを再生成
   ```matlab
   cd GenerateData
   sim_generate
   cd ..
   ```

3. ESKF初期化パラメータを確認
   ```matlab
   edit run_simulation.m
   ```

## 📚 C++ API リファレンス

### QuaternionCompute (14関数)

```matlab
% 基本演算
q_out = KalmanCompute.quat_multiply(q1, q2)           % [4,4] -> [4]
q_out = KalmanCompute.quat_normalize(q)               % [4] -> [4]
q_out = KalmanCompute.quat_conjugate(q)               % [4] -> [4]

% 変換
R = KalmanCompute.quat_to_rotation_matrix(q)          % [4] -> [9] (row-major)
euler = KalmanCompute.quat_to_euler(q)                % [4] -> [3] (degrees)
q = KalmanCompute.quat_from_euler(euler)              % [3] -> [4] (degrees)

% 積分・補間
q_out = KalmanCompute.quat_integrate(q, omega, dt)    % [4,3,1] -> [4]
q_out = KalmanCompute.quat_slerp(q1, q2, t)          % [4,4,1] -> [4]

% その他
q_small = KalmanCompute.quat_small_angle(theta)       % [3] -> [4]
v_rot = KalmanCompute.quat_rotate_vector(q, v)       % [4,3] -> [3]
axis_angle = KalmanCompute.quat_to_axis_angle(q)     % [4] -> [4]
q = KalmanCompute.quat_from_axis_angle(axis_angle)   % [4] -> [4]
```

### RotationCompute (12関数)

```matlab
% 基本演算
R_out = KalmanCompute.rot_multiply(R1, R2)            % [9,9] -> [9]
S = KalmanCompute.rot_skew_symmetric(v)               % [3] -> [9]
R_ortho = KalmanCompute.rot_orthonormalize(R)         % [9] -> [9]

% 変換
R = KalmanCompute.rot_from_euler(euler)               % [3] -> [9] (degrees)
euler = KalmanCompute.rot_to_euler(R)                 % [9] -> [3] (degrees)

% Rodrigues公式
R = KalmanCompute.rot_rodrigues(axis, angle)          % [3,1] -> [9] (radians)
[axis,angle] = KalmanCompute.rot_to_axis_angle(R)    % [9] -> [4]

% その他
R = KalmanCompute.rot_from_two_vectors(v1, v2)       % [3,3] -> [9]
```

**注意事項:**
- 全ての`[9]`出力は **行優先 (row-major)** で保存された3×3行列
- MATLAB使用時は自動的に列優先に変換されます（`reshape(..., [3,3])'`）
- 角度は **度 (degrees)** で入力/出力（内部でラジアン変換）

## 🔄 ワークフロー

### 開発サイクル

1. **C++コード変更時:**
   ```matlab
   cd cpp
   build_mex        % 変更した部分だけ再コンパイル
   cd ..
   ```

2. **MATLABコード変更時:**
   ```matlab
   clear all        % メモリクリア
   run_simulation   % 直接実行
   ```

3. **新規関数追加時:**
   - `cpp/include/Common/Math/*.hpp` にプロトタイプ追加
   - `cpp/src/Common/Math/*.cpp` に実装追加
   - `cpp/MEX/mex_kalman_compute.cpp` にハンドラ追加
   - `Common/Math/KalmanCompute.m` にMATLABラッパー追加
   - `build_mex` で再ビルド

### パフォーマンス測定

```matlab
% MEX vs MATLAB比較
N = 10000;

% MEX版
tic;
for i = 1:N
    q = KalmanCompute.quat_normalize([1;0.1;0.2;0.3]);
end
t_mex = toc;

% MATLAB版（QuaternionLib使用）
tic;
for i = 1:N
    q = QuaternionLib.normalize([1;0.1;0.2;0.3]);
end
t_matlab = toc;

fprintf('MEX: %.4f秒, MATLAB: %.4f秒 (高速化: %.1fx)\n', ...
        t_mex, t_matlab, t_matlab/t_mex);
```

## 📝 次のステップ

1. ✅ `run_build_and_test` を実行して環境を確認
2. ✅ `Results/estimation.csv` で推定結果を確認
3. ✅ グラフ出力やログを確認
4. 🔄 必要に応じてMATLABコードを `KalmanCompute.*` に変換
5. 🚀 パフォーマンスを測定してC++の効果を確認

## 🆘 サポート

問題が発生した場合:
1. `analysis_log.txt` を確認
2. MATLABコマンドウィンドウのエラーメッセージを確認
3. `cpp/README.md` でC++実装の詳細を確認
4. `md/KALMAN_COMPUTE_REFACTORING.md` で設計資料を確認

---

**更新日:** 2024
**バージョン:** 1.0.0

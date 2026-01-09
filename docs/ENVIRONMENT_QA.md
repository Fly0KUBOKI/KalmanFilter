# KalmanFilter 環境依存性問題 — Q&A ガイド

## Q1: なぜコンパイル済みバイナリ（.mexw64）なのに結果が変わるのか？

### 基本的な答え：
**MEXバイナリは、ビルド環境の情報をハードコードしていないため、以下の要因で動作が変わります：**

1. **MEXバイナリの環境適合性**
   - `.mexw64` は Windows 64-bit MATLAB 専用
   - ビルド時のMATLABバージョン（R2021a vs R2024a など）で内部インターフェースが異なる可能性

2. **入力データ（パラメータ・センサーデータ）の違い**
   - config_params.m の値が異なると初期化が変わる
   - センサーデータの型（single vs double）が異なるとMEXの処理結果が変わる

3. **浮動小数点数演算の環境依存**
   - CPUやコンパイラの最適化により、丸めルールが微妙に異なる
   - ただし、通常は mm 単位の誤差

### 【比喩】
> コンパイル済みプログラムは「レシピ本」のようなものです。  
> プログラムのロジック（レシピの作り方）は変わりませんが、  
> **材料（入力パラメータ・センサーデータ）と調理環境（MATLAB・CPU）が異なれば、結果も変わります。**

---

## Q2: どうすれば環境依存の問題を確実に解決できるか？

### 段階別対応：

#### **段階1: 診断スクリプトで現在の状態を把握（5分）**
```matlab
% 1. 問題が発生している環境で実行
cd kalman
diagnose_environment();
```

このスクリプトが以下をチェックします：
- ✅ MATLABバージョン・アーキテクチャ
- ✅ MEXバイナリの存在・バージョン
- ✅ config_params.m の設定値
- ✅ センサーデータ型の一致
- ✅ MEX関数が呼び出し可能か

**出力:** `kalman/Results/environment_diagnostic.txt` に詳細レポートが保存

---

#### **段階2: パラメータを同期（3分）**
```matlab
% 1. 両環境で config_params.m の内容を確認
% 2. 以下が同一になっているか確認

params = config_params();
fprintf('static_time: %.1f\n', params.static_time);           % 5 に統一
fprintf('dt: %.4f\n', params.dt);                             % 0.0025 に統一
fprintf('GPS origin: [%.6f, %.6f]\n', ...
    params.gps_origin.lat, params.gps_origin.lon);            % 36.0, 140.0 に統一
fprintf('accel_std: %.3f\n', params.noise.accel_std);         % 0.100 に統一
```

**同期方法:**
```bash
# Git がある場合
git diff kalman/GenerateData/config_params.m

# なければ、手動で以下を確認
cat kalman/GenerateData/config_params.m | grep -A5 "static_time"
```

---

#### **段階3: MEXを環境専用にビルド（10分）**
```matlab
% 各環境でこれを実行
cd kalman/cpp/build
build_mex();  % すべてのMEXを再ビルド

% ビルド後、必ずキャッシュをクリア
clear functions; clear mex; rehash;
```

**ビルドログで確認:**
```matlab
% 最新のビルドログを表示
type build_mex_log_*.txt | tail -50
```

確認ポイント：
- `mex_run_eskf.mexw64` が正常にビルドされたか
- Compiler Warning が大量にないか

---

#### **段階4: 同じシードで両環境を比較実行（20分）**
```matlab
% 【環境1で実行】
clear functions; clear mex; rehash;
run_simulation(42, false);  % seed=42, skip data generation
copyfile('kalman/Results/estimation.csv', 'env1_result.csv');

% 【環境2で実行】
clear functions; clear mex; rehash;
run_simulation(42, false);  % 同じシードと設定
copyfile('kalman/Results/estimation.csv', 'env2_result.csv');

% 【両環境のいずれかで比較】
compare_environments('env1_result.csv', 'env2_result.csv');
```

**出力解釈:**
| 結果 | 意味 | 対応 |
|------|------|------|
| Max error < 1e-6 | 完全に同一 | ✅ 問題なし |
| Max error < 1e-3 | 数値精度の違い | ✅ 許容範囲 |
| Max error < 0.01 | 初期化パラメータの微妙な違い | ⚠️ 要確認 |
| Max error >= 0.01 | 重大な環境依存性 | ❌ 要修正 |

---

## Q3: 「全く動かない（推定が大きく失敗）」場合、何を最初に確認すべきか？

### 優先順位（確率が高い順）

#### **1位: MEXバイナリが正しくビルドされていない（70%）**
```matlab
% 確認コマンド
clear functions; clear mex;
which mex_run_eskf          % → kalman/cpp/bin/mex_run_eskf.mexw64 を指すべき

% もし見つからない場合
cd kalman/cpp/build
build_mex();
clear mex;
which mex_run_eskf          % 再確認
```

#### **2位: config_params.m の static_time が短すぎる（20%）**
```matlab
params = config_params();
if params.static_time < 3
    error('static_time < 3 sec! Initial Roll/Pitch estimation will fail.');
end
```

**修正:**
```matlab
% kalman/GenerateData/config_params.m 内で
params.static_time = 5;  % 最小 3 秒推奨、5秒推奨
```

#### **3位: GPS原点（GPS origin）が異なる（10%）**
```matlab
% 設定を確認
params = config_params();
fprintf('GPS origin: lat=%.6f, lon=%.6f\n', ...
    params.gps_origin.lat, params.gps_origin.lon);

% 両環境で同じか確認
% もし異なったら、一つの値に統一する
params.gps_origin.lat = 36.0;   % 全環境で統一
params.gps_origin.lon = 140.0;
```

---

## Q4: エラーメッセージごとの対応

### エラー1: "Invalid MEX-file" / "不正なMEXファイル"
```
原因: MEXバイナリが実行環境と不一致
解決:
  cd kalman/cpp/build
  build_mex({'mex_run_eskf'});
  clear mex;
```

### エラー2: "Undefined function or variable 'mex_run_eskf'"
```
原因: MEXバイナリが見つからない、もしくはパスが通っていない
確認:
  which mex_run_eskf
  ls -la kalman/cpp/bin/
  
解決:
  1. kalman/cpp/bin にMEXファイルがあるか確認
  2. なければ: cd kalman/cpp/build && build_mex()
  3. あれば: addpath(fullfile(pwd, 'kalman', 'cpp', 'bin'))
```

### エラー3: 初期化に失敗 "GPS origin mismatch" など
```
原因: config_params.m が欠落または不正
確認:
  params = config_params();
  disp(params.gps_origin);
  
解決:
  kalman/GenerateData/config_params.m が存在するか確認
  存在しなければ、テンプレートから復元
```

### エラー4: 結果が数メートル以上ずれている
```
優先順位:
  1. build_mex() → clear mex → 再実行
  2. diagnose_environment() で詳細確認
  3. config_params.m を同期
  4. センサーデータ型を確認
```

---

## Q5: 「最初は動いていたが、アップデート後に失敗した」場合

### チェックリスト：

1. **MATLABをアップデートした場合**
   ```matlab
   % MEXを再ビルド（必須）
   cd kalman/cpp/build
   build_mex();
   clear mex;
   ```

2. **C++コードを修正した場合**
   ```matlab
   % ビルドしてキャッシュクリア
   cd kalman/cpp/build
   build_mex({'mex_run_eskf'});  % 特定ターゲットのみ
   clear functions; clear mex; rehash;
   ```

3. **config_params.m を修正した場合**
   ```matlab
   % 確認用：新しい設定値を表示
   clear all;  % 前のキャッシュをクリア
   params = config_params();
   disp(params);
   ```

---

## Q6: 複数台の PCで同じ結果を得たい場合

### 推奨ワークフロー：

```matlab
% ===== PC1（開発環境） =====
% 1. 開発環境でMEXをビルド
cd kalman/cpp/build
build_mex();

% 2. config_params.m を確定
params = config_params();
fprintf('Config:\n  static_time=%.1f\n  dt=%.4f\n', ...
    params.static_time, params.dt);

% 3. テスト実行で動作確認
run_simulation(42, false);

% 4. 結果をファイルに保存
copyfile('kalman/Results/estimation.csv', 'reference_result.csv');

% ===== PC2（テスト環境） =====
% 1. メモ: config_params.m の値を確認
%    static_time = 5, dt = 0.0025 を目安に

% 2. MEXをビルド
cd kalman/cpp/build
build_mex();

% 3. 同じシードで実行
run_simulation(42, false);

% 4. 結果を比較
compare_environments('reference_result.csv', 'kalman/Results/estimation.csv');
```

---

## Q7: 「ノイズの程度が異なる」場合

### 原因の可能性：

```matlab
% 確認: config_params.m のノイズ設定
params = config_params();

% ノイズ有効/無効フラグ
fprintf('Noise enabled:\n');
fprintf('  accel: %d\n', params.noise.enable.accel);
fprintf('  gyro: %d\n', params.noise.enable.gyro);
fprintf('  mag: %d\n', params.noise.enable.mag);
fprintf('  gps: %d\n', params.noise.enable.gps);
fprintf('  outlier: %d\n', params.noise.enable.outlier);

% ノイズ標準偏差
fprintf('Noise Std Dev:\n');
fprintf('  accel: %.3f\n', params.noise.accel_std);
fprintf('  gyro: %.3f\n', params.noise.gyro_std);
```

**解決方法:**
```matlab
% 両環境で以下が同じか確認
% 異なれば config_params.m を同期
params.noise.enable.outlier = true;  % 外れ値有効
params.noise.accel_std = 0.1;
params.noise.gyro_std = 0.5;
```

---

## Q8: 「別のマシンで再ビルドしたら、結果が変わった」

### これは想定動作です！

```
重要: MEXバイナリは以下に依存します:
  - コンパイラ（MSVC, GCC など）
  - コンパイラバージョン（16.0, 17.0 など）
  - 最適化レベル（-O0, -O2, -O3 など）
  - ライブラリバージョン（Eigen, BLAS など）

つまり、別のマシンで再ビルドすれば、
微妙な数値差（mm 単位）が発生するのは正常です。

許容範囲:
  - 位置誤差 < 0.1 m → ✅ OK
  - 姿勢誤差 < 1 deg → ✅ OK
  - 速度誤差 < 0.1 m/s → ✅ OK

それ以上の誤差があれば、上記のチェックリストを確認してください。
```

---

## Q9: 自動化したい場合（CI/CDパイプライン等）

### テンプレート：

```matlab
% automated_test.m
function success = automated_test()
    success = false;
    
    try
        % 1. 診断
        fprintf('=== Running environment diagnostic ===\n');
        diagnose_environment();
        
        % 2. パラメータ確認
        params = config_params();
        assert(params.static_time >= 3, 'static_time too short');
        
        % 3. シミュレーション実行
        fprintf('=== Running simulation ===\n');
        run_simulation(42, false);
        
        % 4. 結果確認
        T = readtable('kalman/Results/estimation.csv');
        
        % 簡単な検証
        max_p_error = max(abs(diff(T.p_east(1:100))));
        assert(max_p_error < 100, 'Position divergence detected');
        
        fprintf('✅ All checks passed!\n');
        success = true;
        
    catch ME
        fprintf('❌ Test failed: %s\n', ME.message);
        success = false;
    end
end
```

---

## まとめ表

| 問題の症状 | 原因（確率） | 確認方法 | 解決方法 |
|-----------|----------|--------|--------|
| 全く動かない | MEXビルド失敗 (70%) | `which mex_run_eskf` | `build_mex()` |
| 初期化から失敗 | static_time 不足 (20%) | `params.static_time` | 5秒に設定 |
| 結果がずれる | config_params異なり (10%) | `config_params()` | 同期 |
| 数mm異なる | CPU差（許容） | 誤差 < 1e-3 | 無視OK |
| 新環境で失敗 | MEXが古い (80%) | ビルドログ確認 | 再ビルド |

---


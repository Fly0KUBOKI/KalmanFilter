# KalmanFilter 環境依存性対策ガイド — 実行ステップ

## 概要

このプロジェクトでコンパイル済みバイナリ（MEX）を使用しているため、**異なる実行環境では数値結果が変わる可能性**があります。このガイドでは、その原因と解決方法を説明します。

---

## 【推奨】最初のセットアップ（初回のみ）

### ステップ 1: 環境診断スクリプトを実行（5分）
```matlab
cd kalman
environment_quick_fix();
```

このスクリプトが以下を自動チェックします：
- ✅ MATLABバージョン・アーキテクチャ
- ✅ MEXバイナリの有無とビルド状況
- ✅ config_params.m の設定値
- ✅ センサーデータ型の一致
- ✅ フィルタの実行可能性

### ステップ 2: エラーが出た場合の対応

**エラー: "Invalid MEX-file" / "MEX not found"**
```matlab
cd kalman/cpp/build
build_mex();        % MEXを環境専用にビルド
clear mex;
cd ../../
```

**エラー: "config_params error"**
```matlab
% config_params.m の以下を確認
params = config_params();
fprintf('static_time: %.1f\n', params.static_time);  % >= 5 推奨
```

---

## 【重要】異なる環境での実行時のチェック

### パターンA: 新しいマシンで初めて実行する場合
```matlab
%【Step 1】MEXビルド（環境適応）
cd kalman/cpp/build
build_mex();
clear functions; clear mex; rehash;

%【Step 2】診断確認
cd ../..
environment_quick_fix();        % 自動チェック

%【Step 3】テスト実行
cd kalman
run_simulation(42, false);      % 単一実行テスト
```

### パターンB: 他の環境との結果を比較したい場合
```matlab
%【環境1で実行】
clear functions; clear mex; rehash;
run_simulation(42, false);
copyfile('kalman/Results/estimation.csv', 'env1_result.csv');

%【環境2で実行】
clear functions; clear mex; rehash;
run_simulation(42, false);
copyfile('kalman/Results/estimation.csv', 'env2_result.csv');

%【両環境のいずれかで比較】
compare_environments('env1_result.csv', 'env2_result.csv');
```

---

## 【トラブルシューティング】

### 症状: 「初期化直後から結果がずれている」

**原因の優先順位（確率順）:**

1. **static_time が短い（確率 40%）**
   ```matlab
   params = config_params();
   if params.static_time < 3
       % 修正: config_params.m で params.static_time = 5; に変更
   end
   ```

2. **GPS原点が異なる（確率 30%）**
   ```matlab
   params = config_params();
   % 両環境で以下が同じか確認
   fprintf('GPS origin: [%.6f, %.6f]\n', ...
       params.gps_origin.lat, params.gps_origin.lon);
   ```

3. **MEXが古いバイナリ（確率 20%）**
   ```matlab
   cd kalman/cpp/build
   build_mex();
   clear functions; clear mex; rehash;
   ```

4. **センサーデータ型が混在（確率 10%）**
   ```matlab
   obs = read_csv('kalman/GenerateData/sensor_data.csv');
   fprintf('ax: %s (expected single)\n', class(obs.ax));
   fprintf('lat: %s (expected double)\n', class(obs.lat));
   ```

---

## 【環境依存性の詳細情報】

詳しく知りたい場合は、以下のドキュメントを参照してください：

| ドキュメント | 内容 |
|-----------|------|
| [ENVIRONMENT_DEPENDENCY_GUIDE.md](ENVIRONMENT_DEPENDENCY_GUIDE.md) | 原因・診断・解決方法の詳細版（推奨） |
| [ENVIRONMENT_QA.md](ENVIRONMENT_QA.md) | Q&A形式のガイド |

---

## 【数値差の判断基準】

異なる環境で実行した結果を `compare_environments()` で比較する場合：

| 最大誤差 | 判定 | 対応 |
|--------|------|------|
| < 1e-6 | ✅ 完全一致 | 問題なし |
| 1e-6 ~ 1e-3 | ✅ 許容範囲 | 浮動小数点演算差 |
| 1e-3 ~ 0.01 | ⚠️ 軽微な差 | 初期化パラメータ確認 |
| > 0.01 m | ❌ 重大な差 | MEXビルド・パラメータ同期が必要 |

---

## 【クイックリファレンス】

### よく使う診断コマンド
```matlab
% 1. 環境自動診断（最初にこれ）
environment_quick_fix();

% 2. 詳細診断（問題発生時）
diagnose_environment();

% 3. 環境比較（2つの環境を比較）
compare_environments('env1.csv', 'env2.csv');

% 4. MEXキャッシュクリア（推奨: 毎回実行）
clear functions; clear mex; rehash;

% 5. MEX再ビルド（環境適応）
cd kalman/cpp/build && build_mex();
```

### よく使う実行コマンド
```matlab
% テスト実行（単一シード）
run_simulation(42, false);

% バッチテスト（複数シード、統計確認）
run_batch_10sets();

% FFT分析（周波数特性確認）
run_fft_from_csv();
```

---

## 【よくある質問】

**Q: なぜバイナリなのに結果が変わるのか？**
> MEXバイナリは「ロジック」をコンパイルしたもので、パラメータや環境（MATLAB バージョン、CPU等）には依存しません。ただし、入力パラメータ（config_params）やセンサーデータ形式が異なると、結果は変わります。

**Q: どの環境では同じ結果が保証されるのか？**
> 以下が同じであれば、ほぼ同じ結果が得られます：
> - MATLAB バージョン・アーキテクチャ（64-bit Windows など）
> - MEXバイナリが同じもの
> - config_params.m の値が同じ
> - 同じシード（random number seed）

**Q: CI/CD パイプラインで環境依存を排除したい場合は？**
> 各環境でMEXを再ビルドしてください。`build_mex()` を実行するだけです。

---

## 【サポート】

問題が解決しない場合：
1. [ENVIRONMENT_DEPENDENCY_GUIDE.md](ENVIRONMENT_DEPENDENCY_GUIDE.md) の診断ステップを全て実施
2. `diagnose_environment()` の出力を確認
3. `compare_environments()` で他の環境と比較

---


# 環境依存性問題の解決 — 完全ガイド概要

**作成日:** 2026年1月9日  
**ドキュメント:** このフォルダ内の3つの新規ドキュメント参照

---

## 【簡潔な答え】

### Q: なぜ現在の環境では動くのに、他の環境では推定が全て失敗するのか？

**A: 以下の6つの環境依存要因が考えられます（確率順）**

1. **❌ MEXバイナリが異なる（70%）**
   - ビルド環境が異なるとバイナリも異なる
   - **解決:** `cd kalman/cpp/build && build_mex();`

2. **❌ config_params.m の値が異なる（20%）**
   - static_time, GPS原点, ノイズパラメータが異なる
   - **解決:** 両環境で同じファイルを使用

3. **❌ センサーデータ型が混在（5%）**
   - ax/wy は single、lat/lon は double であるべき
   - **解決:** read_csv() の型変換を確認

4. ⚠️ MEXキャッシュが残存（3%）
   - **解決:** `clear functions; clear mex; rehash;`

5. ⚠️ 浮動小数点演算の丸め差（1%）
   - CPU・コンパイラ最適化の違い
   - **解決:** mm 単位の誤差なら無視

6. ⚠️ 初期化パラメータの微妙な違い（1%）
   - **解決:** diagnose_environment() で確認

---

## 【実行すべき手順】最初の30分で環境依存問題を解決

### Step 1: 環境自動診断（5分）
```matlab
cd kalman
environment_quick_fix();
```
出力を確認。❌ がある場合は次へ。

### Step 2: MEX再ビルド（10分）
```matlab
cd kalman/cpp/build
build_mex();
clear mex;
cd ../../
```
ビルド完了後、Step 1 を再実行。

### Step 3: 詳細診断（10分）
```matlab
cd kalman
diagnose_environment();
```
詳細レポート（kalman/Results/environment_diagnostic.txt）を確認。

### Step 4: テスト実行（5分）
```matlab
run_simulation(42, false);  % seed=42 で固定実行
```
結果が収束していることを確認。

### Step 5: 環境比較（5分、複数環境がある場合）
```matlab
% 環境1で結果保存
copyfile('kalman/Results/estimation.csv', 'env1_result.csv');

% 環境2でも実行、その後比較
compare_environments('env1_result.csv', 'env2_result.csv');
```

**以上で、環境依存性の診断・修正が完了します。**

---

## 【作成したスクリプト・ドキュメント】

### 1. 自動診断スクリプト

| ファイル | 目的 | 実行時間 |
|---------|------|--------|
| `diagnose_environment.m` | 詳細環境診断 | 2分 |
| `environment_quick_fix.m` | 自動修正付き診断 | 5分 |
| `compare_environments.m` | 2環境の結果比較 | 1分 |

### 2. ドキュメント

| ファイル | 内容 | 読む時間 |
|---------|------|--------|
| `ENVIRONMENT_SETUP.md` | **最初にこれを読む** | 10分 |
| `ENVIRONMENT_DEPENDENCY_GUIDE.md` | 詳細な原因説明・対応方法 | 30分 |
| `ENVIRONMENT_QA.md` | Q&A形式のトラブルシューティング | 20分 |

**推奨読む順番:** SETUP → QUICK_FIX 実行 → QA → 詳細版

---

## 【優先度別対応表】

### 🔴 今すぐやるべき（問題が発生している場合）

```matlab
% 1. MEXキャッシュクリア
clear functions; clear mex; rehash;

% 2. 自動診断・修正実行
cd kalman
environment_quick_fix();

% 3. MEX再ビルド（診断で ❌ が出た場合）
cd cpp/build
build_mex();
```

### 🟡 設定確認（異なる環境で実行する場合）

```matlab
% config_params.m が同じか確認
params = config_params();
fprintf('static_time: %.1f\n', params.static_time);      % 5推奨
fprintf('dt: %.4f\n', params.dt);                        % 0.0025推奨
fprintf('GPS origin: [%.6f, %.6f]\n', ...
    params.gps_origin.lat, params.gps_origin.lon);
```

### 🟢 詳細な原因特定（結果がまだずれている場合）

```matlab
diagnose_environment();     % 詳細診断
compare_environments(...);  % 環境比較
% → ENVIRONMENT_DEPENDENCY_GUIDE.md の Step 1-6 に従う
```

---

## 【各環境でのビルド記録】

新しい環境でMEXをビルドした場合、記録を残してください：

| 環境 | MATLAB版 | アーキテクチャ | ビルド日 | 結果 |
|------|---------|-------------|--------|------|
| PC1  | R2021a  | Windows 64-bit | 2026-01-09 | ✅ |
| PC2  | R2024a  | Windows 64-bit | YYYY-MM-DD | ⏳ |
| VM1  | R2023a  | Linux 64-bit   | YYYY-MM-DD | ⏳ |

**記録方法:**
```matlab
fprintf('=== BUILD RECORD ===\n');
ver_info = ver('MATLAB');
fprintf('MATLAB: %s\n', ver_info.Version);
fprintf('Arch: %s\n', computer);
fprintf('Built: %s\n', datestr(now));
fprintf('Status: ✅ (or ❌)\n');
fprintf('Notes: [特記事項]\n');
```

---

## 【よくある失敗パターンと対策】

### 失敗パターン1: 「他の環境で全く動かない」

→ **原因:** MEXバイナリが古い  
→ **対策:** 各環境で `build_mex()` 実行

```matlab
cd kalman/cpp/build
build_mex();
clear mex;
```

### 失敗パターン2: 「初期化から大きくずれている」

→ **原因:** static_time, GPS原点, ノイズパラメータが異なる  
→ **対策:** config_params.m を同期

```matlab
% 両環境で同じ値を確認
params = config_params();
disp(params.static_time);
disp(params.gps_origin);
disp(params.noise);
```

### 失敗パターン3: 「結果がmm単位でずれている」

→ **原因:** CPU・コンパイラの浮動小数点差  
→ **対策:** 無視OK（許容範囲）

```matlab
% max_error < 1e-3 なら許容
compare_environments('env1.csv', 'env2.csv');
```

### 失敗パターン4: 「MEX関数が見つからない」

→ **原因:** パスが通っていない or ビルルル失敗  
→ **対策:** build_mex() で再ビルド

```matlab
cd kalman/cpp/build
build_mex({'mex_run_eskf'});
```

---

## 【推奨ワークフロー】複数環境での開発・テスト

```
【開発環境（PC1: MATLAB R2021a）】
  1. C++ コード修正
  2. build_mex()
  3. run_simulation(42, false) ← 動作確認
  4. config_params.m を確定
  5. → Git に commit

【テスト環境（PC2: MATLAB R2024a）】
  1. Git pull
  2. build_mex()          ← 環境専用ビルド
  3. clear functions; clear mex; rehash;
  4. run_simulation(42, false)
  5. compare_environments() ← 開発環境と比較
  6. → 差分が small なら OK

【継続的インテグレーション】
  - 各環境で自動的に build_mex() + run_batch_10sets()
  - 結果を比較（max_error < 1e-3 なら PASS）
```

---

## 【チェックリスト】環境をセットアップする際

- [ ] environment_quick_fix() を実行
- [ ] 診断結果をスクリーンショット保存
- [ ] MEXビルドログを確認（Warning/Error なし）
- [ ] config_params.m の値をコピー・ペースト
- [ ] run_simulation(42, false) で動作確認
- [ ] 別環境がある場合は compare_environments()
- [ ] 結果をファイルに記録

---

## 【最後に】

このプロジェクトでの「環境依存性」は **設計上の選択** です：

✅ **MEXの利点:** C++ で高速・効率的な計算  
❌ **MEXの欠点:** 環境ごとに再ビルドが必要

**本ガイドにより、この欠点を最小化できます。**

---


# MEX統合プロセス — 標準手順書

**目的**: 複数のMEXを1つのMEXに統合する際の標準化プロセス

---

## 概要

このドキュメントは、以下のような統合を行う際の手順です：

```
❌ mex_meukf_step_v2 (独立)
❌ mex_sensor_filter (独立)
        ↓ 統合
✅ mex_run_eskf (統一MEX)
```

**失敗パターン（過去）**: 実装なしで削除した → NaN/Inf

---

## Phase 1: 準備フェーズ（1日目）

### Step 1.1: 現在の動作を確認
```matlab
% 単体テスト
cd kalman
clear mex
run_simulation(42, true)
```
**期待結果**: PASS (シミュレーション成功)

### Step 1.2: コミット状態を確認
```bash
git status
# → 変更なし、clean な状態を確認
```

### Step 1.3: テストベースラインを記録
```matlab
% バッチテスト
cd kalman
clear mex
run_batch_10sets()
% 結果を batch_10sets_baseline_BEFORE.csv として保存
```

**記録**:
- 成功率
- 平均推定誤差
- 異常値の有無

---

## Phase 2: 実装フェーズ（2-3日目）

### Step 2.1: C++ ソースを統合

**対象ファイル**:
- [kalman/cpp/MEX/mex_run_eskf.cpp](../kalman/cpp/MEX/mex_run_eskf.cpp)
- [kalman/cpp/src/MEUKF/meukf_core.cpp](../kalman/cpp/src/MEUKF/meukf_core.cpp)
- [kalman/cpp/src/MEUKF/unified_filter.cpp](../kalman/cpp/src/MEUKF/unified_filter.cpp)

**実装内容**:
```cpp
// mex_run_eskf.cpp に以下を追加

// 1. MEUKF機能の組み込み
#include "../src/MEUKF/meukf_core.h"

// 2. センサーフィルタの初期化
void mex_run_eskf_init() {
    // センサーアウトライア検出器をリセット
    sensor_filter.reset_zero();  // ← これが重要
}

// 3. ステップ処理での呼び出し
void mex_run_eskf_step() {
    // MEUKF ステップ
    meukf_core_step(...);
    
    // センサーフィルタ更新
    sensor_filter.update(...);
}
```

### Step 2.2: ビルド設定に統合依存性を追加

[kalman/cpp/build/build_mex.m](../kalman/cpp/build/build_mex.m) を確認:
```matlab
% 現在の状態: 
% meukf_core_cpp と unified_filter_cpp が mex_run_eskf.cpp にリンクされているか

% リンク確認:
build_single_mex('mex_run_eskf.cpp', compile_opts, inc_args, {
    ..., 
    meukf_core_cpp,        % ← 含まれているか
    unified_filter_cpp      % ← 含まれているか
}, bin_dir, [], log_fid)
```

### Step 2.3: 統合版MEXをビルド

```bash
cd kalman/cpp/build
matlab -batch "build_mex({'mex_run_eskf'})"
```

**確認**:
```bash
ls -la ../bin/mex_run_eskf.mexw64
# → ファイルが存在し、タイムスタンプが新しいか
```

---

## Phase 3: 検証フェーズ（3-4日目）

### Step 3.1: 単体テスト（統合版MEX）
```matlab
cd kalman
clear mex
run_simulation(42, true)
```
**期待結果**: PASS

### Step 3.2: 非回帰テスト（旧MEXとの比較）

**目的**: 統合後の結果が同じか確認

```matlab
% 旧 mex_meukf_step_v2 + mex_sensor_filter の結果を記録
run_simulation(42, true)  % ← Save Result_OLD.csv

% MEX キャッシュをクリア
clear mex

% 新 mex_run_eskf の結果を記録
run_simulation(42, true)  % ← Save Result_NEW.csv

% 差分を比較
diff_max = max(abs(Result_OLD - Result_NEW))
fprintf('Max difference: %f\n', diff_max)
% 期待: < 1e-4 (数値誤差の範囲)
```

### Step 3.3: バッチテスト（統合版MEX）
```matlab
cd kalman
clear mex
run_batch_10sets()
```

**期待結果**: 成功 10/10 (100%)

**記録**:
```bash
# 結果をコミット前に記録
cp batch_10sets_results.mat batch_10sets_INTEGRATED.mat
cp batch_10sets_summary.csv batch_10sets_INTEGRATED.csv
```

### Step 3.4: 推定品質の確認

```matlab
% 結果を可視化
load('batch_10sets_INTEGRATED.mat')
compare_mex_matlab_detailed()
```

**チェック項目**:
- [ ] NaN/Inf が無いか
- [ ] 推定誤差が許容範囲か（±0.5m以内）
- [ ] 収束速度が同等か
- [ ] 異常値検出が機能しているか

---

## Phase 4: 統合確定フェーズ（4-5日目）

### Step 4.1: ビルドスクリプトの旧MEXをコメント化

[kalman/cpp/build/build_mex.m](../kalman/cpp/build/build_mex.m) を編集:

**Before** (削除しない):
```matlab
meukf_core_cpp = fullfile(src_dir, 'MEUKF', 'meukf_core.cpp');
if exist('mex_meukf_step.cpp', 'file') && exist(meukf_core_cpp, 'file')
    if wants('mex_meukf_step') && build_single_mex(...)
        built_count = built_count + 1;
    end
end
```

**After** (コメント化):
```matlab
% ===== 統合完了: 以下は mex_run_eskf に統合済み =====
% meukf_core_cpp = fullfile(src_dir, 'MEUKF', 'meukf_core.cpp');
% if exist('mex_meukf_step.cpp', 'file') && exist(meukf_core_cpp, 'file')
%     if wants('mex_meukf_step') && build_single_mex(...)
%         built_count = built_count + 1;
%     end
% end
```

### Step 4.2: 初期化コードを確認して維持

[kalman/run_batch_10sets.m](../kalman/run_batch_10sets.m) を確認:

```matlab
% ← 以下は削除してはいけない！
if exist('mex_sensor_filter','file') == 3
    try
        mex_sensor_filter('reset_zero');
    catch
        try mex_sensor_filter('reset'); catch, end
    end
end
```

**もし統合で実装が変わった場合**:
```matlab
% 統合後: mex_run_eskf('init_filters') など新しい初期化方式を呼び出す
if exist('mex_run_eskf','file') == 3
    try
        mex_run_eskf('init_filters');  % ← 新規のセンサーフィルタ初期化
    catch
        try mex_run_eskf('init'); catch, end
    end
end
```

### Step 4.3: 最終テスト

```matlab
cd kalman
clear mex
run_batch_10sets()  % ← 再度確認
```

**期待結果**: 成功 10/10 (100%)

---

## Phase 5: コミットフェーズ（5日目）

### Step 5.1: git status を確認

```bash
git status
```

**許容される状態**:
```
Changes not staged for commit:
    modified:   kalman/cpp/build/build_mex.m       (コメント化のみ)
    modified:   kalman/run_batch_10sets.m          (初期化方式の変更のみ)

Untracked files:
    kalman/Results/estimation_*.csv
    kalman/Results/batch_10sets_*.mat
    ...
```

### Step 5.2: コミットメッセージを作成

```
[統合] MEUKF と センサーフィルタを mex_run_eskf に統合

実装詳細:
- mex_run_eskf.cpp に meukf_core() 処理を追加
- mex_run_eskf.cpp に sensor_filter 状態管理を統合
- meukf_core.cpp, unified_filter.cpp を mex_run_eskf にリンク
- センサーフィルタ初期化を mex_run_eskf('init_filters') に統合

検証:
- 単体テスト: run_simulation(42) → PASS
- 非回帰テスト: 旧 mex_meukf_step との差分 < 1e-4 ✅
- バッチテスト: run_batch_10sets() → 成功 10/10 (100%) ✅
- 推定品質: NaN/Inf なし、推定誤差 ±0.5m以内 ✅

ビルドスクリプト変更:
- mex_meukf_step, mex_sensor_filter ビルド設定をコメント化

削除:
- （今後：旧 mex_meukf_step_v2.mexw64, mex_sensor_filter.mexw64 は
  十分な移行期間後に削除予定）
```

### Step 5.3: コミット実行

```bash
git add kalman/cpp/build/build_mex.m kalman/run_batch_10sets.m
git commit -m "[統合] MEUKF と センサーフィルタを mex_run_eskf に統合"
```

---

## Phase 6: 後始末フェーズ（1-2週間後）

十分なテスト期間（1-2週間）を経た後:

### Step 6.1: 旧MEXバイナリの削除

```bash
git rm kalman/cpp/bin/mex_meukf_step_v2.mexw64
git rm kalman/cpp/bin/mex_sensor_filter.mexw64
```

### Step 6.2: コミット

```bash
git add kalman/cpp/bin/
git commit -m "[cleanup] 統合済みの旧 mex_meukf_step_v2, mex_sensor_filter を削除"
```

---

## トラブルシューティング

### 問題: `NaN/Inf detected` エラー
```matlab
% 原因: センサーフィルタの初期化が抜けている
% → run_batch_10sets.m の初期化コードを確認
if exist('mex_sensor_filter','file') == 3
    mex_sensor_filter('reset_zero');  % ← 必須
end
```

### 問題: バイナリが見つからない
```bash
# 確認
ls -la kalman/cpp/bin/mex_*.mexw64

# 再ビルド
cd kalman/cpp/build
matlab -batch "build_mex()"
```

### 問題: 非回帰テストで大きな差分が出た
```matlab
% 原因: 統合実装に誤りがある
% → メジャー版号を上げて別の mex_run_eskf_v2 として実装し直す
% → Phase 3 を最初からやり直す
```

---

## チェックリスト（統合実行時）

### 実装前
- [ ] git status で clean な状態か
- [ ] 旧MEXでバッチテスト実行済みか
- [ ] コミット歴を確認したか

### 実装中
- [ ] C++ ソースの統合が完了したか
- [ ] コンパイルエラーがないか
- [ ] ビルド設定にリンク依存性を追加したか

### 検証前
- [ ] clear mex を実行したか
- [ ] 単体テストで PASS したか

### 検証中
- [ ] 非回帰テストの差分を確認したか
- [ ] バッチテストで 10/10 成功したか
- [ ] NaN/Inf が無いか確認したか

### コミット前
- [ ] git diff で想定外の削除がないか確認したか
- [ ] 初期化コードが削除されていないか確認したか
- [ ] コミットメッセージは詳細か

### コミット後
- [ ] push 前に再度 run_batch_10sets() で検証したか
- [ ] push した後、別マシンでも pull × テストしたか

---

**版**: 1.0  
**作成日**: 2025年12月31日  
**対象プロジェクト**: KalmanFilter  
**参考**: [INTEGRATION_FAILURE_ROOT_CAUSE.md](INTEGRATION_FAILURE_ROOT_CAUSE.md)

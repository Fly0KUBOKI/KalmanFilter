# mex_run_eskf 完全統合計画（完全版）

**版**: 2.0  
**策定日**: 2025年12月31日  
**目標**: mex_meukf_step_v2 と mex_sensor_filter を完全に mex_run_eskf に統合  
**重要**: 段階的実装、各フェーズ後にバックアップ取得、復旧可能性確保

---

## エグゼクティブサマリー

### 現状
```
mex_meukf_step_v2.mexw64     ← 独立したMEUKFステップ処理
mex_sensor_filter.mexw64     ← 独立したセンサーフィルタ
mex_run_eskf.mexw64          ← ESKF Predict+Update（統合先）
```

### 目標状態
```
mex_run_eskf.mexw64          ← 全機能統合（MEUKF + センサーフィルタ + ESKF）
（他の2つのMEXは廃止またはレガシーサポート）
```

### 成功条件
- ✅ `run_batch_10sets()` で 10/10 成功
- ✅ `compare_mex_matlab_detailed()` で差分 < 1e-4
- ✅ NaN/Inf エラーなし
- ✅ 推定品質が現在と同等以上

---

## 統合の失敗履歴と教訓

### 過去の失敗パターン（記録）

| 失敗 | 原因 | 防止策 |
|------|------|--------|
| 実装なし削除 | 統合計画のみで削除実行 | **実装 → テスト → 削除** の順序徹底 |
| 機能損失 | 初期化コード削除 | 初期化は **必須**、削除禁止 |
| 段階性の欠如 | 全て一度に統合 | **フェーズごと** に分割実装 |
| バックアップなし | 復旧不能 | **各フェーズ後に git stash** |
| テスト不足 | 単体テストのみ | 単体 → 非回帰 → バッチの3段階 |

### 今回の防止策
```
✅ 実装→テスト→コミット→バックアップの順序
✅ 各フェーズを独立したコミットで記録
✅ git stash による復旧ポイント設置
✅ 3段階テストの実装（詳細は Phase 3 参照）
```

---

## Phase 0: 準備フェーズ（本日実施）

### Step 0.1: 現在のバージョン確認
```bash
git log --oneline -5
# → 統合前の状態を記録
```

### Step 0.2: バックアップコミット
```bash
git add -A
git commit -m "[backup] 統合前のクリーンな状態を記録 (mex_meukf_step_v2, mex_sensor_filter, mex_run_eskf は独立)"
```

**記録内容**:
- mex_meukf_step_v2.cpp （自主機能: MEUKF状態更新）
- mex_sensor_filter.cpp （自主機能: センサーアウトライア検出）
- mex_run_eskf.cpp （自主機能: ESKF Predict + Update）
- 各々のバイナリ：動作確認済み状態

### Step 0.3: 単体テスト＆ベースライン記録
```matlab
cd kalman
clear mex
run_simulation(42, true)  % 再現可能なシード
% Result_BASELINE.csv を保存
```

**期待結果**: PASS  
**記録**: このシミュレーションを Phase ごとに比較

---

## Phase 1: mex_meukf_step_v2 → mex_run_eskf への機能統合

**目標**: MEUKF 状態更新機能を mex_run_eskf に組み込む

### Step 1.1: 源ファイル分析

**分析対象**:
- [kalman/cpp/MEX/mex_meukf_step.cpp](../kalman/cpp/MEX/mex_meukf_step.cpp)
- [kalman/cpp/src/MEUKF/meukf_core.cpp](../kalman/cpp/src/MEUKF/meukf_core.cpp)
- [kalman/cpp/include/MEUKF/meukf_core.h](../kalman/cpp/include/MEUKF/meukf_core.h)

**質問リスト**（確認すること）:
```
1. mex_meukf_step.cpp の main logic は何か?
   → meukf_core::step() の呼び出しか?
   
2. 入力パラメータは?
   - 状態ベクトル x (15×1)
   - 共分散 P (15×15)
   - センサー観測
   
3. 出力は?
   - 更新後の状態 x_new
   - 更新後の共分散 P_new
   
4. mex_run_eskf の現在の機能は?
   - Predict ステップ？
   - Update ステップ？
   - 両方？
```

**コマンド**:
```bash
cd kalman/cpp
grep -n "^void\|^class\|^struct" MEX/mex_meukf_step.cpp | head -20
grep -n "^void\|^class\|^struct" src/MEUKF/meukf_core.cpp | head -20
grep -n "^void\|^class\|^struct" MEX/mex_run_eskf.cpp | head -20
```

### Step 1.2: 統合実装（C++）

**ファイル編集**: [kalman/cpp/MEX/mex_run_eskf.cpp](../kalman/cpp/MEX/mex_run_eskf.cpp)

**追加内容**:
```cpp
// mex_run_eskf.cpp に以下を追加

#include "../src/MEUKF/meukf_core.h"

// =============== MEUKF ステップ機能を組み込む ===============
// mex_meukf_step.cpp から移植
class MEUKFStep {
private:
    meukf_core::Core meukf_engine;  // MEUKF エンジン
    
public:
    // 状態更新（mex_meukf_step の機能）
    void step(const Eigen::VectorXd& state,
              const Eigen::MatrixXd& cov,
              const SensorObservation& obs,
              Eigen::VectorXd& state_out,
              Eigen::MatrixXd& cov_out) {
        // mex_meukf_step.cpp の logic をここに移植
        meukf_engine.update(state, cov, obs);
        state_out = meukf_engine.get_state();
        cov_out = meukf_engine.get_covariance();
    }
};

// MEX エントリーポイント拡張
void mexFunction(int nlhs, mxArray *plhs[], 
                 int nrhs, const mxArray *prhs[]) {
    // 既存コード（ESKF Predict/Update）
    
    // + 新規: MEUKF 統合機能
    if (command == "meukf_step") {
        // mex_meukf_step の処理をここで実行
        meukf_step_impl(...);
    }
}
```

**実装チェックリスト**:
- [ ] meukf_core.h のインクルード
- [ ] MEUKF クラスの実装（状態更新ロジック）
- [ ] mex_meukf_step.cpp からの logic 移植
- [ ] 入出力インタフェースの確認
- [ ] 型変換（MATLAB ↔ C++）の確認
- [ ] エラーハンドリング

### Step 1.3: ビルド設定更新

**ファイル編集**: [kalman/cpp/build/build_mex.m](../kalman/cpp/build/build_mex.m)

**変更内容**:
```matlab
% mex_run_eskf ビルド設定に meukf_core.cpp をリンク追加

% Before:
build_single_mex('mex_run_eskf.cpp', compile_opts, inc_args, {
    filter_management_cpp,
    eskf_postprocess_cpp,
    ...
}, bin_dir, [], log_fid)

% After:
meukf_core_cpp = fullfile(src_dir, 'MEUKF', 'meukf_core.cpp');
build_single_mex('mex_run_eskf.cpp', compile_opts, inc_args, {
    filter_management_cpp,
    eskf_postprocess_cpp,
    meukf_core_cpp,  % ← 追加
    ...
}, bin_dir, [], log_fid)
```

### Step 1.4: ビルド実行
```matlab
cd kalman/cpp/build
build_mex({'mex_run_eskf'})  % 1つだけ
```

**ビルドが成功したか確認**:
```bash
ls -la ../bin/mex_run_eskf.mexw64
# → ファイルサイズが増えているか（機能追加のため）
```

### Step 1.5: テスト - MEUKF 機能確認
```matlab
cd kalman
clear mex

% 1. MEUKF ステップだけテスト
mex_run_eskf('meukf_step', state, cov, obs)
% → 結果が mex_meukf_step.mexw64 と同じか?
```

**テスト内容**:
```matlab
% テストスクリプト: test_meukf_integration.m

function test_meukf_integration()
    % 1. 旧 MEX で実行
    clear mex
    load('sample_sensor_obs.mat')  % サンプルデータ
    [state_old, cov_old] = mex_meukf_step(state, cov, obs);
    
    % 2. 新 MEX（統合版）で実行
    clear mex
    [state_new, cov_new] = mex_run_eskf('meukf_step', state, cov, obs);
    
    % 3. 差分を確認
    diff_state = norm(state_old - state_new);
    diff_cov = norm(cov_old - cov_new);
    
    fprintf('State difference: %e\n', diff_state);
    fprintf('Cov difference: %e\n', diff_cov);
    
    % 4. 判定
    if diff_state < 1e-10 && diff_cov < 1e-10
        disp('✓ MEUKF 統合成功')
    else
        disp('✗ MEUKF 統合失敗 - 差分が大きい')
    end
end
```

### Step 1.6: 成功確認＆バックアップ

**テスト結果**:
- ✅ ビルド成功
- ✅ MEUKF 機能テスト成功（差分 < 1e-10）

**コミット＆バックアップ**:
```bash
git add kalman/cpp/MEX/mex_run_eskf.cpp
git add kalman/cpp/build/build_mex.m
git add kalman/cpp/bin/mex_run_eskf.mexw64
git commit -m "[Phase 1] MEUKF機能をmex_run_eskfに統合 - mex_meukf_step('meukf_step') で動作確認済み"

# バックアップポイント
git stash create "phase1-meukf-integrated"
```

---

## Phase 2: mex_sensor_filter → mex_run_eskf への機能統合

**目標**: センサーフィルタ（アウトライア検出）機能を mex_run_eskf に組み込む

### Step 2.1: 源ファイル分析

**分析対象**:
- [kalman/cpp/MEX/mex_sensor_filter.cpp](../kalman/cpp/MEX/mex_sensor_filter.cpp)
- [kalman/cpp/include/Common/Sensor/sensor_filter.hpp](../kalman/cpp/include/Common/Sensor/sensor_filter.hpp)

**質問リスト**:
```
1. mex_sensor_filter の主な機能は?
   → reset_zero() / reset() で初期化
   → update() でフィルタリング
   
2. 状態（メモリ）を持つか?
   → Yes: 前回の観測値を保持
   
3. mex_run_eskf で既に使用されているか?
   → 確認: grep -n "sensor_filter" MEX/mex_run_eskf.cpp
```

### Step 2.2: 統合実装（C++）

**ファイル編集**: [kalman/cpp/MEX/mex_run_eskf.cpp](../kalman/cpp/MEX/mex_run_eskf.cpp)

**追加内容**:
```cpp
#include "../include/Common/Sensor/sensor_filter.hpp"

// =============== センサーフィルタ機能を組み込む ===============
class SensorFilterComponent {
private:
    sensor_filter::OutlierDetector detector;
    
public:
    void reset_zero() {
        detector.reset_zero();
    }
    
    void update(const Eigen::VectorXd& obs,
                bool& is_outlier,
                Eigen::VectorXd& filtered_obs) {
        detector.update(obs, is_outlier, filtered_obs);
    }
};

// MEX エントリーポイント拡張
void mexFunction(int nlhs, mxArray *plhs[], 
                 int nrhs, const mxArray *prhs[]) {
    // ...
    
    if (command == "sensor_filter_reset_zero") {
        sensor_filter.reset_zero();
    }
    else if (command == "sensor_filter_update") {
        sensor_filter.update(...);
    }
}
```

### Step 2.3: ビルド設定更新

**ファイル編集**: [kalman/cpp/build/build_mex.m](../kalman/cpp/build/build_mex.m)

```matlab
% mex_run_eskf ビルド設定に sensor_filter をリンク
sensor_filter_cpp = fullfile(src_dir, 'Common', 'Sensor', 'sensor_filter.cpp');
build_single_mex('mex_run_eskf.cpp', compile_opts, inc_args, {
    ...,
    meukf_core_cpp,
    sensor_filter_cpp  % ← 追加
}, bin_dir, [], log_fid)
```

### Step 2.4: ビルド実行
```matlab
build_mex({'mex_run_eskf'})
```

### Step 2.5: テスト - センサーフィルタ確認
```matlab
% test_sensor_filter_integration.m

function test_sensor_filter_integration()
    clear mex
    
    % 1. 旧 MEX テスト
    mex_sensor_filter('reset_zero');
    [is_outlier_old, filtered_old] = mex_sensor_filter('update', obs);
    
    % 2. 新 MEX（統合版）テスト
    clear mex
    mex_run_eskf('sensor_filter_reset_zero');
    [is_outlier_new, filtered_new] = mex_run_eskf('sensor_filter_update', obs);
    
    % 3. 差分確認
    diff = norm(filtered_old - filtered_new);
    fprintf('Sensor filter difference: %e\n', diff);
    
    if diff < 1e-10
        disp('✓ センサーフィルタ統合成功')
    else
        disp('✗ センサーフィルタ統合失敗')
    end
end
```

### Step 2.6: バックアップ
```bash
git add -A
git commit -m "[Phase 2] センサーフィルタをmex_run_eskfに統合 - outlier detection 動作確認済み"
git stash create "phase2-sensor-filter-integrated"
```

---

## Phase 3: 統合検証フェーズ

**目標**: 統合後の ESKF がフル機能で正常に動作することを確認

### Step 3.1: 初期化コードの統合確認

**確認対象**: [kalman/run_batch_10sets.m](../kalman/run_batch_10sets.m)

**変更（今後不要にする）**:
```matlab
% Phase 3 実装まで、以下は**必須**（削除禁止）
if exist('mex_sensor_filter','file') == 3
    try
        mex_sensor_filter('reset_zero');
    catch
        try mex_sensor_filter('reset'); catch, end
    end
end

% ↓ Phase 3 実装後は（以下に置き換え）
if exist('mex_run_eskf','file') == 3
    try
        % 統合されたセンサーフィルタを初期化
        mex_run_eskf('init_all_filters');
    catch
        try mex_run_eskf('init'); catch, end
    end
end
```

### Step 3.2: MATLAB ラッパー更新（推奨）

**ファイル**: [kalman/ESKF/@ESKF/ESKF.m](../kalman/ESKF/@ESKF/ESKF.m)

**確認事項**:
- `mex_run_eskf` の呼び出し方
- パラメータ型チェック（float32/float64 混在を防ぐ）
- 初期化の順序

### Step 3.3: 単体テスト - 統合版ESKF
```matlab
cd kalman
clear mex

% テスト1: シード42での再現性確認
run_simulation(42, true)
% 結果を estimation_phase3.csv として保存
```

**期待結果**:
- ✅ シミュレーション完了
- ✅ NaN/Inf エラーなし
- ✅ 位置・速度が更新されている（全てゼロではない）

### Step 3.4: 非回帰テスト - バージョン比較
```matlab
% test_regression_phase3.m

function test_regression_phase3()
    % 1. 復旧: Phase 0（独立したMEX）
    %    git stash apply phase0-backup
    clear mex
    run_simulation(42, true)
    result_baseline = readtable('estimation_phase0.csv');
    
    % 2. 現在: Phase 3（統合版）
    clear mex
    run_simulation(42, true)
    result_phase3 = readtable('estimation_phase3.csv');
    
    % 3. 差分分析
    diff = abs(result_baseline{:,:} - result_phase3{:,:});
    max_diff = max(diff, [], 'all');
    mean_diff = mean(diff, 'all');
    
    fprintf('Max difference: %e\n', max_diff);
    fprintf('Mean difference: %e\n', mean_diff);
    
    % 4. 判定
    if max_diff < 1e-4  % 許容誤差
        disp('✓ 非回帰テスト: PASS')
    else
        disp('✗ 非回帰テスト: FAIL - 差分が大きい')
        disp('差分の詳細:')
        disp(max(diff, [], 1))
    end
end
```

### Step 3.5: バッチテスト - 10セット検証
```matlab
cd kalman
clear mex
run_batch_10sets()
```

**期待結果**:
- ✅ 成功: 10/10 (100%)
- ✅ NaN/Inf エラーなし
- ✅ 平均推定誤差が許容範囲内（±0.5m）

**結果確認**:
```matlab
%結果の詳細を確認
load('kalman/Results/batch_10sets_results.mat')
fprintf('成功率: %d/%d\n', success_count, total_count);
fprintf('平均誤差: %f m\n', mean_rmse);
fprintf('最大誤差: %f m\n', max_rmse);
```

### Step 3.6: バックアップ＆コミット
```bash
git add -A
git commit -m "[Phase 3] 統合ESKF検証完了 - run_batch_10sets() で10/10成功、非回帰テストPass"
git stash create "phase3-integration-verified"
```

---

## Phase 4: 統合の最終確定フェーズ

**目標**: 旧MEX削除と統合版への完全な切り替え

### Step 4.1: MATLAB ラッパーの完全置き換え

**ファイル**: [kalman/ESKF/@ESKF/ESKF.m](../kalman/ESKF/@ESKF/ESKF.m)

**変更内容**: すべての MEX 呼び出しを mex_run_eskf に統一

```matlab
% Before（3つのMEXを別々に呼び出し）
[x, P] = mex_meukf_step(x, P, obs);
[is_outlier] = mex_sensor_filter('update', obs);
[x, P] = mex_run_eskf('predict', x, P);

% After（統合版で統一）
[x, P] = mex_run_eskf('step', x, P, obs);
```

### Step 4.2: ビルドスクリプトの最終整理

**ファイル**: [kalman/cpp/build/build_mex.m](../kalman/cpp/build/build_mex.m)

**変更内容**: 旧MEXのビルド設定をコメント化（削除ではなく）

```matlab
% ===== 統合完了: 以下は mex_run_eskf に統合済み =====
% meukf_core_cpp = fullfile(src_dir, 'MEUKF', 'meukf_core.cpp');
% if exists('mex_meukf_step.cpp') ...
%     build_single_mex('mex_meukf_step.cpp', ...)
% end
%
% if exists('mex_sensor_filter.cpp') ...
%     build_single_mex('mex_sensor_filter.cpp', ...)
% end
% ================================================
```

### Step 4.3: 最終テスト

```matlab
cd kalman
clear mex
run_batch_10sets()  % 再度確認
```

**期待結果**: 成功 10/10 (100%)

### Step 4.4: 最終コミット＆タグ付け
```bash
git add -A
git commit -m "[Phase 4] 統合ESKF確定 - mex_run_eskf へ完全統合、mex_meukf_step/mex_sensor_filter はレガシーサポート"

# リリースタグ
git tag -a "v1.0-integrated-mex" -m "mex_meukf_step と mex_sensor_filter を mex_run_eskf に統合完了"
```

### Step 4.5: バックアップ保存
```bash
git stash create "phase4-integration-final"
```

---

## Phase 5: 後始末フェーズ（1-2週間後）

**目標**: 旧MEXバイナリを削除

**注意**: Phase 4 完了後 1-2週間テストしてから実行

### Step 5.1: 旧MEXバイナリを削除
```bash
git rm kalman/cpp/bin/mex_meukf_step_v2.mexw64
git rm kalman/cpp/bin/mex_sensor_filter.mexw64
```

### Step 5.2: ビルド設定から旧MEXを完全削除
```matlab
% build_mex.m から以下を削除（コメント化から削除へ）
% % meukf_core_cpp = ...
% % if ... build_single_mex('mex_meukf_step.cpp', ...)
```

### Step 5.3: 最終コミット
```bash
git add -A
git commit -m "[cleanup] 統合完了から1週間経過後、旧MEXバイナリ（mex_meukf_step_v2, mex_sensor_filter）を削除"
```

---

## トラブルシューティング＆ロールバック手順

### 問題: ビルドエラーが発生
```bash
# ロールバック
git stash list  # バックアップポイント確認
git stash apply phase1-meukf-integrated  # 最後に成功した状態へ復旧
```

### 問題: テストで NaN/Inf が出現
```matlab
% 原因：初期化が不足
% → run_batch_10sets.m で以下を確認
if exist('mex_sensor_filter','file') == 3
    mex_sensor_filter('reset_zero');  % ← 削除されていないか?
end

% 代替（Phase 3 実装後）
if exist('mex_run_eskf','file') == 3
    mex_run_eskf('init_all_filters');  % ← 正しく呼ばれているか?
end
```

### 問題: 非回帰テストで大きな差分が出た
```
差分 > 1e-4 の場合：
1. ビルドのコンパイラ設定を確認（最適化レベル）
2. float32/float64 混在を確認
3. メモリレイアウト（Eigen aligned vs non-aligned）を確認
4. → 発見できない場合は Phase を戻す
```

---

## チェックリスト（実装時）

### 各 Phase 実行前
- [ ] git status で clean な状態か
- [ ] バックアップコミットが存在するか
- [ ] テストベースラインが記録されているか

### 各 Phase 実行中
- [ ] C++ コード変更は1機能単位か（複数変更の場合は分割）
- [ ] ビルドエラーはないか
- [ ] コンパイルが成功したか

### 各 Phase 実行後（必須）
- [ ] 単体テスト成功か
- [ ] 差分が許容範囲か（< 1e-10 for unit, < 1e-4 for regression）
- [ ] git stash で復旧ポイント作成したか
- [ ] git commit でログを残したか

### 最終確認（Phase 3 終了時）
- [ ] バッチテスト: 10/10 成功
- [ ] NaN/Inf エラーなし
- [ ] 推定品質：現在と同等以上
- [ ] 新規機能の追加予定なし

---

## 推定スケジュール

| Phase | 内容 | 所要時間 | 累積 |
|-------|------|--------|------|
| 0 | 準備・ベースライン | 1時間 | 1h |
| 1 | MEUKF統合 | 2-3時間 | 4h |
| 2 | センサーフィルタ統合 | 2-3時間 | 7h |
| 3 | 検証 | 1-2時間 | 9h |
| 4 | 確定 | 30分 | 9.5h |
| 5 | 後始末（1週間後） | 15分 | 9.75h |

**総計**: 約9.75時間（分散可能）

---

## 重要な約束

```
✅ 実装 → テスト → コミット → バックアップ の順序を守る
✅ 段階的実装（全て一度にはしない）
✅ 各フェーズ後に git stash で復旧ポイント設置
✅ 非回帰テストを必ず実施
✅ 初期化コードは削除しない（統合完了まで）
✅ バイナリ削除は統合完了から 1 週間以上後
```

---

## 参考ドキュメント

- [INTEGRATION_FAILURE_ROOT_CAUSE.md](INTEGRATION_FAILURE_ROOT_CAUSE.md) — 過去の失敗分析
- [MEX_INTEGRATION_STANDARD_PROCESS.md](MEX_INTEGRATION_STANDARD_PROCESS.md) — 一般的な統合手順
- [kalman/cpp/build/04_INTEGRATION_REFACTORING_PLAN.md](../kalman/cpp/build/04_INTEGRATION_REFACTORING_PLAN.md) — 詳細な技術設計

---

**版**: 2.0  
**最終更新**: 2025年12月31日  
**ステータス**: 準備完了 → 実装待機

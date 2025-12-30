# 再統合時の注意事項チェックリスト

## 前提

コミット `6207225a0ead9496f713c16ad8aa832f48d52137` の失敗原因：
1. **座標系変換エラー** - 共分散行列 P の行列形式（row-major ↔ column-major）の誤り
2. **dt の未設定** - 時刻ステップが 0 に固定
3. **MEUKF 直接呼び出しの仕様不一致** - 出力形式の検証不足
4. **センサー前フレーム値の初期化問題** - 0 固定による変化検出エラー

これら4つの問題を再統合時に確実に解決するための方法を以下に示します。

---

## Phase 1: 事前準備（コード変更前）

### 1.1 基準となるコミット状態を確認

```bash
cd /path/to/KalmanFilter

# 正常なベースラインを確認
git log --oneline d3dab8c1887dd734b4641bc080436b266a47bf6c -1

# 期待: 
# d3dab8c MEUKF integration (working state)
```

### 1.2 テスト環境を準備

```matlab
% MATLAB で実施
cd kalman

% MEX をクリア
clear mex

% ベースラインテストを実行して期待値を記録
run_batch_10sets()

% 結果を保存（後で比較用）
baseline_results = readtable('Results/batch_10sets_summary.csv');
save baseline_results.mat baseline_results
```

**期待される PASS/FAIL 状況を記録しておく**

### 1.3 変更内容を書き出す

```bash
# 現在のコミット 6207225 から d3dab8c への差分を確認
git diff d3dab8c1887dd734b4641bc080436b266a47bf6c 6207225a0ead9496f713c16ad8aa832f48d52137 > /tmp/proposed_changes.patch

# ファイル一覧を確認
git diff --name-only d3dab8c1887dd734b4641bc080436b266a47bf6c 6207225a0ead9496f713c16ad8aa832f48d52137
```

---

## Phase 2: 変更の分解と個別検証

変更を **1 つの目的 = 1 コミット** の単位に分解します。

### 2.1 【パート A】センサー入力の型変換と座標系（最重大）

**目的：** double (MATLAB) → float (C++) の変換、座標系の正確性確保

**対象ファイル：**
- `kalman/cpp/Inc/MEX/mex_run_eskf_sensor_updates.hpp`（行 ~280-350）

**実装時の確認項目：**

```cpp
// ✓ 共分散行列の変換は対称性を保つ
// MATLAB (column-major): P[c*15 + r] で要素にアクセス
// C++ (row-major):      P[r*15 + c] で要素にアクセス

// ❌ 誤った変換（現在の失敗コード）:
for (int r = 0; r < 15; ++r) {
    for (int c = 0; c < 15; ++c) {
        P_cpp[r*15 + c] = static_cast<float>(P_matlab[c*15 + r]);
    }
}
// 理由: これは転置を伴わないシンプルなインデックス変換だが、
//      入力 P_matlab 自体が MATLAB の列優先形式のため、
//      結果は 90° 回転した行列になる可能性

// ✓ 正しい変換（旧コードの手法を確認）:
// mex_meukf_step_v2 を経由して、既に正しく変換されたものを使用
// または、明示的に転置を行う:
for (int r = 0; r < 15; ++r) {
    for (int c = 0; c < 15; ++c) {
        float val = static_cast<float>(P_matlab[c*15 + r]);
        // 対称性の確認
        if (r != c && abs(P_cpp[r*15+c] - P_cpp[c*15+r]) > 1e-5) {
            mexWarnMsgTxt("Covariance not symmetric!");
        }
        P_cpp[r*15 + c] = val;
    }
}
```

**検証方法：**

```matlab
% MATLAB で単一テスト
state_before = run_simulation(42, false);

% C++ が受け取った P が対称であることを確認
P = state_before.P;
is_symmetric = max(max(abs(P - P'))) < 1e-6;
assert(is_symmetric, "Covariance matrix not symmetric!");
```

**検証 OK なら次へ進む**

---

### 2.2 【パート B】dt パラメータの正確な渡し込み

**目的：** 時刻ステップが正確に MEX に渡されることを確認

**対象ファイル：**
- `kalman/cpp/Inc/MEX/mex_run_eskf_sensor_updates.hpp`（行 ~360-380）

**実装時の確認項目：**

```cpp
// ❌ 現在の失敗コード:
mxSetField(sensor_data, 0, "dt", mxCreateDoubleScalar(0));

// ✓ 修正コード:
// dt は関数の引数または呼び出し元から渡された値を使用すること
// 呼び出し元を確認:

// mex_run_eskf_impl.hpp の handle_sensor_update_internal 関数シグネチャを確認
void handle_sensor_update_internal(
    const char* sensor_type,
    const double* meas, int meas_len,
    double* p, double* v, double* q, double* ba, double* bg, double* P,
    double dt,  // ← これが来ているはず
    double* out_p, double* out_v, double* out_q, double* out_ba, double* out_bg,
    double* out_P,
    bool& should_skip
);

// dt を使用:
mxSetField(sensor_data, 0, "dt", mxCreateDoubleScalar(dt));
```

**検証方法：**

```matlab
% センサー更新の dt パラメータを検査
% mex_run_eskf('step', ...) で dt が正確に渡されていることを確認

% シミュレーション内で dt を記録
[state, dt_used] = run_simulation_with_dt_log(42);

% dt_used が期待値（通常 0.01 秒など）と一致することを確認
assert(all(dt_used > 0), "dt should be positive!");
assert(abs(mean(dt_used) - expected_dt) < 1e-6, "dt mismatch!");
```

**検証 OK なら次へ進む**

---

### 2.3 【パート C】MEUKF 出力形式の仕様確認

**目的：** 直接 C++ 呼び出しの入出力形式が MATLAB MEX ラッパーと一致することを確認

**対象ファイル：**
- `kalman/cpp/Inc/MEX/mex_run_eskf_sensor_updates.hpp`（行 ~380-420）
- `kalman/cpp/MEUKF/meukf_core.cpp`
- `kalman/cpp/MEUKF/meukf_types.hpp`

**実装時の確認項目：**

```cpp
// meukf_types.hpp で MEUKFInput / MEUKFOutput の定義を確認

struct MEUKFOutput {
    State new_state;           // 更新後の状態
    float last_y[3];           // 最後のイノベーション（観測残差）
    int last_y_len;            // last_y の有効要素数
    float last_H[3*15];        // 最後の観測行列 H (row-major 3x15)
    float last_K[15*3];        // 最後のカルマンゲイン K (row-major 15x3)
    float pred_P[15*15];       // 予測共分散 P (row-major 15x15)
};

// ✓ C++ 側の出力を MATLAB 互換形式で検証:
// - last_H が row-major であることを確認
// - pred_P が row-major であることを確認
// - last_K の形式を確認

// ❌ 出力を直接使用する場合、形式が MEX ラッパーの出力と一致するか検証必須
```

**検証方法：**

```matlab
% 旧 MEX ラッパーの出力形式と比較
% mex_meukf_step_v2 の出力形式を確認
[state_old, dbg_old] = mex_meukf_step_v2(state_struct, sensor_data, params);

% C++ 直接呼び出しの出力形式と比較
[state_new, dbg_new] = MEUKFCore::step(input);

% innov, H, K, P の値が一致することを検証
assert(max(abs(dbg_old.innov - dbg_new.last_y)) < 1e-4, "Innovation mismatch!");
assert(max(max(abs(dbg_old.H - dbg_new.last_H))) < 1e-4, "H matrix mismatch!");
```

**検証 OK なら次へ進む**

---

### 2.4 【パート D】センサー前フレーム値の状態管理

**目的：** prev_mag, prev_gps_pos, prev_baro_alt が正確に保持・更新されること

**対象ファイル：**
- `kalman/cpp/Inc/MEX/mex_run_eskf_sensor_updates.hpp`（行 ~350-370）

**実装時の確認項目：**

```cpp
// ❌ 現在の失敗コード（初期化時に 0 で固定）:
for (int i = 0; i < 3; ++i) {
    input.sensor.prev_mag[i] = 0.0f;
    input.sensor.prev_gps_pos[i] = 0.0f;
}
input.sensor.prev_baro_alt = 0.0f;

// ✓ 修正方法：
// 前フレーム値を ESKFState グローバル変数に保存し、更新時に取得
namespace mex_run_eskf_impl {
    std::map<uint64_t, ESKFState*> g_states;  // 既に存在
    
    // ESKFState 内に前フレーム値を追加:
    struct ESKFState {
        // ... 既存のメンバー
        float last_mag[3];
        float last_gps_pos[3];
        float last_baro_alt;
    };
}

// step 関数内で更新時に保存:
// センサー更新後、新しい値を保存
for (int i = 0; i < 3; ++i) {
    state->last_mag[i] = sensor_data.mag[i];
    state->last_gps_pos[i] = sensor_data.gps_pos[i];
}
state->last_baro_alt = sensor_data.alt_baro;
```

**検証方法：**

```matlab
% センサー値の変化検出が正常に機能することを確認
state1 = run_simulation_step(42, 1, sensor_data_1);
state2 = run_simulation_step(42, 2, sensor_data_2);

% prev_sensor 値が正確に保持されていることを確認
% （このチェックは内部で行われるべきだが、ログ出力で確認）
```

**検証 OK なら次へ進む**

---

## Phase 3: 統合テスト

### 3.1 単一テストの実行

```matlab
% MATLAB コマンドライン
cd kalman
clear mex

% 最初のテストで動作確認
run_simulation(42, true);

% 期待値:
% - 位置推定エラー < 5m
% - 速度推定エラー < 0.1 m/s
% - 姿勢推定エラー < 5°
```

### 3.2 バッチテストの実行

```matlab
% 10 セットのテストを実行
run_batch_10sets();

% 結果を保存
new_results = readtable('Results/batch_10sets_summary.csv');

% ベースライン と比較
baseline = load('baseline_results.mat').baseline_results;
diff_results = new_results - baseline;

% 統計的に有意な悪化がないことを確認
failed_new = sum(strcmp(new_results.Status, 'FAILED'));
failed_old = sum(strcmp(baseline.Status, 'FAILED'));

if failed_new > failed_old
    error("More failures than baseline!");
end
```

### 3.3 詳細比較

```matlab
% MEX と MATLAB の詳細比較
compare_mex_matlab_detailed();

% 期待値:
% - 数値差異 < 1% （float32/float64 の混在は許容）
% - NaN/Inf の出現 = 0
% - 発散の兆候なし
```

---

## Phase 4: 修正時の段階的アプローチ

修正を行う場合、以下の順序で **1 つずつ統合** してください：

```
Step 1: Part A (座標系変換) を修正
        ↓ テスト実行 → PASS?
Step 2: Part B (dt パラメータ) を修正
        ↓ テスト実行 → PASS?
Step 3: Part C (MEUKF 出力形式) を修正
        ↓ テスト実行 → PASS?
Step 4: Part D (センサー前フレーム値) を修正
        ↓ テスト実行 → PASS?
```

各ステップで **1 つの commit** を作成：

```bash
git add kalman/cpp/Inc/MEX/mex_run_eskf_sensor_updates.hpp
git commit -m "Fix Part A: Covariance matrix coordinate system transformation

- Ensure P matrix maintains symmetry during MATLAB->C++ conversion
- Add symmetry validation
- Test: run_batch_10sets() passes with baseline metrics"
```

---

## Phase 5: リスク管理

### 5.1 ロールバック計画

各修正ステップ後に問題が発生した場合：

```bash
# 最後の commit に戻す
git reset --soft HEAD~1

# または、特定の commit に戻す
git reset --hard <commit_hash>

# テスト環境をリセット
cd kalman
clear mex
```

### 5.2 バージョン管理

```bash
# 修正ブランチを作成（メインブランチは保護）
git checkout -b feature/meukf-integration-fix

# 各パートごとに commit
# ... Part A, B, C, D の修正

# テストが全て PASS した後、メインブランチへマージ
git checkout phase6
git merge feature/meukf-integration-fix
```

---

## Phase 6: ドキュメント更新

修正が完了したら、以下を更新：

### 6.1 座標系変換の仕様書を作成

**ファイル:** `kalman/cpp/markdown/COORDINATE_SYSTEM_SPEC.md`

```markdown
# 座標系変換仕様

## MATLAB ↔ C++ の変換方法

### 行列（shared_variance, covariance）
- MATLAB: 列優先 (column-major)
  - 要素アクセス: A[c*rows + r] (c: 列, r: 行)
- C++: 行優先 (row-major)
  - 要素アクセス: A[r*cols + c] (r: 行, c: 列)

### 変換コード例
```cpp
// MATLAB (15x15 列優先) → C++ (15x15 行優先)
for (int r = 0; r < 15; ++r) {
    for (int c = 0; c < 15; ++c) {
        P_cpp[r*15 + c] = static_cast<float>(P_matlab[c*15 + r]);
    }
}
// 対称性の確認:
assert(abs(P_cpp[r*15+c] - P_cpp[c*15+r]) < 1e-5);
```

## テスト方法
...
```

### 6.2 MEX インターフェース仕様書を更新

**ファイル:** `kalman/cpp/markdown/MEX_INTERFACE_SPEC.md`

```markdown
# MEX インターフェース仕様

## mex_run_eskf の I/O

### input
- sensor_type: char
- meas: double[3] or double[scalar]
- p, v, q, ba, bg: double vectors
- P: double[15*15] (MATLAB column-major)
- dt: double (time step in seconds)

### output
- out_p, out_v, out_q, out_ba, out_bg: double
- out_P: double[15*15] (MATLAB column-major, symmetrized)

## 重要: dt の扱い
- dt は常に > 0 であること
- dt = 0 の場合は予測ステップがスキップされる
- デフォルト: 0.01 秒

## 出力の対称化
```cpp
P = (P + P') / 2;  // MATLAB で実行
```
...
```

---

## チェックリスト（修正実施時）

```
[ ] Phase 1: ベースライン測定と期待値の記録
    [ ] baseline_results.mat を作成
    [ ] 期待される PASS/FAIL 数を記録
    
[ ] Phase 2.1: 座標系変換の修正
    [ ] コード変更
    [ ] 対称性テスト実行
    [ ] run_simulation(42, true) で検証
    
[ ] Phase 2.2: dt パラメータの修正
    [ ] 関数シグネチャ確認
    [ ] dt が 0 でないことを確認
    [ ] run_simulation(42, true) で検証
    
[ ] Phase 2.3: MEUKF 出力形式の検証
    [ ] MEUKFOutput の定義を確認
    [ ] 出力形式が MATLAB MEX と一致することを確認
    [ ] run_simulation(42, true) で検証
    
[ ] Phase 2.4: センサー前フレーム値の修正
    [ ] ESKFState に last_* フィールドを追加
    [ ] step 関数内で更新を追加
    [ ] run_simulation(42, true) で検証
    
[ ] Phase 3: 統合テスト
    [ ] run_batch_10sets() を実行
    [ ] new_results を baseline と比較
    [ ] compare_mex_matlab_detailed() で検証
    
[ ] Phase 4: Commit 作成
    [ ] Part A の commit 作成
    [ ] Part B の commit 作成
    [ ] Part C の commit 作成
    [ ] Part D の commit 作成
    
[ ] Phase 5: メインブランチへマージ
    [ ] feature branch から phase6 へマージ
    [ ] 最終テスト実行
    
[ ] Phase 6: ドキュメント更新
    [ ] COORDINATE_SYSTEM_SPEC.md 作成
    [ ] MEX_INTERFACE_SPEC.md 更新
    [ ] この checklist を archive
```

---

## 参考資料

- **失敗分析:** [FAILURE_ANALYSIS_REPORT.md](FAILURE_ANALYSIS_REPORT.md)
- **コミット履歴:** `git log --oneline -20 phase6`
- **型混在分析:** [kalman/cpp/TYPE_MIX_REPORT.md](kalman/cpp/TYPE_MIX_REPORT.md)


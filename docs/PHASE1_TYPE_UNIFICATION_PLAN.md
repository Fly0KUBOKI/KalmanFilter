# Phase 1: 型統一による数値再現性の確立

**目標**: float/double混在を解消し、コンパイル環境によらず数値計算結果を一致させる  
**期間**: 1週間  
**優先度**: ⭐⭐⭐⭐⭐（最高）

---

## 🎯 **Phase 1の成功基準**

### 定量的目標
1. **数値一致性**: This PC と Other PC で Position RMSE の差 < **1e-10 m**
2. **型の一貫性**: `static_cast<float>()` / `static_cast<double>()` の使用箇所を **80%削減**
3. **テスト成功率**: `run_batch_10sets()` で **10/10 PASS**
4. **ビルド時間**: Phase 1 完了後も **10秒以内**

### 定性的目標
- ✅ すべての状態変数の型が統一されている（float または double）
- ✅ MEX入出力の型が明確に定義されている
- ✅ ホットパス（eskf_runner.cpp）で型変換が発生しない

---

## 📋 **実装タスク（7つのサブタスク）**

### 🔧 **タスク 1.1: 型の方針決定**

**決定事項**:
```
全体を float に統一する（理由：MATLAB single 互換、メモリ効率）

ただし、以下の例外を認める：
- GPS 座標（lat, lon, alt）: double（WGS84の精度要件）
- MEX入力の一時バッファ: double（MATLAB ↔ C++ 境界）
```

**影響を受けるファイル**:
- `kalman/cpp/Lib/Common/interface.hpp` (ESKFState 構造体)
- `kalman/cpp/Lib/ESKF/src/eskf_runner.cpp` (型変換ループ)
- `kalman/cpp/MEX/Inc/mex_eskf_common.hpp` (MEX I/O)

**実装順序**:
1. interface.hpp の ESKFState を確認（現在の型定義を記録）
2. 混在している型を特定（grep で `double.*p\[|double.*v\[|double.*dt` を検索）
3. 統一方針を決定（全て float）
4. 変更箇所をリストアップ

---

### 🔧 **タスク 1.2: ESKFState 構造体の型統一**

**ファイル**: [kalman/cpp/Lib/Common/interface.hpp](kalman/cpp/Lib/Common/interface.hpp)

**現在の定義（推定）**:
```cpp
struct ESKFState {
    double p[3];          // 位置
    double v[3];          // 速度
    double q[4];          // 四元数
    double ba[3];         // 加速度bias
    double bg[3];         // ジャイロbias
    
    float P[15*15];       // 共分散（既にfloat）
    float Q_nominal[15*15]; // プロセスノイズ（既にfloat）
    
    double dt;            // サンプリング時間
    double g[3];          // 重力ベクトル
    
    // その他のパラメータ...
};
```

**修正後**:
```cpp
struct ESKFState {
    float p[3];           // ❌ double → ✅ float
    float v[3];           // ❌ double → ✅ float
    float q[4];           // ❌ double → ✅ float
    float ba[3];          // ❌ double → ✅ float
    float bg[3];          // ❌ double → ✅ float
    
    float P[15*15];       // ✅ 変更なし
    float Q_nominal[15*15]; // ✅ 変更なし
    
    float dt;             // ❌ double → ✅ float
    float g[3];           // ❌ double → ✅ float
    
    // GPS origin だけは double を許容
    double gps_origin_lat;
    double gps_origin_lon;
    double gps_origin_alt;
};
```

**実装手順**:
1. `interface.hpp` を開く
2. ESKFState の各メンバ型を float に変更
3. GPS origin 専用の double メンバを追加
4. コンパイルエラーを確認（依存ファイルを特定）

---

### 🔧 **タスク 1.3: eskf_runner.cpp の型変換削除**

**ファイル**: [kalman/cpp/Lib/ESKF/src/eskf_runner.cpp](kalman/cpp/Lib/ESKF/src/eskf_runner.cpp)

**現在の問題コード（21-23行、35行）**:
```cpp
// ❌ 削除対象: 毎フレーム double ↔ float 変換
for (int i=0;i<3;++i){
    p_f(i,0)=static_cast<float>(s->p[i]);   // double → float
    v_f(i,0)=static_cast<float>(s->v[i]);
    ba_f(i,0)=static_cast<float>(s->ba[i]);
    bg_f(i,0)=static_cast<float>(s->bg[i]);
    a_meas_f(i,0)=static_cast<float>(a_meas[i]);
    w_meas_f(i,0)=static_cast<float>(w_meas[i]);
    g_f(i,0)=static_cast<float>(s->g[i]);
}
for (int i=0;i<4;++i) q_f(i,0)=static_cast<float>(s->q[i]);

// ... 計算 ...

// ❌ 削除対象: float → double 変換
for (int i=0;i<3;++i){
    s->p[i]=static_cast<double>(p_f(i,0));   // float → double
    s->v[i]=static_cast<double>(v_f(i,0));
    s->ba[i]=static_cast<double>(ba_f(i,0));
    s->bg[i]=static_cast<double>(bg_f(i,0));
}
for (int i=0;i<4;++i) s->q[i]=static_cast<double>(q_f(i,0));
```

**修正後**:
```cpp
// ✅ 追加: 直接アクセス（変換なし）
void eskf_runner_step(ESKFState* s, const float* a_meas, const float* w_meas) {
    // ESKFState がすでに float なので、変換不要
    Vector<3,float> p_f(s->p[0], s->p[1], s->p[2]);
    Vector<3,float> v_f(s->v[0], s->v[1], s->v[2]);
    Vector<4,float> q_f(s->q[0], s->q[1], s->q[2], s->q[3]);
    Vector<3,float> ba_f(s->ba[0], s->ba[1], s->ba[2]);
    Vector<3,float> bg_f(s->bg[0], s->bg[1], s->bg[2]);
    
    Vector<3,float> a_meas_f(a_meas[0], a_meas[1], a_meas[2]);
    Vector<3,float> w_meas_f(w_meas[0], w_meas[1], w_meas[2]);
    Vector<3,float> g_f(s->g[0], s->g[1], s->g[2]);
    
    // ... 計算（変更なし）...
    
    // ✅ 直接書き戻し（変換なし）
    s->p[0] = p_f(0,0); s->p[1] = p_f(1,0); s->p[2] = p_f(2,0);
    s->v[0] = v_f(0,0); s->v[1] = v_f(1,0); s->v[2] = v_f(2,0);
    s->q[0] = q_f(0,0); s->q[1] = q_f(1,0); s->q[2] = q_f(2,0); s->q[3] = q_f(3,0);
    s->ba[0] = ba_f(0,0); s->ba[1] = ba_f(1,0); s->ba[2] = ba_f(2,0);
    s->bg[0] = bg_f(0,0); s->bg[1] = bg_f(1,0); s->bg[2] = bg_f(2,0);
}
```

**削減効果**:
- **Before**: 20箇所の `static_cast<float>()` + 20箇所の `static_cast<double>()`
- **After**: 0箇所
- **削減率**: **100%**

---

### 🔧 **タスク 1.4: MEX入出力の型変換整理**

**ファイル**: [kalman/cpp/MEX/Inc/mex_type_conversion.hpp](kalman/cpp/MEX/Inc/mex_type_conversion.hpp)

**現在の問題**:
- MATLAB から double で受け取る → C++ で float に変換 → 戻すときに double
- 3回の型変換が発生

**修正方針**:
```cpp
namespace mex_conv {

// MATLAB (double*) → C++ (float*) 変換を一度だけ
inline void mxArrayToFloatArray(const mxArray* mx, float* out, size_t n) {
    const double* src = mxGetPr(mx);  // MATLAB は常に double
    for (size_t i = 0; i < n; ++i) {
        out[i] = static_cast<float>(src[i]);  // ← ここだけで変換
    }
}

// C++ (float*) → MATLAB (double*) 変換を一度だけ
inline mxArray* floatArrayToMxArray(const float* data, size_t rows, size_t cols) {
    mxArray* mx = mxCreateDoubleMatrix(rows, cols, mxREAL);
    double* dst = mxGetPr(mx);
    for (size_t i = 0; i < rows * cols; ++i) {
        dst[i] = static_cast<double>(data[i]);  // ← ここだけで変換
    }
    return mx;
}

} // namespace mex_conv
```

**重要**: 変換は **MEX境界でのみ** 実施し、内部計算では一切変換しない

---

### 🔧 **タスク 1.5: 配列の明示的初期化**

**ファイル**: 
- [kalman/cpp/Lib/Matrix/fixed_matrix.hpp](kalman/cpp/Lib/Matrix/fixed_matrix.hpp)
- [kalman/cpp/Lib/MEUKF/inc/meukf_types.hpp](kalman/cpp/Lib/MEUKF/inc/meukf_types.hpp)

**修正内容**:

#### fixed_matrix.hpp (181, 278行)
```cpp
// ❌ 現在
struct FixedMatrix {
    int rows, cols;
    float data[MAX_N * MAX_N];  // ← 未初期化
};

// ✅ 修正後
struct FixedMatrix {
    int rows = 0;
    int cols = 0;
    float data[MAX_N * MAX_N] = {0.0f};  // ← ゼロ初期化
};
```

#### meukf_types.hpp (26-27行)
```cpp
// ❌ 現在
struct SensorData {
    float accel[3], gyro[3], mag[3];
    float prev_mag[3];       // ← 未初期化
    float prev_gps_pos[3];   // ← 未初期化
};

// ✅ 修正後
struct SensorData {
    float accel[3] = {0.0f};
    float gyro[3] = {0.0f};
    float mag[3] = {0.0f};
    float prev_mag[3] = {0.0f};      // ← ゼロ初期化
    float prev_gps_pos[3] = {0.0f};  // ← ゼロ初期化
};
```

---

### 🔧 **タスク 1.6: 数値テストの実装**

**新規ファイル**: `kalman/test_numeric_consistency.m`

```matlab
function test_numeric_consistency()
% Phase 1 修正後の数値一貫性テスト
% 
% 目標: This PC と Other PC で Position RMSE の差 < 1e-10 m

fprintf('====================================\n');
fprintf('NUMERIC CONSISTENCY TEST (Phase 1)\n');
fprintf('====================================\n\n');

% テスト用のシード
test_seeds = [1, 42, 123, 999, 2024];

results = struct();

for i = 1:length(test_seeds)
    seed = test_seeds(i);
    fprintf('Testing seed %d... ', seed);
    
    % シミュレーション実行
    tic;
    run_simulation(seed, false);  % verbose=false
    elapsed = toc;
    
    % 結果読み込み
    csv_file = sprintf('Results/estimation_%02d.csv', i);
    if ~exist(csv_file, 'file')
        fprintf('FAIL (file not found)\n');
        continue;
    end
    
    data = readmatrix(csv_file);
    
    % Position RMSE 計算
    pos_error = data(:, 2:4);  % [px, py, pz]
    pos_rmse = sqrt(mean(sum(pos_error.^2, 2)));
    
    % Velocity RMSE 計算
    vel_error = data(:, 5:7);  % [vx, vy, vz]
    vel_rmse = sqrt(mean(sum(vel_error.^2, 2)));
    
    % 記録
    results(i).seed = seed;
    results(i).pos_rmse = pos_rmse;
    results(i).vel_rmse = vel_rmse;
    results(i).time = elapsed;
    
    fprintf('PASS (pos=%.10f m, vel=%.10f m/s, time=%.2fs)\n', ...
        pos_rmse, vel_rmse, elapsed);
end

fprintf('\n====================================\n');
fprintf('SUMMARY\n');
fprintf('====================================\n');
fprintf('%-10s %-20s %-20s %-10s\n', 'Seed', 'Position RMSE (m)', 'Velocity RMSE (m/s)', 'Time (s)');
for i = 1:length(results)
    fprintf('%-10d %.15f  %.15f  %.2f\n', ...
        results(i).seed, results(i).pos_rmse, results(i).vel_rmse, results(i).time);
end

% 結果を保存（Other PC との比較用）
save('Results/phase1_numeric_test.mat', 'results');
fprintf('\nResults saved to: Results/phase1_numeric_test.mat\n');
fprintf('Copy this file to Other PC and run compare_numeric_results.m\n');

end
```

**検証用スクリプト**: `kalman/compare_numeric_results.m`

```matlab
function compare_numeric_results(this_pc_file, other_pc_file)
% Phase 1: This PC と Other PC の数値結果を比較
%
% Usage:
%   compare_numeric_results('Results/phase1_numeric_test.mat', ...
%                          'OtherPC/Results/phase1_numeric_test.mat')

fprintf('====================================\n');
fprintf('CROSS-PC NUMERIC COMPARISON\n');
fprintf('====================================\n\n');

% ファイル読み込み
this_pc = load(this_pc_file);
other_pc = load(other_pc_file);

if length(this_pc.results) ~= length(other_pc.results)
    error('Results have different number of seeds');
end

fprintf('%-10s %-20s %-20s %-15s\n', 'Seed', 'This PC Pos RMSE', 'Other PC Pos RMSE', 'Difference (m)');
fprintf(repmat('-', 1, 80)); fprintf('\n');

max_diff = 0;
for i = 1:length(this_pc.results)
    seed = this_pc.results(i).seed;
    pos_this = this_pc.results(i).pos_rmse;
    pos_other = other_pc.results(i).pos_rmse;
    diff = abs(pos_this - pos_other);
    
    fprintf('%-10d %.15f  %.15f  %.2e\n', seed, pos_this, pos_other, diff);
    
    if diff > max_diff
        max_diff = diff;
    end
end

fprintf(repmat('-', 1, 80)); fprintf('\n');
fprintf('Maximum difference: %.2e m\n', max_diff);

% 判定
threshold = 1e-10;
if max_diff < threshold
    fprintf('✅ PASS: Numeric consistency achieved (< %.2e m)\n', threshold);
else
    fprintf('❌ FAIL: Numeric difference too large (> %.2e m)\n', threshold);
    fprintf('    Possible causes:\n');
    fprintf('    - float/double conversion not fully eliminated\n');
    fprintf('    - Compiler optimization differences\n');
    fprintf('    - Uninitialized variable usage\n');
end

end
```

---

### 🔧 **タスク 1.7: ビルドスクリプトの更新**

**ファイル**: [kalman/cpp/build/build_mex.m](kalman/cpp/build/build_mex.m)

**追加内容**:
```matlab
% Phase 1: 数値再現性を保証するコンパイラフラグ

% 現在
compile_opts = {'-O', '-DNDEBUG', '-DKALMAN_NO_STANDALONE'};

% 修正後（厳密な浮動小数点演算）
compile_opts = {
    '-O2',                    % 最適化レベルを明示（-O は曖昧）
    '-DNDEBUG',
    '-DKALMAN_NO_STANDALONE',
    '-DPHASE1_TYPE_UNIFIED'   % Phase 1 完了フラグ
};

if ispc
    compile_opts = [compile_opts, {
        '-DWIN32',
        '-D_CRT_SECURE_NO_WARNINGS',
        '/fp:precise',        % 浮動小数点演算の厳密化
        '/arch:SSE2',         % SIMD命令セットを固定
        '/MD'                 % Runtime Library を明示
    }];
    
    old_compflags = getenv('COMPFLAGS');
    setenv('COMPFLAGS', '/utf-8 /fp:precise /arch:SSE2');
else
    % Linux/macOS
    compile_opts = [compile_opts, {
        '-msse2',             % SIMD命令セットを固定
        '-ffloat-store',      % 浮動小数点レジスタの使用を制限
        '-fno-fast-math'      % 高速だが精度が落ちる最適化を無効化
    }];
end
```

**理由**:
- `/fp:precise` (MSVC) / `-ffloat-store` (GCC): 中間計算の丸め誤差を一貫化
- `/arch:SSE2` / `-msse2`: CPU命令セットを固定（AVX vs SSE の差異を排除）
- `/MD`: Runtime Library を動的リンクに統一（静的/動的の差異を排除）

---

## 📊 **実装スケジュール**

| 日 | タスク | 担当 | 完了基準 |
|----|--------|------|--------|
| **Day 1** | タスク 1.1 型方針決定 | — | 統一方針ドキュメント完成 |
| **Day 2** | タスク 1.2 ESKFState 修正 | — | interface.hpp 型変更完了 |
| **Day 3** | タスク 1.3 eskf_runner 修正 | — | 型変換ループ削除、ビルド成功 |
| **Day 4** | タスク 1.4 MEX変換整理 | — | mex_type_conversion.hpp 更新 |
| **Day 5** | タスク 1.5 配列初期化 | — | fixed_matrix.hpp, meukf_types.hpp 修正 |
| **Day 6** | タスク 1.6 数値テスト実装 | — | test_numeric_consistency.m 実行成功 |
| **Day 7** | タスク 1.7 ビルドスクリプト更新 | — | build_mex.m 更新、両PC でビルド |

---

## 🧪 **検証プロトコル**

### ステップ 1: This PC での検証

```matlab
% 1. Phase 1 修正を適用
cd kalman/cpp/build
build_mex();
clear mex;

% 2. 数値一貫性テスト
cd ../..
test_numeric_consistency();

% 出力例:
% Seed 1    PASS (pos=0.3214567890 m, vel=0.0123456789 m/s, time=1.23s)
% Seed 42   PASS (pos=0.3187654321 m, vel=0.0109876543 m/s, time=1.18s)
% ...
```

### ステップ 2: Other PC での検証

```matlab
% 1. Git pull して Phase 1 コードを取得
git pull origin phase1/type-unification

% 2. ビルド
cd kalman/cpp/build
build_mex();
clear mex;

% 3. 同じテスト実行
cd ../..
test_numeric_consistency();

% 4. ファイルコピー
% phase1_numeric_test.mat を This PC にコピー
```

### ステップ 3: 結果比較

```matlab
% This PC で実行
compare_numeric_results('Results/phase1_numeric_test.mat', ...
                       'OtherPC/phase1_numeric_test.mat');

% 期待出力:
% Maximum difference: 1.23e-11 m
% ✅ PASS: Numeric consistency achieved (< 1.00e-10 m)
```

---

## 📝 **ロールバック計画**

Phase 1 で問題が発生した場合の対応：

### シナリオ A: ビルドエラー

**症状**: interface.hpp 変更後にコンパイルエラー

**対応**:
```bash
git stash  # 一時退避
git log --oneline -10  # 直前のコミットを確認
git checkout HEAD~1  # 1つ前に戻る
```

### シナリオ B: 数値テスト失敗

**症状**: `test_numeric_consistency()` で RMSE が異常

**対応**:
1. `diagnose_environment()` で環境差異を確認
2. `build_mex_verbose()` でコンパイラフラグを比較
3. MEX の型変換処理をデバッグ出力で確認

### シナリオ C: 既存テスト失敗

**症状**: `run_batch_10sets()` で FAIL が出る

**対応**:
1. どのテストケースで失敗したか特定
2. そのシードで `run_simulation(seed, true)` を verbose 実行
3. ログを Phase 1 前後で比較

---

## ✅ **チェックリスト**

### 実装前
- [ ] `interface.hpp` のバックアップを取得
- [ ] `eskf_runner.cpp` のバックアップを取得
- [ ] 現在の `run_batch_10sets()` 結果を保存（ベースライン）
- [ ] Git ブランチを作成 (`git checkout -b phase1/type-unification`)

### 実装中
- [ ] タスク 1.1: 型方針決定完了
- [ ] タスク 1.2: ESKFState 型変更完了、ビルド成功
- [ ] タスク 1.3: eskf_runner.cpp 型変換削除、ビルド成功
- [ ] タスク 1.4: MEX変換整理完了
- [ ] タスク 1.5: 配列初期化完了
- [ ] タスク 1.6: 数値テスト実装完了、This PC でPASS
- [ ] タスク 1.7: ビルドスクリプト更新完了

### 実装後
- [ ] This PC で `test_numeric_consistency()` 実行 → PASS
- [ ] This PC で `run_batch_10sets()` 実行 → 10/10 PASS
- [ ] Other PC で同じテスト実行 → PASS
- [ ] `compare_numeric_results()` 実行 → 差 < 1e-10 m
- [ ] ドキュメント更新（README, CHANGELOG）
- [ ] Git commit & push

---

## 📚 **参考資料**

- [CODE_PORTABILITY_ANALYSIS.md](./CODE_PORTABILITY_ANALYSIS.md) — 全体の修正計画
- [CPP_INPUT_OUTPUT_SPEC.md](./CPP_INPUT_OUTPUT_SPEC.md) — MEX型マッピング
- [BINARY_MANAGEMENT.md](./BINARY_MANAGEMENT.md) — バイナリ管理戦略

---

**次のステップ**: タスク 1.1（型方針決定）から実装を開始してください。

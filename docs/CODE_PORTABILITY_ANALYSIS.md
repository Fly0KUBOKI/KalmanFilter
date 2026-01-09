# コード移植性分析と修正計画

**作成日**: 2026-01-09  
**目的**: 環境依存によって実行結果が変わるコードを特定し、基礎的なC++への書き換え計画を作成  
**問題**: コンパイル環境によってバイナリサイズだけでなく、**数値計算結果**も変化する可能性がある

---

## 🚨 **重大な懸念事項**

### ユーザーの指摘
> コンパイル環境に依存して、バイナリの変化、実行結果の変化が生まれるのはプログラムとして失格である

### 現状の問題
1. **バイナリサイズが6.3倍異なる** (155KB vs 976KB)
2. **実行結果の一致性が未検証**（Other PC での数値結果が This PC と一致するか不明）
3. **モダンC++機能の多用** → コンパイラ依存の最適化や未定義動作のリスク
4. **float/double混在** → 精度損失、丸め誤差の蓄積

---

## 📊 **検出された環境依存コードのカテゴリ**

### 🔴 **カテゴリ A: モダンC++機能の使用**（11箇所）

| ファイル | 行 | 問題コード | リスク |
|---------|-----|-----------|--------|
| meukf_update.cpp | 43, 105, 153, 218, 411 | `auto h_func = [](...)` | ラムダ式のキャプチャ方式がコンパイラ依存 |
| assemble_measurements.cpp | 14 | `auto add_block = [&](...)` | 参照キャプチャの最適化挙動が不確定 |
| kalman_filter.cpp | 159 | `auto is_well_conditioned = [&]()` | コンパイラによるインライン化が異なる |
| mex_run_eskf.cpp | 45, 52 | `auto it = g_states.find()` | `std::map` のイテレータ型推論 |
| mex_meukf_step.cpp | 33, 107 | `auto set_vec3_float = [&]()` | 型推論の精度（float/double） |

**問題点**:
- `auto` 型推論 → コンパイラによって推論される型が微妙に異なる可能性（`float` vs `double` の曖昧性）
- ラムダ式 → インライン化の挙動がコンパイラ最適化に依存
- C++11/14 機能 → 古いコンパイラではサポート不足

**修正方針**:
```cpp
// ❌ 現在（auto + lambda）
auto h_func = [](const Vector15& xv) {
    return compute_measurement(xv);
};

// ✅ 修正後（明示的な関数ポインタ）
Vector3 h_func_accel(const Vector15& xv) {
    return compute_measurement(xv);
}
```

---

### 🟠 **カテゴリ B: constexpr と if constexpr**（20箇所以上）

| ファイル | 行 | 問題コード | リスク |
|---------|-----|-----------|--------|
| quaternion_functions.hpp | 114, 151 | `constexpr T EPS = static_cast<T>(1e-9)` | テンプレート型Tによって精度が異なる |
| fixed_matrix.hpp | 473 | `if constexpr (N == 3)` | C++17 機能、古いコンパイラで失敗 |
| meukf_observation_models.hpp | 21, 67 | `static constexpr float g_ned[3]` | 配列の初期化順序がコンパイラ依存 |
| types.hpp | 14, 15 | `static constexpr Index MAX_STATE_DIM = 20` | ヘッダオンリーで重複定義リスク |

**問題点**:
- `constexpr` → コンパイル時定数の評価がコンパイラ最適化に依存
- `if constexpr` → C++17 必須、古い環境では非対応
- `static constexpr` 配列 → メモリレイアウトがコンパイラ依存

**修正方針**:
```cpp
// ❌ 現在（constexpr + if constexpr）
template <typename T>
constexpr T EPS = static_cast<T>(1e-9);

if constexpr (N == 3) {
    // 3x3 専用処理
}

// ✅ 修正後（明示的な #define + テンプレート特殊化）
#define EPS_FLOAT 1.0e-9f
#define EPS_DOUBLE 1.0e-9

template <int N>
void process();

template <>
void process<3>() {
    // 3x3 専用処理（特殊化）
}
```

---

### 🔴 **カテゴリ C: float/double混在による精度損失**（20箇所以上）

| ファイル | 行 | 問題コード | リスク |
|---------|-----|-----------|--------|
| eskf_runner.cpp | 21-23 | `static_cast<float>(s->p[i])` の大量使用 | double→float変換で精度損失 |
| eskf_runner.cpp | 35 | `static_cast<double>(p_f(i,0))` | float→double変換で復元不能 |
| meukf_predict.cpp | 142 | `double val = static_cast<double>(P_new(i,j))` | 型変換の往復で誤差蓄積 |
| meukf_sigma_points.cpp | 32 | `const_cast<Matrix&>(sigmas)` | const違反、未定義動作のリスク |

**問題点**:
- **精度の二重劣化**: double（内部） → float（計算） → double（出力）
- **蓄積誤差**: 1ステップごとの変換誤差がフィルタ全体で累積
- **丸め誤差の非再現性**: コンパイラ最適化によって丸め方向が変わる

**例（eskf_runner.cpp:21-23）**:
```cpp
// ❌ 現在（ホットパスで毎フレーム変換）
for (int i=0;i<3;++i){
    p_f(i,0)=static_cast<float>(s->p[i]);   // double → float（精度損失1）
    v_f(i,0)=static_cast<float>(s->v[i]);
    ba_f(i,0)=static_cast<float>(s->ba[i]);
    bg_f(i,0)=static_cast<float>(s->bg[i]);
}
// ... 計算 ...
for (int i=0;i<3;++i){
    s->p[i]=static_cast<double>(p_f(i,0));  // float → double（精度損失2）
}
```

**数値例**:
```
Original double: 36.123456789012345   (15桁精度)
      ↓ static_cast<float>
Float: 36.1234589                     (7桁精度、丸め誤差発生)
      ↓ static_cast<double>
Final double: 36.123458862304688     (元の値に戻らない)

累積誤差: 10,000ステップで ~1e-4m の位置誤差
```

**修正方針**:
```cpp
// ✅ 修正後（全体をdoubleで統一）
struct ESKFState {
    double p[3], v[3], q[4], ba[3], bg[3];  // float → double
    double P[15*15], Q_nominal[15*15];      // float → double
    double dt;
};

// または、全体をfloatで統一（メモリ効率重視の場合）
struct ESKFState {
    float p[3], v[3], q[4], ba[3], bg[3];
    float P[15*15], Q_nominal[15*15];
    float dt;  // double → float
};
```

---

### 🟡 **カテゴリ D: 未初期化変数のリスク**（検出なし、但し潜在的リスクあり）

| ファイル | 行 | 問題コード | リスク |
|---------|-----|-----------|--------|
| fixed_matrix.hpp | 181 | `float data[MAX_N * MAX_N]` | 配列が初期化されていない |
| fixed_matrix.hpp | 278 | `float aug[MAX_N][MAX_N * 2]` | スタック配列、初期化なし |
| meukf_types.hpp | 26-27 | `float prev_mag[3], prev_gps_pos[3]` | コンストラクタで初期化されていない可能性 |

**問題点**:
- **未定義動作**: 初期化されていない変数を読むと、スタックのゴミ値を使用
- **環境依存**: コンパイラ最適化によって初期値が異なる（0埋めされる場合とされない場合）

**修正方針**:
```cpp
// ❌ 現在（初期化なし）
struct FixedMatrix {
    float data[MAX_N * MAX_N];  // ← ゴミ値
};

// ✅ 修正後（ゼロ初期化）
struct FixedMatrix {
    float data[MAX_N * MAX_N] = {0.0f};  // C++11
};

// または
FixedMatrix() {
    memset(data, 0, sizeof(data));  // C互換
}
```

---

### 🟠 **カテゴリ E: コンパイラ依存の最適化**

| ファイル | 行 | 問題コード | リスク |
|---------|-----|-----------|--------|
| eskf_core.cpp | 17 | `static Vector3 prev_a_world` | 静的変数の初期化タイミング |
| eskf_core.cpp | 19 | `static bool prev_initialized = false` | スレッドセーフティ未保証 |

**問題点**:
- **静的変数の初期化順序**: コンパイラによって異なる
- **最適化による削除**: `-O2` では未使用変数が削除される可能性

---

## 🎯 **修正計画（3段階アプローチ）**

### 📋 **Phase 1: 緊急対応（1週間）** — 数値精度の統一

#### 1.1 型の統一（float vs double）

**方針**: **全体を float に統一**（メモリ効率 + MATLAB single互換）

**対象ファイル**:
- [kalman/cpp/Lib/ESKF/src/eskf_runner.cpp](kalman/cpp/Lib/ESKF/src/eskf_runner.cpp)
- [kalman/cpp/Lib/ESKF/inc/eskf_types.hpp](kalman/cpp/Lib/ESKF/inc/eskf_types.hpp)
- [kalman/cpp/MEX/Inc/mex_eskf_common.hpp](kalman/cpp/MEX/Inc/mex_eskf_common.hpp)

**変更内容**:
```cpp
// eskf_types.hpp（interface.hpp も同様）
struct ESKFState {
    float p[3], v[3], q[4], ba[3], bg[3];  // ✓ 既に float
    float P[15*15], Q_nominal[15*15];      // ✓ 既に float
    float dt;                              // ❌ double → float に変更
    float g[3];                            // ✓ 既に float
    // ...
};
```

**eskf_runner.cpp の修正**:
```cpp
// ❌ 削除: 毎フレームの型変換ループ（21-23行、35行）
// ✅ 追加: 一度だけの変換（初期化時）

void eskf_runner_step(ESKFState* s, const float* a_meas, const float* w_meas) {
    // 直接 float で処理（変換なし）
    Vector<3,float> a_meas_f(a_meas[0], a_meas[1], a_meas[2]);
    Vector<3,float> w_meas_f(w_meas[0], w_meas[1], w_meas[2]);
    
    // ... 計算 ...
    
    // 直接書き戻し（変換なし）
    s->p[0] = p_f(0,0);
    s->p[1] = p_f(1,0);
    s->p[2] = p_f(2,0);
}
```

#### 1.2 配列の明示的初期化

**対象**:
- [kalman/cpp/Lib/Matrix/fixed_matrix.hpp](kalman/cpp/Lib/Matrix/fixed_matrix.hpp) (181, 278行)
- [kalman/cpp/Lib/MEUKF/inc/meukf_types.hpp](kalman/cpp/Lib/MEUKF/inc/meukf_types.hpp) (26-27行)

**変更**:
```cpp
// fixed_matrix.hpp
struct FixedMatrix {
    int rows = 0, cols = 0;
    float data[MAX_N * MAX_N] = {0.0f};  // ← ゼロ初期化
};

// meukf_types.hpp
struct SensorData {
    float accel[3] = {0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float mag[3] = {0.0f, 0.0f, 0.0f};
    float prev_mag[3] = {0.0f, 0.0f, 0.0f};      // ← ゼロ初期化
    float prev_gps_pos[3] = {0.0f, 0.0f, 0.0f};  // ← ゼロ初期化
};
```

#### 1.3 コンパイラフラグの統一

**build_mex.m の修正**:
```matlab
% 現在
compile_opts = {'-O', '-DNDEBUG', '-DKALMAN_NO_STANDALONE'};

% 修正後（厳密な再現性）
compile_opts = {
    '-O2',                    % 最適化レベルを明示
    '-DNDEBUG',
    '-DKALMAN_NO_STANDALONE',
    '-fp:precise',            % 浮動小数点演算の厳密化（MSVC）
    '-ffloat-store'           % 浮動小数点レジスタの使用を制限（GCC）
};

% MSVC 固有設定
if ispc
    compile_opts = [compile_opts, {
        '/fp:precise',        % 浮動小数点の精度優先
        '/arch:SSE2',         % CPU命令セットを統一
        '/MD'                 % Runtime Library を動的リンクに固定
    }];
end
```

---

### 📋 **Phase 2: モダンC++機能の基礎化（2週間）**

#### 2.1 auto キーワードの削除

**対象**:
- meukf_update.cpp (43, 105, 153, 218, 411行)
- assemble_measurements.cpp (14行)
- mex_run_eskf.cpp (45, 52行)

**変更例**:
```cpp
// ❌ 現在
auto h_func = [](const Vector15& xv) -> Vector3 {
    return compute_accel_measurement(xv);
};

// ✅ 修正後
typedef Vector3 (*H_Function)(const Vector15&);

Vector3 h_func_accel(const Vector15& xv) {
    return compute_accel_measurement(xv);
}

H_Function h_func = h_func_accel;
```

#### 2.2 constexpr の置き換え

**対象**:
- quaternion_functions.hpp (114, 151行)
- types.hpp (14-15行)

**変更例**:
```cpp
// ❌ 現在
template <typename T>
constexpr T EPS = static_cast<T>(1e-9);

// ✅ 修正後
#define EPS_FLOAT 1.0e-9f
#define EPS_DOUBLE 1.0e-9

template <typename T>
inline T get_eps() {
    return sizeof(T) == sizeof(float) ? EPS_FLOAT : EPS_DOUBLE;
}
```

#### 2.3 if constexpr の削除

**対象**:
- fixed_matrix.hpp (473行)

**変更例**:
```cpp
// ❌ 現在
template <int N, typename T>
void invert(const Matrix<N,N,T>& A, Matrix<N,N,T>& A_inv) {
    if constexpr (N == 3) {
        invert3x3(A, A_inv);  // 3x3 専用
    } else {
        general_invert(A, A_inv);
    }
}

// ✅ 修正後（テンプレート特殊化）
template <int N, typename T>
void invert(const Matrix<N,N,T>& A, Matrix<N,N,T>& A_inv) {
    general_invert(A, A_inv);  // 一般実装
}

template <typename T>
void invert<3, T>(const Matrix<3,3,T>& A, Matrix<3,3,T>& A_inv) {
    invert3x3(A, A_inv);  // 3x3 専用（特殊化）
}
```

---

### 📋 **Phase 3: 数値安定性の強化（3週間）**

#### 3.1 浮動小数点演算の厳密化

**新規ファイル**: `kalman/cpp/Lib/Common/inc/Math/fp_strict.hpp`

```cpp
#pragma once

namespace common {
namespace fp {

// 厳密な浮動小数点比較
inline bool nearly_equal(float a, float b, float epsilon = 1.0e-6f) {
    return fabsf(a - b) <= epsilon * fmaxf(1.0f, fmaxf(fabsf(a), fabsf(b)));
}

// 厳密な加算（Kahan summation）
inline float stable_sum(const float* data, int n) {
    float sum = 0.0f;
    float compensation = 0.0f;
    
    for (int i = 0; i < n; ++i) {
        float y = data[i] - compensation;
        float t = sum + y;
        compensation = (t - sum) - y;
        sum = t;
    }
    return sum;
}

// 数値的に安定な内積計算
inline float stable_dot(const float* a, const float* b, int n) {
    float sum = 0.0f;
    float c = 0.0f;
    
    for (int i = 0; i < n; ++i) {
        float prod = a[i] * b[i];
        float y = prod - c;
        float t = sum + y;
        c = (t - sum) - y;
        sum = t;
    }
    return sum;
}

} // namespace fp
} // namespace common
```

#### 3.2 静的解析ツールの導入

**cppcheck の実行**:
```bash
cd kalman/cpp
cppcheck --enable=all --suppress=missingIncludeSystem \
         --platform=win64 --std=c++14 \
         -I Lib -I MEX/Inc \
         Lib/ MEX/ \
         2>&1 | tee cppcheck_report.txt
```

**clang-tidy の実行**:
```bash
clang-tidy kalman/cpp/Lib/**/*.cpp \
    -checks='-*,bugprone-*,cert-*,modernize-*,performance-*' \
    -- -std=c++14 -I kalman/cpp/Lib
```

---

## 🧪 **検証計画**

### ステップ 1: 型統一後の数値一致テスト

**テストスクリプト**: `kalman/verify_numeric_stability.m`

```matlab
function verify_numeric_stability()
    % Phase 1 修正後の検証
    seeds = [1, 42, 123, 999, 2024];
    
    for seed_idx = 1:length(seeds)
        seed = seeds(seed_idx);
        
        % シミュレーション実行
        run_simulation(seed, true);
        
        % 結果の保存
        result = readmatrix(sprintf('Results/estimation_%02d.csv', seed_idx));
        
        % 数値精度の検証
        fprintf('Seed %d:\n', seed);
        fprintf('  Position RMSE: %.10f m\n', mean(sqrt(sum(result(:,2:4).^2, 2))));
        fprintf('  Velocity RMSE: %.10f m/s\n', mean(sqrt(sum(result(:,5:7).^2, 2))));
    end
end
```

**期待結果**:
- This PC と Other PC で **Position RMSE の差 < 1e-10 m**
- Velocity RMSE の差 < 1e-10 m/s

### ステップ 2: バイナリサイズの監視

```bash
# Phase 1-3 修正後
cd kalman/cpp/bin
ls -lh *.mexw64

# 期待:
# - サイズが This PC と Other PC で一致（±1KB以内）
# - 155KB 前後（デバッグシンボルなし）
```

---

## 📊 **修正優先順位マトリクス**

| カテゴリ | 影響度 | 実装難易度 | 優先度 | 期限 |
|---------|--------|----------|--------|------|
| **C. float/double混在** | ⭐⭐⭐⭐⭐ | ⭐⭐ | **最高** | 1週間 |
| **D. 未初期化変数** | ⭐⭐⭐⭐ | ⭐ | **高** | 1週間 |
| **A. auto/lambda** | ⭐⭐⭐ | ⭐⭐⭐ | **中** | 2週間 |
| **B. constexpr** | ⭐⭐ | ⭐⭐ | **中** | 2週間 |
| **E. 静的変数** | ⭐⭐ | ⭐⭐⭐ | **低** | 3週間 |

---

## 🎯 **成功基準**

### Phase 1 完了時
- ✅ This PC と Other PC で **数値結果が完全一致**（RMSE差 < 1e-10）
- ✅ バイナリサイズ差が ±10% 以内
- ✅ 全テストケース（10 seeds）で PASS

### Phase 2 完了時
- ✅ C++11/14 機能を C++98/03 互換コードに置き換え
- ✅ `-std=c++98` でコンパイル成功
- ✅ cppcheck でエラー0件

### Phase 3 完了時
- ✅ 数値安定性テスト（Kahan summation）で誤差 < 1e-8
- ✅ 静的解析ツールで警告0件
- ✅ 5つの異なるコンパイラ（MSVC, GCC, Clang, MinGW, Intel）でビルド成功

---

## 📝 **まとめ**

### 現在の問題
1. **実行結果の非再現性**: コンパイラによって数値計算結果が異なる可能性
2. **モダンC++依存**: auto, constexpr, lambda などが環境差を生む
3. **float/double混在**: 精度損失が蓄積

### 解決方針
1. **型の統一**: 全体を float に統一（Phase 1）
2. **基礎的なC++**: C++98互換コードへの書き換え（Phase 2）
3. **数値安定化**: Kahan summation, 厳密な浮動小数点演算（Phase 3）

### 期待効果
- ✅ **完全な数値再現性**: 任意のPC・コンパイラで同じ結果
- ✅ **移植性の向上**: 古い環境でもビルド可能
- ✅ **保守性の向上**: シンプルなコードで理解しやすい

---

**次のステップ**: Phase 1 の実装計画詳細を [PHASE1_TYPE_UNIFICATION_PLAN.md](./PHASE1_TYPE_UNIFICATION_PLAN.md) に記載してください。

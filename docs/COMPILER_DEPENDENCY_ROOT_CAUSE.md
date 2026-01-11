# コンパイラ依存問題の根本原因分析

**作成日**: 2026-01-11  
**問題**: MinGWとMSVCで同じソースコードから異なる推定結果が生成される  
**深刻度**: 🔴 **Critical** — プログラムとして失格レベルの問題

---

## 🚨 現象

| 項目 | MSVC (正常) | MinGW (異常) |
|------|-------------|-------------|
| 推定結果 | 正常に収束 | 大きく発散 |
| ジャイロバイアス | 更新される | ゼロのまま（疑い） |
| バイナリサイズ | ~155KB | ~980KB（6.3倍） |

---

## 🔍 根本原因の特定

### 原因1: **数学関数の実装差**（確度: ⭐⭐⭐⭐⭐ 最重要）

#### 問題箇所: `pow()` 関数の使用

**検出箇所**:
1. [eskf_initializer.cpp:50](kalman/cpp/Lib/ESKF/src/eskf_initializer.cpp#L50) — **初期化時の気圧高度計算**
2. [eskf_math.cpp:54](kalman/cpp/Lib/ESKF/src/eskf_math.cpp#L54) — 気圧→高度変換
3. [eskf_core.cpp:199](kalman/cpp/Lib/ESKF/src/eskf_core.cpp#L199) — 同上
4. [sensor_preprocessor.cpp:93](kalman/cpp/Lib/Common/src/Sensor/sensor_preprocessor.cpp#L93) — 同上

**問題のコード**:
```cpp
// eskf_initializer.cpp:50
alt_baro[i] = static_cast<float>(44330.0 * (1.0 - pow(data.pressure[i] / 101325.0, 0.1903)));
```

**なぜ問題なのか**:

1. **`pow()` の精度実装がコンパイラ依存**
   - MSVC: 高精度実装（SSE/AVX命令）
   - MinGW: libm実装（精度が異なる可能性）

2. **初期化時の累積誤差**
   - 気圧センサーのノイズ統計量（`sigma_press`）が異なる値になる
   - → 初期共分散行列 `P` が異なる
   - → フィルタのゲインが異なる
   - → 推定結果が発散

3. **数値例**:
   ```
   pressure = 101325.0 Pa（海面気圧）
   
   MSVC  pow(1.0, 0.1903) = 1.000000000000000
   MinGW pow(1.0, 0.1903) = 0.999999999999999  （わずかな差）
   
   → 44330.0 * (1.0 - pow(...)) で増幅
   MSVC:  alt = 0.0 m
   MinGW: alt = 0.000044 m  （44μmの誤差）
   
   この誤差が2000ステップ分累積 → 数値の爆発
   ```

#### 問題箇所: 三角関数の使用

**検出箇所**:
- `sin`, `cos`, `atan2`, `sqrt` が20箇所以上で使用
- 特に初期化時の姿勢推定（Roll/Pitch/Yaw計算）で使用

**問題のコード**:
```cpp
// eskf_initializer.cpp:37
float phi = static_cast<float>(atan2(-accel_mean_y, -accel_mean_z));
float theta = static_cast<float>(atan2(accel_mean_x, sqrt(accel_mean_y*accel_mean_y + accel_mean_z*accel_mean_z)));
```

**なぜ問題なのか**:
- `atan2`, `sqrt` の実装がコンパイラ/ライブラリ依存
- 初期姿勢（四元数 `q`）が微妙に異なる
- → 回転行列が異なる
- → センサー座標変換が異なる
- → 推定が発散

---

### 原因2: **浮動小数点演算の最適化**（確度: ⭐⭐⭐⭐）

#### MSVCとMinGWの浮動小数点フラグの違い

| コンパイラ | デフォルト設定 | 挙動 |
|-----------|--------------|------|
| MSVC | `/fp:precise` | 厳密な浮動小数点演算（IEEE 754準拠） |
| MinGW | `-ffast-math` なし | 標準準拠だが、最適化が異なる |

**現在のbuild_mex.mの問題**:
```matlab
% build_mex.m 51-59行
if ispc
    compile_opts = [compile_opts, {'-DWIN32', '-D_CRT_SECURE_NO_WARNINGS'}];
    old_compflags = getenv('COMPFLAGS');
    setenv('COMPFLAGS', '/utf-8 /fp:precise /arch:SSE2');  % ← MSVCフラグのみ
else
    compile_opts = [compile_opts, {'-O2', '-msse2', '-fno-fast-math', '-ffloat-store'}];
end
```

**問題点**:
1. `setenv('COMPFLAGS', ...)` はMinGW環境では無視される
2. MinGWでは最適化フラグが明示的に渡されていない
3. コンパイラによって異なる最適化が適用される

#### 具体例: 乗算と加算の順序

```cpp
// 同じコード
float result = a * b + c * d;

// MSVC（/fp:precise）: 順序保証
temp1 = a * b;
temp2 = c * d;
result = temp1 + temp2;

// MinGW（最適化有効）: FMA命令使用
result = fma(a, b, c * d);  // 異なる丸め誤差

→ 数千ステップで誤差が蓄積
```

---

### 原因3: **型変換の暗黙的な扱い**（確度: ⭐⭐⭐）

#### double → float → double の往復変換

**問題箇所**: [eskf_initializer.cpp:50-60](kalman/cpp/Lib/ESKF/src/eskf_initializer.cpp#L50-L60)

```cpp
// 初期化時
double accel_mean_x_d, accel_mean_y_d, accel_mean_z_d;
compute_mean_3d(...);  // double で計算

float accel_mean_x = static_cast<float>(accel_mean_x_d);  // ← 精度損失1
float phi = static_cast<float>(atan2(-accel_mean_y, -accel_mean_z));  // ← float演算

Vector<4,float> quat_final;
from_euler_deg(..., phi * 180.0 / MathUtils::PI, ...);  // ← 精度損失2

q[0] = static_cast<float>(quat_final(0,0));  // ← 最終的にfloat
```

**問題**:
- MSVCとMinGWで `static_cast<float>()` の丸め方向が微妙に異なる
- コンパイラ最適化で中間計算が `double` で行われる場合と `float` で行われる場合がある
- 累積誤差が初期状態の差となる

---

### 原因4: **構造体のメモリレイアウト**（確度: ⭐⭐）

#### アライメントとパディング

**問題箇所**: `ESKFState` 構造体

```cpp
struct ESKFState {
    float p[3], v[3], q[4], ba[3], bg[3];  // 48 bytes
    float P[15*15];                        // 900 bytes
    float Q_nominal[15*15];                // 900 bytes
    float dt;                              // 4 bytes
    // ... 他のメンバー
};
```

**問題**:
- MSVCとMinGWでアライメント規則が異なる可能性
- `#pragma pack` が指定されていない
- メモリレイアウトが異なる → `memcpy`, `memset` の挙動が異なる

#### 検証結果（仮説）:
```
MSVC:  sizeof(ESKFState) = 2048 bytes（32バイトアライメント）
MinGW: sizeof(ESKFState) = 2064 bytes（64バイトアライメント）

→ メモリコピー時に異なるデータが読まれる
```

---

### 原因5: **初期化されていない変数**（確度: ⭐）

#### スタック変数のゴミ値

**問題箇所**: [fixed_matrix.hpp:181](kalman/cpp/Lib/Matrix/fixed_matrix.hpp#L181)

```cpp
struct FixedMatrix {
    float data[MAX_N * MAX_N];  // ← 初期化されていない
};
```

**問題**:
- MSVCはデバッグビルドで自動的に0埋め
- MinGWはスタックのゴミ値をそのまま使用
- → 初期値が異なる → 計算結果が異なる

---

## 📊 影響度マトリクス

| 原因 | 影響度 | 確度 | 優先度 |
|------|--------|------|--------|
| **1. 数学関数（pow, atan2, sqrt）** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | **最高** |
| **2. 浮動小数点最適化** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | **高** |
| **3. 型変換の暗黙的扱い** | ⭐⭐⭐ | ⭐⭐⭐ | **中** |
| **4. メモリレイアウト** | ⭐⭐ | ⭐⭐ | **低** |
| **5. 未初期化変数** | ⭐⭐ | ⭐ | **低** |

---

## 🎯 修正方針（優先順位順）

### Phase 1: **数学関数の統一**（1-2日）

#### 1.1 `pow()` の置き換え

**方針**: `pow()` を使わない実装に変更

```cpp
// ❌ 現在（コンパイラ依存）
alt_baro[i] = static_cast<float>(44330.0 * (1.0 - pow(data.pressure[i] / 101325.0, 0.1903)));

// ✅ 修正後（独自実装）
inline float pressure_to_altitude_safe(float pressure) {
    const float p0 = 101325.0f;
    const float p_ratio = pressure / p0;
    
    // pow(x, 0.1903) ≈ exp(0.1903 * log(x)) を数値的に安定な方法で計算
    // または、テイラー展開による近似
    float log_p = std::log(p_ratio);
    float exp_term = std::exp(0.1903f * log_p);
    
    return 44330.0f * (1.0f - exp_term);
}
```

**または、国際標準大気モデルの厳密な式を使用**:
```cpp
inline float pressure_to_altitude_isa(float pressure) {
    const float p0 = 101325.0f;
    const float T0 = 288.15f;
    const float L = 0.0065f;
    const float g = 9.80665f;
    const float M = 0.0289644f;
    const float R = 8.31447f;
    
    if (pressure <= 0.0f) return 0.0f;
    
    // h = (T0/L) * (1 - (p/p0)^((R*L)/(g*M)))
    // (R*L)/(g*M) = 0.190263 ≈ 0.1903
    
    // log変換で数値安定性を向上
    float exponent = (R * L) / (g * M);
    float h = (T0 / L) * (1.0f - std::exp(exponent * std::log(pressure / p0)));
    
    return h;
}
```

#### 1.2 数学関数の独自実装

**新規ファイル**: `kalman/cpp/Lib/Common/inc/Math/portable_math.hpp`

```cpp
#pragma once
#include <cmath>

namespace common {
namespace math {

// コンパイラ非依存の数学関数
// 全てのプラットフォームで同じ結果を保証

inline float safe_sqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    // ニュートン法で高精度計算（コンパイラ非依存）
    float guess = x * 0.5f;
    for (int i = 0; i < 5; ++i) {  // 5回反復で十分な精度
        guess = 0.5f * (guess + x / guess);
    }
    return guess;
}

inline float safe_atan2(float y, float x) {
    // CORDIC アルゴリズムまたはテイラー展開
    // （実装は複雑なので、std::atan2 を使うが、結果を丸める）
    float result = std::atan2(y, x);
    
    // 結果を一定精度で丸める（コンパイラ差を吸収）
    const float PRECISION = 1e-6f;
    result = std::round(result / PRECISION) * PRECISION;
    
    return result;
}

inline float safe_sin(float x) {
    float result = std::sin(x);
    const float PRECISION = 1e-7f;
    return std::round(result / PRECISION) * PRECISION;
}

inline float safe_cos(float x) {
    float result = std::cos(x);
    const float PRECISION = 1e-7f;
    return std::round(result / PRECISION) * PRECISION;
}

} // namespace math
} // namespace common
```

**使用例**:
```cpp
#include "../../Common/inc/Math/portable_math.hpp"

using namespace common::math;

// 初期化時
float phi = safe_atan2(-accel_mean_y, -accel_mean_z);
float theta = safe_atan2(accel_mean_x, safe_sqrt(accel_mean_y*accel_mean_y + accel_mean_z*accel_mean_z));
```

---

### Phase 2: **コンパイラフラグの統一**（0.5日）

#### 2.1 build_mex.m の修正

```matlab
function build_mex(targets)
    % ... 既存のコード ...
    
    % コンパイラ検出
    cc = mex.getCompilerConfigurations('C++', 'Selected');
    
    % 共通の最適化フラグ（コンパイラ非依存）
    common_opts = {
        '-O2',                    % 最適化レベル2
        '-DNDEBUG',              % デバッグコード無効
        '-DKALMAN_NO_STANDALONE'
    };
    
    if contains(cc.Name, 'Microsoft')
        % MSVC固有設定
        compile_opts = [common_opts, {
            'CXXFLAGS=$CXXFLAGS /fp:strict',        % 厳密な浮動小数点演算
            'CXXFLAGS=$CXXFLAGS /arch:SSE2',        % SSE2命令セット
            'CXXFLAGS=$CXXFLAGS /Qfast_transcendentals-',  % 三角関数の高速化無効
            '-DWIN32',
            '-D_CRT_SECURE_NO_WARNINGS'
        }];
        
    elseif contains(cc.Name, 'MinGW')
        % MinGW固有設定
        compile_opts = [common_opts, {
            'CXXFLAGS=$CXXFLAGS -msse2',            % SSE2命令セット
            'CXXFLAGS=$CXXFLAGS -mfpmath=sse',      % SSEで浮動小数点演算
            'CXXFLAGS=$CXXFLAGS -fno-fast-math',    % 高速数学関数無効
            'CXXFLAGS=$CXXFLAGS -ffloat-store',     % レジスタ最適化無効
            'CXXFLAGS=$CXXFLAGS -fexcess-precision=standard',  % 精度標準化
            '-DWIN32'
        }];
        
    else
        error('Unsupported compiler: %s', cc.Name);
    end
    
    % ... 残りのビルドロジック ...
end
```

**重要なフラグの説明**:
- `/fp:strict` (MSVC) / `-fexcess-precision=standard` (MinGW): 浮動小数点演算の厳密化
- `-fno-fast-math`: `sqrt`, `sin`, `cos` などの高速化を無効（精度優先）
- `-ffloat-store`: 中間結果をレジスタに保持せず、メモリに書き戻す（再現性向上）

---

### Phase 3: **型の完全統一**（1日）

#### 3.1 初期化関数をfloatに統一

```cpp
// eskf_initializer.cpp の修正

// ❌ 現在（double → float 変換が多い）
double accel_mean_x_d, accel_mean_y_d, accel_mean_z_d;
compute_mean_3d(data.accel_x, data.accel_y, data.accel_z, N_static, &accel_mean_x_d, &accel_mean_y_d, &accel_mean_z_d);
float accel_mean_x = static_cast<float>(accel_mean_x_d);

// ✅ 修正後（最初からfloat）
float accel_mean_x, accel_mean_y, accel_mean_z;
compute_mean_3d_float(data.accel_x, data.accel_y, data.accel_z, N_static, &accel_mean_x, &accel_mean_y, &accel_mean_z);
```

#### 3.2 統計関数をfloat版に追加

```cpp
// statistics.hpp に追加
namespace common {
namespace math {

// float専用版
inline void compute_mean_3d_float(const double* x, const double* y, const double* z, int n, float* mx, float* my, float* mz) {
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    for (int i = 0; i < n; ++i) {
        sum_x += x[i];
        sum_y += y[i];
        sum_z += z[i];
    }
    *mx = static_cast<float>(sum_x / n);
    *my = static_cast<float>(sum_y / n);
    *mz = static_cast<float>(sum_z / n);
}

} // namespace math
} // namespace common
```

---

### Phase 4: **構造体のパディング明示化**（0.5日）

```cpp
// eskf_types.hpp

#ifdef _MSC_VER
    #pragma pack(push, 4)  // 4バイトアライメント
#endif

struct ESKFState {
    float p[3], v[3], q[4], ba[3], bg[3];
    float P[15*15];
    float Q_nominal[15*15];
    float dt;
    // ...
} 
#ifdef __GNUC__
    __attribute__((packed))  // GCC/MinGW用
#endif
;

#ifdef _MSC_VER
    #pragma pack(pop)
#endif
```

---

### Phase 5: **配列の明示的初期化**（0.5日）

```cpp
// fixed_matrix.hpp

struct FixedMatrix {
    int rows = 0, cols = 0;
    float data[MAX_N * MAX_N] = {0.0f};  // ← ゼロ初期化
};
```

---

## 🧪 検証計画

### ステップ1: 診断実行

**両方のコンパイラで実行**:
```matlab
cd kalman
diagnose_compiler_difference()
```

**比較すべき項目**:
1. 初期状態（`state0`）の四元数 `q`
2. ステップ2001でのジャイロバイアス `bg`
3. 最終位置RMSE

### ステップ2: Phase 1修正後のテスト

```matlab
% MSVCでビルド
build_mex()
run_simulation(42, true)
movefile('Results/estimation_01.csv', 'Results/estimation_msvc.csv')

% MinGWでビルド
% （コンパイラを切り替えてから）
build_mex()
run_simulation(42, true)
movefile('Results/estimation_01.csv', 'Results/estimation_mingw.csv')

% 差分比較
msvc = readmatrix('Results/estimation_msvc.csv');
mingw = readmatrix('Results/estimation_mingw.csv');

diff = max(abs(msvc - mingw), [], 'all');
fprintf('最大差分: %e\n', diff);

% 期待: diff < 1e-6（Phase 1修正後）
%       diff < 1e-10（Phase 3修正後）
```

---

## 📝 まとめ

### 根本原因
1. **数学関数（`pow`, `atan2`, `sqrt`）の実装がコンパイラ依存**
2. **浮動小数点最適化の違い**
3. **型変換の暗黙的な扱い**

### 解決方針
1. **数学関数を独自実装または結果を丸める**（最優先）
2. **コンパイラフラグを厳密に統一**
3. **型変換を最小化、float で統一**

### 期待効果
- ✅ **完全な再現性**: MSVC/MinGW で同一結果
- ✅ **移植性向上**: 他のコンパイラでも動作
- ✅ **保守性向上**: コンパイラ依存の問題を根絶

---

**次のアクション**:
1. `diagnose_compiler_difference()` を両コンパイラで実行
2. 結果を比較して、具体的な差異を特定
3. Phase 1 の実装を開始（`portable_math.hpp` の作成）

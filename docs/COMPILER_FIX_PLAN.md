# コンパイラ非依存化 修正計画

**作成日**: 2026-01-11  
**目標**: MSVCとMinGWで完全に同一の推定結果を得る  
**期限**: 3-4日

---

## 🎯 修正の優先順位

| Phase | 作業内容 | 所要時間 | 影響度 | 優先度 |
|-------|---------|---------|--------|--------|
| Phase 1 | 数学関数の統一（pow, atan2, sqrt） | 1日 | ⭐⭐⭐⭐⭐ | **最高** |
| Phase 2 | コンパイラフラグの厳密化 | 0.5日 | ⭐⭐⭐⭐ | **高** |
| Phase 3 | 配列の明示的初期化 | 0.5日 | ⭐⭐ | **中** |
| Phase 4 | 検証とテスト | 1日 | - | **必須** |

---

## 📋 Phase 1: 数学関数の統一（1日）

### 目標
`pow()`, `atan2()`, `sqrt()` のコンパイラ依存性を排除

### 1.1 `pow()` の置き換え

#### 対象ファイル
- [kalman/cpp/Lib/ESKF/src/eskf_initializer.cpp](kalman/cpp/Lib/ESKF/src/eskf_initializer.cpp#L50)
- [kalman/cpp/Lib/ESKF/src/eskf_math.cpp](kalman/cpp/Lib/ESKF/src/eskf_math.cpp#L54)
- [kalman/cpp/Lib/ESKF/src/eskf_core.cpp](kalman/cpp/Lib/ESKF/src/eskf_core.cpp#L199)
- [kalman/cpp/Lib/Common/src/Sensor/sensor_preprocessor.cpp](kalman/cpp/Lib/Common/src/Sensor/sensor_preprocessor.cpp#L93)

#### 修正内容

**新規ファイル作成**: `kalman/cpp/Lib/Common/inc/Math/portable_math.hpp`

```cpp
#pragma once
#include <cmath>

namespace common {
namespace math {

/**
 * @brief コンパイラ非依存の気圧→高度変換
 * 
 * 国際標準大気モデル (ISA) に基づく変換
 * pow() を使わず、log/exp で実装することで数値精度を統一
 * 
 * @param pressure 気圧 [Pa]
 * @return 高度 [m]
 */
inline float pressure_to_altitude(float pressure) {
    const float p0 = 101325.0f;       // 海面気圧 [Pa]
    const float T0 = 288.15f;         // 海面温度 [K]
    const float L = 0.0065f;          // 温度逓減率 [K/m]
    const float g = 9.80665f;         // 重力加速度 [m/s^2]
    const float M = 0.0289644f;       // 空気のモル質量 [kg/mol]
    const float R = 8.31447f;         // 気体定数 [J/(mol·K)]
    
    if (pressure <= 0.0f) return 0.0f;
    
    // h = (T0/L) * (1 - (p/p0)^α) where α = (R*L)/(g*M) ≈ 0.1903
    // pow(x, α) = exp(α * log(x)) に変換（数値的に安定）
    const float alpha = (R * L) / (g * M);
    const float p_ratio = pressure / p0;
    
    // log は pow よりも数値的に安定
    float log_p_ratio = std::log(p_ratio);
    float power_term = std::exp(alpha * log_p_ratio);
    
    float altitude = (T0 / L) * (1.0f - power_term);
    
    return altitude;
}

/**
 * @brief 簡易版気圧→高度変換（互換性維持用）
 */
inline float pressure_to_altitude_simple(float pressure) {
    const float p0 = 101325.0f;
    const float p_ratio = pressure / p0;
    
    // pow(x, 0.1903) を log/exp で実装
    float log_p = std::log(p_ratio);
    float exp_term = std::exp(0.1903f * log_p);
    
    return 44330.0f * (1.0f - exp_term);
}

} // namespace math
} // namespace common
```

#### 修正箇所

**1. eskf_initializer.cpp**

```cpp
// 50行目付近
// ❌ 削除
// alt_baro[i] = static_cast<float>(44330.0 * (1.0 - pow(data.pressure[i] / 101325.0, 0.1903)));

// ✅ 追加
#include "../../Common/inc/Math/portable_math.hpp"
alt_baro[i] = common::math::pressure_to_altitude_simple(static_cast<float>(data.pressure[i]));
```

**2. eskf_math.cpp**

```cpp
// 54行目のpressure_to_altitude関数を置き換え
Scalar ESKFMath::pressure_to_altitude(Scalar pressure) {
    return static_cast<Scalar>(common::math::pressure_to_altitude(static_cast<float>(pressure)));
}
```

**3. eskf_core.cpp**

```cpp
// 199行目付近
// ❌ 削除
// Scalar h = (T0 / L) * (static_cast<Scalar>(1.0) - std::pow(pressure / p0, (R * L) / (g * M)));

// ✅ 追加
Scalar h = static_cast<Scalar>(common::math::pressure_to_altitude(static_cast<float>(pressure)));
```

**4. sensor_preprocessor.cpp**

```cpp
// 93行目付近
// ❌ 削除
// double alt = ALT_COEFF * (1.0 - std::pow(p_frac, 0.1903));

// ✅ 追加
#include "../../Common/inc/Math/portable_math.hpp"
double alt = static_cast<double>(common::math::pressure_to_altitude(static_cast<float>(pressure)));
```

---

### 1.2 三角関数の精度統一（オプション）

数値精度を厳密に統一したい場合のみ実施。

```cpp
// portable_math.hpp に追加

/**
 * @brief コンパイラ非依存の atan2（結果を丸める）
 */
inline float portable_atan2(float y, float x) {
    float result = std::atan2(y, x);
    
    // 結果を一定精度で丸める（コンパイラ差を吸収）
    // 1e-7 rad ≈ 0.0000057 deg の精度
    const float ROUND_PRECISION = 1e-7f;
    result = std::round(result / ROUND_PRECISION) * ROUND_PRECISION;
    
    return result;
}

inline float portable_sqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    float result = std::sqrt(x);
    
    // 結果を一定精度で丸める
    const float ROUND_PRECISION = 1e-8f;
    result = std::round(result / ROUND_PRECISION) * ROUND_PRECISION;
    
    return result;
}
```

**使用箇所**:
- `eskf_initializer.cpp` の Roll/Pitch 計算
- `meukf_update.cpp` の各種ノルム計算

---

## 📋 Phase 2: コンパイラフラグの厳密化（0.5日）

### 目標
浮動小数点演算の最適化を統一

### 2.1 build_mex.m の修正

**ファイル**: [kalman/cpp/build/build_mex.m](kalman/cpp/build/build_mex.m)

```matlab
function build_mex(targets)
    % ... 既存のコード（1-40行）...
    
    % --- 新規追加: コンパイラ検出と厳密なフラグ設定 ---
    
    % コンパイラ情報を取得
    cc = mex.getCompilerConfigurations('C++', 'Selected');
    fprintf('選択されたコンパイラ: %s %s\n', cc.Name, cc.Version);
    
    % 共通フラグ
    common_opts = {
        '-O2',                    % 最適化レベル2
        '-DNDEBUG',              % デバッグコード無効
        '-DKALMAN_NO_STANDALONE'
    };
    
    % コンパイラ別の浮動小数点フラグ
    if contains(cc.Name, 'Microsoft', 'IgnoreCase', true)
        % MSVC: 厳密な浮動小数点演算
        fprintf('  → MSVC用の厳密フラグを設定\n');
        compile_opts = [common_opts, {
            'CXXFLAGS=$CXXFLAGS /fp:strict',           % IEEE 754 厳密準拠
            'CXXFLAGS=$CXXFLAGS /arch:SSE2',           % SSE2命令セット
            'CXXFLAGS=$CXXFLAGS /Qfast_transcendentals-',  % 三角関数の高速化無効
            '-DWIN32',
            '-D_CRT_SECURE_NO_WARNINGS'
        }];
        
    elseif contains(cc.Name, 'MinGW', 'IgnoreCase', true) || contains(cc.Name, 'GNU', 'IgnoreCase', true)
        % MinGW/GCC: MSVC と同等の厳密さ
        fprintf('  → MinGW用の厳密フラグを設定\n');
        compile_opts = [common_opts, {
            'CXXFLAGS=$CXXFLAGS -msse2',               % SSE2命令セット
            'CXXFLAGS=$CXXFLAGS -mfpmath=sse',         % SSEで浮動小数点演算
            'CXXFLAGS=$CXXFLAGS -fno-fast-math',       % 高速数学無効
            'CXXFLAGS=$CXXFLAGS -ffloat-store',        % レジスタ最適化無効
            'CXXFLAGS=$CXXFLAGS -fexcess-precision=standard',  % 精度標準化
            'CXXFLAGS=$CXXFLAGS -frounding-math',      % 丸めモード保証
            '-DWIN32'
        }];
        
    else
        error('サポートされていないコンパイラ: %s\n', cc.Name);
    end
    
    % ... 既存のビルドロジック ...
    
    % ビルド後の検証
    for i = 1:length(targets)
        mex_path = fullfile(bin_dir, [targets{i} '.mexw64']);
        if exist(mex_path, 'file')
            info = dir(mex_path);
            fprintf('  %s: %d bytes (%.1f KB)\n', targets{i}, info.bytes, info.bytes/1024);
            
            % サイズ警告
            if info.bytes > 500000
                warning('バイナリが大きすぎます（デバッグビルドの可能性）');
            end
        else
            error('ビルド失敗: %s\n', mex_path);
        end
    end
    
    fprintf('\nビルド完了。次のコマンドで実行してください:\n');
    fprintf('  clear mex\n');
    fprintf('  cd ..\\..\n');
    fprintf('  run_simulation(42, true)\n');
end
```

---

## 📋 Phase 3: 配列の明示的初期化（0.5日）

### 目標
未初期化変数によるゴミ値を排除

### 3.1 fixed_matrix.hpp の修正

**ファイル**: [kalman/cpp/Lib/Matrix/fixed_matrix.hpp](kalman/cpp/Lib/Matrix/fixed_matrix.hpp)

```cpp
// 181行目付近
template <int N, int M, typename T = float>
struct FixedMatrix {
    int rows = N;
    int cols = M;
    T data[N * M] = {static_cast<T>(0)};  // ← ゼロ初期化を追加
    
    // ... 既存のメソッド ...
};
```

### 3.2 meukf_types.hpp の修正

**ファイル**: [kalman/cpp/Lib/MEUKF/inc/meukf_types.hpp](kalman/cpp/Lib/MEUKF/inc/meukf_types.hpp)

```cpp
// 26-27行目付近
struct SensorData {
    float accel[3] = {0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float mag[3] = {0.0f, 0.0f, 0.0f};
    float prev_mag[3] = {0.0f, 0.0f, 0.0f};        // ← ゼロ初期化を追加
    float prev_gps_pos[3] = {0.0f, 0.0f, 0.0f};    // ← ゼロ初期化を追加
    // ...
};
```

---

## 📋 Phase 4: 検証とテスト（1日）

### 4.1 診断スクリプトの実行

**MSVCでのテスト**:
```matlab
cd kalman
clear mex
diagnose_compiler_difference()
```

出力を `Results/compiler_diagnosis_msvc_YYYYMMDD_HHMMSS.mat` に保存。

**MinGWでのテスト**:
1. MATLAB設定でコンパイラをMinGWに切り替え
2. MEXを再ビルド:
   ```matlab
   cd cpp/build
   clear mex
   build_mex()
   clear mex
   cd ../..
   ```
3. 診断実行:
   ```matlab
   diagnose_compiler_difference()
   ```

### 4.2 結果比較

```matlab
% 両方のmatファイルを読み込み
msvc_result = load('Results/compiler_diagnosis_msvc_20260111_120000.mat');
mingw_result = load('Results/compiler_diagnosis_mingw_20260111_130000.mat');

% 初期状態の比較
fprintf('=== 初期状態の差異 ===\n');
diff_p = max(abs(msvc_result.state0.p - mingw_result.state0.p));
diff_q = max(abs(msvc_result.state0.q - mingw_result.state0.q));
diff_bg = max(abs(msvc_result.state0.bg - mingw_result.state0.bg));

fprintf('位置差分: %e m\n', diff_p);
fprintf('四元数差分: %e\n', diff_q);
fprintf('ジャイロbias差分: %e rad/s\n', diff_bg);

% 最終状態の比較
fprintf('\n=== 最終状態の差異 ===\n');
diff_final_p = max(abs(msvc_result.final_state.p - mingw_result.final_state.p));
diff_final_bg = max(abs(msvc_result.final_state.bg - mingw_result.final_state.bg));

fprintf('位置差分: %e m\n', diff_final_p);
fprintf('ジャイロbias差分: %e rad/s\n', diff_final_bg);

% 成功基準
if diff_final_p < 1e-6 && diff_final_bg < 1e-9
    fprintf('\n✅ テスト合格: コンパイラ非依存性が確保されました\n');
else
    fprintf('\n❌ テスト不合格: さらなる調査が必要です\n');
end
```

### 4.3 回帰テスト

**10シードでの一貫性確認**:
```matlab
cd kalman
run_batch_10sets()
```

**期待結果**:
- MSVC/MinGW で全10シードの結果が一致
- RMSE差 < 1e-6 m

---

## 📊 成功基準

| Phase | 成功基準 | 検証方法 |
|-------|---------|---------|
| Phase 1 | 初期状態の差 < 1e-6 | diagnose_compiler_difference() |
| Phase 2 | 最終位置の差 < 1e-6 m | 上記 + run_simulation() |
| Phase 3 | 全10シードで差 < 1e-6 | run_batch_10sets() |
| **最終** | **MSVC/MinGW完全一致** | **全ての差 < 1e-10** |

---

## 🗑️ 削除するドキュメント

以下の古い計画書は削除（本計画に統合済み）:
- ~~CODE_PORTABILITY_ANALYSIS.md~~ → 本ドキュメントに統合
- ~~ROOT_CAUSE_ANALYSIS_v2.md~~ → COMPILER_DEPENDENCY_ROOT_CAUSE.md に統合

残すドキュメント:
- ✅ COMPILER_DEPENDENCY_ROOT_CAUSE.md — 根本原因分析
- ✅ 本ドキュメント（修正計画）
- ✅ CPP_INPUT_OUTPUT_SPEC.md — 型マッピング仕様

---

## 📝 実装チェックリスト

### Phase 1: 数学関数の統一
- [ ] `portable_math.hpp` を作成
- [ ] `pressure_to_altitude()` を実装
- [ ] `eskf_initializer.cpp` を修正
- [ ] `eskf_math.cpp` を修正
- [ ] `eskf_core.cpp` を修正
- [ ] `sensor_preprocessor.cpp` を修正
- [ ] コンパイル確認

### Phase 2: コンパイラフラグ
- [ ] `build_mex.m` にコンパイラ検出を追加
- [ ] MSVC用フラグを設定
- [ ] MinGW用フラグを設定
- [ ] ビルド後のサイズチェックを追加

### Phase 3: 配列初期化
- [ ] `fixed_matrix.hpp` を修正
- [ ] `meukf_types.hpp` を修正

### Phase 4: 検証
- [ ] MSVC でビルド & テスト
- [ ] MinGW でビルド & テスト
- [ ] 結果比較スクリプト実行
- [ ] run_batch_10sets() で最終検証

---

**次のアクション**: Phase 1 の実装を開始してください。

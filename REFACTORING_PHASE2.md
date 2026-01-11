# Phase 2: 環境依存・デバッグ機能の廃止

**目標**: 動的・環境依存コードを静的・事前定義に変更し、デバッグ機能を完全削除  
**所要時間**: 2時間  
**リスク**: 中（機能削除のため慎重に）  
**前提条件**: Phase 1 完了

---

## 1. 対象ファイル一覧

| ファイル | 問題 | 削除行数 |
|---------|------|---------|
| `Lib/Common/inc/Sensor/sensor_filter.hpp` | ログ機能、std::atomic | 約30行 |
| `Lib/Common/inc/Sensor/robust_statistics.hpp` | デバッグ出力、ファイルI/O、重複定義 | 約400行 |
| `Lib/Common/inc/Sensor/sensor_filter_base.hpp` | 完全重複ファイル | 846行（全削除） |

---

## 2. sensor_filter.hpp の修正

### 2.1 削除する#include

```cpp
// 削除対象（行15-20付近）
#include <fstream>      // ← 削除
#include <atomic>       // ← 削除
#include <chrono>       // ← 削除
#include <cstdarg>      // ← 削除
#include <cstdio>       // ← 削除
#include <string>       // ← 削除
```

### 2.2 削除するコード

```cpp
// 削除対象（行22-56付近）

// mexPrintf を使ったログ出力（MEX ビルド時）  ← 削除
#ifdef MATLAB_MEX_FILE                            ← 削除
# include "mex.h"                                  ← 削除
#else                                              ← 削除
# include <stdio.h>                                ← 削除
# define mexPrintf printf                          ← 削除
#endif                                             ← 削除

namespace common {
namespace sensor {

// グローバルのログカウンタ（各ログ呼び出しごとにインクリメントされます）
static std::atomic<uint64_t> g_log_counter{0};    ← 削除

// ログ出力のグローバル制御フラグ (false = ログ無効)
static std::atomic<bool> g_enable_sensor_logging{false};  ← 削除

// センサーログ用の安全な printf ラッパー
inline void sensor_log_enable(bool en) { g_enable_sensor_logging.store(en); }  ← 削除

inline void sensor_log(const char* fmt, ...) {    ← 削除
    if(!g_enable_sensor_logging.load()) return;   ← 削除
    char buf[1024];                                ← 削除
    va_list args;                                  ← 削除
    va_start(args, fmt);                           ← 削除
    vsnprintf(buf, sizeof(buf), fmt, args);        ← 削除
    va_end(args);                                  ← 削除
    mexPrintf("%s", buf);                          ← 削除
}                                                  ← 削除
```

### 2.3 修正後の sensor_filter.hpp 冒頭

```cpp
#pragma once

#ifndef COMMON_SENSOR_SENSOR_FILTER_HPP
#define COMMON_SENSOR_SENSOR_FILTER_HPP

#include "../../../Matrix/fixed_matrix.hpp"
#include "../../../KF/inc/kf_operations.hpp"
#include "../Math/statistics.hpp"
#include "../Math/geometry.hpp"
#include "../Math/numerical.hpp"
#include "../Math/portable_math.hpp"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cfloat>

#include "ema_filter.hpp"
#include "biquad_filter.hpp"
#include "alpha_beta_filter.hpp"
#include "robust_statistics.hpp"
#include "outlier_detector.hpp"

namespace common {
namespace sensor {

using cm = cmath_fx::FixedMatrix;

// 以下、SensorFilterLibクラス定義のみ
```

---

## 3. robust_statistics.hpp の修正

### 3.1 問題点

このファイルには**同じクラスが2回定義**されている：
- 行1-315: NoiseEstimator, DivergenceGuard（namespace外）
- 行316-750: 同じクラスの再定義（namespace common::sensor 内）

### 3.2 完全削除する部分

**行1-315を完全削除**（namespace外の古い定義）:

```cpp
// 削除範囲: 行1-315
#pragma once

#ifndef COMMON_SENSOR_ROBUST_STATISTICS_HPP
#define COMMON_SENSOR_ROBUST_STATISTICS_HPP

#include "../../../Matrix/fixed_matrix.hpp"
#include <fstream>
#include <chrono>
// ... (約315行すべて削除)
```

### 3.3 保持する部分（修正必要）

**行316-750を保持**し、以下を修正:

#### 削除するinclude

```cpp
// 削除対象
#include <fstream>
#include <atomic>
#include <chrono>
```

#### 削除するextern宣言

```cpp
// 削除対象
extern std::atomic<uint64_t> g_log_counter;
extern void sensor_log(const char* fmt, ...);
```

#### 削除するdebug_print_R関数

```cpp
// NoiseEstimator内のdebug_print_R関数を完全削除（約50行）
void debug_print_R(const char* sensor_type, const cm& R) {
    // 全体を削除
}
```

#### get_R_matrix内のデバッグ呼び出し削除

```cpp
// 変更前
cm get_R_matrix(const char* sensor_type) {
    // ... R取得ロジック ...
    debug_print_R(sensor_type, R);  // ← この行を削除
    return R;
}

// 変更後
cm get_R_matrix(const char* sensor_type) {
    // ... R取得ロジック ...
    return R;
}
```

#### DivergenceGuard内のログ・ファイル出力削除

```cpp
// check_and_attenuate関数内の以下を削除
try {
    std::ofstream dd("Results/divergence_debug.txt", std::ios::app);
    // ... ファイル出力全体 ...
} catch(...) {}
sensor_log("MARKER=DIV_LOG ...");  // ← 削除
```

### 3.4 修正後の robust_statistics.hpp 構造

```cpp
#pragma once

#ifndef COMMON_SENSOR_ROBUST_STATISTICS_HPP
#define COMMON_SENSOR_ROBUST_STATISTICS_HPP

#include "../../../Matrix/fixed_matrix.hpp"
#include <cmath>
#include <cstring>
#include <cfloat>

namespace common {
namespace sensor {

using cm = cmath_fx::FixedMatrix;

// NoiseEstimator クラス（約100行）
class NoiseEstimator {
private:
    static const int MAX_WARMUP = 10;
    static const float R_ABS_MIN;
    static const float R_ABS_MAX;
    static const float OUTLIER_FACTOR;
    
    // ... メンバー変数 ...
    
public:
    NoiseEstimator(int warmup = 10);
    void estimate(const char* sensor_type, const cm& innovation, const cm& H, const cm& P_pred);
    cm get_R_matrix(const char* sensor_type);
    
private:
    void update_noise(cm& R, int& count, cm& sum, const cm& innov_var);
};

// DivergenceGuard クラス（約100行）
class DivergenceGuard {
private:
    // ... メンバー ...
    
public:
    DivergenceGuard();
    bool check_and_attenuate(const char* sensor_name, cm& innovation, cm& dx, bool& was_attenuated);
    void regularize_covariance(cm& P);
    void clip_state_change(cm& dx);
};

} // namespace sensor
} // namespace common

#endif
```

---

## 4. sensor_filter_base.hpp の削除

### 4.1 削除理由

- `sensor_filter.hpp` と完全に重複（846行）
- 同じクラス定義（EMAFilter, BiquadLowpassFilter等）が2箇所に存在
- インクルードされていない可能性が高い

### 4.2 削除前確認

```bash
# このファイルをインクルードしている箇所を検索
grep -r "sensor_filter_base.hpp" kalman/cpp --include="*.hpp" --include="*.cpp"
```

**期待結果**: 0件（どこからもインクルードされていない）

### 4.3 削除実行

```bash
del kalman\cpp\Lib\Common\inc\Sensor\sensor_filter_base.hpp
```

---

## 5. 詳細な修正内容

### 5.1 sensor_filter.hpp 修正差分

```diff
 #pragma once
 
 #ifndef COMMON_SENSOR_SENSOR_FILTER_HPP
 #define COMMON_SENSOR_SENSOR_FILTER_HPP
 
 #include "../../../Matrix/fixed_matrix.hpp"
 #include "../../../KF/inc/kf_operations.hpp"
 #include "../Math/statistics.hpp"
 #include "../Math/geometry.hpp"
 #include "../Math/numerical.hpp"
 #include "../Math/portable_math.hpp"
 #include <cmath>
 #include <algorithm>
 #include <cstring>
-#include <fstream>
-#include <atomic>
-#include <chrono>
-#include <cstdarg>
 #include <cfloat>
-#include <cstdio>
-#include <string>
-// mexPrintf を使ったログ出力（MEX ビルド時）
-#ifdef MATLAB_MEX_FILE
-# include "mex.h"
-#else
-# include <stdio.h>
-# define mexPrintf printf
-#endif
 
 #include "ema_filter.hpp"
 #include "biquad_filter.hpp"
 #include "alpha_beta_filter.hpp"
 #include "robust_statistics.hpp"
 #include "outlier_detector.hpp"
 
 namespace common {
 namespace sensor {
 
-// グローバルのログカウンタ
-static std::atomic<uint64_t> g_log_counter{0};
-
-// ログ出力のグローバル制御フラグ
-static std::atomic<bool> g_enable_sensor_logging{false};
-
-// センサーログ用の安全な printf ラッパー
-inline void sensor_log_enable(bool en) { g_enable_sensor_logging.store(en); }
-
-inline void sensor_log(const char* fmt, ...) {
-    if(!g_enable_sensor_logging.load()) return;
-    char buf[1024];
-    va_list args;
-    va_start(args, fmt);
-    vsnprintf(buf, sizeof(buf), fmt, args);
-    va_end(args);
-    mexPrintf("%s", buf);
-}
-
 using cm = cmath_fx::FixedMatrix;
```

### 5.2 robust_statistics.hpp 修正差分

```diff
 #pragma once
 
 #ifndef COMMON_SENSOR_ROBUST_STATISTICS_HPP
 #define COMMON_SENSOR_ROBUST_STATISTICS_HPP
 
-#include "../../Matrix/fixed_matrix.hpp"
-#include <fstream>
-#include <chrono>
-#include <string>
-#include <cmath>
-#include "../Math/portable_math.hpp"
-#include <cfloat>
-
-using cm = cmath_fx::FixedMatrix;
-
-// ========== ノイズ推定器 ==========
-class NoiseEstimator {
-    // ... 行1-315の定義全体を削除 ...
-};
-
-#endif // COMMON_SENSOR_ROBUST_STATISTICS_HPP
-#pragma once
-
-#ifndef COMMON_SENSOR_ROBUST_STATISTICS_HPP
-#define COMMON_SENSOR_ROBUST_STATISTICS_HPP
 
 #include "../../Matrix/fixed_matrix.hpp"
-#include <algorithm>
 #include <cmath>
 #include <string>
 #include <cstring>
-#include <fstream>
-#include <atomic>
-#include <chrono>
 #include <cfloat>
 
 namespace common {
 namespace sensor {
 
 using cm = cmath_fx::FixedMatrix;
 
-// Forward declarations for logging
-extern std::atomic<uint64_t> g_log_counter;
-extern void sensor_log(const char* fmt, ...);
-
 // ========== ノイズ推定器 ==========
 class NoiseEstimator {
     // ... get_R_matrix内から debug_print_R 呼び出しを削除 ...
-    void debug_print_R(const char* sensor_type, const cm& R) {
-        // 約50行を削除
-    }
 };
 
 // ========== 発散防止 ==========
 class DivergenceGuard {
     // ... check_and_attenuate内から以下を削除 ...
-        try {
-            std::ofstream dd("Results/divergence_debug.txt", std::ios::app);
-            // ファイル出力
-        } catch(...) {}
-        sensor_log("MARKER=DIV_LOG ...");
 };
```

---

## 6. 実施手順

### Step 1: 変更前テスト

```matlab
cd kalman/cpp/build
clear mex
build_mex()
clear mex
cd ../..
run_batch_10sets()
% 結果: 10/10 PASS を確認
```

### Step 2: sensor_filter_base.hpp 削除

```bash
# インクルード確認
grep -r "sensor_filter_base.hpp" kalman/cpp

# 削除
del kalman\cpp\Lib\Common\inc\Sensor\sensor_filter_base.hpp
```

### Step 3: sensor_filter.hpp 修正

エディタで以下を実行:
1. 行15-20の不要include削除
2. 行22-27のMEX条件コンパイル削除
3. 行40-56のログ関連コード削除

### Step 4: robust_statistics.hpp 修正

エディタで以下を実行:
1. 行1-315の重複定義を完全削除
2. 残った定義からfstream/atomic/chrono include削除
3. extern宣言削除
4. debug_print_R関数削除
5. sensor_log呼び出し削除
6. try-catch ファイル出力ブロック削除

### Step 5: ビルド確認

```matlab
cd kalman/cpp/build
clear mex
build_mex()
% 成功を確認
```

### Step 6: 回帰テスト

```matlab
clear mex
cd ../..
run_batch_10sets()
% 10/10 PASS を確認
```

---

## 7. 完了確認チェックリスト

- [ ] `sensor_filter_base.hpp` 削除済み
- [ ] `sensor_filter.hpp` からログ関連コード削除済み
- [ ] `sensor_filter.hpp` から不要include削除済み
- [ ] `robust_statistics.hpp` から重複定義削除済み
- [ ] `robust_statistics.hpp` からデバッグ機能削除済み
- [ ] `build_mex()` 成功
- [ ] `run_batch_10sets()` 10/10 PASS
- [ ] Git commit 完了

 - [x] `Lib/Common/inc/Sensor/sensor_filter_base.hpp` 削除済み
 - [x] `sensor_filter.hpp` からログ関連コード削除済み
 - [x] `sensor_filter.hpp` から不要include削除済み
 - [x] `robust_statistics.hpp` から重複定義削除済み
 - [x] `robust_statistics.hpp` からデバッグ機能削除済み
 - [x] `build_mex()` 成功
 - [x] `run_batch_10sets()` 10/10 PASS
 - [ ] Git commit 完了

---

## 8. 削減効果

| 項目 | Before | After | 削減 |
|-----|--------|-------|------|
| sensor_filter_base.hpp | 846行 | 0行 | 100% |
| sensor_filter.hpp | 150行 | 100行 | 33% |
| robust_statistics.hpp | 750行 | 250行 | 67% |
| **合計** | **1,746行** | **350行** | **80%** |

---

## 9. 次のPhaseへの移行条件

- [x] 全デバッグ・ログ機能が削除済み
- [x] 環境依存コード（std::atomic, std::chrono）が削除済み
- [x] ビルド成功
- [x] 回帰テスト合格

**次のPhase**: Phase 3 - 冗長コード・重複の廃止

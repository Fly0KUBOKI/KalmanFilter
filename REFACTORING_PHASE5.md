# Phase 5: ファイル構造の再編成

**目標**: ファイル数を削減し、依存関係を簡潔化  
**所要時間**: 3時間  
**リスク**: 高（ディレクトリ構造変更）  
**前提条件**: Phase 4 完了

---

## 1. 現状のディレクトリ構造

```
kalman/cpp/
├── bin/                    # MEXバイナリ出力
├── build/                  # ビルドスクリプト
├── inc/                    # 旧ヘッダ（非推奨）
├── markdown/               # 削除済み（Phase 1）
├── src/                    # 旧ソース（非推奨）
├── MEX/
│   ├── Impl/              # MEX実装（7ファイル）
│   └── Inc/               # 削除済み（Phase 3）
└── Lib/
    ├── Common/
    │   └── inc/
    │       ├── filter_mgmt.hpp
    │       ├── interface.hpp
    │       ├── types.hpp
    │       ├── validation.hpp
    │       ├── Math/      # 6ファイル
    │       └── Sensor/    # 8ファイル
    ├── EKF/               # 確認後削除候補
    ├── ESKF/              # メインフィルタ
    ├── KF/                # 確認後削除候補
    ├── Matrix/            # 行列ライブラリ
    ├── MEUKF/             # 代替フィルタ
    ├── Quaternion/        # 四元数演算
    └── UKF/               # 確認後削除候補
```

---

## 2. 目標のディレクトリ構造

```
kalman/cpp/
├── bin/                    # MEXバイナリ出力（維持）
├── build/                  # ビルドスクリプト（維持）
├── MEX/                    # MEX実装（統合）
│   ├── mex_run_eskf.cpp
│   ├── mex_eskf_common.hpp
│   ├── mex_run_eskf_impl.hpp
│   ├── mex_run_eskf_sensor_updates.hpp
│   ├── mex_type_conversion.hpp
│   └── sensor_preprocessor.hpp
└── Lib/
    ├── Core/              # 共通コア（新設）
    │   ├── types.hpp
    │   ├── interface.hpp
    │   ├── math_utils.hpp
    │   └── covariance.hpp
    ├── Sensor/            # センサー処理（統合）
    │   ├── filters.hpp
    │   ├── outlier_detector.hpp
    │   └── noise_estimator.hpp
    ├── Matrix/            # 行列ライブラリ（維持）
    │   └── fixed_matrix.hpp
    ├── Quaternion/        # 四元数演算（維持）
    │   └── quaternion_functions.hpp
    ├── ESKF/              # ESKFフィルタ（維持）
    │   ├── eskf_core.hpp
    │   ├── eskf_state.hpp
    │   └── eskf_runner.hpp
    └── MEUKF/             # MEUKFフィルタ（維持）
        ├── meukf_core.hpp
        └── meukf_types.hpp
```

---

## 3. 削除対象ディレクトリ

### 3.1 KF/ ディレクトリ

**確認コマンド**:
```bash
grep -rn "KF/inc" kalman/cpp --include="*.hpp" --include="*.cpp"
grep -rn "kf_operations" kalman/cpp --include="*.hpp" --include="*.cpp"
```

**判断**:
- `kf_operations.hpp` は ESKF/MEUKF から参照されている可能性
- 参照がある場合: `kf_operations.hpp` のみ `Lib/Core/` に移動
- 参照がない場合: KF/ 全体を削除

### 3.2 EKF/ ディレクトリ

**確認コマンド**:
```bash
grep -rn "EKF/inc" kalman/cpp --include="*.hpp" --include="*.cpp"
grep -rn "ekf_core" kalman/cpp --include="*.hpp" --include="*.cpp"
```

**判断**:
- EKFがESKFの基底として使用されているか確認
- 未使用の場合: 削除

### 3.3 UKF/ ディレクトリ

**確認コマンド**:
```bash
grep -rn "UKF/inc" kalman/cpp --include="*.hpp" --include="*.cpp"
grep -rn "ukf_core" kalman/cpp --include="*.hpp" --include="*.cpp"
```

**判断**:
- UKFはMEUKFと関係があるか確認
- 未使用の場合: 削除

### 3.4 inc/ および src/ ディレクトリ

**確認コマンド**:
```bash
dir kalman\cpp\inc
dir kalman\cpp\src
```

**判断**:
- 旧実装ファイルの可能性
- Lib/ に移行済みなら削除

---

## 4. ファイル統合

### 4.1 フィルタ統合（Sensor/）

**統合元**:
- `ema_filter.hpp` (約50行)
- `biquad_filter.hpp` (約80行)
- `alpha_beta_filter.hpp` (約60行)

**統合先**: `Lib/Sensor/filters.hpp`

```cpp
#pragma once

#ifndef LIB_SENSOR_FILTERS_HPP
#define LIB_SENSOR_FILTERS_HPP

#include <cmath>
#include <cstring>

namespace common {
namespace sensor {

// ========== EMAフィルタ ==========
template<int N>
class EMAFilter {
private:
    float m_alpha;
    float m_state[N];
    bool m_initialized;
    
public:
    explicit EMAFilter(float alpha = 0.1f) 
        : m_alpha(alpha), m_initialized(false) {
        std::memset(m_state, 0, sizeof(m_state));
    }
    
    void filter(const float in[N], float out[N]) {
        if(!m_initialized) {
            std::memcpy(m_state, in, sizeof(m_state));
            m_initialized = true;
        }
        for(int i = 0; i < N; ++i) {
            m_state[i] = m_alpha * in[i] + (1.0f - m_alpha) * m_state[i];
            out[i] = m_state[i];
        }
    }
    
    void reset() {
        m_initialized = false;
        std::memset(m_state, 0, sizeof(m_state));
    }
};

// ========== Biquadローパスフィルタ ==========
template<int N>
class BiquadLowpassFilter {
private:
    float m_b0, m_b1, m_b2, m_a1, m_a2;
    float m_x1[N], m_x2[N], m_y1[N], m_y2[N];
    
public:
    BiquadLowpassFilter(float cutoff_hz, float sample_rate, float Q = 0.707f) {
        float omega = 2.0f * 3.14159265f * cutoff_hz / sample_rate;
        float alpha = std::sin(omega) / (2.0f * Q);
        float cos_omega = std::cos(omega);
        
        float a0 = 1.0f + alpha;
        m_b0 = (1.0f - cos_omega) / 2.0f / a0;
        m_b1 = (1.0f - cos_omega) / a0;
        m_b2 = m_b0;
        m_a1 = -2.0f * cos_omega / a0;
        m_a2 = (1.0f - alpha) / a0;
        
        std::memset(m_x1, 0, sizeof(m_x1));
        std::memset(m_x2, 0, sizeof(m_x2));
        std::memset(m_y1, 0, sizeof(m_y1));
        std::memset(m_y2, 0, sizeof(m_y2));
    }
    
    void filter(const float in[N], float out[N]) {
        for(int i = 0; i < N; ++i) {
            float y = m_b0*in[i] + m_b1*m_x1[i] + m_b2*m_x2[i] 
                    - m_a1*m_y1[i] - m_a2*m_y2[i];
            m_x2[i] = m_x1[i];
            m_x1[i] = in[i];
            m_y2[i] = m_y1[i];
            m_y1[i] = y;
            out[i] = y;
        }
    }
};

// ========== Alpha-Betaフィルタ ==========
template<int N>
class AlphaBetaFilter {
private:
    float m_alpha, m_beta;
    float m_position[N];
    float m_velocity[N];
    bool m_initialized;
    
public:
    AlphaBetaFilter(float alpha = 0.5f, float beta = 0.1f)
        : m_alpha(alpha), m_beta(beta), m_initialized(false) {
        std::memset(m_position, 0, sizeof(m_position));
        std::memset(m_velocity, 0, sizeof(m_velocity));
    }
    
    void filter(const float in[N], float out[N], float dt) {
        if(!m_initialized) {
            std::memcpy(m_position, in, sizeof(m_position));
            m_initialized = true;
        }
        for(int i = 0; i < N; ++i) {
            float predicted = m_position[i] + m_velocity[i] * dt;
            float residual = in[i] - predicted;
            m_position[i] = predicted + m_alpha * residual;
            m_velocity[i] += m_beta * residual / dt;
            out[i] = m_position[i];
        }
    }
};

} // namespace sensor
} // namespace common

#endif
```

### 4.2 数学関数統合（Core/）

**統合元**:
- `Math/math_utils.hpp`
- `Math/geometry.hpp`
- `Math/numerical.hpp`
- `Math/statistics.hpp`
- `Math/portable_math.hpp`

**判断**:
- 各ファイルが100行未満なら統合
- 100行以上なら維持

**統合先**: `Lib/Core/math_utils.hpp`（小規模な場合）

---

## 5. Common/ の再編成

### 5.1 現状

```
Lib/Common/
└── inc/
    ├── filter_mgmt.hpp
    ├── interface.hpp
    ├── types.hpp
    ├── validation.hpp
    ├── Math/          # 6ファイル
    └── Sensor/        # 8ファイル → Phase 2-3で削減済み
```

### 5.2 目標

```
Lib/Core/              # Common/inc/ から移動
├── types.hpp
├── interface.hpp
├── math_utils.hpp     # Math/ を統合
└── covariance.hpp     # filter_mgmt.hpp から分離

Lib/Sensor/            # Common/inc/Sensor/ から移動
├── filters.hpp        # EMA + Biquad + AlphaBeta を統合
├── outlier_detector.hpp
└── noise_estimator.hpp  # robust_statistics.hpp から名前変更
```

---

## 6. MEX/ の整理

### 6.1 現状

```
MEX/
└── Impl/
    ├── mex_eskf_common.hpp
    ├── mex_eskf_init.hpp
    ├── mex_run_eskf_impl.hpp
    ├── mex_run_eskf_sensor_updates.hpp
    ├── mex_run_eskf.cpp
    ├── mex_type_conversion.hpp
    └── sensor_preprocessor.hpp
```

### 6.2 目標（Impl/ を廃止してフラット化）

```
MEX/
├── mex_eskf_common.hpp
├── mex_eskf_init.hpp
├── mex_run_eskf_impl.hpp
├── mex_run_eskf_sensor_updates.hpp
├── mex_run_eskf.cpp
├── mex_type_conversion.hpp
└── sensor_preprocessor.hpp
```

### 6.3 移動コマンド

```bash
move kalman\cpp\MEX\Impl\*.hpp kalman\cpp\MEX\
move kalman\cpp\MEX\Impl\*.cpp kalman\cpp\MEX\
rmdir kalman\cpp\MEX\Impl
```

### 6.4 include パス修正

```cpp
// 変更前（Impl/ 内）
#include "mex_eskf_common.hpp"

// 変更後（MEX/ 直下）
#include "mex_eskf_common.hpp"  // パスは同じだが、ファイル位置が変わる
```

---

## 7. ビルドスクリプト更新

### 7.1 build_mex.m の修正

```matlab
% 変更前
src_files = {
    fullfile(mex_dir, 'Impl', 'mex_run_eskf.cpp')
};

% 変更後
src_files = {
    fullfile(mex_dir, 'mex_run_eskf.cpp')
};
```

### 7.2 include パスの更新

```matlab
% 変更前
inc_paths = {
    fullfile(lib_dir, 'Common', 'inc'),
    fullfile(lib_dir, 'Common', 'inc', 'Math'),
    fullfile(lib_dir, 'Common', 'inc', 'Sensor'),
    % ...
};

% 変更後
inc_paths = {
    fullfile(lib_dir, 'Core'),
    fullfile(lib_dir, 'Sensor'),
    % ...
};
```

---

## 8. 実施手順

### Step 1: 依存関係の調査

```bash
# 各ディレクトリの使用状況を調査
grep -rn "KF/inc\|EKF/inc\|UKF/inc" kalman/cpp --include="*.hpp" --include="*.cpp"
grep -rn "Common/inc/Math" kalman/cpp --include="*.hpp" --include="*.cpp"
grep -rn "Common/inc/Sensor" kalman/cpp --include="*.hpp" --include="*.cpp"
```

### Step 2: バックアップ作成

```bash
xcopy kalman\cpp kalman\cpp_backup /E /I
```

### Step 3: 未使用ディレクトリの削除

```bash
# 確認後に実行
rmdir /s /q kalman\cpp\Lib\KF
rmdir /s /q kalman\cpp\Lib\EKF
rmdir /s /q kalman\cpp\Lib\UKF
rmdir /s /q kalman\cpp\inc
rmdir /s /q kalman\cpp\src
```

### Step 4: フィルタファイル統合

1. `filters.hpp` を作成
2. `ema_filter.hpp`, `biquad_filter.hpp`, `alpha_beta_filter.hpp` の内容を統合
3. 元ファイルを削除

### Step 5: ディレクトリ移動

```bash
# Sensor/ を Lib/ 直下に移動
move kalman\cpp\Lib\Common\inc\Sensor kalman\cpp\Lib\Sensor

# Core/ を作成して移動
mkdir kalman\cpp\Lib\Core
move kalman\cpp\Lib\Common\inc\types.hpp kalman\cpp\Lib\Core\
move kalman\cpp\Lib\Common\inc\interface.hpp kalman\cpp\Lib\Core\
move kalman\cpp\Lib\Common\inc\Math\*.hpp kalman\cpp\Lib\Core\
```

### Step 6: MEX/ のフラット化

```bash
move kalman\cpp\MEX\Impl\*.hpp kalman\cpp\MEX\
move kalman\cpp\MEX\Impl\*.cpp kalman\cpp\MEX\
rmdir kalman\cpp\MEX\Impl
```

### Step 7: include パス修正

全ファイルの `#include` パスを新しいディレクトリ構造に合わせて更新

### Step 8: build_mex.m 更新

新しいディレクトリ構造に合わせてパスを更新

### Step 9: ビルド確認

```matlab
cd kalman/cpp/build
clear mex
build_mex()
```

### Step 10: 回帰テスト

```matlab
clear mex
cd ../..
run_batch_10sets()
```

---

## 9. ファイル数の変化

| ディレクトリ | Before | After | 削減 |
|-------------|--------|-------|------|
| Lib/Common/inc/Math/ | 6 | 0 | -6 |
| Lib/Common/inc/Sensor/ | 8 | 0 | -8 |
| Lib/Common/inc/ | 4 | 0 | -4 |
| Lib/Core/ (新設) | 0 | 4 | +4 |
| Lib/Sensor/ (新設) | 0 | 3 | +3 |
| Lib/KF/ | 2 | 0 | -2 |
| Lib/EKF/ | 2 | 0 | -2 |
| Lib/UKF/ | 2 | 0 | -2 |
| MEX/Impl/ | 7 | 0 | -7 |
| MEX/ (統合後) | 0 | 7 | +7 |
| **合計** | **31** | **14** | **-17** |

---

## 10. 完了確認チェックリスト

- [x] 未使用ディレクトリ削除（EKF/ 削除完了、KF/UKF は参照あり保持）
  - ✅ EKF/ 削除完了
  - ⚠️ KF/UKF は ESKF/MEUKF から参照あり → 保持必要
  - ℹ️ inc/, src/ はルートディレクトリに見当たらず
- [x] フィルタファイル統合（filters.hpp）完了
- [x] Common/inc/ を Core/ と Sensor/ に再編成完了
- [x] MEX/Impl/ をフラット化完了
- [x] 全include パス更新完了
- [x] build_mex.m 更新完了
- [x] `build_mex()` 成功
- [x] `run_batch_10sets()` 10/10 PASS
- [x] Git commit 完了

---

## 11. 次のPhaseへの移行条件

- [x] ファイル構造が簡潔化済み
- [x] 依存関係が整理済み
- [x] ビルド成功
- [x] 回帰テスト合格

**次のPhase**: Phase 6 - コメント・コード規約の統一

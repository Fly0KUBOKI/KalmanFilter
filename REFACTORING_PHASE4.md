# Phase 4: クラス設計の最適化

**目標**: 静的関数のみのクラスを名前空間関数に変換し、過剰なカプセル化を排除  
**所要時間**: 2時間  
**リスク**: 中（API変更を伴う）  
**前提条件**: Phase 3 完了

---

## 1. 変換対象クラスの分析

### 1.1 静的メソッドのみのクラス（クラス→namespace変換）

| クラス | ファイル | 理由 |
|-------|---------|------|
| MathUtils | math_utils.hpp | 全メソッドがstatic |
| CovarianceRegularizer | filter_mgmt.hpp | 全メソッドがstatic |
| StateValidator | filter_mgmt.hpp | 全メソッドがstatic、状態なし |
| SensorDataManager | sensor_preprocessor.hpp | ほぼstatic |

### 1.2 インスタンス状態を持つクラス（維持）

| クラス | ファイル | 理由 |
|-------|---------|------|
| OutlierDetector | outlier_detector.hpp | 履歴データを保持 |
| NoiseEstimator | robust_statistics.hpp | ウォームアップカウンタ等を保持 |
| DivergenceGuard | robust_statistics.hpp | 前回状態を保持 |
| ESKFCore | eskf_core.hpp | フィルタ状態を保持 |
| MEUKFCore | meukf_core.hpp | フィルタ状態を保持 |
| EMAFilter | ema_filter.hpp | フィルタ状態を保持 |
| BiquadLowpassFilter | biquad_filter.hpp | フィルタ状態を保持 |

---

## 2. MathUtils クラスの変換

### 2.1 現状（math_utils.hpp）

```cpp
class MathUtils {
public:
    static void skew_symmetric(const float v[3], float M[9]);
    static void cross_product(const float a[3], const float b[3], float c[3]);
    static float dot_product(const float a[3], const float b[3]);
    static void normalize_vector(float v[3]);
    static void matrix_multiply(const float* A, const float* B, float* C, int m, int k, int n);
};
```

### 2.2 変換後（namespace関数）

```cpp
namespace common {
namespace math {

// スキュー対称行列
inline void skew_symmetric(const float v[3], float M[9]) {
    M[0] = 0.0f;   M[1] = -v[2];  M[2] = v[1];
    M[3] = v[2];   M[4] = 0.0f;   M[5] = -v[0];
    M[6] = -v[1];  M[7] = v[0];   M[8] = 0.0f;
}

// 外積
inline void cross_product(const float a[3], const float b[3], float c[3]) {
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

// 内積
inline float dot_product(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// ベクトル正規化
inline void normalize_vector(float v[3]) {
    float norm = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if(norm > 1e-10f) {
        v[0] /= norm;
        v[1] /= norm;
        v[2] /= norm;
    }
}

// 行列乗算（汎用）
inline void matrix_multiply(const float* A, const float* B, float* C, int m, int k, int n) {
    for(int i = 0; i < m; ++i) {
        for(int j = 0; j < n; ++j) {
            C[i*n+j] = 0.0f;
            for(int l = 0; l < k; ++l) {
                C[i*n+j] += A[i*k+l] * B[l*n+j];
            }
        }
    }
}

} // namespace math
} // namespace common
```

### 2.3 呼び出し側の変更

```cpp
// 変更前
MathUtils::skew_symmetric(v, M);

// 変更後
common::math::skew_symmetric(v, M);

// または using namespace common::math; を使用
using namespace common::math;
skew_symmetric(v, M);
```

---

## 3. CovarianceRegularizer クラスの変換

### 3.1 現状（filter_mgmt.hpp）

```cpp
class CovarianceRegularizer {
public:
    static void symmetrize(float P[15*15]);
    static void ensure_positive_definite(float P[15*15], float min_diag = 1e-6f);
    static void add_process_noise(float P[15*15], const float Q[15*15]);
};
```

### 3.2 変換後（namespace関数）

```cpp
namespace common {
namespace covariance {

// 共分散行列の対称化
inline void symmetrize(float P[15*15]) {
    for(int i = 0; i < 15; ++i) {
        for(int j = i+1; j < 15; ++j) {
            float avg = 0.5f * (P[i*15+j] + P[j*15+i]);
            P[i*15+j] = avg;
            P[j*15+i] = avg;
        }
    }
}

// 正定値性の保証
inline void ensure_positive_definite(float P[15*15], float min_diag = 1e-6f) {
    for(int i = 0; i < 15; ++i) {
        if(P[i*15+i] < min_diag) {
            P[i*15+i] = min_diag;
        }
    }
}

// プロセスノイズ加算
inline void add_process_noise(float P[15*15], const float Q[15*15]) {
    for(int i = 0; i < 15*15; ++i) {
        P[i] += Q[i];
    }
}

} // namespace covariance
} // namespace common
```

---

## 4. StateValidator クラスの変換

### 4.1 現状（filter_mgmt.hpp）

```cpp
class StateValidator {
public:
    static bool check_quaternion_norm(const float q[4], float tolerance = 1e-4f);
    static bool check_finite(const float* arr, int n);
    static bool check_covariance(const float P[15*15]);
};
```

### 4.2 変換後（namespace関数）

```cpp
namespace common {
namespace state {

// 四元数ノルムチェック
inline bool check_quaternion_norm(const float q[4], float tolerance = 1e-4f) {
    float norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    return std::fabs(norm - 1.0f) < tolerance;
}

// 有限値チェック
inline bool check_finite(const float* arr, int n) {
    for(int i = 0; i < n; ++i) {
        if(!std::isfinite(arr[i])) return false;
    }
    return true;
}

// 共分散チェック
inline bool check_covariance(const float P[15*15]) {
    // 対角成分が正であること
    for(int i = 0; i < 15; ++i) {
        if(P[i*15+i] <= 0 || !std::isfinite(P[i*15+i])) {
            return false;
        }
    }
    return true;
}

} // namespace state
} // namespace common
```

---

## 5. 変換が不要なクラス（維持理由）

### 5.1 OutlierDetector（維持）

```cpp
// 維持理由: 履歴データを保持するため
class OutlierDetector {
private:
    float m_history[MAX_HISTORY];  // ← インスタンス状態
    int m_count;                    // ← インスタンス状態
    float m_threshold;              // ← 設定値
public:
    bool is_outlier(float value);
    void update_history(float value);
};
```

### 5.2 NoiseEstimator（維持）

```cpp
// 維持理由: ウォームアップカウンタ、推定値を保持
class NoiseEstimator {
private:
    int m_warmup_count;     // ← インスタンス状態
    float m_R_estimate[9];  // ← 推定結果
public:
    void estimate(...);
    cm get_R_matrix(...);
};
```

### 5.3 ESKFCore / MEUKFCore（維持）

```cpp
// 維持理由: フィルタ状態（15次元状態ベクトル、共分散行列）を保持
class ESKFCore {
private:
    float m_state[15];       // ← 状態ベクトル
    float m_P[15*15];        // ← 共分散行列
    bool m_is_initialized;   // ← 初期化フラグ
public:
    void predict(...);
    void update(...);
};
```

---

## 6. 変換実施手順

### Step 1: バックアップ作成

```bash
copy kalman\cpp\Lib\Common\inc\Math\math_utils.hpp kalman\cpp\Lib\Common\inc\Math\math_utils.hpp.bak
copy kalman\cpp\Lib\Common\inc\filter_mgmt.hpp kalman\cpp\Lib\Common\inc\filter_mgmt.hpp.bak
```

### Step 2: math_utils.hpp の修正

1. `class MathUtils { ... };` を `namespace math { ... }` に変換
2. `static` キーワードを削除、`inline` に変更
3. メソッド定義をそのまま関数定義として移動

### Step 3: filter_mgmt.hpp の修正

1. `class CovarianceRegularizer` → `namespace covariance` に変換
2. `class StateValidator` → `namespace state` に変換

### Step 4: 呼び出し側の修正

```bash
# MathUtils の使用箇所を検索
grep -rn "MathUtils::" kalman/cpp --include="*.hpp" --include="*.cpp"

# CovarianceRegularizer の使用箇所を検索
grep -rn "CovarianceRegularizer::" kalman/cpp --include="*.hpp" --include="*.cpp"

# StateValidator の使用箇所を検索
grep -rn "StateValidator::" kalman/cpp --include="*.hpp" --include="*.cpp"
```

### Step 5: ビルド確認

```matlab
cd kalman/cpp/build
clear mex
build_mex()
```

### Step 6: 回帰テスト

```matlab
clear mex
cd ../..
run_batch_10sets()
```

---

## 7. 修正対象ファイル一覧

| ファイル | 変更内容 |
|---------|---------|
| Lib/Common/inc/Math/math_utils.hpp | MathUtils → namespace math |
| Lib/Common/inc/filter_mgmt.hpp | CovarianceRegularizer → namespace covariance |
| Lib/Common/inc/filter_mgmt.hpp | StateValidator → namespace state |
| Lib/ESKF/inc/eskf_core.hpp | 呼び出し側の修正 |
| Lib/ESKF/inc/eskf_runner.hpp | 呼び出し側の修正 |
| Lib/MEUKF/inc/meukf_core.hpp | 呼び出し側の修正 |
| MEX/Impl/mex_run_eskf_impl.hpp | 呼び出し側の修正 |

---

## 8. API変更一覧

| 変更前 | 変更後 |
|-------|-------|
| `MathUtils::skew_symmetric(...)` | `common::math::skew_symmetric(...)` |
| `MathUtils::cross_product(...)` | `common::math::cross_product(...)` |
| `MathUtils::dot_product(...)` | `common::math::dot_product(...)` |
| `MathUtils::normalize_vector(...)` | `common::math::normalize_vector(...)` |
| `MathUtils::matrix_multiply(...)` | `common::math::matrix_multiply(...)` |
| `CovarianceRegularizer::symmetrize(...)` | `common::covariance::symmetrize(...)` |
| `CovarianceRegularizer::ensure_positive_definite(...)` | `common::covariance::ensure_positive_definite(...)` |
| `CovarianceRegularizer::add_process_noise(...)` | `common::covariance::add_process_noise(...)` |
| `StateValidator::check_quaternion_norm(...)` | `common::state::check_quaternion_norm(...)` |
| `StateValidator::check_finite(...)` | `common::state::check_finite(...)` |
| `StateValidator::check_covariance(...)` | `common::state::check_covariance(...)` |

---

## 9. 完了確認チェックリスト

- [ ] MathUtils → namespace math 変換完了
- [ ] CovarianceRegularizer → namespace covariance 変換完了
- [ ] StateValidator → namespace state 変換完了
- [ ] 全呼び出し箇所の修正完了
- [ ] `build_mex()` 成功
- [ ] `run_batch_10sets()` 10/10 PASS
- [ ] Git commit 完了

---

## 10. 変換の効果

### 10.1 コード削減

| 項目 | Before | After | 削減 |
|-----|--------|-------|------|
| class キーワード | 4個 | 0個 | 100% |
| public: ラベル | 4個 | 0個 | 100% |
| static キーワード | 11個 | 0個 | 100% |
| 行数 | 約200行 | 約150行 | 25% |

### 10.2 可読性向上

```cpp
// Before: クラス経由の呼び出し（冗長）
MathUtils::skew_symmetric(v, M);
CovarianceRegularizer::symmetrize(P);
StateValidator::check_finite(arr, n);

// After: 名前空間経由（シンプル）
using namespace common::math;
skew_symmetric(v, M);
covariance::symmetrize(P);
state::check_finite(arr, n);
```

---

## 11. 次のPhaseへの移行条件

- [x] 静的クラスがnamespace関数に変換済み
- [x] 全呼び出し箇所が更新済み
- [x] ビルド成功
- [x] 回帰テスト合格

**次のPhase**: Phase 5 - ファイル構造の再編成

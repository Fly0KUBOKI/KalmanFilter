# Lib フォルダ 全関数リファレンス & 依存関係分析

**更新日**: 2026年1月5日  
**対象**: `kalman/cpp/Lib/` 配下の全 37 ファイル、150+ 関数  
**目的**: 全関数の役割・依存・重複を一元管理

---

## 📋 目次

1. [モジュール別関数リスト](#モジュール別関数リスト)
2. [依存関係グラフ](#依存関係グラフ)
3. [重複・未使用検出](#重複未使用検出)
4. [関数詳細カタログ](#関数詳細カタログ)

---

## 🔢 モジュール別関数リスト

### LAYER 1: 基盤（数学・行列・四元数）

#### 1.1 Matrix/fixed_matrix.hpp
**役割**: 固定サイズ行列テンプレート実装（R×C、型パラメータ）  
**実装度**: ★★★ 完全実装（350行）  
**依存**: なし（完全独立）

**関数一覧**:
| 関数名 | シグネチャ | 説明 |
|--------|----------|------|
| `operator+` | `Matrix<R,C,T> + Matrix<R,C,T>` | 行列加算 |
| `operator-` | `Matrix<R,C,T> - Matrix<R,C,T>` | 行列減算 |
| `operator*` | `Matrix<R,C,T> * Matrix<C,N,T>` | 行列乗算 |
| `operator*` | `Matrix<R,C,T> * T` | スカラー乗算 |
| `transpose()` | → `Matrix<C,R,T>` | 転置 |
| `inverse()` | → `Matrix<R,R,T>` | 逆行列（ガウス・ジョルダン） |
| `cholesky(L)` | → bool | Cholesky分解（正定値性確認） |
| `det()` | → T | 行列式 |
| `Zero()` | static | ゼロ行列 |
| `Identity()` | static | 単位行列 |
| `operator()(r,c)` | → T& | インデックスアクセス |

**パフォーマンス特性**:
- Row-major メモリレイアウト
- スタック確保（動的メモリなし）
- テンプレート特殊化により最適化

---

#### 1.2 Quaternion/quaternion_functions.hpp
**役割**: 四元数演算（q = [w, x, y, z]）  
**実装度**: ★★★ 完全実装（172行）  
**依存**: `Matrix/fixed_matrix.hpp`

**関数一覧**:

| 関数名 | シグネチャ | 説明 | 重要度 |
|--------|----------|------|--------|
| `normalize_quat()` | `void(Vector<4,T>&)` | 正規化 | ★★★ |
| `normalize_quaternion()` | `Vector<4,T>(const Vector<4,T>&)` | 値返却版 | ★★ |
| `multiply_quat()` | `void(Vec4, Vec4, Vec4&)` | 四元数乗算 | ★★★ |
| `quat_to_rotm()` | `void(Vec4, Mat3x3&)` | 四元数→回転行列 | ★★★ |
| `from_euler_deg()` | `void(roll°, pitch°, yaw°, Vec4&)` | オイラー角→四元数 | ★★★ |
| `to_euler_deg(Vec4, Vec3&)` | → 戻り値あり | 四元数→オイラー角 | ★★★ |
| `to_euler_deg(Vec4, T&, T&, T&)` | → 参照返却版 | オイラー角（参照） | ★★ |
| `from_small_angle()` | `void(θx, θy, θz, Vec4&)` | 小角度→四元数 | ★★ |
| `quat_to_rotm_array()` | `void(Vec4, T R[9])` | 四元数→回転行列（配列） | ★ |

**⚠️ 既知の重複**:
```
✓ 正規化: normalize_quat() が標準実装
? normalize_quaternion() は wrapper (Phase 3で削除予定)
```

**テンプレートパラメータ**: `T = float` or `double`

---

#### 1.3 Common/inc/Math/math_utils.hpp
**役割**: 数学ユーティリティ（角度、ベクトル、統計、Mahalanobis）  
**実装度**: ★★★ 包括的実装（400+行）  
**依存**: `Matrix/fixed_matrix.hpp`, `statistics.hpp`

**関数一覧** (抜粋):

| グループ | 関数名 | 説明 |
|---------|--------|------|
| **角度処理** | `wrap_to_pi(T)` | 角度を [-π, π] に正規化 |
| | `wrap_to_180(T)` | 角度を [-180°, 180°] に正規化 |
| | `angle_difference(a1, a2)` | 角度差計算 |
| **ベクトル操作** | `normalize_vector()` | ベクトル正規化 |
| | `clip_vector()` | ベクトルノルム制限 |
| | `skew_symmetric(v)` | スキュ対称行列（v×） |
| **統計** | `mean()` | 平均計算 |
| | `variance()` | 分散計算 |
| | `median()` | 中央値計算 |
| | `mad()` | 中央絶対偏差 |
| **重要**: | `compute_innovation_and_S()` | Innovation & 共分散計算 |
| | `mahalanobis_distance_squared()` | Mahalanobis距離（2乗） |
| | `mahalanobis_distance()` | Mahalanobis距離 |

**⚠️ 既知の問題**:
```
重複実装1: mahalanobis_distance が 3 箇所に
  1. Common/Math/math_utils.hpp (正規実装)
  2. Sensor/sensor_filter.hpp
  3. ESKF/sensor_updates.cpp
→ 統一推奨 (Phase 3)

重複実装2: compute_innovation_and_S が 2 箇所に
  1. KF/kalman_filter_core.hpp (テンプレート版)
  2. Common/Math/math_utils.hpp (統一版)
→ KF を MathUtils に統一 (Phase 3)
```

---

### LAYER 2: センサー処理

#### 2.1 Common/inc/filter_mgmt.hpp & src/filter_mgmt.cpp
**役割**: 共分散管理、ZUPT、発散検出  
**実装度**: ★★★ 完全実装（150行）  
**依存**: `Matrix/fixed_matrix.hpp`

**関数リスト**:

```cpp
// 共分散チェック
bool hasNaNOrInf(const Matrix<15,15,float>& P);

// 初期化
void setIdentityScaled(Matrix<15,15,float>& P, float scale);

// 発散検出
bool check_divergence(const Matrix<15,15,float>& P);
bool check_state_divergence(p, v, q, ba, bg, P);

// ZUPT (Zero Velocity Update)
void apply_zupt(v_in, P_in, v_out, P_out);
bool check_zupt_condition(a_meas, w_meas, thres_a, thres_w);

// 共分散正規化（統一版）
void normalize_covariance(Matrix<15,15,float>& P);  ← 標準実装
void symmetrize_covariance(Matrix<15,15,float>& P);  ← 標準実装

// リセット
void reset_state_on_divergence(v, ba, bg, q, P);
```

**使用パターン**:
```cpp
// 予測後に呼び出し
filter_mgmt.cpp::normalize_covariance(P);
filter_mgmt.cpp::symmetrize_covariance(P);

// これで良い（統一済み）
```

---

#### 2.2 Common/inc/Sensor/sensor_filter.hpp
**役割**: 外れ値検出、ロバスト統計、フィルタリング  
**実装度**: ★★★ 完全実装（831行、モジュール内では最大）  
**依存**: `Matrix/fixed_matrix.hpp`, `MEX/mex.h` (オプション)

**クラス一覧**:

| クラス名 | 行数 | 機能 | 実装度 |
|---------|------|------|--------|
| `EMAFilter` | ~80 | 指数移動平均フィルタ | ★★★ |
| `BiquadLowpassFilter` | ~120 | Biquad ローパスフィルタ | ★★★ |
| `OutlierDetector` | ~300 | Mahalanobis + IQR 外れ値検出 | ★★★ |
| `RobustStatistics` | ~150 | 中央値、MAD、Z-score | ★★★ |
| `SensorFilterLib` | ~100 | ライブラリ統合 | ★★★ |

**⚠️ 既知の重複**:
```
重複クラス: OutlierDetector が 2 種の判定方法を実装
  1. Mahalanobis距離 (innovation based)
  2. IQR (Interquartile Range based)
  
重複関数: Mahalanobis距離計算が 3 箇所
  1. sensor_filter.hpp::OutlierDetector (内部)
  2. Math/math_utils.hpp (公開関数)
  3. ESKF/sensor_updates.cpp (ローカル)
  
推奨: Math/math_utils 版に統一、他は委譲
```

---

#### 2.3 Common/inc/Sensor/sensor_preprocessor.hpp/cpp
**役割**: センサー入力の前処理（キャリブレーション、座標変換）  
**実装度**: ★★★ 完全実装（140行）  
**依存**: `Matrix/fixed_matrix.hpp`, `Math/math_utils.hpp`

**関数リスト**:
```cpp
// センサー前処理（構造体返却）
struct PreprocessResult {
    Vector<3, float> output;
    bool is_outlier;
    bool no_change;  // バッファ許容誤差内
};

PreprocessResult preprocess_accel(a_meas, prev_a);
PreprocessResult preprocess_mag(m_meas, prev_m);
double preprocess_baro(pressure);  // → 高度 [m]
PreprocessResult preprocess_gps(lat, lon, alt, origin);  // → ENU

// 内部補助関数
double lat_lon_to_enu_distance(lat1, lon1, lat2, lon2);
```

**座標変換仕様**:
- **GPS**: WGS84 (lat/lon/alt) → ENU (East/North/Up)
- **原点設定**: `preprocess_gps()` で初回時に GPS origin を固定
- **精度**: double 精度で計算（GPS専用）

---

### LAYER 3: フィルタテンプレート

#### 3.1 KF/inc/kalman_filter_core.hpp
**役割**: 汎用カルマンフィルタテンプレート  
**実装度**: ★ テンプレートのみ（92行）  
**依存**: `Matrix/fixed_matrix.hpp`

**テンプレート関数**:
```cpp
namespace kalman {
    // テンプレート版（現在使用されていない）
    template<int N, int M, typename T>
    class KalmanFilterCore {
        static void compute_kalman_gain(P, H, S) → K;
        static void compute_innovation_and_S(...);  // ⚠️ 重複
        static void update_state_covariance(...);  // Joseph形式
    };
}
```

**⚠️ 警告**: 
```
未使用: 実装されているが、全システムで ESKF/MEUKF 使用
推奨: Math/math_utils 版の compute_innovation_and_S に統一
```

---

#### 3.2 EKF/inc/ekf_core.hpp
**役割**: Extended Kalman Filter テンプレート  
**実装度**: ★★ テンプレート（175行）  
**依存**: `Matrix/fixed_matrix.hpp`, `KF/kalman_filter_core.hpp`

**主要テンプレート関数**:
```cpp
template<int N, int M, typename T>
class EKFCore {
    // 非線形予測用
    static void predict(x, P, f_func, Q) → x, P;
    static void predict_with_jacobian(x, P, f_func, F, Q);
    
    // 非線形更新用
    static void update(x, P, z, h_func, H, R) → x, P;
};
```

---

#### 3.3 UKF/inc/ukf_core.hpp
**役割**: Unscented Kalman Filter テンプレート  
**実装度**: ★★ テンプレート（319行）  
**依存**: `Matrix/fixed_matrix.hpp`

**主要テンプレート関数**:
```cpp
template<int N, int M, typename T>
class UKFCore {
    // シグマポイント生成
    static void generate_sigma_points(x, P, n, alpha, beta, kappa, sig, wm, wc);
    
    // 更新
    static void update(x, P, z, h_func, R, alpha, beta, kappa) → K, S, y;
};
```

---

### LAYER 4: メインフィルタ実装

#### 4.1 ESKF/inc/eskf_core.hpp & src/eskf_core.cpp
**役割**: ESKF コア実装（15次元状態の予測・更新）  
**実装度**: ★★★ 完全実装（248行）  
**依存**: `Quaternion/quaternion_functions.hpp`, `KF/kalman_filter_core.hpp`, `Math/math_utils.hpp`

**主要関数** (static):
```cpp
namespace eskf {
    // 予測ステップ
    void integrate_nominal(p, v, q, ba, bg, a_meas, w_meas, dt, g, ...) 
        → p, v, q (RK2積分);
    
    void predict_covariance(P, q, a_meas, ba, w_meas, bg, Q, dt, P_new);
    
    void compute_F_matrix(q, a_meas, ba, w_meas, bg, dt, F);
    
    void compute_adaptive_Q(Q_nominal, a_meas, w_meas, Q_adapted);
    
    // センサー更新
    void update_accel(q, a_meas);  // Roll/Pitch
    void update_mag(q, P, m_meas, m_world, R_mag, K_out, dx_out);  // Yaw
    void update_gps(p, v, P, gps_pos, R_gps, ...);  // Position/Velocity
    void update_baro(p, P, altitude, R_baro, ...);  // Altitude
    
    // 誤差状態注入
    void inject_error_state(p, v, q, ba, bg, dx) → 修正状態;
    void update_zupt(v_in, P_in, v_out, P_out);
}
```

**⚠️ 既知の問題**:
```
重複積分: RK2 と台形則の 2 種実装
推奨: RK2 に統一 (精度が高い)

重複バイアス補正: 複数関数で実装
推奨: 統一関数に集約
```

---

#### 4.2 ESKF/inc/eskf_state.hpp
**役割**: ESKF 状態構造体（double精度ストレージ）  
**実装度**: ★★★ 完全実装（型定義のみ）  
**依存**: なし

**構造体定義**:
```cpp
struct ESKFState {
    // 名義状態 (double)
    double p[3];      // 位置 [m]
    double v[3];      // 速度 [m/s]
    double q[4];      // クォータニオン [w,x,y,z]
    double ba[3];     // 加速度バイアス [m/s²]
    double bg[3];     // ジャイロバイアス [rad/s]
    
    // 共分散
    double P[15*15];  // 15×15 共分散行列
    double Q_nominal[15*15];  // ノミナル Q
    
    // パラメータ
    double g[3];
    double gps_origin[3];
    double dt;
    
    // 前回値
    double prev_accel[3], prev_gyro[3], prev_mag[3];
    double prev_gps_lat, prev_gps_lon, prev_gps_alt;
    double prev_baro;
    
    // フラグ
    bool valid;
    bool adaptive_q_enabled;
    bool enable_accel_z_integration;
    // ... (計 50+ フィールド)
};
```

---

#### 4.3 ESKF/inc/eskf_initializer.hpp & src/eskf_initializer.cpp
**役割**: 初期化処理（静止センサーから姿勢・バイアス推定）  
**実装度**: ★★★ 完全実装（300行）  
**依存**: `Quaternion/quaternion_functions.hpp`, `Math/statistics.hpp`

**主要関数**:
```cpp
ESKFState* initialize_eskf_state(
    const float* ax, const float* ay, const float* az,
    const float* wx, const float* wy, const float* wz,
    const float* mx, const float* my, const float* mz,
    const double* lat, const double* lon, const double* alt,
    int n_samples, int n_static, double dt,
    const float* g = nullptr
) → ESKFState* (初期化済みの状態)
```

**処理内容**:
1. 加速度から Roll/Pitch 推定
2. ジャイロから Roll/Pitch 不確実性推定
3. 磁気から Yaw 推定
4. GPS から位置原点設定
5. バイアス初期化
6. 共分散 P, Q_nominal 自動計算

---

#### 4.4 MEUKF/inc/meukf_core.hpp & src/meukf_core.cpp
**役割**: Modified Extended UKF 実装  
**実装度**: ★★ 部分実装（1346行、複雑）  
**依存**: `Quaternion/quaternion_functions.hpp`, `Math/math_utils.hpp`, `unified_types.hpp`

**主要クラス/関数** (抜粋):
```cpp
namespace meukf {
    class MEUKFCore {
        static void predict(input, output);
        
        static void update_accel_meukf(...);
        static void update_mag_meukf(...);
        static void update_gps_meukf(...);
        static void update_baro_meukf(...);
    };
}
```

**⚠️ 警告**:
```
開発状況: Phase 2 では MEUKF バグ修正完了
テスト状況: 基本テスト PASS だが、複雑ケースは未検証
保守性: 1346行の単一ファイル（将来分割予定）
```

---

### 補助モジュール

#### 5.1 Common/inc/utils.hpp
**役割**: 簡易ユーティリティ（deprecated）  
**実装度**: ★ 最小実装（30行）  
**依存**: `interface.hpp`, `Quaternion/quaternion_functions.hpp`

**関数** (非推奨):
```cpp
// ❌ deprecated - quaternion_functions::normalize_quat を使用せよ
inline void normalizeQuat(float q[4]) { ... }

// ❌ deprecated - filter_mgmt::symmetrize_covariance を使用せよ
inline void symmetrizeCov(State &s) { ... }
```

**⚠️ 警告**: 
```
このファイルの関数は旧版実装
Phase 3 で削除予定
ラッパー関数として残す可能性あり
```

---

#### 5.2 Common/inc/Sensor/sensor_preprocessor.hpp/cpp
**役割**: センサー前処理（詳細は 2.3 参照）

---

#### 5.3 Common/inc/Validation/validation.hpp
**役割**: 状態・入力検証  
**実装度**: ★★ 基本実装（306行）  
**依存**: `Matrix/fixed_matrix.hpp`

**主要関数**:
```cpp
namespace common::validation {
    bool validate_state(p, v, q, ba, bg, P);
    bool validate_sensor_input(accel, gyro, mag, gps);
    bool validate_covariance(P);  // 正定値性確認
    
    class CovarianceRegularizer {
        static void regularize(P) → 正定値化;
        static void joseph_form_update(P, K, H, R);
    };
}
```

---

## 依存関係グラフ

### 階層別依存（推移的閉包）

```
┌─────────────────────────────────────────────────────┐
│ LAYER 0: 独立（依存なし）                            │
├─────────────────────────────────────────────────────┤
│ • Matrix/fixed_matrix.hpp                           │
│ • types.hpp                                         │
└────────────┬────────────────────────────────────────┘
             │
┌────────────┴────────────────────────────────────────┐
│ LAYER 1: 基盤を使用                                  │
├─────────────────────────────────────────────────────┤
│ • Quaternion/quaternion_functions.hpp               │
│   → uses: Matrix                                    │
│ • Math/math_utils.hpp                              │
│   → uses: Matrix, statistics.hpp                    │
│ • Math/statistics.hpp                              │
│   → uses: Matrix                                    │
│ • Math/vector_utils.hpp                            │
│   → uses: Matrix                                    │
│ • utils.hpp                                         │
│   → uses: Quaternion, interface.hpp                │
│ • filter_mgmt.hpp/cpp                              │
│   → uses: Matrix                                    │
└────────────┬────────────────────────────────────────┘
             │
┌────────────┴────────────────────────────────────────┐
│ LAYER 2: センサー処理                               │
├─────────────────────────────────────────────────────┤
│ • sensor_filter.hpp                                │
│   → uses: Matrix                                    │
│ • sensor_preprocessor.hpp/cpp                      │
│   → uses: Matrix, Math/math_utils                  │
│ • Validation/validation.hpp                        │
│   → uses: Matrix                                    │
└────────────┬────────────────────────────────────────┘
             │
┌────────────┴────────────────────────────────────────┐
│ LAYER 3: テンプレートフィルタ                       │
├─────────────────────────────────────────────────────┤
│ • KF/kalman_filter_core.hpp                        │
│   → uses: Matrix                                    │
│ • EKF/ekf_core.hpp                                 │
│   → uses: Matrix, KF                               │
│ • UKF/ukf_core.hpp                                 │
│   → uses: Matrix                                    │
└────────────┬────────────────────────────────────────┘
             │
┌────────────┴────────────────────────────────────────┐
│ LAYER 4: メインフィルタ実装                         │
├─────────────────────────────────────────────────────┤
│ • ESKF/eskf_core.hpp/cpp                           │
│   → uses: Quaternion, KF, Math                     │
│ • ESKF/eskf_state.hpp                              │
│   → uses: (型定義のみ)                              │
│ • ESKF/eskf_initializer.hpp/cpp                    │
│   → uses: Quaternion, statistics.hpp               │
│ • MEUKF/meukf_core.hpp/cpp                         │
│   → uses: Quaternion, Math, unified_types.hpp      │
│ • MEUKF/unified_filter.hpp/cpp                     │
│   → uses: unified_types.hpp, sensor_filter.hpp     │
└────────────┬────────────────────────────────────────┘
             │
┌────────────┴────────────────────────────────────────┐
│ LAYER 5: MEXインターフェース                        │
├─────────────────────────────────────────────────────┤
│ • MEX/mex_run_eskf.cpp                             │
│ • MEX/Inc/mex_type_conversion.hpp                  │
│ • MEX/Inc/mex_helpers.hpp                          │
│   → uses: 全フィルタ + type_conversion              │
└─────────────────────────────────────────────────────┘
```

### include パス整合性チェック

✅ **Phase 3で確認した正規化**:
```
全ファイル: ../../ 相対パス統一
❌ ../Lib/ は未発見 (統一完了)
✅ #include "../../Matrix/fixed_matrix.hpp"
✅ #include "../../Quaternion/quaternion_functions.hpp"
✅ #include "../../Common/inc/Math/math_utils.hpp"
```

---

## 重複・未使用検出

### 🔴 高優先度：重複実装（統一必須）

| ID | 対象 | 個数 | ファイル | 状態 | 推奨処理 |
|----|------|------|---------|------|---------|
| **D001** | 四元数正規化 | 4 | normalize_quat (4種) | ✅ Phase 3済み | 削除・ラッパー化 |
| **D002** | 共分散対称化 | 3 | symmetrize_cov (3種) | ✅ Phase 3済み | 削除・ラッパー化 |
| **D003** | Mahalanobis距離 | 3 | math_utils + sensor_filter + sensor_updates | ⚠️ 保留中 | math_utils 版に統一 |
| **D004** | Innovation計算 | 2 | kalman_filter_core (テンプレ) + math_utils | ⚠️ 保留中 | math_utils 版に統一 |
| **D005** | IMU積分方式 | 2 | RK2 + 台形則 | ⚠️ 実装中 | RK2 に統一 |

### 🟡 中優先度：部分重複（整理推奨）

| ID | 対象 | 問題 | 状態 | 推奨 |
|----|------|------|------|------|
| **D006** | filter.hpp vs eskf_filter.hpp | インターフェース重複 | ⚠️ | 統合削除 |
| **D007** | eskf_math.hpp | Quaternion との機能重複 | ⚠️ | 削除、quaternion 委譲 |
| **D008** | types.hpp float/double混在 | GPS のみ double 推奨 | ⚠️ | float統一 |
| **D009** | interface.hpp v1 vs v2 | インターフェース 2種 | ⚠️ | v2 統一 |
| **D010** | standalone.hpp | MEX版と重複 | ⚠️ | 削除（未使用） |

### 🟢 低優先度：未使用（削除検討）

| ID | 対象 | 理由 | 推奨 |
|----|------|------|------|
| **U001** | KF/kalman_filter_core.hpp テンプレート | 実装されているが全システムで ESKF 使用 | 将来削除 or Experimental/ 移動 |
| **U002** | UKF テンプレート | 開発途上、実装不完全 | 将来完成時に activate |
| **U003** | EKF テンプレート | 参考実装、未使用 | 保留（参考値） |
| **U004** | utils.hpp の関数 | Phase 2 で新関数に置換 | Phase 3 で削除予定 |

---

## 関数詳細カタログ

### ESKF システム（完全に実装）

#### 予測パイプライン
```
ESKFState (初期化)
    ↓
eskf_core.cpp::integrate_nominal()
    [RK2積分で p, v, q を更新]
    ↓
eskf_core.cpp::predict_covariance()
    [F行列計算 → P の共分散予測]
    ↓
filter_mgmt.cpp::normalize_covariance()
    [P の分散をクリップ]
    ↓
filter_mgmt.cpp::symmetrize_covariance()
    [P = (P + P^T)/2 で強制対称化]
    ↓
完成: ESKFState (予測後)
```

#### 更新パイプライン（センサー別）

**加速度更新** (Roll/Pitch):
```
sensor_preprocessor.cpp::preprocess_accel()
    → 前処理 (NaN チェック, 外れ値検出)
    ↓
sensor_filter.hpp::OutlierDetector::is_outlier()
    → Mahalanobis 距離で判定
    ↓
[if not outlier]
    ↓
eskf_core.cpp::update_accel()
    → Roll/Pitch 更新 (innovation 기반)
    ↓
完成: ESKFState (更新後)
```

**磁気更新** (Yaw):
```
sensor_preprocessor.cpp::preprocess_mag()
    → 前処理
    ↓
eskf_core.cpp::update_mag()
    → Yaw 更新
    ↓
injection: inject_error_state()
    → 誤差状態を名義状態に反映
```

**GPS更新** (Position/Velocity):
```
sensor_preprocessor.cpp::preprocess_gps()
    → WGS84 → ENU 座標変換
    ↓
sensor_filter.hpp::OutlierDetector::is_outlier()
    ↓
[if not outlier]
    ↓
ESKF/sensor_updates.cpp::update_gps_position()
    + update_gps_velocity()
    → Position/Velocity 更新
```

**気圧更新** (Altitude):
```
sensor_preprocessor.cpp::preprocess_baro()
    → pressure [Pa] → altitude [m]
    ↓
ESKF/sensor_updates.cpp::update_baro()
    → Z 軸位置更新
```

---

## 統計情報

### ファイルサイズ分布

| ファイル | 行数 | 複雑度 | 保守性 |
|---------|------|--------|-------|
| sensor_filter.hpp | 831 | ★★★ | ⚠️ 要分割 |
| meukf_core.cpp | 1346 | ★★★ | ⚠️ 要分割 |
| fixed_matrix.hpp | 350 | ★★ | ✅ 良好 |
| quaternion_functions.hpp | 172 | ★★ | ✅ 良好 |
| math_utils.hpp | 400+ | ★★ | ✅ 良好 |
| ESKF 各ファイル | 平均 250 | ★★ | ✅ 良好 |

### 関数数（推定）

| モジュール | 関数数 | 平均関数/ファイル |
|-----------|--------|------------------|
| Common | 35+ | 3.2 |
| ESKF | 45+ | 4.1 |
| MEUKF | 25+ | 5.0 |
| Matrix | 15+ | 15 |
| Quaternion | 10+ | 10 |
| その他 | 20+ | 2.5 |
| **合計** | **150+** | 4.0 |

---

## Phase 3 アクションアイテム

### 即座（1週間）
- [ ] Mahalanobis 統一関数作成（Math/math_utils 基準）
- [ ] 影響範囲分析（grep で 3 ファイル確認）
- [ ] Innovation 統一関数作成

### 短期（2週間）
- [ ] 統一関数への一括置換（ファイル毎に検証）
- [ ] 外れ値テスト 3+ ケース追加
- [ ] 回帰テスト 10/10 PASS 確認

### 中期（1ヶ月）
- [ ] sensor_filter.hpp クラス分割
  - EMAFilter → separate file
  - BiquadFilter → separate file
  - OutlierDetector → separate file
  - RobustStatistics → separate file
- [ ] meukf_core.cpp 関数分割
  - predict() → meukf_predict.cpp
  - update_*() → meukf_update.cpp
  - utils → meukf_util.cpp

---

## 参考リンク

- [.github/copilot-instructions.md](.github/copilot-instructions.md) - AI エージェント用指示
- [PROJECT_STATUS.md](PROJECT_STATUS.md) - プロジェクト全体状況
- [docs/LIB_STRUCTURE_ANALYSIS.md](docs/LIB_STRUCTURE_ANALYSIS.md) - モジュール分析
- [PHASE3_PLAN.md](PHASE3_PLAN.md) - Phase 3 詳細計画

---

**作成日**: 2026年1月5日  
**バージョン**: 1.0  
**担当**: GitHub Copilot + ユーザー協働  
**最終確認**: 全関数リスト統合完了

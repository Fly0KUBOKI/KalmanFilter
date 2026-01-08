# Lib フォルダ構造・依存関係・品質分析

## 📋 概要

KalmanFilterプロジェクトの `Lib/` フォルダは、**7つのサブモジュール・37ファイル** で構成される、モジュール化された数値計算ライブラリです。

### モジュール構成

| モジュール | ファイル数 | 主な機能 | 実装度 | 状態 |
|-----------|---------|---------|--------|------|
| **Common** | 11 | センサー・フィルタ・統計・検証 | ★★★ | ✓ |
| **ESKF** | 11 | Error-State Kalmanフィルタ | ★★★ | ✓ |
| **MEUKF** | 5 | Modified Extended UKF | ★★ | ⚠️ |
| **EKF** | 3 | Extended Kalmanフィルタ | ★★ | ⚠️ |
| **UKF** | 3 | Unscented Kalmanフィルタ | ★★ | ⚠️ |
| **KF** | 2 | 線形Kalmanフィルタ | ★ | ⚠️ |
| **Matrix** | 1 | 固定サイズ行列演算 | ★★★ | ✓ |
| **Quaternion** | 1 | 四元数演算 | ★★★ | ✓ |

---

## 🏗️ 1. Common モジュール (11ファイル)

全フィルタ実装が依存する共通基盤ライブラリ。センサー処理・統計・検証機能を提供。

### 1.1 フィルタ管理 (`filter_mgmt.hpp/cpp`)

**ファイル**: `Common/inc/filter_mgmt.hpp`, `Common/src/filter_mgmt.cpp`

**機能**:
- 共分散行列の整合性チェック (NaN/Inf検出)
- 共分散行列の初期化 (スケール付き単位行列)
- 発散検知・検出
- ZUPT (Zero Velocity Update) 処理
- 共分散正規化・スケーリング

**主要関数**:
```cpp
namespace common::filter {
    bool hasNaNOrInf(const Matrix<15,15,float>& P);
    void setIdentityScaled(Matrix<15,15,float>& P, float scale);
    bool check_divergence(const Matrix<15,15,float>& P);
    void apply_zupt(...);  // 速度ゼロ適用
    void normalize_covariance(Matrix<15,15,float>& P);
}
```

**依存関係**:
- `Matrix/fixed_matrix.hpp`
- `cmath` 標準ライブラリ

**実装度**: ★★★ (完全実装、計約150行)

---

### 1.2 インターフェース管理 (`interface.hpp`, `interface_stub.cpp`)

**ファイル**: `Common/inc/interface.hpp`, `Common/src/interface_stub.cpp`

**機能**:
- スタンドアロン版との互換インターフェース
- フィルタ初期化・実行のラッパー
- MEX版との切り替え対応

**主要関数**:
```cpp
namespace kalman {
    namespace standalone {
        void initialize_filter(...);
        void run_filter_step(...);
        void cleanup_filter();
    }
}
```

**依存関係**:
- ESKF モジュール各ファイル
- Common/types.hpp

**実装度**: ★★ (スタブ実装、主にヘッダー定義)

---

### 1.3 数学ユーティリティ

#### 1.3.1 統計計算 (`Math/statistics.hpp`)

**機能**:
- 中央値計算
- 平均・分散計算  
- 四分位数計算
- ロバスト統計 (MAD, Z-score)

**主要関数**:
```cpp
namespace cmath_fx::statistics {
    float mean(const Vector<N, float>& v);
    float variance(const Vector<N, float>& v);
    float median(const Vector<N, float>& v);
    float mad(const Vector<N, float>& v);  // Median Absolute Deviation
}
```

**⚠️ 問題**: `<1行コード>` と `<複数行展開>` が混在 → **統一推奨**

#### 1.3.2 ベクトル演算 (`Math/vector_utils.hpp`)

**機能**:
- ベクトル正規化・スケーリング
- 内積・外積計算
- 要素ごとの演算

**主要関数**:
```cpp
namespace cmath_fx::vector {
    float norm(const Vector<N, float>& v);
    void normalize(Vector<N, float>& v);
    float dot(const Vector<N, float>& a, const Vector<N, float>& b);
}
```

#### 1.3.3 数学関数 (`Math/math_utils.hpp`)

**機能**:
- 三角関数 (sin, cos, tan)
- 指数・対数関数
- 行列分解 (Cholesky)
- 固有値計算

**実装度**: ★★★ (包括的、計約400行)

**注記（統一方針）**:
- `MathUtils` が `compute_innovation_and_S` と `mahalanobis_distance_squared` の正規実装を提供します。
- 一部古いモジュール（例: `KF/inc/kalman_filter_core.hpp`）に同等のテンプレート実装が残っています。将来的には `KF` 側を `MathUtils` 呼び出しに置換し、重複を削除することを推奨します。

---

### 1.4 センサー処理

#### 1.4.1 外れ値検出 (`Sensor/sensor_filter.hpp`)

**機能**:
- IMUセンサー異常値検出
- Mahalanobis距離計算
- ロバスト統計ベース異常判定
- フィルタリング

**主要クラス**:
```cpp
namespace common::sensor {
    class OutlierDetector {
        bool detect_outlier(const Vector3& measurement, ...);
        float mahalanobis_distance(...);
    };
}
```

**⚠️ 問題**: 
- **3種類の外れ値検出方法が実装** → 統一推奨
- Mahalanobis距離計算 2回実装

#### 1.4.2 センサー前処理 (`Sensor/sensor_preprocessor.hpp/cpp`)

**ファイル**: `Common/inc/Sensor/sensor_preprocessor.hpp`, `Common/src/Sensor/sensor_preprocessor.cpp`

**機能**:
- センサー入力の正規化
- キャリブレーション
- スケーリング適用
- ノイズ減算

**主要関数**:
```cpp
namespace common::sensor {
    void preprocess_imu(const RawIMU& raw, ProcessedIMU& processed);
    void preprocess_gps(const RawGPS& raw, ProcessedGPS& processed);
    void apply_calibration(...);
}
```

**依存関係**:
- `types.hpp`
- `Math/math_utils.hpp`

---

### 1.5 検証・品質管理 (`Validation/validation.hpp`)

**機能**:
- 状態ベクトル検証 (NaN/Inf/範囲)
- 共分散正定値性確認
- センサー入力妥当性チェック
- アサーション・デバッグ出力

**主要関数**:
```cpp
namespace common::validation {
    bool validate_state(...);      // 状態検証
    bool validate_covariance(...); // 共分散検証
    bool validate_sensor_input(...); // センサー入力検証
}
```

**実装度**: ★★ (基本検証のみ実装)

---

### 1.6 スタンドアロン対応 (`standalone.hpp/cpp`)

**機能**:
- MEX非依存の実行インターフェース
- CSVファイル入出力
- 時間管理

**⚠️ 問題**: 
- MEX版との重複機能が多い
- 現在はほぼ未使用

---

### 1.7 共通型定義 (`types.hpp`)

**機能**:
- 構造体定義 (RawIMU, ProcessedIMU, RawGPS, etc.)
- 定数定義 (重力加速度, 地磁気, など)
- センサー設定構造体

**⚠️ 問題**:
- **float/double 混在**: センサー入力構造体がdoubleの場所もある → float統一推奨
- **フィールド命名不統一**: `accel_x` vs `ax` 混在

---

## 🧮 2. ESKF モジュール (11ファイル)

**Error-State Kalman Filter** の完全実装。メインフィルタエンジン。

### 2.1 状態管理 (`eskf_state.hpp`)

**機能**:
- 15次元状態ベクトル定義: `[p(3), v(3), q(4), ba(3), bg(3)]`
- 状態構造体初期化
- メモリ管理

**主要構造体**:
```cpp
namespace eskf {
    struct ESKFState {
        float p[3];    // 位置 [m]
        float v[3];    // 速度 [m/s]
        float q[4];    // 四元数 [w,x,y,z]
        float ba[3];   // 加速度バイアス [m/s²]
        float bg[3];   // ジャイロバイアス [rad/s]
        
        float P[15*15]; // 共分散行列
        
        // 診断用フィールド
        double last_innov_norm;
        double last_maha_dist;
    };
}
```

**依存関係**:
- `Matrix/fixed_matrix.hpp`
- `Quaternion/quaternion_functions.hpp`

---

### 2.2 初期化処理 (`eskf_initializer.hpp/cpp`)

**ファイル**: `ESKF/inc/eskf_initializer.hpp`, `ESKF/src/eskf_initializer.cpp`

**機能**:
- 初期姿勢推定 (加速度・磁気から)
- 初期速度設定 (GPS/IMU融合)
- バイアス初期化
- 共分散初期化

**主要関数**:
```cpp
namespace eskf {
    ESKFState initialize_from_observations(
        const Vector3& accel_mean,
        const Vector3& mag_mean,
        const Vector3& gps_pos_init,
        const Vector3& gps_vel_init,
        float dt
    );
    
    void calibrate_gyro_bias(...);
}
```

**実装度**: ★★★ (計約300行、詳細初期化ロジック)

---

### 2.3 フィルタコア (`eskf_core.hpp/cpp`)

**ファイル**: `ESKF/inc/eskf_core.hpp`, `ESKF/src/eskf_core.cpp`

**機能**:
- IMU積分 (RK2/台形則)
- 四元数更新・正規化
- 加速度計更新 (Roll/Pitch)
- 磁気計更新 (Yaw)
- プロセスノイズ計算

**主要クラス/関数**:
```cpp
namespace eskf {
    class ESKFCore {
    public:
        static void integrate_nominal(...);     // IMU積分
        static void update_accel(...);          // 加速度更新
        static void update_mag(...);            // 磁気更新
        static void compute_process_noise(...); // プロセスノイズ
        static void propagate_covariance(...);  // 共分散伝播
    };
}
```

**⚠️ 問題**:
- **IMU積分が2種類実装** (RK2 vs 台形則) → 1つに統一
- **加速度スケール計算が複雑** (多重フォーム) → 簡潔化推奨

**実装度**: ★★★ (計約800行、複雑度高)

---

### 2.4 センサー更新 (`eskf_sensor_updates.hpp/cpp`)

**ファイル**: `ESKF/inc/eskf_sensor_updates.hpp`, `ESKF/src/eskf_sensor_updates.cpp`

**機能**:
- GPS位置更新
- GPS速度更新
- 気圧計高度更新
- 外れ値検出・棄却

**主要関数**:
```cpp
namespace eskf {
    void update_gps_position(...);
    void update_gps_velocity(...);
    void update_barometer(...);
    
    float compute_innovation(...);
    float compute_mahalanobis_distance(...);
}
```

**⚠️ 問題**:
- **Innovation計算が2箇所に分散** → 統合推奨
- **Mahalanobis距離3種実装** → 統一推奨

**実装度**: ★★★ (計約600行)

---

### 2.5 数学補助 (`eskf_math.hpp/cpp`)

**機能**:
- 四元数関数 (乗算・共役・逆数)
- 回転行列計算
- Jacobian計算

**⚠️ 問題**:
- `Quaternion/quaternion_functions.hpp` と機能重複 → 統合推奨

---

### 2.6 後処理 (`eskf_postprocess.hpp/cpp`)

**機能**:
- 四元数正規化
- 共分散対称化・正定値化
- Euler角計算
- 結果フォーマット

**主要関数**:
```cpp
void normalize_quaternion(float q[4]);
void symmetrize_covariance(float P[15*15]);
void compute_euler_angles(const float q[4], float& roll, float& pitch, float& yaw);
```

**⚠️ 問題**:
- **正規化処理4種類実装** (normalize_quaternion, normalize, quat_normalize, quaternion_normalize)
- **共分散対称化3種実装** (symmetrize, force_symmetric, make_symmetric)

---

### 2.7 実行制御 (`eskf_runner.hpp/cpp`)

**機能**:
- ステップバイステップ実行制御
- 時間管理
- センサー同期

---

### 2.8 ハイレベルインターフェース (`filter.hpp/cpp`, `eskf_filter.hpp`)

**機能**:
- ユーザー向けAPI
- 初期化・実行・状態取得
- エラーハンドリング

**⚠️ 問題**:
- `filter.hpp` と `eskf_filter.hpp` で機能重複 → 統合推奨

---

## ⚡ 3. MEUKF モジュール (5ファイル)

**Modified Extended Unscented Kalman Filter** 実装。ESKF代替フィルタ。

### 3.1 コア実装 (`meukf_core.hpp/cpp`)

**機能**:
- MEUKF予測ステップ
- シグマポイント生成
- 重み計算

**⚠️ 問題**:
- 開発途上（保守段階）
- テストが不十分

### 3.2 型定義 (`meukf_types.hpp`)

**機能**:
- MEUKF専用の状態・パラメータ型

---

## 📐 4. EKF モジュール (3ファイル)

**Extended Kalman Filter** 基本実装。テンプレート的位置づけ。

### 4.1 線形更新 (`ekf_linear_update.hpp/cpp`)

**機能**:
- 線形観測モデルでの更新

**実装度**: ★★ (基本実装)

---

## 🎲 5. UKF モジュール (3ファイル)

**Unscented Kalman Filter** 実装。非線形フィルタリング用テンプレート。

### 5.1 シグマポイント (`ukf_sigma_points.hpp`)

**機能**:
- Unscented変換によるシグマポイント生成
- 重み計算

---

## 🔤 6. KF モジュール (2ファイル)

**基本的なKalmanフィルタ** テンプレート。線形フィルタリング用。

**実装度**: ★ (テンプレート、ほぼ未使用)

---

## 🔢 7. 基本数学モジュール

### 7.1 固定サイズ行列 (`Matrix/fixed_matrix.hpp`)

**ファイル**: `Matrix/fixed_matrix.hpp` (350行、実装含む)

**機能**:
- 固定サイズ行列テンプレート (R×C, 型パラメータ)
- 基本演算 (+, -, *, transpose)
- Cholesky分解
- 行列式・逆行列
- メモリ効率的な実装 (スタック確保)

**主要クラス/関数**:
```cpp
namespace cmath_fx {
    template <int R, int C, typename T = float>
    struct Matrix {
        static Matrix Zero();
        static Matrix Identity();
        
        Matrix operator+(const Matrix& other) const;
        Matrix operator*(const Matrix& other) const;
        Matrix transpose() const;
        bool cholesky_decomposition(Matrix& L) const;
        float determinant() const;
        Matrix inverse() const;
    };
    
    // 型エイリアス
    template <int N, typename T = float>
    using Vector = Matrix<N, 1, T>;
}
```

**実装度**: ★★★ (完全実装、ハイパフォーマンス)

**依存関係**: なし (スタンドアロン)

**活用状況分析**:
✅ **appropriate使用**:
- ESKF: 15x15共分散行列 → `Matrix<15,15,float>`
- センサー処理: 3x3行列 → `Matrix<3,3,float>` 
- Quaternion: Vector<4,float> として活用

⚠️ **問題検出**: Sensor層で `cmath_fx::FixedMatrix` という可変長マトリクスも独立実装 → 冗長

---

### 7.2 四元数演算 (`Quaternion/quaternion_functions.hpp`)

**ファイル**: `Quaternion/quaternion_functions.hpp` (174行、実装含む)

**機能**:
- 四元数乗算
- 四元数正規化 (複数実装)
- 四元数共役
- 四元数逆数
- 回転行列変換
- Euler角変換

**主要関数**:
```cpp
namespace cquat {
    template <typename T>
    inline void normalize_quat(cmath_fx::Vector<4, T>& q);           // ✅ 推奨標準
    
    template <typename T>
    inline cmath_fx::Vector<4, T> normalize_quaternion(const cmath_fx::Vector<4, T>& q_in);  // ⚠️ 重複 (wrapper)
    
    template <typename T>
    inline void multiply_quat(const cmath_fx::Vector<4, T>& a, const cmath_fx::Vector<4, T>& b, cmath_fx::Vector<4, T>& out);
    
    template <typename T>
    inline void quat_to_rotm(const cmath_fx::Vector<4, T>& q, cmath_fx::Matrix<3, 3, T>& R);
    
    // Euler角変換も実装
}
```

**実装度**: ★★★ (完全実装、パフォーマンス最適化済み)

**依存関係**: `Matrix/fixed_matrix.hpp`, `Common/Math/*.hpp` (不適切)

**活用状況分析**:
✅ **appropriate使用**:
- ESKF/MEUKF: `cquat::normalize_quat()` で四元数正規化 → 統一済み
- 四元数乗算: `multiply_quat()` で統一

⚠️ **重大な問題**:
1. **インクルード構造が不適切**:
   ```cpp
   // ❌ quaternion_functions.hpp の異常なインクルード
   #include "../Matrix/fixed_matrix.hpp"     // パス1
   #include "../../Matrix/fixed_matrix.hpp"  // パス2 (重複)
   #include "../Common/inc/Math/statistics.hpp"     // ← 後方依存
   #include "../Common/inc/Math/geometry.hpp"
   #include "../Common/inc/Math/numerical.hpp"
   #include "../Common/inc/Math/math_utils.hpp"     // ← 循環依存予備軍
   ```
   
2. **normalize_quaternion()` wrapper が重複**:
   - `normalize_quat()` のwrapper版なのに別命名
   - 呼び出し箇所でどちらを使うか混乱

3. **後方依存**: Common/Math層がQuaternion層より下位なのに、Quaternionが依存している

---

## 📊 依存関係グラフ

```
┌──────────────────────────────────────┐
│ MEX インターフェース層                 │
│ (MEX/*.cpp)                           │
└─────────────────────────────────────┬┘
                                      │
                    ┌─────────────────┴──────────────┐
                    │                                │
        ┌───────────▼──────────┐      ┌────────────▼──────────┐
        │ ESKF                 │      │ MEUKF / EKF / UKF     │
        │ • eskf_core          │      │ • meukf_core          │
        │ • sensor_updates     │      │ • ekf_linear_update   │
        │ • initializer        │      │ • ukf_sigma_points    │
        │ • math/postprocess   │      │                       │
        └───────────┬──────────┘      └────────────┬──────────┘
                    │                              │
        ┌───────────▼──────────────────────────────┴──────┐
        │ Common (共通基盤)                              │
        │ • filter_mgmt (共分散管理)                    │
        │ • sensor_filter/preprocessor (センサー処理)     │
        │ • Math (統計・ベクトル・数学関数)              │
        │ • Validation (検証)                           │
        └───────────┬───────────────────────────┬────────┘
                    │                           │
        ┌───────────▼──────────┐      ┌────────▼──────────┐
        │ Matrix                │      │ Quaternion        │
        │ fixed_matrix.hpp      │      │ quaternion_       │
        │ (固定サイズ行列)       │      │ functions.hpp     │
        │ ✅活用度: 高            │      │ (四元数演算)       │
        └───────────────────────┘      │ ⚠️複数重複実装      │
                                       └───────────────────┘
```

### ⚠️ **重大な依存関係問題**

#### 1️⃣ **インクルード重複 (Quaternion層)**
```cpp
// quaternion_functions.hpp の異常なインクルード
#include "../Matrix/fixed_matrix.hpp"     // ← 相対パス混在 ❌
#include "../../Matrix/fixed_matrix.hpp"  // ← 2重インクルード ❌
#include "../Common/inc/Math/statistics.hpp"    // ← 非対称な構造 ❌
#include "../Common/inc/Math/geometry.hpp"
#include "../Common/inc/Math/numerical.hpp"
#include "../Common/inc/Math/math_utils.hpp"    // ← 後方依存 ❌
```
**影響**: コンパイル遅延、include guard の脆弱性

#### 2️⃣ **Sensor層の過度なインクルード**
```cpp
// sensor_filter.hpp (846行) の問題
#include "../../../Matrix/fixed_matrix.hpp"     // 重複
#include "../../Matrix/fixed_matrix.hpp"         // 重複
#include "../../../KF/inc/kf_operations.hpp"    // 不要
#include <atomic>, <chrono>, <cstdarg>, <cfloat> // 過度
```
**影響**: ビルド時間増加、メモリ使用量増加

#### 3️⃣ **ESKF層での不完全な分離**
```cpp
// filter.hpp vs eskf_filter.hpp → 機能重複・混乱
// eskf_math.hpp が Quaternion と同じ機能を実装
// eskf_postprocess.hpp が Quaternion 正規化機能を重複実装
```
**影響**: 状態同期の複雑化、バグ発生リスク

---

## ⚠️ 重複・不要な実装リスト

### 高優先度（即座対応）

| # | 対象 | 問題 | 削除/統合予定 | 影響度 | 実装箇所 |
|---|------|------|-------------|--------|--------|
| 1 | **四元数正規化** | 4種実装 (normalize_quat, normalize_quaternion など) | → `cquat::normalize_quat<T>()` に統一 | **HIGH** | quaternion_functions.hpp, eskf_postprocess.cpp, unified_filter.cpp |
| 2 | **共分散対称化** | 3種実装 (symmetrize, force_symmetric, make_symmetric) | → `common::filter::symmetrize_covariance()` に統一 | **HIGH** | filter_mgmt.cpp, sensor_filter.hpp, eskf_core.cpp |
| 3 | **Mahalanobis距離** | 3箇所で計算（独立実装） | → `Common/Math/` に統一実装、reference提供 | **MEDIUM** | sensor_filter.hpp(L250-300), sensor_updates.cpp(L180-220), kf_operations.hpp(L350-380) |
| 4 | **Innovation計算** | 2箇所分散 | → `sensor_updates.cpp` に統一 | **MEDIUM** | sensor_updates.cpp, eskf_math.cpp, kf_operations.hpp |
| 5 | **IMU積分** | 2種実装 (RK2, 台形則) | → RK2に統一（計算精度面から） | **MEDIUM** | eskf_core.cpp(L55), eskf_math.cpp(L120) |
| 6 | **filter.hpp vs eskf_filter.hpp** | 機能重複・インターフェース混在 | → `eskf_filter.hpp` に統合, filter.hpp削除 | **MEDIUM** | ESKF/inc/filter.hpp, ESKF/inc/eskf_filter.hpp |
| 7 | **インクルード重複** | Matrix include 2重、パス混在 | → 相対パス統一（`../../`） | **HIGH** | quaternion_functions.hpp(L3,5), sensor_filter.hpp(L6,7), validation.hpp(L3,5) |

### 中優先度（近期対応）

| # | 対象 | 問題 | 削除/統合予定 | 影響度 | ファイル |
|---|------|------|-------------|--------|---------|
| 8 | **eskf_math.hpp** | Quaternion/quaternion_functions.hpp と重複 | → quaternion_functions.hpp に統合 | LOW | ESKF/inc/eskf_math.hpp |
| 9 | **types.hpp float/double混在** | GPS入力がdouble | → 全float32に統一 | MEDIUM | Common/types.hpp |
| 10 | **interface.hpp/standalone.hpp** | 重複インターフェース | → 削除（未使用） | LOW | Common/inc/interface.hpp, Common/inc/standalone.hpp |
| 11 | **KF, EKF テンプレート** | 現在未使用（MEUKF/ESKF使用） | → アーカイブフォルダに移動 | LOW | KF/, EKF/ |
| 12 | **sensor_filter.hpp 巨大ファイル** | 831行、複数フィルタ実装混在 | → EMA, Biquad, Outlier を分割 | MEDIUM | Common/inc/Sensor/sensor_filter.hpp |
| 13 | **meukf_core.cpp超長** | 1346行、複数フェーズ混在 | → predict/update を .cpp に分割 | MEDIUM | MEUKF/src/meukf_core.cpp |
| 14 | **可変長Matrix実装** | sensor_filter.hpp で `cmath_fx::FixedMatrix` (動的サイズ) | → fixed_matrix<R,C> に統一 | MEDIUM | Common/inc/Sensor/sensor_filter.hpp(L50-100) |

### 低優先度（将来対応）

| # | 対象 | 問題 | 削除/統合予定 | 影響度 |
|---|------|------|-------------|--------|
| 15 | **統計関数コード形式混在** | statistics.hpp で `<1行>` と `<複数行展開>` 混在 | → 統一 | LOW |
| 16 | **Validation層の検証不足** | validation.hpp で実装が基本検証のみ | → 拡張（NaN/Inf/有限チェック） | LOW |
| 17 | **未使用な数学関数** | geometry.hpp, numerical.hpp 内で実装されているが呼ばれていない関数 | → deprecated マーク + ログ | LOW |

---

---

---

## 🔍 詳細分析：実装冗長性と不要コード

### Phase 1: インクルード構造の問題（高優先度）

#### 問題 A: 相対パス混在と重複

**quaternion_functions.hpp**:
```cpp
#pragma once
#include "../Matrix/fixed_matrix.hpp"     // ❌ パス1
#include <cmath>
#include "../../Matrix/fixed_matrix.hpp"  // ❌ パス2（重複インクルード）
#include "../Common/inc/Math/statistics.hpp"    // ❌ 後方依存
#include "../Common/inc/Math/geometry.hpp"
#include "../Common/inc/Math/numerical.hpp"
#include "../Common/inc/Math/math_utils.hpp"    // ❌ 循環予備軍
```

**sensor_filter.hpp**:
```cpp
#include "../../../Matrix/fixed_matrix.hpp"     // ❌ パス1
#include "../../Matrix/fixed_matrix.hpp"         // ❌ パス2（重複）
#include "../../../KF/inc/kf_operations.hpp"    // ❌ 不要な依存
#include <atomic>, <chrono>, <cstdarg> ...       // ❌ 過度なヘッダー
```

**validation.hpp**:
```cpp
#include "../../../Matrix/fixed_matrix.hpp"     // ❌ パス1
#include "../../../KF/inc/kf_operations.hpp"    // ❌ 不要
#include "../../Matrix/fixed_matrix.hpp"         // ❌ パス2（重複）
```

**修正案**:
```bash
# 相対パスをすべて ../../ に統一
find kalman/cpp/Lib -name "*.hpp" -exec sed -i \
  's|#include "../Matrix/|#include "../../Matrix/|g' {} \;

# 重複インクルードを削除
sed -i '5d' kalman/cpp/Lib/Quaternion/quaternion_functions.hpp
sed -i '7d' kalman/cpp/Lib/Common/inc/Sensor/sensor_filter.hpp
```

---

### Phase 2: 関数重複実装（高優先度）

#### 問題 B: 四元数正規化の4重実装

**現状マッピング**:
| 実装 | ファイル | 行 | 用途 | 状態 |
|------|---------|-----|------|------|
| `normalize_quat<T>()` | quaternion_functions.hpp | 13 | ✅推奨：テンプレート版 | **採用予定** |
| `normalize_quaternion()` | quaternion_functions.hpp | 29 | wrapper（返値版） | ⚠️ 重複 |
| `normalize_quaternion()` | unified_filter.cpp | 211 | MEUKF内独立実装 | ⚠️ 重複 |
| `quat_normalize()` | utils.hpp | 23 | Deprecated | ❌ 削除推奨 |

**削除対象**:
```cpp
// ❌ unified_filter.cpp の重複版 → cquat::normalize_quat() に置換
Vec4 UnifiedFilter::normalize_quaternion(const Vec4& q) const {
    // 削除 → cquat::normalize_quat(q) 呼び出しに統一
}

// ❌ normalize_quaternion wrapper → デッドコード扱い
template <typename T>
inline cmath_fx::Vector<4, T> normalize_quaternion(const cmath_fx::Vector<4, T>& q_in) {
    // 不要 → normalize_quat() で十分
}
```

#### 問題 C: 共分散対称化の3重実装

**呼び出し箇所一覧**:
| 関数名 | ファイル | 行 | 説明 |
|--------|---------|-----|------|
| `symmetrize_covariance()` | filter_mgmt.cpp | 85 | ✅推奨版 |
| `force_symmetric()` | sensor_filter.hpp | 278 | ⚠️ 同じ処理 |
| `make_symmetric()` | eskf_core.cpp | 450 | ⚠️ 同じ処理 |

**コード例**:
```cpp
// filter_mgmt.cpp ✅ 推奨
void symmetrize_covariance(Matrix<15,15,float>& P) {
    for (int i = 0; i < 15; ++i) {
        for (int j = i+1; j < 15; ++j) {
            float avg = (P(i,j) + P(j,i)) / 2.0f;
            P(i,j) = avg;
            P(j,i) = avg;
        }
    }
}

// sensor_filter.hpp ❌ 重複
void force_symmetric(Matrix<15,15,float>& P) {
    for (int i = 0; i < 15; ++i)
        for (int j = i+1; j < 15; ++j)
            P(i,j) = P(j,i) = (P(i,j) + P(j,i)) / 2.0f;
}
```

#### 問題 D: Mahalanobis距離の3重実装

**発見位置**:
| ファイル | 行 | 計算方式 | 精度 |
|---------|-----|---------|------|
| sensor_filter.hpp | 250-300 | 完全実装（外れ値検出用） | ⭐ 最も詳細 |
| sensor_updates.cpp | 180-220 | 簡略版 | ⭐⭐ 基本形 |
| kf_operations.hpp | 350-380 | 汎用テンプレート版 | ⭐⭐⭐ 最高精度（複雑） |

**統一推奨アプローチ**:
```cpp
// → Common/Math/mahalanobis.hpp に統一
namespace cmath_fx::stats {
    // テンプレート汎用版（全次元対応）
    template <int N, typename T>
    T compute_mahalanobis_distance_squared(
        const Vector<N, T>& innovation,
        const Matrix<N, N, T>& S_inv  // S^{-1} (innovation covariance inverse)
    ) {
        Vector<N, T> temp = S_inv * innovation;
        return dot(innovation, temp);
    }
}
```

---

### Phase 3: ファイル構造の問題

#### 問題 E: filter.hpp vs eskf_filter.hpp 重複

**対比表**:
| インターフェース | filter.hpp | eskf_filter.hpp | 推奨 |
|--------|-----------|-----------------|------|
| 初期化 | `init()` | `initialize()` | ⚠️ 名称不統一 |
| 更新 | `update()` | `update()` | ✅ 一致 |
| 状態取得 | `getState()` | `get_state()` | ⚠️ 命名規則不統一 |
| 継承 | `Filter` 基底クラス | スタンドアロン | ⚠️ 設計不統一 |

**削除対象**: `ESKF/inc/filter.hpp` → `eskf_filter.hpp` に統合

#### 問題 F: MEUKF内の独立実装

**unified_filter.hpp, meukf_core.hpp, meukf_core.cpp**:
```cpp
// MEUKF が独自で実装している機能（重複）
Vec4 normalize_quaternion(const Vec4& q) const;     // ← cquat::normalize_quat() で十分
Vec3 quaternion_to_euler(const Vec4& q) const;      // ← quaternion_functions.hpp 使用可
Mat3 quaternion_to_rotation_matrix(const Vec4& q) const;  // ← 重複
```

---

### Phase 4: センサー層の過度な実装（中優先度）

#### 問題 G: sensor_filter.hpp (846行)の巨大化

**含まれている要素**:
1. **EMAフィルタ** (90行) → 分割推奨: `Sensor/ema_filter.hpp`
2. **Biquad ローパスフィルタ** (130行) → 分割推奨: `Sensor/biquad_filter.hpp`
3. **外れ値検出** (200行) → 分割推奨: `Sensor/outlier_detector.hpp`
4. **ロバスト統計** (150行) → 分割推奨: `Sensor/robust_statistics.hpp`
5. **その他センサー管理** (276行)

**分割案**:
```bash
Common/inc/Sensor/
├── sensor_filter.hpp          (削減: 396行 → dispatcher)
├── ema_filter.hpp             (新規: 90行)
├── biquad_filter.hpp          (新規: 130行)
├── outlier_detector.hpp       (新規: 200行)
└── robust_statistics.hpp      (新規: 150行)
```

#### 問題 H: meukf_core.cpp (1346行)の巨大化

**含まれている要素**:
1. 予測ステップ (400行)
2. シグマポイント処理 (350行)
3. 更新処理 (400行)
4. 補助関数 (196行)

**分割案**:
```bash
MEUKF/src/
├── meukf_core.cpp             (削減: 1346行 → 200行)
├── meukf_predict.cpp          (新規: 400行)
├── meukf_sigma_points.cpp     (新規: 350行)
└── meukf_update.cpp           (新規: 400行)
```

---



| ファイル | 行数 | 複雑度 | 関数/クラス数 | 優先順位 |
|---------|------|--------|-------------|---------|
| `eskf_core.cpp` | 800+ | ★★★ | 8+ | 1 |
| `fixed_matrix.hpp` | 350 | ★★ | 20+ | 2 |
| `sensor_updates.cpp` | 600+ | ★★★ | 10+ | 1 |
| `math_utils.hpp` | 400+ | ★★ | 15+ | 2 |
| `quaternion_functions.hpp` | 200 | ★★ | 8+ | 3 |
| `sensor_filter.hpp` | 250+ | ★★ | 5+ | 3 |

---

## ✅ 推奨リファクタリングロードマップ

### **Phase 0: 準備・分析 (3日)**
- [x] 依存関係グラフ分析完了
- [x] インクルード重複特定
- [x] 関数重複マッピング完了
- [ ] MEXインターフェース変更影響度評価

### **Phase 1: インクルード統一 (3-5日)**
**目的**: ビルド安定化・コンパイル時間削減
- [ ] **相対パス統一**: すべてのヘッダーを `../../` 形式に統一
  - `quaternion_functions.hpp`: L3, L5 の重複削除
  - `sensor_filter.hpp`: L6, L7 の重複削除
  - `validation.hpp`: L3, L5 の重複削除
- [ ] **後方依存削除**: Quaternion層から Common/Math層への依存削除
- [ ] **ビルド検証**: `build_mex()` で全MEX再ビルド & 実行テスト

### **Phase 2: 関数重複統一 (1週間)**
**目的**: 状態同期バグ削減、保守性向上
- [ ] **四元数正規化** 統一
  - 標準: `cquat::normalize_quat<T>()`
  - deprecated: `normalize_quaternion()`, `quat_normalize()`
  - 削除: `UnifiedFilter::normalize_quaternion()` (unified_filter.cpp L211)
  - テスト: `run_simulation(42, true)` で数値差なし確認

- [ ] **共分散対称化** 統一
  - 標準: `common::filter::symmetrize_covariance()`
  - 削除: `sensor_filter.hpp::force_symmetric()`, `eskf_core.cpp::make_symmetric()`
  - テスト: P が対称性を保つ確認

- [ ] **Mahalanobis距離** 統一
  - 標準関数: `Common/Math/mahalanobis.hpp` 作成
  - 全3実装を統一インターフェースに置換
  - テスト: 外れ値検出精度が変わらない確認

- [ ] **Innovation計算** 統一
  - 標準: `sensor_updates.cpp::compute_innovation()`
  - 削除: `eskf_math.cpp` 内の重複実装

### **Phase 3: ファイル分割・整理 (1ヶ月)**
**目的**: 可読性向上・複雑度低下
- [ ] **sensor_filter.hpp 分割** (846行 → 396行)
  - [ ] `Sensor/ema_filter.hpp` (90行)
  - [ ] `Sensor/biquad_filter.hpp` (130行)
  - [ ] `Sensor/outlier_detector.hpp` (200行)
  - [ ] `Sensor/robust_statistics.hpp` (150行)

- [ ] **meukf_core.cpp 分割** (1346行 → 200行 + 3新ファイル)
  - [ ] `meukf_predict.cpp` (400行)
  - [ ] `meukf_sigma_points.cpp` (350行)
  - [ ] `meukf_update.cpp` (400行)

- [ ] **インターフェース統一**
  - [ ] `filter.hpp` → `eskf_filter.hpp` に統合削除

- [ ] **未使用ファイル移動**
  - [ ] `KF/`, `EKF/` → `Archive/` (テンプレート用途のみ)
  - [ ] `interface.hpp`, `standalone.hpp` → deprecated化

### **Phase 4: 型統一・最適化 (2週間)**
**目的**: 数値安定性向上、バグリスク削減
- [ ] **float/double 統一**
  - GPS座標系データのみ double → ENU変換時点で float に
  - 内部計算はすべて float32
  - 出力も float32 (MATLAB側で double変換)

- [ ] **可変長Matrix削除**
  - `sensor_filter.hpp` の `cmath_fx::FixedMatrix` → `Matrix<R,C>` に統一

- [ ] **パフォーマンス最適化**
  - [ ] SIMD命令導入 (SSE/AVX)
  - [ ] キャッシュ局所性改善
  - [ ] プロファイリング・ベンチマーク

### **Phase 5: テスト・検証 (1-2週間)**
- [ ] 単体テスト: `run_simulation(seed, verbose)` 複数seed実行
- [ ] 回帰テスト: `run_batch_10sets()` で統計的安定性確認
- [ ] MEX vs MATLAB差分: `compare_mex_matlab_detailed()` で数値精度確認
- [ ] パリティ確認: 修正前後で推定精度変わらない

---

## � ファイルサイズ・複雑度分析

| ファイル | 行数 | 複雑度 | 関数数 | 優先順位 | 推奨アクション |
|---------|------|--------|--------|---------|---------------|
| `sensor_filter.hpp` | 846 | ★★★ | 8+ | **HIGH** | 4ファイルに分割 |
| `meukf_core.cpp` | 1346 | ★★★ | 12+ | **HIGH** | 4ファイルに分割 |
| `eskf_core.cpp` | 800+ | ★★★ | 8+ | MEDIUM | 関数の単体化・テンプレ化 |
| `fixed_matrix.hpp` | 350 | ★★ | 20+ | ✅ | 現状維持（最適化済み） |
| `sensor_updates.cpp` | 600+ | ★★★ | 10+ | MEDIUM | Innovation統一後簡潔化 |
| `math_utils.hpp` | 400+ | ★★ | 15+ | MEDIUM | カテゴリ別ヘッダ分割検討 |
| `quaternion_functions.hpp` | 174 | ★★ | 8+ | **HIGH** | インクルード最適化、重複削除 |
| `utils.hpp` | 100+ | ★ | 5+ | LOW | deprecated関数削除 |

---

## 🎯 未使用・廃止予定コード一覧

### 確認済み未使用関数

```cpp
// utils.hpp
inline void normalizeQuat(float q[4]) {      // ❌ deprecated → cquat::normalize_quat() 使用
    // 実装は normalize_quat と同一だが、命名が異なる
}

// standalone.hpp
void initialize_filter(...);                   // ❌ 呼び出し箇所なし（スタブのみ）
void run_filter_step(...);                     // ❌ 呼び出し箇所なし
void cleanup_filter();                         // ❌ 呼び出し箇所なし
```

### 削除予定コード

```cpp
// EKF/inc/ekf_core.hpp
// KF/inc/kalman_filter_core.hpp
// → MEUKF, ESKF が存在するため、テンプレート用途のみ
// → Archive/ に移動推奨
```

---

## 🔗 参考文献・関連ドキュメント

- [PHASE3_PLAN.md](../../PHASE3_PLAN.md) — 進捗・成功指標
- [ROADMAP_TO_PHASE_13.md](../../ROADMAP_TO_PHASE_13.md) — 全体ロードマップ
- [kalman/cpp/FILE_DUPLICATION_REPORT.md](../FILE_DUPLICATION_REPORT.md) — ファイル重複解析
- [kalman/cpp/Lib/README.md](../README.md) — ライブラリモジュール説明
- [CPP_INPUT_OUTPUT_SPEC.md](../markdown/CPP_INPUT_OUTPUT_SPEC.md) — 型マッピング仕様
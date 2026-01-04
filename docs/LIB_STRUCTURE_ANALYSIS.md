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

---

### 7.2 四元数演算 (`Quaternion/quaternion_functions.hpp`)

**ファイル**: `Quaternion/quaternion_functions.hpp` (約200行)

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
    struct Quaternion { float w, x, y, z; };
    
    Quaternion quat_multiply(const Quaternion& q1, const Quaternion& q2);
    void normalize(Quaternion& q);              // 方法1
    void normalize_quaternion(Quaternion& q);   // 方法2 (重複)
    void quat_normalize(float q[4]);            // 方法3 (重複)
    
    Quaternion conjugate(const Quaternion& q);
    Quaternion inverse(const Quaternion& q);
    
    void quat_to_euler_angles(...);
    void euler_angles_to_quat(...);
}
```

**⚠️ 問題**:
- **正規化処理4種実装** → 統一推奨
- **複数の命名規則** (normalize, quat_normalize, normalize_quaternion)

**実装度**: ★★★ (完全実装、パフォーマンス最適化済み)

**依存関係**: `Matrix/fixed_matrix.hpp`

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
        └───────────────────────┘      │ (四元数演算)       │
                                       └───────────────────┘
```

---

## ⚠️ 重複・不要な実装リスト

### 高優先度（即座対応）

| # | 対象 | 問題 | 削除/統合予定 | 影響度 |
|---|------|------|-------------|--------|
| 1 | **四元数正規化** | 4種実装 (normalize, quat_normalize, normalize_quaternion,他) | →  `normalize()` 1つに統一 | **HIGH** |
| 2 | **共分散対称化** | 3種実装 (symmetrize, force_symmetric, make_symmetric) | → `symmetrize()` に統一 | **HIGH** |
| 3 | **Mahalanobis距離** | 3箇所で計算 (sensor_filter.hpp, sensor_updates.cpp, eskf_core.cpp) | → `Common/Math/` に統一実装 | **MEDIUM** |
| 4 | **Innovation計算** | 2箇所分散 (sensor_updates.cpp, eskf_math.cpp) | → `sensor_updates.cpp` に統一 | **MEDIUM** |
| 5 | **IMU積分** | 2種実装 (RK2, 台形則) | → 1つに統一（RK2推奨） | **MEDIUM** |
| 6 | **filter.hpp vs eskf_filter.hpp** | 機能重複 | → `eskf_filter.hpp` に統合削除 | **MEDIUM** |

### 中優先度（近期対応）

| # | 対象 | 問題 | 削除/統合予定 | 影響度 |
|---|------|------|-------------|--------|
| 7 | **eskf_math.hpp** | Quaternion/quaternion_functions.hpp と重複 | → quaternion_functions.hpp に統合 | LOW |
| 8 | **types.hpp float/double混在** | GPS入力がdouble → float統一推奨 | → 全float32に | MEDIUM |
| 9 | **interface.hpp/standalone.hpp** | 重複インターフェース | → 削除（未使用） | LOW |
| 10 | **KF, EKF テンプレート** | 現在未使用 | → アーカイブフォルダに移動 | LOW |

---

## 🔍 詳細分析：ファイルサイズ・複雑度

| ファイル | 行数 | 複雑度 | 関数/クラス数 | 優先順位 |
|---------|------|--------|-------------|---------|
| `eskf_core.cpp` | 800+ | ★★★ | 8+ | 1 |
| `fixed_matrix.hpp` | 350 | ★★ | 20+ | 2 |
| `sensor_updates.cpp` | 600+ | ★★★ | 10+ | 1 |
| `math_utils.hpp` | 400+ | ★★ | 15+ | 2 |
| `quaternion_functions.hpp` | 200 | ★★ | 8+ | 3 |
| `sensor_filter.hpp` | 250+ | ★★ | 5+ | 3 |

---

## ✅ 推奨フェーズ計画

### Phase 1: インクルード統一 (1週間)
- [ ] インクルードパス統一 (`../Lib/` vs `Lib/` 混在解決)
- [ ] マクロ多重定義チェック
- [ ] 循環依存確認・解決

### Phase 2: 型・関数統一 (2週間)
- [ ] 正規化処理 4→1 に統一
- [ ] 対称化処理 3→1 に統一
- [ ] float/double を float32 に統一
- [ ] Mahalanobis距離 3箇所→1箇所

### Phase 3: ファイル分割・整理 (1ヶ月)
- [ ] `eskf_core.cpp` を複数ファイルに分割
- [ ] `math_utils.hpp` をカテゴリ別に分割
- [ ] `types.hpp` をセンサー型・フィルタ型に分割
- [ ] interface統一、未使用ファイル削除

### Phase 4: パフォーマンス最適化 (1ヶ月)
- [ ] SIMD 命令対応
- [ ] キャッシュ局所性改善
- [ ] プロファイリング・ベンチマーク

---

## 📚 参考：ファイル間依存関係マトリクス

```
            | Common | ESKF | MEUKF | EKF | UKF | KF | Matrix | Quaternion
-----------|--------|------|-------|-----|-----|----| -------|----------
Common     |   -    |  ←   |  ←    |  ← |  ← |  ← |  ←     |  ←
ESKF       |   →    |  -   |      |     |     |     |  →     |  →
MEUKF      |   →    |      |  -   |     |     |     |  →     |  →
EKF        |   →    |      |      |  -  |     |     |  →     |
UKF        |   →    |      |      |     |  -  |     |  →     |
KF         |   →    |      |      |     |     |  -  |  →     |
Matrix     |   -    |  -   |  -   |  -  |  -  |  -  |  -     |
Quaternion |   -    |  →   |      |     |     |     |  ←     |  -
```

**凡例**: 
- `→` : 下側が上側に依存
- `←` : 上側が下側に依存
- `-` : 依存なし

---

## 🎯 品質メトリクス

| メトリクス | 評価 | コメント |
|-----------|------|---------|
| **モジュール化度** | ★★★ | 7つの独立モジュール |
| **コードの重複** | ★★ | 同じ処理が複数実装 |
| **ドキュメント** | ★★ | ヘッダーコメント充実も、全体設計書不足 |
| **テストカバレッジ** | ★★ | MEUKF/EKF/UKFはテスト不十分 |
| **パフォーマンス** | ★★★ | ESKF+Commonは最適化済み |
| **保守性** | ★★ | インクルード混在・関数重複で低下 |
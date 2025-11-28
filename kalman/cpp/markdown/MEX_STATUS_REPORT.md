# MEX化状況レポート

## ✅ 完了済みMEX実装

### 1. **ESKF コア関数** (`cpp/ESKF/`)
- **ファイル**: `eskf_core.cpp/hpp`, `mex_eskf_core.cpp`
- **関数**:
  - `integrate_nominal` - Adams-Bashforth 2次積分
  - `update_accel` - 加速度計姿勢更新
  - `update_mag` - 磁気計Yaw更新
  - `update_gps` - GPS位置更新
  - `update_baro` - 気圧計高度更新
- **状態**: ✅ C++実装完了、MEXラッパー作成済み
- **統合**: `ESKF.m`の2関数で使用中（integrate_nominal, update_accel）

### 2. **カルマンフィルタコア** (`cpp/KF/Core/`)
- **ファイル**: `kalman_filter_core.cpp/hpp`, `mex_kalman_filter_core.cpp`
- **関数**:
  - `predict_step` - 共分散予測
  - `compute_kalman_gain` - カルマンゲイン計算
  - `update_state_covariance` - 状態・共分散更新
  - `compute_jacobian` - ヤコビアン計算
  - `regularize_covariance` - 共分散正則化
- **状態**: ✅ C++実装完了、MEXラッパー作成済み
- **統合**: `kalman_filter_core.m`から自動使用

### 3. **UKF シグマポイント** (`cpp/UKF/Core/`)
- **ファイル**: `ukf_sigma_points.cpp/hpp`, `mex_ukf_sigma_points.cpp`
- **関数**:
  - `ukf_sigma_points` - シグマポイント生成
- **状態**: ✅ C++実装完了、MEXラッパー作成済み

---

## 🔧 C++実装済み（MEXラッパー未作成）

### 4. **Common/Math ユーティリティ** (`cpp/Common/Math/`)
- **ファイル**: `math_utils.hpp`
- **クラス/関数**:
  - `MathUtils` クラス
    - 角度処理（wrap_to_pi, wrap_to_180, angle_difference）
    - ベクトル操作（normalize, clip, enforce_symmetry）
    - 数値安定化（safe_divide, safe_sqrt, safe_asin/acos）
    - 統計（median, MAD, robust_statistics）
    - 座標変換（lla_to_enu, enu_to_lla）
    - 行列分解（safe_cholesky）
- **MATLABソース**: `Common/Math/MathUtils.m`
- **状態**: ✅ C++実装完了、❌ MEXラッパー未作成
- **推奨**: ヘッダーオンリーライブラリとして他のMEXから直接使用

### 5. **Common/Sensor フィルタ** (`cpp/Common/Sensor/`)
- **ファイル**: `sensor_filter.hpp`
- **クラス**:
  - `EMAFilter` - 指数移動平均
  - `BiquadLowpassFilter` - 2次ローパス
  - `AlphaBetaFilter` - Alpha-Betaトラッキング
  - `OutlierDetector` - 外れ値検出
  - `SensorFilterLib` - 統合インターフェース
- **MATLABソース**: `Common/Sensor/SensorFilterLib.m`
- **状態**: ✅ C++実装完了、❌ MEXラッパー未作成
- **推奨**: ESKFのMEXから内部で使用（既に組み込み可能）

### 6. **Common/Validation** (`cpp/Common/Validation/`)
- **ファイル**: `validation.hpp`
- **クラス**:
  - `CovarianceRegularizer` - 共分散正則化
  - `OutlierDetector` - 外れ値検出（マハラノビス距離、Chi-square）
  - `StateValidator` - 状態妥当性チェック
  - `NoiseEstimator` - ノイズ推定
- **MATLABソース**: `Common/Validation/*.m`
- **状態**: ✅ C++実装完了、❌ MEXラッパー未作成
- **推奨**: カルマンフィルタコアから内部で使用

---

## ⚠️ 未実装（MEX化推奨）

### 7. **ESKF 補助関数** (`ESKF/Core/`)
- `ESKFCovariancePrediction.m` - 共分散予測（F行列計算）
- `ESKFErrorInjection.m` - 誤差注入
- `ESKFMeasurementUpdate.m` - 観測更新ラッパー
- `covariance_prediction_optimized.m` - 最適化版共分散予測
- `adaptive_innovation_gating.m` - 適応的イノベーションゲーティング
- `GSF_YawEstimator.m` - GSF Yaw推定器
- **推奨**: 🔴 高頻度呼び出しのためMEX化推奨

### 8. **KF Utils** (`KF/Utils/`)
- `DivergenceGuard.m` - 発散防止
- `NoiseEstimator.m` - ノイズ推定
- `BiquadFilter.m` - Biquadフィルタ
- `AccelFilter.m` - 加速度フィルタ
- `alpha_beta_step.m` - Alpha-Betaステップ
- `ema_update.m` - EMA更新
- **推奨**: 🟡 使用頻度次第でMEX化検討

### 9. **Sensor専用フィルタ** (`KF/Utils/`)
- `SensorAccelFilter.m`
- `SensorGyroFilter.m`
- `SensorMagFilter.m`
- `SensorGPSFilter.m`
- `SensorBaroFilter.m`
- `SensorFilterFactory.m`
- **推奨**: 🟢 Common/Sensorの統合版があるため優先度低

### 10. **UKF追加関数** (`UKF/Core/`)
- `ukf_update.m` - UKF更新ステップ
- **推奨**: 🟡 UKFクラスから頻繁に呼ばれるためMEX化検討

---

## 📊 MEX化優先度

| 優先度 | カテゴリ | 理由 |
|--------|----------|------|
| 🔴 **最高** | ESKF補助関数 | 毎ステップ呼び出し、計算量大 |
| 🟠 **高** | UKF update | UKFシミュレーション高速化 |
| 🟡 **中** | KF Utils (DivergenceGuard, NoiseEstimator) | 使用頻度は高いが計算量は中程度 |
| 🟢 **低** | Sensor個別フィルタ | Commonライブラリで代替可能 |
| ⚪ **不要** | Graph, GenerateData, FFT | ビジュアライゼーション・前処理 |

---

## 🎯 次のアクション

### フェーズ1: 現在のMEXビルド（修正版）
```matlab
cd('cpp')
build_mex  % 3つのMEXをビルド
```

### フェーズ2: ESKF補助関数のMEX化
1. `ESKFCovariancePrediction` → C++実装
2. `covariance_prediction_optimized` → C++実装
3. `ESKFErrorInjection` → C++実装
4. MEXラッパー作成
5. `ESKF.m`から呼び出し

### フェーズ3: UKF updateのMEX化
1. `ukf_update` → C++実装
2. MEXラッパー作成
3. `UKF.m`から呼び出し

### フェーズ4: Common libraryの統合
1. 既存MEXから`math_utils.hpp`, `sensor_filter.hpp`, `validation.hpp`を活用
2. 必要に応じて独立MEXラッパー作成

---

## 📈 期待される性能向上

| 実装 | 現在 | 完全MEX化後 | 速度向上 |
|------|------|-------------|----------|
| ESKF.integrate_nominal | ✅ MEX | ✅ MEX | - |
| ESKF.update_accel | ✅ MEX | ✅ MEX | - |
| ESKF.predict (F行列) | ⚠️ MATLAB | → C++ | **5-10倍** |
| ESKF.update_* (残り) | ⚠️ MATLAB | → C++ | **3-7倍** |
| KalmanFilterCore | ✅ MEX | ✅ MEX | - |
| UKF.update | ⚠️ MATLAB | → C++ | **4-8倍** |
| 全体シミュレーション | - | - | **3-5倍** |

---

## ✅ ビルド済みMEXファイル

確認コマンド:
```matlab
cd('cpp')
dir('*.mexw64')
```

期待されるファイル:
- `mex_kalman_filter_core.mexw64` ✅
- `mex_ukf_sigma_points.mexw64` ✅
- `mex_eskf_core.mexw64` ⚠️ (ビルドエラー修正済み、再ビルド必要)

---

**作成日**: 2025年11月18日  
**ステータス**: 基盤MEX完了、補助関数MEX化待ち

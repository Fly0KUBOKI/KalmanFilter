# ESKF MATLAB→MEX段階的移行計画

**作成日**: 2025年12月24日  
**目的**: ESKFクラスの残存MATLAB実装をC++ MEX化し、計算性能を向上させる  
**戦略**: 依存関係の少ないレイヤーから段階的に移行、各段階でビルド・バッチテストで検証

---

## 1. 現在のESKF.mの関数構成と役割

### 1.1 主要パブリックメソッド

| メソッド | 行番号 | 役割 | 現状 | 優先度 |
|---------|--------|------|------|--------|
| **ESKF()** | 33-283 | コンストラクタ・初期化 | MATLAB | 低 |
| **predict()** | 417-554 | IMU予測ステップ（核心） | MATLAB | 高 |
| **update_filter()** | 555-579 | メインループ呼び出し | MATLAB (薄い) | 低 |
| **sensor_updates()** | 580-595 | センサー更新分岐 | MATLAB (薄い) | 低 |
| **update_sensor_impl()** | 600-720 | センサー変更検知・前処理 | MATLAB | 中 |
| **do_cpp_update()** | 723-895 | C++呼び出しラッパー | MATLAB | 中 |
| **call_meukf_step()** | 896-910 | MEX_meukf_step_v2呼び出し | MATLAB (薄い) | 低 |
| **reset()** / **check_and_reset_impl()** | 911-978 | 発散チェック・リセット | MATLAB | 中 |
| **zupt()** / **check_stationary_impl()** | 979-1047 | 静止判定・ZUPT更新 | MATLAB | 低 |
| **divergence_check_velocity_impl()** | 1068-1095 | 速度クリップ・分散制限 | MATLAB | 中 |

### 1.2 ヘルパー・ユーティリティ

| メソッド | 行番号 | 役割 | 現状 |
|---------|--------|------|------|
| **utils()** | 362-372 | ユーティリティ分岐 | MATLAB (薄い) |
| **get_euler_impl()** | 374-377 | オイラー角取得 | MATLAB（MEX呼び出し） |
| **get_field_impl()** | 379-406 | フィールド検索・変換 | MATLAB |
| **has_field_impl()** | 408-415 | フィールド存在確認 | MATLAB |
| **divergence_check()** | 357-360 | 発散チェック分岐 | MATLAB (薄い) |
| **estimate_noise()** | 340-355 | ノイズ推定 | MATLAB (薄い) |
| **get_sensor_R()** | 328-338 | センサー分散取得 | MATLAB (薄い) |
| **reset_sensor_filters()** | 307-318 | フィルタリセット | MATLAB |
| **delete()** | 1059-1066 | デストラクタ | MATLAB (薄い) |

### 1.3 プロパティ初期化（コンストラクタ内）

**状態変数**: `p`, `v`, `q`, `ba`, `bg`, `P`（MATLAB→MEX化済）  
**ノイズパラメータ**: `Q`（MATLAB→MEX化済）  
**フィルタ設定**: `noiseEstimator`, `sensor_filters`, `divergence_guard`  
**バッファ・フラグ**: `prev_accel`, `prev_gyro`, `zupt_counter` など

---

## 2. 既存C++ファイル構造の分析

### 2.1 ディレクトリ構成

```
cpp/
├── bin/                          # MEXバイナリ出力
│   ├── mex_*.mexw64             # 既存MEX実装(15個)
│
├── include/
│   ├── Common/                  # 共通型・ヘッダ
│   │   ├── filter_interface.hpp # センサー入出力構造体
│   │   ├── Math/
│   │   │   ├── fixed_matrix.hpp # float固定サイズ行列
│   │   │   ├── quaternion.hpp   # クォータニオン演算
│   │   │   └── ...
│   │   ├── Sensor/
│   │   │   └── sensor_filter.hpp# センサーフィルタ
│   │   └── Validation/
│   │
│   ├── EKF/
│   ├── ESKF/                    # ESKF固有ヘッダ
│   │   └── eskf_core.hpp        # メインESKFクラス
│   ├── KF/                      # Kalmanフィルタ基本
│   ├── MEUKF/
│   ├── UKF/
│   └── kalman_filters.hpp       # 統合エクスポート
│
├── src/                         # C++実装（不使用）
│   ├── Common/
│   ├── EKF/
│   ├── ESKF/
│   │   └── eskf_core.cpp        # ESKFコア実装
│   └── ...
│
└── MEX/                         # MEX実装ファイル
    ├── mex_meukf_step_v2.mexw64 # 現在のメイン実装
    ├── mex_sensor_filter.mexw64 # センサーフィルタ
    ├── mex_quaternion_lib.mexw64# クォータニオン
    └── ...
```

### 2.2 既存MEX実装状況

**完全なMEX実装**:
- `mex_meukf_step_v2.cpp`: 予測＋全センサー更新（UKF/MEUKF使用）
- `mex_sensor_filter.cpp`: センサー異常検知・ノイズ推定
- `mex_quaternion_lib.cpp`: クォータニオン演算（to_euler, multiply, from_euler等）
- `mex_unified_filter.cpp`: 統合フィルタ（実験的）

**部分的MEX**: `mex_ukf_update.cpp`, `mex_eskf_step.cpp` (legacy)

### 2.3 既存C++型システム

```cpp
// Common/Math/fixed_matrix.hpp で定義
namespace cmath_fx {
    template<int Rows, int Cols, typename Scalar>
    class Matrix { /* float のみ対応 */ };
    
    using Vector3 = Vector<3, float>;
    using Vector4 = Vector<4, float>;
    using Matrix15x15 = Matrix<15, 15, float>;
}

// Common/filter_interface.hpp で定義
struct SensorInput {
    Vector3 accel, gyro, mag, gps_pos;
    Scalar pressure;
    bool accel_updated, gyro_updated, mag_updated, gps_updated, baro_updated;
    Scalar dt;
};

struct FilterOutput {
    Vector3 position, velocity, euler, accel_bias, gyro_bias;
    Vector4 quaternion;
};

struct FilterParams {
    Matrix3x3 Q_gyro, Q_accel, Q_gyro_bias, Q_accel_bias;
    Matrix3x3 R_mag, R_gps_pos, R_gps_vel;
    Scalar R_baro;
    Vector3 gravity, mag_world, gps_origin;
};
```

---

## 3. MATLAB実装の分類と依存関係

### 3.1 階層モデル

```
Layer 4 (高レベル)
├─ update_filter()               [薄い・外部インタフェース]
├─ sensor_updates() 分岐         [薄い・分岐のみ]
└─ reset() 系                    [中・リセット＆検証]

Layer 3 (中レベル)
├─ update_sensor_impl()          [厚い・変更検知・前処理]
├─ do_cpp_update()               [中・パラメータ整形]
├─ divergence_check()            [薄い・分岐]
└─ divergence_check_velocity_impl()[中・物理制約)

Layer 2 (低レベル・ロジック)
├─ predict()                     [厚い・IMU積分・Q適応]
├─ zupt()系                       [中・静止判定]
└─ check_and_reset_impl()        [中・GPS統合)

Layer 1 (基盤・ユーティリティ)
├─ get_euler_impl()              [薄い・MEX呼び出し]
├─ get_field_impl()              [薄い・フィールド抽出]
├─ has_field_impl()              [薄い・フィールド確認]
└─ コンストラクタ初期化          [厚い・初期状態計算]
```

### 3.2 依存グラフ（縮約版）

```
predict()
  ↓ (呼び出し: mex_meukf_step)
do_cpp_update()
  ↓
  ├─ noiseEstimator.getRnoise()      [MEX優先]
  ├─ divergence_guard.check...()     [MEX優先]
  └─ call_meukf_step() → mex_meukf_step_v2

update_filter()
  ↓
  ├─ predict()
  ├─ sensor_updates()
  │   ↓
  │   └─ update_sensor_impl()        [変更検知後、do_cpp_updateを呼び出し]
  └─ reset() → check_and_reset_impl()

check_and_reset_impl()
  ↓
  └─ reset_filter_impl()             [GPS統合時に計算]

zupt() 系
  ↓
  ├─ check_stationary_impl()         [判定ロジック]
  └─ update_zupt_impl()              [ZUPT更新]

utils('get_field')  ← get_field_impl()
utils('get_euler')  ← get_euler_impl() [mex_quaternion_lib呼び出し]
```

---

## 4. 移行計画のロードマップ

### フェーズ1: 基盤レイヤー（影響最小・テスト容易）

**対象関数**:
1. `get_field_impl()`
2. `has_field_impl()`
3. `get_euler_impl()`

**C++設計**:
```cpp
// New: include/Common/MATLAB/matlab_helpers.hpp
namespace matlab_helpers {
    // フィールド抽出（obs構造体をベクトル化）
    template<typename T>
    std::vector<float> get_field(const MATLABStruct& obs, 
                                  const std::vector<std::string>& candidates,
                                  int index, int num_cols);
    
    // フィールド確認
    bool has_field(const MATLABStruct& obs, 
                   const std::vector<std::string>& candidates);
    
    // オイラー角取得（既存 mex_quaternion_libを使用）
    Vector3 get_euler_from_quat(const Vector4& q);
}
```

**実装**:
- `MEX/mex_matlab_helpers.cpp` 新規作成
- 既存の `mex_quaternion_lib.cpp` を再利用
- テスト: `get_field_impl` → `mex_matlab_helpers('get_field', ...)` に置き換え

**期待効果**: 依存性なし、テスト容易  
**リスク**: 低（基盤レイヤー、MATLAB互換性影響なし）

**ステップ**:
1. C++実装: `mex_matlab_helpers.cpp`
2. build_mex.m に追加
3. MATLAB側呼び出しを置き換え
4. テスト: `run_simulation(42, true)` で基本動作確認

---

### フェーズ2: ユーティリティ・助言関数（Low-hanging fruit）

**対象関数**:
1. `divergence_check_velocity_impl()` → MEXユーティリティ化
2. `estimate_noise()` → 既存 `mex_sensor_filter` に統合
3. `reset_sensor_filters()` → 既存 `mex_sensor_filter` に統合
4. `get_sensor_R()` → 既存 `mex_sensor_filter` に統合

**C++設計**:
- 既存 `mex_sensor_filter.cpp` を拡張
- 新規関数: `divergence_clip_velocity()`, `divergence_regularize_covariance()`
- インタフェース: MATLAB struct → C++構造体

**期待効果**: 
- MATLAB層を簡素化
- センサーフィルタと発散ガード統合

**リスク**: 低〜中（既存構造に追加のみ）

**ステップ**:
1. `mex_sensor_filter.cpp` にメソッド追加
2. MATLAB側で条件分岐を簡略化
3. テスト: `run_simulation(42, true)` 

---

### フェーズ3: センサー前処理層（中核機能）

**対象関数**:
- `update_sensor_impl()` の変更検知＋前処理ロジック
  - 加速度異常検知
  - 磁気計異常検知
  - GPS座標変換
  - 気圧計高度換算

**C++設計**:
```cpp
// New: include/ESKF/sensor_preprocessor.hpp
namespace eskf {
    struct SensorPreprocessOutput {
        bool is_outlier;
        Vector3 corrected_measurement;
        float confidence;
    };
    
    class SensorPreprocessor {
    public:
        SensorPreprocessOutput preprocess_accel(const Vector3& a_meas, const Vector3& prev_a);
        SensorPreprocessOutput preprocess_mag(const Vector3& m_meas, const Vector3& prev_m);
        SensorPreprocessOutput preprocess_gps(float lat, float lon, float alt, 
                                              const Vector3& origin, bool first_time);
        SensorPreprocessOutput preprocess_baro(float pressure);
    };
}
```

**実装**:
- `src/ESKF/sensor_preprocessor.cpp` 新規作成
- `MEX/mex_sensor_preprocessor.cpp` 新規作成
- `mex_sensor_filter.cpp` の異常検知ロジック再利用

**期待効果**: MATLAB側 `update_sensor_impl()` を削減  
**リスク**: 中（ロジック複雑、数値精度影響）

**ステップ**:
1. C++実装: `src/ESKF/sensor_preprocessor.cpp`
2. MEX作成: `mex_sensor_preprocessor.cpp`
3. MATLAB `update_sensor_impl()` の前処理を置き換え
4. テスト: 
   - `run_simulation(seed, true)` で精度確認
   - `run_batch_10sets()` で安定性確認

---

### フェーズ4: 予測ステップのコア化（最優先・最高効果）

**対象関数**:
- `predict()` の核心ロジック
  - ジャイロ・加速度フィルタ適用
  - Adaptive Q計算
  - MATLAB側 `call_meukf_step()` をラップ
  - 速度クリップ・分散制限

**現状**:
- `mex_meukf_step_v2` でIMU積分＋センサー更新を一括処理
- MATLAB側で補足的な処理（フィルタ適用、Q適応）

**C++設計**:
```cpp
// New: include/ESKF/adaptive_predict.hpp
namespace eskf {
    struct AdaptivePredictParams {
        bool enable_gyro_filter;
        bool enable_accel_filter;
        bool adaptive_q_enabled;
        float velocity_damping;
    };
    
    class AdaptivePredictor {
    public:
        // MATLAB predict()全体をC++化
        static void predict_step(
            Vector3& p, Vector3& v, Vector4& q, Vector3& ba, Vector3& bg,
            Matrix15x15& P,
            const Vector3& a_meas, const Vector3& w_meas,
            const Vector3& w_body_prev,
            const Matrix15x15& Q_nominal,
            const AdaptivePredictParams& params,
            float dt, const Vector3& g
        );
    };
}
```

**実装**:
- `src/ESKF/adaptive_predict.cpp` 新規作成
- 既存 `mex_meukf_step_v2` を再利用＋ラップ
- パラメータ受け渡し最小化

**期待効果**: 
- 予測ステップ最適化（速度向上）
- 数値精度向上（float一貫性）

**リスク**: 高（コア機能、数値差異可能性）

**ステップ**:
1. C++実装: `src/ESKF/adaptive_predict.cpp`
2. 既存 `mex_meukf_step_v2` との結合検討
3. MATLAB `predict()` を置き換え
4. テスト:
   - `run_simulation(42, true)` で差分確認
   - `compare_mex_matlab_detailed()` で詳細検証
   - `run_batch_10sets()` で回帰テスト

---

### フェーズ5: リセット・発散チェック

**対象関数**:
- `check_and_reset_impl()`
- `reset_filter_impl()`
- `zupt()` 系

**C++設計**:
```cpp
// New: include/ESKF/filter_management.hpp
namespace eskf {
    class FilterManager {
    public:
        bool check_divergence(const Vector3& p, const Vector3& v, 
                             const Vector4& q, const Matrix15x15& P);
        void reset_state(Vector3& p, Vector3& v, Vector4& q, Vector3& ba, Vector3& bg,
                        Matrix15x15& P);
        void reset_with_gps(Vector3& p, const Vector3& gps_origin, 
                           float lat, float lon, float alt);
        
        bool check_stationary(const Vector3& a_meas, const Vector3& w_meas,
                             float threshold_a, float threshold_w);
        void apply_zupt(Vector3& v, Matrix15x15& P);
    };
}
```

**期待効果**: 発散ガード一元化  
**リスク**: 中（物理制約ロジック）

**ステップ**:
1. C++実装: `src/ESKF/filter_management.cpp`
2. MEX作成: `mex_filter_management.cpp`
3. MATLAB側を置き換え
4. テスト: リセット・ZUPT動作確認

---

### フェーズ6: 初期化（最後）

**対象**: `ESKF()` コンストラクタ

**現状**: 複雑な初期パラメータ計算（MATLAB統計処理）

**戦略**: 
- MATLAB側で初期化を維持（複雑な信号処理）
- MEX化は不要（初期化1回のみ）

**結論**: **フェーズ6では実施しない**

---

## 5. ビルド・テスト戦略

### 5.1 各フェーズのビルド手順

```matlab
% フェーズ1: 基盤
cd kalman/cpp/build
build_mex({'mex_matlab_helpers'})
clear mex
cd ../..
run_simulation(42, true)

% フェーズ2: ユーティリティ拡張
build_mex({'mex_sensor_filter'})
clear mex
run_simulation(42, true)

% フェーズ3: センサー前処理
build_mex({'mex_sensor_preprocessor'})
clear mex
run_simulation(42, true)

% フェーズ4: 予測ステップ（最重要）
build_mex({'mex_meukf_step_v2'})  % または新規 adaptive_predict
clear mex
run_simulation(42, true)
compare_mex_matlab_detailed()
run_batch_10sets()

% フェーズ5: フィルタ管理
build_mex({'mex_filter_management'})
clear mex
run_batch_10sets()
```

### 5.2 テスト項目

**各フェーズ共通**:

1. **基本動作確認**
   ```matlab
   run_simulation(42, true)  % シード42で確定的テスト
   % 出力: Results/estimation_0.csv で最初の数行を目視確認
   ```

2. **数値差分確認**
   ```matlab
   compare_mex_matlab_detailed()  % MATLAB vs MEX比較
   % 許容差: 1e-5（float精度）以内
   ```

3. **回帰テスト**
   ```matlab
   run_batch_10sets()  % 10セット連続実行
   % 各セット: RMS Error が許容値以内か確認
   ```

4. **パフォーマンス測定**
   ```matlab
   tic; run_simulation(42, true); toc  % 実行時間計測
   % 期待: フェーズ進行につれて高速化
   ```

### 5.3 検証クライテリア

| フェーズ | 数値精度 | 安定性 | 性能向上 |
|---------|---------|--------|---------|
| Phase 1 | EXACT | STABLE | 1.0x |
| Phase 2 | <1e-5 | STABLE | 1.05x |
| Phase 3 | <1e-4 | STABLE | 1.15x |
| Phase 4 | <1e-4 | STABLE | 1.5x |
| Phase 5 | <1e-4 | STABLE | 1.6x |

---

## 6. C++実装ガイドライン

### 6.1 コーディング規約

**型システム**:
- ✅ `float` のみ使用（double不可）
- ✅ `cmath_fx::Vector<N, float>` 使用
- ✅ `cmath_fx::Matrix<R, C, float>` 使用
- ❌ Eigen ライブラリ禁止
- ❌ STL `std::vector<double>` 禁止

**入出力インタフェース**:
- MATLAB struct → C++構造体への変換は `mex_type_conv.hpp` を使用
- 状態ベクトル: 厳格に `[p(3), v(3), q(4), ba(3), bg(3)]` = 15要素
- クォータニオン: 必ず `[w, x, y, z]` スカラー先頭

**共分散対称化**:
```cpp
// 出力前に必ず実施
P = (P + P.transpose()) / 2.0f;
```

**メモリ管理**:
- スタック割り当てのみ（`fixed_matrix` は固定サイズ）
- 動的割り当て禁止

### 6.2 既存構造の適合方法

**例: Phase 1 - mex_matlab_helpers.cpp**
```cpp
#include <mex.h>
#include "../include/Common/Math/fixed_matrix.hpp"
#include "../include/Common/filter_interface.hpp"

using Vector3 = cmath_fx::Vector<3, float>;
using Vector4 = cmath_fx::Vector<4, float>;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Input: command (string)
    if (nrhs < 1) mexErrMsgIdAndTxt("mex_matlab_helpers:input", "Usage: ...");
    
    std::string cmd = mxArrayToString(prhs[0]);
    
    if (cmd == "get_field") {
        // Extract field from MATLAB struct
        // Type conversion: prhs[1] (struct) → C++ Vector
        // ...implementation using cmath_fx
    } else if (cmd == "has_field") {
        // Check field existence
    }
}
```

### 6.3 テスト・デバッグ

**デバッグ出力**:
```cpp
// 本番コード内では使用禁止
#ifdef DEBUG_MATLAB_HELPERS
    mexPrintf("Debug: value = %f\n", value);
#endif
```

**単体テスト**: 各MEX関数に対し MATLAB スクリプトで最小ケース検証

---

## 7. 既存MEX実装との結合点

### mex_meukf_step_v2 との統合

**現在**:
```matlab
% predict() 内で
state_out = obj.call_meukf_step(state_in, sensor_data, mex_params);
```

**フェーズ4後の設計**:
```cpp
// adaptive_predict.cpp でラップ
Vector3 p, v; Vector4 q; Vector3 ba, bg;
Matrix15x15 P;
// ...
ESKFCore::integrate_nominal(p, v, q, ba, bg, a, w, dt, g, ...);
// または
AdaptivePredictor::predict_step(...);  // こちらをフェーズ4で実装
```

### mex_sensor_filter との連携

**Phase 2**: 既存機能拡張
```cpp
// mex_sensor_filter.cpp内に新規メソッド
"divergence_clip_velocity", "divergence_regularize"
```

**Phase 3**: 前処理統合
```cpp
// mex_sensor_preprocessor.cpp 新規
// または mex_sensor_filter.cpp にさらに拡張
"preprocess_accel", "preprocess_mag", "preprocess_gps"
```

---

## 8. リスク管理

### 8.1 主要リスク

| リスク | 発生可能性 | 影響度 | 対策 |
|--------|----------|--------|------|
| float精度不足 | 中 | 高 | Phase 4で`compare_mex_matlab_detailed()`実施 |
| 共分散発散 | 低 | 高 | 共分散対称化ルール遵守 |
| クォータニオン順序誤り | 低 | 高 | [w,x,y,z]を徹底確認 |
| MEX更新ビルド失敗 | 低 | 中 | build_mex()エラーハンドリング改善 |
| 数値差異累積 | 中 | 中 | 定期的な`run_batch_10sets()`検証 |

### 8.2 ロールバック計画

各フェーズ完了後、検証失敗時:
1. MEX バイナリを `bin/` から削除
2. MATLAB の `call_*()` 呼び出しをコメント化
3. 前フェーズの実装に戻す

---

## 9. 実装スケジュール（参考）

| フェーズ | 作業項目 | 推定工数 |
|---------|---------|---------|
| **Phase 1** | mex_matlab_helpers 作成・テスト | 1-2日 |
| **Phase 2** | mex_sensor_filter 拡張 | 1-2日 |
| **Phase 3** | mex_sensor_preprocessor 新規 | 2-3日 |
| **Phase 4** | adaptive_predict 実装・検証 | 3-5日 |
| **Phase 5** | filter_management 実装 | 1-2日 |
| **総計** | | 8-14日 |

---

## 10. 統合フェーズ（Phase 6-10）

**現状**: Phase 1-5のMEXファイルはビルド済みだが、ESKF.mに統合されていない。

### Phase 6: Phase 1統合（基盤レイヤー）

**対象**: `get_field_impl()`, `has_field_impl()`

**実装手順**:
```matlab
% get_field_impl() の置き換え
function data = get_field_impl(obj, obs, field_names, idx, num_cols)
    if exist('mex_matlab_helpers', 'file') == 3
        data = mex_matlab_helpers('get_field', obs, field_names, idx, num_cols);
    else
        % MATLAB フォールバック（既存実装）
        for i = 1:length(field_names)
            if isfield(obs, field_names{i})
                % ...既存ロジック...
            end
        end
    end
end

% has_field_impl() の置き換え
function tf = has_field_impl(obj, obs, field_names)
    if exist('mex_matlab_helpers', 'file') == 3
        tf = mex_matlab_helpers('has_field', obs, field_names);
    else
        % MATLAB フォールバック
        tf = false;
        for ii = 1:length(field_names)
            if isfield(obs, field_names{ii})
                tf = true; return;
            end
        end
    end
end
```

**テスト**: `run_simulation(42, true)` で動作確認

---

### Phase 7: Phase 4統合（予測ステップ）⭐最重要⭐

**対象**: `predict()` メソッド

**実装手順**:
```matlab
function predict(obj, a_meas, w_meas)
    % NaN check
    if any(isnan(obj.p)) || any(isnan(obj.v)) || any(isnan(obj.q)) || any(isnan(obj.P(:)))
        warning('ESKF:predict:NaN', 'NaN detected before predict');
        return;
    end
    
    % MEX実装を使用（存在する場合）
    if exist('mex_adaptive_predict', 'file') == 3
        % 状態構造体準備
        state_in.p = obj.p(:);
        state_in.v = obj.v(:);
        state_in.q = obj.q(:);
        state_in.ba = obj.ba(:);
        state_in.bg = obj.bg(:);
        state_in.P = obj.P;
        
        % パラメータ準備
        params.g = obj.g;
        params.dt = obj.dt;
        params.Q_nominal = obj.Q_nominal;
        params.adaptive_q_enabled = obj.adaptive_q_enabled;
        params.enable_gyro_filter = isprop(obj, 'enable_gyro_filter') && obj.enable_gyro_filter;
        params.enable_accel_filter = ~isempty(obj.accel_filter);
        params.velocity_damping = obj.velocity_damping;
        params.w_body_prev = obj.w_body;
        
        % MEX呼び出し
        state_out = mex_adaptive_predict('predict', state_in, a_meas, w_meas, params);
        
        % 状態更新
        obj.p = state_out.p;
        obj.v = state_out.v;
        obj.q = state_out.q;
        obj.ba = state_out.ba;
        obj.bg = state_out.bg;
        obj.P = state_out.P;
        
        % 補助変数更新
        obj.w_body = w_meas;
        obj.quaternion_norm = norm(obj.q);
        
        return;
    end
    
    % MATLAB フォールバック（既存実装）
    % ...既存のpredict()ロジック...
end
```

**テスト**: 
- `run_simulation(42, true)` で動作確認
- `run_batch_10sets()` で回帰テスト
- 数値精度確認（位置RMS < 1e-3 m, 速度RMS < 1e-4 m/s）

---

### Phase 8: Phase 3統合（センサー前処理）

**対象**: `update_sensor_impl()` の前処理部分

**実装手順**:
```matlab
function update_sensor_impl(obj, sensor_type, varargin)
    switch sensor_type
        case 'accel'
            a_meas = varargin{1};
            
            % MEX前処理を使用
            if exist('mex_sensor_preprocessor', 'file') == 3
                [a_corrected, is_outlier] = mex_sensor_preprocessor('preprocess_accel', a_meas, obj.prev_accel);
                if is_outlier || any(isnan(a_corrected))
                    return;
                end
                obj.prev_accel = a_meas;
                do_cpp_update(obj, 'accel', a_corrected);
            else
                % MATLAB フォールバック（既存実装）
                % ...既存ロジック...
            end
            
        case 'mag'
            m_meas = varargin{1};
            if exist('mex_sensor_preprocessor', 'file') == 3
                [m_filtered, is_outlier] = mex_sensor_preprocessor('preprocess_mag', m_meas, obj.prev_mag);
                if is_outlier || any(isnan(m_filtered))
                    return;
                end
                obj.prev_mag = m_meas;
                do_cpp_update(obj, 'mag', m_filtered);
            else
                % MATLAB フォールバック
            end
            
        case 'gps'
            lat = varargin{1}; lon = varargin{2}; alt = varargin{3};
            if exist('mex_sensor_preprocessor', 'file') == 3
                [z_gps, is_outlier] = mex_sensor_preprocessor('preprocess_gps', lat, lon, alt, obj.gps_origin);
                if is_outlier
                    return;
                end
                obj.prev_gps_lat = lat; obj.prev_gps_lon = lon; obj.prev_gps_alt = alt;
                do_cpp_update(obj, 'gps', z_gps);
            else
                % MATLAB フォールバック
            end
            
        case 'baro'
            pressure = varargin{1};
            if exist('mex_sensor_preprocessor', 'file') == 3
                [alt_baro, is_outlier] = mex_sensor_preprocessor('preprocess_baro', pressure);
                if is_outlier || any(isnan(alt_baro))
                    return;
                end
                obj.prev_baro = pressure;
                do_cpp_update(obj, 'baro', alt_baro);
            else
                % MATLAB フォールバック
            end
    end
end
```

**テスト**: `run_simulation(42, true)` + `run_batch_10sets()`

---

### Phase 9: Phase 5統合（フィルタ管理）

**対象**: `check_and_reset_impl()`, `reset_filter_impl()`, `check_stationary_impl()`, `update_zupt_impl()`

**実装手順**:
```matlab
function check_and_reset_impl(obj, obs, k)
    if isempty(k); return; end
    
    if exist('mex_filter_management', 'file') == 3
        % MEX実装を使用
        diverged = mex_filter_management('check_divergence', obj.P);
        if diverged
            if ~isempty(obs)
                reset_filter_impl(obj, obs, k);
            else
                [obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P] = ...
                    mex_filter_management('reset_state', obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P, 0.01);
            end
        end
    else
        % MATLAB フォールバック（既存実装）
        % ...既存ロジック...
    end
end

function is_stat = check_stationary_impl(obj, a_meas, w_meas)
    if exist('mex_filter_management', 'file') == 3
        % MEX実装（必要に応じて実装）
        % 現状はMATLAB実装を維持
    end
    % MATLAB フォールバック（既存実装）
    % ...既存ロジック...
end

function update_zupt_impl(obj)
    if ~obj.is_stationary; return; end
    
    if exist('mex_filter_management', 'file') == 3
        [v_out, P_out] = mex_filter_management('apply_zupt', obj.v, obj.P);
        obj.v = v_out;
        obj.P = P_out;
    else
        % MATLAB フォールバック（既存実装）
        % ...既存ロジック...
    end
end
```

**テスト**: `run_batch_10sets()` でリセット・ZUPT動作確認

---

### Phase 10: Phase 2完了（発散チェック）

**対象**: `divergence_check_velocity_impl()`

**実装手順**:
```matlab
function [vel_out, P_out, was_clipped] = divergence_check_velocity_impl(obj, vel_in, P_in, vel_indices)
    if exist('mex_sensor_filter', 'file') == 3
        % mex_sensor_filterにdivergence_clip_velocity機能を追加する必要がある
        % または、既存のMATLAB実装を維持
    end
    % MATLAB フォールバック（既存実装）
    % ...既存ロジック...
end
```

---

## 11. 統合スケジュール（再計画）

| Phase | 作業内容 | 推定工数 | 優先度 |
|-------|---------|---------|--------|
| **Phase 6** | Phase 1統合（get_field, has_field） | 0.5日 | 低 |
| **Phase 7** | Phase 4統合（predict）⭐ | 2-3日 | **最高** |
| **Phase 8** | Phase 3統合（update_sensor_impl前処理） | 1-2日 | 高 |
| **Phase 9** | Phase 5統合（reset, ZUPT） | 1日 | 中 |
| **Phase 10** | Phase 2完了（divergence_check_velocity） | 0.5日 | 低 |
| **総計** | | 5-7日 | |

---

## 12. 参考資料

- [型混在分析レポート](TYPE_MIX_REPORT.md): float/double混在の既知問題
- [既存ビルド手順](../../cpp/build/build_mex.m): MEX コンパイル方法
- [ESKF クラス](../../ESKF.m): 現在の実装
- [テスト スクリプト](../../run_simulation.m): 単体テスト
- [バッチテスト](../../run_batch_10sets.m): 回帰テスト
- [進捗状況](MIGRATION_PROGRESS.md): 最新の進捗状況

---

**現状**: Phase 1-5のMEXファイルはビルド済み  
**次ステップ**: Phase 6（Phase 1統合）から開始、またはPhase 7（Phase 4統合）を優先


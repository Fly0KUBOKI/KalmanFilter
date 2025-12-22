# ファイル構造と MEX 化状況レポート

**作成日**: 2025年12月22日  
**目的**: MATLAB実装の依存関係整理とMEX化未完了ファイルの列挙

---

## 📊 実行フロー概要

```
run_simulation.m
├─ sim_generate.m (データ生成)
├─ read_csv.m (CSV読み込み)
├─ config_params.m (パラメータ設定)
├─ ESKF.m (メインフィルタクラス)
│   ├─ predict.m ──────────► mex_meukf_step_v2
│   ├─ sensor_updates.m ───► mex_meukf_step_v2
│   ├─ zupt.m ─────────────► mex_meukf_step_v2
│   └─ utils.m (内部ユーティリティ)
├─ SensorFilters.m ────────► mex_sensor_filter
│   ├─ accel(), mag(), gps(), baro()
│   ├─ get_R(), noise_estimate()
│   └─ divergence_check(), divergence_regularize()
└─ Graph/plot_*.m (可視化)
```

---

## 🟢 MEX化完了（C++実装済み）

### メインMEXバイナリ（cpp/bin/）

| MEX ファイル | 主な機能 | 呼び出し元 |
|-------------|---------|-----------|
| `mex_meukf_step_v2.mexw64` | ESKF予測・更新ループ | `predict.m`, `sensor_updates.m` |
| `mex_sensor_filter.mexw64` | センサーフィルタ統合 | `SensorFilters.m` |
| `mex_quaternion_lib.mexw64` | クォータニオン演算 | `ESKF.m` |
| `mex_eskf_core.mexw64` | ESKF基本計算 | (内部) |
| `mex_eskf_math.mexw64` | ESKF数学関数 | (内部) |
| `mex_ukf_update.mexw64` | UKF更新 | (内部) |
| `mex_ukf_sigma_points.mexw64` | シグマ点生成 | (内部) |
| `mex_unified_filter.mexw64` | 統合フィルタ | (内部) |

### C++ ヘッダー依存関係

```
cpp/include/
├─ Common/
│   ├─ Sensor/
│   │   └─ sensor_filter.hpp    ← センサーフィルタ・ノイズ推定・外れ値検出
│   ├─ Math/                    ← 行列・クォータニオン演算
│   └─ Validation/              ← バリデーションユーティリティ
├─ ESKF/
│   ├─ eskf_core.hpp            ← ESKF コア計算
│   ├─ eskf_math.hpp            ← 数学関数
│   └─ eskf_helper.hpp          ← ヘルパー関数
├─ MEUKF/
│   ├─ unified_filter.hpp       ← MEUKF 統合フィルタ
│   └─ unified_types.hpp        ← 型定義
└─ UKF/
    ├─ ukf_core.hpp             ← UKF コア
    └─ ukf_sigma_points.hpp     ← シグマ点
```

---

## 🟡 部分的にMEX化（MATLABフォールバックあり）

これらのファイルは MEX が利用可能な場合は MEX を呼び出し、利用不可能な場合は MATLAB にフォールバックします。

| MATLAB ファイル | MEX 連携 | フォールバック有無 | 備考 |
|----------------|---------|------------------|------|
| `KF/Utils/SensorFilters.m` | `mex_sensor_filter` | ✅ あり（現在コメントアウト中） | 統合ラッパー |
| `KF/Utils/SensorAccelFilter.m` | `mex_sensor_filter` | ✅ あり | 加速度フィルタ |
| `KF/Utils/AccelFilter.m` | `mex_sensor_filter` | ✅ あり | 簡易加速度フィルタ |
| `KF/Utils/NoiseEstimator.m` | `mex_sensor_filter('get_R')` | ✅ あり | ノイズ推定 |
| `KF/Utils/DivergenceGuard.m` | `mex_sensor_filter` | ✅ あり | 発散防止 |

---

## 🔴 MEX化未完了（純粋MATLAB実装）

### カテゴリA: フィルタコア（高優先度）

| ファイル | 機能 | 依存先 | MEX化推奨度 |
|---------|------|--------|------------|
| `ESKF/@ESKF/ESKF.m` | ESKFクラス本体・初期化 | すべて | 🔴 高 |
| `ESKF/@ESKF/predict.m` | 予測ステップ呼び出し | `mex_meukf_step_v2` | 🟡 中（既にMEX呼出） |
| `ESKF/@ESKF/sensor_updates.m` | センサー更新呼び出し | `mex_meukf_step_v2` | 🟡 中（既にMEX呼出） |
| `ESKF/@ESKF/zupt.m` | ZUPT更新 | `mex_meukf_step_v2` | 🟡 中（既にMEX呼出） |
| `ESKF/@ESKF/utils.m` | ユーティリティ | ESKF内部 | 🟢 低 |

### カテゴリB: センサーフィルタ（中優先度）

| ファイル | 機能 | 依存先 | MEX化推奨度 |
|---------|------|--------|------------|
| `KF/Utils/SensorMagFilter.m` | 磁気計フィルタ | SensorFilters | 🟡 中 |
| `KF/Utils/SensorGPSFilter.m` | GPSフィルタ | SensorFilters | 🟡 中 |
| `KF/Utils/SensorBaroFilter.m` | 気圧計フィルタ | SensorFilters | 🟡 中 |
| `KF/Utils/SensorGyroFilter.m` | ジャイロフィルタ（廃止予定） | - | 🟢 低 |
| `KF/Utils/OutlierGuard.m` | 外れ値検出 | SensorFilters | 🟢 低（C++に統合済） |
| `KF/Utils/FilterUtils.m` | フィルタユーティリティ | 各フィルタ | 🟢 低 |

### カテゴリC: 基本演算（低優先度）

| ファイル | 機能 | 依存先 | MEX化推奨度 |
|---------|------|--------|------------|
| `KF/Utils/alpha_beta_step.m` | α-β フィルタ | フィルタ全般 | ✅ 済（C++実装あり） |
| `KF/Utils/ema_update.m` | EMA更新 | フィルタ全般 | ✅ 済（C++実装あり） |
| `KF/Utils/hampel_causal.m` | Hampelフィルタ | フィルタ全般 | ✅ 済（C++実装あり） |
| `KF/Utils/BiquadFilter.m` | Biquadフィルタ | フィルタ全般 | ✅ 済（C++実装あり） |

### カテゴリD: データ生成・I/O（MEX化不要）

| ファイル | 機能 | MEX化 | 備考 |
|---------|------|------|------|
| `GenerateData/sim_generate.m` | データ生成 | ❌ 不要 | MATLAB で十分 |
| `GenerateData/config_params.m` | パラメータ設定 | ❌ 不要 | MATLAB で十分 |
| `GenerateData/read_csv.m` | CSV読み込み | ❌ 不要 | MATLAB で十分 |
| `GenerateData/add_sensor_noise.m` | ノイズ追加 | ❌ 不要 | シミュレーション用 |
| `GenerateData/generate_*.m` | 軌道生成 | ❌ 不要 | シミュレーション用 |
| `Graph/plot_*.m` | 可視化 | ❌ 不要 | MATLAB 専用 |

### カテゴリE: 未使用/レガシー

| ファイル | 機能 | ステータス |
|---------|------|----------|
| `KF/Utils/alpha_beta_step_cpp.m` | C++版ラッパー | 🔵 削除候補 |
| `KF/Utils/ema_update_cpp.m` | C++版ラッパー | 🔵 削除候補 |
| `KF/Utils/hampel_causal_cpp.m` | C++版ラッパー | 🔵 削除候補 |
| `KF/Utils/SensorFilter.m` | 旧ファクトリ | 🔵 削除候補 |
| `KF/Utils/SensorFilterFactory.m` | 旧ファクトリ | 🔵 削除候補 |
| `UKF/@UKF/*.m` | 旧UKFクラス | 🔵 ESKF に統合済 |
| `EKF/@EKF/*.m` | 旧EKFクラス | 🔵 ESKF に統合済 |

---

## 📈 現在の実行パス分析

### MEX使用時の呼び出しフロー

```
run_simulation.m
 │
 ├─► ESKF() コンストラクタ
 │    ├─► mex_quaternion_lib('from_euler')
 │    ├─► mex_quaternion_lib('to_rotation_matrix')
 │    └─► SensorFilters.reset() → mex_sensor_filter('reset')
 │
 └─► for k = 1:n_samples
      │
      ├─► eskf.predict(a, w)
      │    └─► mex_meukf_step_v2(state, sensor_data, params)
      │
      ├─► eskf.zupt('check', a, w)
      │    └─► (MATLAB内部判定)
      │
      ├─► eskf.sensor_updates('accel', a)
      │    ├─► SensorFilters.accel() → mex_sensor_filter('accel')
      │    └─► mex_meukf_step_v2()
      │
      ├─► eskf.sensor_updates('mag', m)
      │    ├─► SensorFilters.mag() → mex_sensor_filter('mag')
      │    └─► mex_meukf_step_v2()
      │
      ├─► eskf.sensor_updates('gps', lat, lon, alt)
      │    └─► mex_meukf_step_v2()
      │
      └─► eskf.sensor_updates('baro', pressure)
           ├─► SensorFilters.baro() → mex_sensor_filter('baro')
           └─► mex_meukf_step_v2()
```

### MATLABのみで残る処理

1. **ループ制御**: `run_simulation.m` のメインループ
2. **条件分岐**: センサー更新頻度判定（`mod(k, freq_*)`)
3. **初期化**: ESKF コンストラクタの大部分
4. **ZUPT判定**: `zupt.m` の静止検出ロジック
5. **データ変換**: GPS座標変換（lat/lon → m）
6. **Adaptive Q**: 動的ノイズ調整計算

---

## 🎯 MEX化優先度まとめ

### Tier 1: 即座に削除可能（MATLABフォールバック不要確認済み）
- `SensorFilters.m` の MATLAB フォールバック部分 ✅ 済

### Tier 2: 次フェーズでMEX統合
- `ESKF.m` 初期化部分 → C++ 初期化関数
- `zupt.m` 静止判定 → C++ に移植
- `predict.m` / `sensor_updates.m` のラッパー削減

### Tier 3: 長期的リファクタリング
- `run_simulation.m` のメインループ → 単一 MEX 呼び出し
- データ生成のC++化（オプション）

---

## 📝 備考

- **Eigenライブラリ**: 現在使用中。将来的に標準C++配列への移行を検討
- **浮動小数点精度**: 一部 float32 → float64 への移行が必要（PHASE 5）
- **テスト結果**: 10/10 PASS（Roll RMSE: 0.27°, Pitch RMSE: 0.28°）

---

**次ステップ**: `cpp_migration_plan.md` を更新して段階的移行計画を立案

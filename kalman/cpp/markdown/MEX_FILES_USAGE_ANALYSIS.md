# MEXファイルの使用状況分析

## 概要

このドキュメントは、MATLABコードから実際に使用されているMEX関数と、それらの依存関係を分析した結果です。

## 重要な発見

**MATLABコードから直接呼び出されるMEX関数は2つのみです：**

1. **`mex_run_eskf`** - ESKF処理のメインエントリーポイント
2. **`mex_sensor_filter`** - センサーフィルターの初期化のみ

## 使用状況の詳細

### 1. MATLABコードからの直接呼び出し

#### `run_simulation.m` と `run_batch_10sets.m`

```matlab
% メインのESKF処理
handle = mex_run_eskf('init', obs, params.static_time, dt);
mex_run_eskf('step', handle, obs, k);
state = mex_run_eskf('get_state', handle);
mex_run_eskf('free', handle);

% センサーフィルターの初期化（run_batch_10sets.mのみ）
mex_sensor_filter('reset_zero');
mex_sensor_filter('reset');
```

### 2. `mex_run_eskf` が内部で呼び出すMEX関数

`mex_run_eskf`は`mexCallMATLAB`を使用して、以下のMEX関数を内部で呼び出します：

#### 初期化時
- **`mex_eskf_constructor`** - ESKF状態の初期化

#### 予測ステップ
- **`mex_adaptive_predict`** - 予測ステップ（状態と共分散の予測）
- **`mex_eskf_predict_postprocess`** - 予測後処理（速度ダンピング、加速度Z統合）

#### センサー更新
- **`mex_eskf_sensor_updates_full`** - センサー更新処理（accel, mag, baro, gps）

#### フィルター管理
- **`mex_filter_management`** - 発散チェックとリセット処理
  - `check_divergence` - 共分散行列の発散チェック
  - `reset_state` - 状態のリセット

#### ゼロ速度更新（ZUPT）
- **`mex_eskf_zupt`** - 静止状態の検出と速度更新

### 3. 下位レベルのMEX関数（間接的に使用）

以下のMEX関数は、上記の関数からさらに呼び出されます：

#### `mex_eskf_sensor_updates_full` が呼び出す関数
- **`mex_sensor_preprocessor`** - センサーデータの前処理
- **`mex_eskf_do_update`** - カルマンフィルター更新処理

#### `mex_eskf_do_update` が呼び出す関数
- **`mex_sensor_filter`** - ノイズ推定と外れ値検出
- **`mex_meukf_step_v2`** - MEUKF（Multiple Extended Unscented Kalman Filter）ステップ
- **`mex_eskf_update_postprocess`** - 更新後処理（エラー状態注入、外れ値検出）

## 呼び出し階層図

```
MATLABコード
│
├─ mex_run_eskf (直接呼び出し)
│  │
│  ├─ mex_eskf_constructor (初期化)
│  │
│  ├─ mex_adaptive_predict (予測)
│  │
│  ├─ mex_eskf_predict_postprocess (予測後処理)
│  │
│  ├─ mex_eskf_sensor_updates_full (センサー更新)
│  │  │
│  │  ├─ mex_sensor_preprocessor (前処理)
│  │  │
│  │  └─ mex_eskf_do_update (更新処理)
│  │     │
│  │     ├─ mex_sensor_filter (ノイズ推定)
│  │     │
│  │     ├─ mex_meukf_step_v2 (MEUKF)
│  │     │
│  │     └─ mex_eskf_update_postprocess (更新後処理)
│  │
│  ├─ mex_filter_management (発散チェック/リセット)
│  │
│  └─ mex_eskf_zupt (ゼロ速度更新)
│
└─ mex_sensor_filter (直接呼び出し - 初期化のみ)
```

## 実際に使用されているMEX関数のリスト

### 必須（実際に使用されている）
1. `mex_run_eskf` ⭐ **メインエントリーポイント**
2. `mex_sensor_filter` ⭐ **初期化用**
3. `mex_eskf_constructor` - 初期化処理
4. `mex_adaptive_predict` - 予測処理
5. `mex_eskf_predict_postprocess` - 予測後処理
6. `mex_eskf_sensor_updates_full` - センサー更新
7. `mex_sensor_preprocessor` - センサー前処理
8. `mex_eskf_do_update` - 更新処理
9. `mex_meukf_step_v2` - MEUKF処理
10. `mex_eskf_update_postprocess` - 更新後処理
11. `mex_filter_management` - フィルター管理
12. `mex_eskf_zupt` - ゼロ速度更新

### 未使用またはレガシー（使用されていない可能性が高い）
以下のMEX関数は、現在のコードからは呼び出されていません：

- `mex_eskf_core` - レガシー関数
- `mex_eskf_math` - レガシー関数
- `mex_eskf_init` - レガシー関数
- `mex_eskf_get_state` - レガシー関数（`mex_run_eskf`内で直接実装）
- `mex_eskf_free` - レガシー関数（`mex_run_eskf`内で直接実装）
- `mex_eskf_set_state` - レガシー関数
- `mex_eskf_step` - レガシー関数
- `mex_eskf_step_handle` - レガシー関数
- `mex_eskf_full` - レガシー関数（`mex_run_eskf`に統合）
- `mex_eskf_sensor_update` - レガシー関数（`mex_eskf_sensor_updates_full`に統合）
- `mex_eskf_sensor_updates` - レガシー関数（`mex_eskf_sensor_updates_full`に統合）
- `mex_ukf` - 未使用
- `mex_ukf_update` - 未使用
- `mex_ukf_sigma_points` - 未使用
- `mex_ekf` - 未使用
- `mex_kalman_filter_core` - 未使用
- `mex_unified_filter` - 未使用（`mex_eskf_step_handle`から呼び出されるが、`mex_eskf_step_handle`自体が未使用）
- `mex_matlab_helpers` - 未使用（`.m`ファイルとして存在）

## 結論

1. **実際に使用されているのは`mex_run_eskf`と`mex_sensor_filter`のみ**
   - 他のMEX関数は`mex_run_eskf`の内部実装の一部として使用されています

2. **多くのMEX関数はレガシーまたは未使用**
   - 開発過程で段階的に実装されたが、最終的に`mex_run_eskf`に統合された関数が多数存在します

3. **ビルドの最適化**
   - 実際に使用されている関数のみをビルドすることで、ビルド時間とバイナリサイズを削減できます
   - ただし、`mex_run_eskf`が依存する関数はすべて必要です

4. **推奨事項**
   - 未使用のMEX関数のソースコードは保持するが、ビルドから除外することを検討
   - または、レガシー関数を別ディレクトリに移動して整理

## 関連ドキュメント

- [MEX_FILES_OVERVIEW.md](./MEX_FILES_OVERVIEW.md) - MEXフォルダの概要
- [MEX_FILES_DEPENDENCIES.md](./MEX_FILES_DEPENDENCIES.md) - 依存関係の詳細
- [MEX_FILES_CLEANUP_RECOMMENDATIONS.md](./MEX_FILES_CLEANUP_RECOMMENDATIONS.md) - クリーンアップの推奨事項





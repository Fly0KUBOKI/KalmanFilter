# ESKF.m 関数リファレンス

## 概要
ESKF.mは14個のメソッドで構成され、MEX関数を中心とした実装となっている。

## 関数一覧と依存関係

### 1. 初期化・構築
#### `ESKF(obs, static_time, dt)` - コンストラクタ
- **機能**: 初期状態・共分散・ノイズパラメータの設定
- **依存**: `get_field_impl`, `has_field_impl`, `mex_quaternion_lib`, `mex_eskf_init`, `mex_sensor_filter`
- **出力**: なし（オブジェクト初期化）

#### `delete(obj)` - デストラクタ
- **機能**: C++状態ハンドルの解放
- **依存**: `mex_eskf_free`
- **出力**: なし

---

### 2. ユーティリティ
#### `utils(obj, method, varargin)` - 統合ユーティリティ
- **機能**: `get_euler`, `get_field`の統合エントリ
- **依存**: `mex_quaternion_lib`, `get_field_impl`
- **出力**: メソッド依存

#### `get_field_impl(obj, obs, field_names, idx, num_cols)` - フィールド取得
- **機能**: 構造体から指定フィールドを取得
- **依存**: `mex_matlab_helpers('get_field')`
- **出力**: `data`

#### `has_field_impl(obj, obs, field_names)` - フィールド存在確認
- **機能**: 構造体に指定フィールドが存在するか確認
- **依存**: `mex_matlab_helpers('has_field')`
- **出力**: `tf` (logical)

---

### 3. センサーフィルタ管理
#### `reset_sensor_filters(obj, method)` - フィルタリセット
- **機能**: センサーフィルタをリセット（`method='reset'`または`'reset_zero'`）
- **依存**: `mex_sensor_filter('reset')` / `mex_sensor_filter('reset_zero')`
- **出力**: なし

---

### 4. 予測・更新
#### `predict(obj, a_meas, w_meas)` - 予測ステップ
- **機能**: IMUデータによる状態予測
- **依存**: `mex_adaptive_predict`, `mex_quaternion_lib`, `divergence_guard`
- **出力**: なし（状態更新）

#### `update_filter(obj, obs, k)` - 1ステップ更新
- **機能**: 予測→センサー更新→発散チェックの一連処理
- **依存**: `predict`, `sensor_updates`, `reset('check')`
- **出力**: なし

#### `sensor_updates(obj, sensor_type, varargin)` - センサー更新
- **機能**: センサー種別ごとの前処理と更新呼び出し
- **依存**: `mex_sensor_preprocessor`, `do_cpp_update`
- **出力**: なし

#### `do_cpp_update(obj, sensor_type, meas, sample)` - C++更新呼び出し
- **機能**: MEX関数への更新処理委譲
- **依存**: `call_meukf_step`, `noiseEstimator.estimate`, `divergence_guard`
- **出力**: なし（状態更新）

#### `call_meukf_step(obj, state, sensor_data, mex_params, sensor_type, sample)` - MEUKF呼び出し
- **機能**: `mex_meukf_step_v2`のラッパー
- **依存**: `mex_meukf_step_v2`
- **出力**: `[new_state, dbg_out, mex_debug]`

---

### 5. リセット・発散管理
#### `reset(obj, method, varargin)` - リセット統合
- **機能**: 発散チェックとリセット処理（`method='check'`または`'filter'`）
- **依存**: `mex_filter_management('check_divergence')`, `mex_filter_management('reset_state')`
- **出力**: なし

---

### 6. ZUPT (Zero Velocity Update)
#### `zupt(obj, method, varargin)` - ZUPT統合
- **機能**: 静止判定とZUPT更新（`method='check'`または`'update'`）
- **依存**: `mex_filter_management('apply_zupt')`
- **出力**: メソッド依存（`'check'`の場合は`is_stat`）

---

### 7. その他
#### `call_unified_filter(obj, input_struct)` - 統合フィルタ呼び出し
- **機能**: `mex_unified_filter`のラッパー
- **依存**: `mex_unified_filter`
- **出力**: `output` (構造体)

---

## 依存関係グラフ

```
ESKF()
├─ get_field_impl() → mex_matlab_helpers
├─ has_field_impl() → mex_matlab_helpers
├─ mex_quaternion_lib
├─ mex_eskf_init
└─ mex_sensor_filter

predict()
├─ mex_adaptive_predict
├─ mex_quaternion_lib
└─ divergence_guard → mex_sensor_filter

update_filter()
├─ predict()
├─ sensor_updates()
│   ├─ mex_sensor_preprocessor
│   └─ do_cpp_update()
│       ├─ call_meukf_step() → mex_meukf_step_v2
│       ├─ noiseEstimator.estimate() → mex_sensor_filter
│       └─ divergence_guard → mex_sensor_filter
└─ reset('check')
    └─ mex_filter_management
```

---

## MEX関数依存関係

| MEX関数 | 使用箇所 | 用途 |
|---------|---------|------|
| `mex_matlab_helpers` | `get_field_impl`, `has_field_impl` | 構造体操作 |
| `mex_quaternion_lib` | `ESKF`, `utils`, `predict` | クォータニオン演算 |
| `mex_eskf_init` | `ESKF` | C++状態初期化 |
| `mex_eskf_free` | `delete` | C++状態解放 |
| `mex_sensor_filter` | 複数 | センサーフィルタ・発散チェック |
| `mex_sensor_preprocessor` | `sensor_updates` | センサー前処理 |
| `mex_adaptive_predict` | `predict` | 予測ステップ |
| `mex_filter_management` | `reset`, `zupt` | フィルタ管理 |
| `mex_meukf_step_v2` | `call_meukf_step` | MEUKF更新 |
| `mex_unified_filter` | `call_unified_filter` | 統合フィルタ |

---

## 統合後の変更点

### 削除された関数（統合済み）
1. **`reset_sensor_filters_zero`** → `reset_sensor_filters(obj, 'reset_zero')`に統合
2. **`get_sensor_R`** → `noiseEstimator.getRnoise()`を直接使用
3. **`estimate_noise`** → `noiseEstimator.estimate()`を直接使用
4. **`divergence_check`** → `divergence_guard.check_and_attenuate_update()`を直接使用
5. **`get_euler_impl`** → `utils('get_euler')`内に直接実装
6. **`update_sensor_impl`** → `sensor_updates()`に統合
7. **`check_and_reset_impl`** → `reset('check')`に統合
8. **`reset_filter_impl`** → `reset('filter')`に統合
9. **`check_stationary_impl`** → `zupt('check')`に統合
10. **`update_zupt_impl`** → `zupt('update')`に統合

### 関数数の変化
- **統合前**: 24個
- **統合後**: 14個
- **削減率**: 約42%

### その他の改善
1. **MEX存在チェック削減**: 各関数内の`exist()`チェックを削除（MEX必須前提）
2. **コメント削減**: 冗長なコメントを削除
3. **重複処理統合**: 初期化処理を`deal()`で統合
4. **エラーハンドリング削減**: `try-catch`を削除
5. **コード簡潔化**: 条件分岐を1行化、変数初期化を統合

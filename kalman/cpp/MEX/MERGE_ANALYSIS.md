# MEXファイル統合分析

## 依存関係ツリー

```
mex_run_eskf (最上位)
├─ mex_adaptive_predict (独立、統合不可)
├─ mex_eskf_predict_postprocess (統合候補)
│  ├─ mex_quaternion_lib (独立、統合不可)
│  └─ mex_sensor_filter (独立、統合不可)
├─ mex_eskf_sensor_updates_full (統合候補)
│  ├─ mex_sensor_preprocessor (独立、統合不可)
│  └─ mex_eskf_do_update (統合候補)
│     ├─ mex_sensor_filter (独立、統合不可)
│     ├─ mex_meukf_step_v2 (独立、統合不可)
│     └─ mex_eskf_update_postprocess (統合候補)
│        └─ mex_sensor_filter (独立、統合不可)
├─ mex_filter_management (独立、統合不可)
├─ mex_eskf_zupt (独立、統合不可)
└─ mex_eskf_constructor (独立、統合不可)
```

## 統合可能なファイル

### 1. `mex_eskf_update_postprocess` → `mex_eskf_do_update`
- **理由**: `mex_eskf_do_update`が`mex_eskf_update_postprocess`を直接呼び出しているのみ
- **統合方法**: `mex_eskf_do_update`内に`handle_postprocess`関数を統合
- **影響**: `mex_eskf_do_update`のみが影響を受ける

### 2. `mex_eskf_do_update` → `mex_eskf_sensor_updates_full`
- **理由**: `mex_eskf_sensor_updates_full`が`mex_eskf_do_update`を直接呼び出しているのみ
- **統合方法**: `mex_eskf_sensor_updates_full`内に`handle_update`関数を統合
- **影響**: `mex_eskf_sensor_updates_full`のみが影響を受ける

### 3. `mex_eskf_sensor_updates_full` → `mex_run_eskf`
- **理由**: `mex_run_eskf`が`mex_eskf_sensor_updates_full`を直接呼び出しているのみ
- **統合方法**: `mex_run_eskf`内に`handle_accel`, `handle_mag`, `handle_baro`, `handle_gps`関数を統合
- **影響**: `mex_run_eskf`のみが影響を受ける

### 4. `mex_eskf_predict_postprocess` → `mex_run_eskf`
- **理由**: `mex_run_eskf`が`mex_eskf_predict_postprocess`を直接呼び出しているのみ
- **統合方法**: `mex_run_eskf`内に`handle_postprocess`関数を統合
- **影響**: `mex_run_eskf`のみが影響を受ける

## 統合順序

1. **第1段階**: `mex_eskf_update_postprocess` → `mex_eskf_do_update`
2. **第2段階**: `mex_eskf_do_update` → `mex_eskf_sensor_updates_full`
3. **第3段階**: `mex_eskf_sensor_updates_full` → `mex_run_eskf`
4. **第4段階**: `mex_eskf_predict_postprocess` → `mex_run_eskf`

## 統合後の構造

```
mex_run_eskf (統合後)
├─ mex_adaptive_predict (独立)
├─ mex_sensor_preprocessor (独立)
├─ mex_sensor_filter (独立)
├─ mex_meukf_step_v2 (独立)
├─ mex_quaternion_lib (独立)
├─ mex_filter_management (独立)
├─ mex_eskf_zupt (独立)
└─ mex_eskf_constructor (独立)
```

## 注意事項

1. **MATLAB呼び出し**: 一部の関数は`mexCallMATLAB`を使用しているため、統合後も`mexCallMATLAB`を維持する必要があります。

2. **関数名の衝突**: 統合時に同じ名前の関数が存在する場合は、名前空間や名前変更が必要です。

3. **ビルド設定**: `build_mex.m`から統合されたファイルを削除する必要があります。


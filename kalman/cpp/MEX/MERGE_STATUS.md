# MEXファイル統合状況

## 完了した統合

### ✅ 1. `mex_eskf_update_postprocess` → `mex_eskf_do_update`
- **状態**: 完了
- **日付**: 2024-12-29
- **詳細**: `mex_eskf_do_update.cpp`に`handle_postprocess`関数を統合
- **影響**: `mex_eskf_update_postprocess.cpp`は不要になったが、互換性のため残す

### ✅ 2. `mex_eskf_predict_postprocess` → `mex_run_eskf`
- **状態**: 完了
- **日付**: 2024-12-29
- **詳細**: `mex_run_eskf.cpp`の`call_predict`関数内に`handle_postprocess`ロジックを統合
- **影響**: `mex_eskf_predict_postprocess.cpp`は不要になったが、互換性のため残す

## 統合困難なファイル

### ⚠️ 3. `mex_eskf_do_update` → `mex_eskf_sensor_updates_full`
- **状態**: 統合困難
- **理由**: `mex_eskf_do_update`は`mex_meukf_step_v2`を呼び出しており、これは独立したMEXファイルとして維持する必要がある
- **代替案**: `mex_eskf_do_update`のロジックを`mex_eskf_sensor_updates_full`に統合することは可能だが、`mex_meukf_step_v2`の呼び出しは維持する必要がある

### ⚠️ 4. `mex_eskf_sensor_updates_full` → `mex_run_eskf`
- **状態**: 統合困難
- **理由**: `mex_eskf_sensor_updates_full`は複数の`handle_*`関数（`handle_accel`, `handle_mag`, `handle_gps`, `handle_baro`）を含んでおり、それぞれが`mex_sensor_preprocessor`と`mex_eskf_do_update`を呼び出している
- **代替案**: 各`handle_*`関数を`mex_run_eskf`に統合することは可能だが、コードが非常に大きくなる

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

## 削除可能なファイル（互換性のため残す）

以下のファイルは統合により不要になったが、互換性のため残すことを推奨：

1. `mex_eskf_update_postprocess.cpp` - `mex_eskf_do_update`に統合済み
2. `mex_eskf_predict_postprocess.cpp` - `mex_run_eskf`に統合済み

## 注意事項

1. **MATLAB呼び出し**: 一部の関数は`mexCallMATLAB`を使用しているため、統合後も`mexCallMATLAB`を維持する必要があります。

2. **型変換**: `mex_run_eskf.cpp`は`double`型を使用しており、統合した関数は`float`型を使用しているため、型変換が必要です。

3. **ビルド設定**: `build_mex.m`から統合されたファイルを削除する必要はありませんが、互換性のため残すことを推奨します。


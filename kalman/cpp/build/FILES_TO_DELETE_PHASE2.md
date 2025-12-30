# 削除可能なファイル一覧（Phase 2: mex_eskf_do_update統合後）

## 実施日時
2025年12月30日

## ✅ 削除可能なファイル

### 1. `mex_eskf_do_update.cpp`
- **パス**: `kalman/cpp/MEX/mex_eskf_do_update.cpp`
- **理由**: 
  - `mex_run_eskf_sensor_updates.hpp`に`handle_sensor_update_internal`として統合済み
  - MATLABコードから直接呼び出されていない
  - C++コード内でも`mexCallMATLAB`による呼び出しがない（既に統合済み）
- **統合先**: `kalman/cpp/Inc/MEX/mex_run_eskf_sensor_updates.hpp`の`handle_sensor_update_internal`関数
- **状態**: 削除可能

### 2. `mex_eskf_do_update.mexw64`
- **パス**: `kalman/cpp/bin/mex_eskf_do_update.mexw64`
- **理由**: 上記ソースファイルに対応するバイナリファイル
- **状態**: 削除可能

### 3. `build_mex.m`のビルド設定
- **変更**: `mex_eskf_do_update`のビルド設定を削除（224-227行目）
- **状態**: 削除可能

## ⚠️ 残す必要があるファイル

### 1. `mex_meukf_step.cpp` (または `mex_meukf_step_v2.mexw64`)
- **理由**: `handle_sensor_update_internal`内で`mexCallMATLAB`経由で`mex_meukf_step_v2`を呼び出している
- **使用箇所**: `mex_run_eskf_sensor_updates.hpp` Line 391
- **注意**: 将来の統合対象（Phase 3として検討）

### 2. `mex_sensor_filter.cpp`
- **理由**: `run_batch_10sets.m`から`reset_zero`と`reset`が呼び出されている
- **使用箇所**: 
  ```matlab
  mex_sensor_filter('reset_zero');
  mex_sensor_filter('reset');
  ```

### 3. `mex_run_eskf.cpp`
- **理由**: MATLABコードから直接呼び出されている（メインエントリーポイント）

## 削除後の確認事項

1. **ビルドテスト**: `mex_run_eskf`が正常にビルドできることを確認
2. **精度テスト**: 統合後も精度が維持されていることを確認（既に確認済み）

## 統合の進捗

- ✅ `mex_eskf_do_update`の統合: 完了
  - `handle_sensor_update_internal`関数として`mex_run_eskf_sensor_updates.hpp`に統合
  - `call_sensor_update`と`call_gps_update`から`mexCallMATLAB`を削除

- ⏳ 残存する`mexCallMATLAB`呼び出し:
  - `mex_meukf_step_v2`（`handle_sensor_update_internal`内、1箇所）
  - 将来の統合対象（Phase 3として検討）




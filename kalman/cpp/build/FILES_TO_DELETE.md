# 削除可能なファイル一覧

## 統合作業完了後の不要ファイル

### ✅ 削除可能なファイル

#### 1. `mex_sensor_preprocessor.cpp`
- **理由**: Phase 4Aで統合済み。C++コード内では`mexCallMATLAB`経由の呼び出しは削除済み。
- **確認**: MATLABコードから直接呼び出されていない（`build_mex.m`でのビルド設定のみ）
- **ファイルパス**: `kalman/cpp/MEX/mex_sensor_preprocessor.cpp`

#### 2. `mex_sensor_preprocessor.mexw64`
- **理由**: 上記ソースファイルに対応するバイナリファイル
- **ファイルパス**: `kalman/cpp/bin/mex_sensor_preprocessor.mexw64`

### ⚠️ 残す必要があるファイル

#### 1. `mex_sensor_filter.cpp`
- **理由**: `run_batch_10sets.m`から`reset_zero`と`reset`が呼び出されている
- **使用箇所**: 
  ```matlab
  mex_sensor_filter('reset_zero');
  mex_sensor_filter('reset');
  ```
- **注意**: Phase 4C, 4Dで統合された機能（`divergence_check`, `get_R`, `noise_estimate`）は削除済みだが、初期化機能は残す必要がある

#### 2. `mex_meukf_step.cpp` (または `mex_meukf_step_v2.mexw64`)
- **理由**: `mex_eskf_do_update.cpp`から`mexCallMATLAB`経由で呼び出されている
- **使用箇所**: `mex_eskf_do_update.cpp` Line 197
- **注意**: 将来の統合対象（Phase 4Dの将来の検討事項）

#### 3. `mex_eskf_do_update.cpp`
- **理由**: `mex_run_eskf_sensor_updates.hpp`から`mexCallMATLAB`経由で呼び出されている
- **使用箇所**: `mex_run_eskf_sensor_updates.hpp` (4箇所)

#### 4. `mex_run_eskf.cpp`
- **理由**: MATLABコードから直接呼び出されている
- **使用箇所**: `run_simulation.m`, `run_batch_10sets.m`

### ❌ 既に削除済み

#### 1. `mex_eskf_update_postprocess.cpp`
- **理由**: Phase 4Bで統合済み（`update_state_from_dx`を直接呼び出し）
- **状態**: ソースファイルは既に存在しない

## 削除手順

### 1. 削除前の確認
```matlab
% MATLABで確認
which mex_sensor_preprocessor
% 結果が空であることを確認
```

### 2. ファイル削除
```bash
# ソースファイル
rm kalman/cpp/MEX/mex_sensor_preprocessor.cpp

# バイナリファイル
rm kalman/cpp/bin/mex_sensor_preprocessor.mexw64
```

### 3. ビルド設定の更新
`kalman/cpp/build/build_mex.m`から`mex_sensor_preprocessor`のビルド設定を削除（Line 117付近）

### 4. 削除後の確認
- ビルドテスト: `build_mex({'mex_run_eskf'})`
- 精度テスト: `run_batch_10sets()`

## 注意事項

1. **`mex_sensor_filter.cpp`は削除しない**
   - `reset_zero`と`reset`機能はMATLABコードから直接呼び出されている
   - これらの機能は`SensorFilterLib`クラスのメソッドとして実装されているが、MATLABインターフェースとして残す必要がある

2. **`mex_meukf_step.cpp`は削除しない**
   - `mex_eskf_do_update.cpp`から呼び出されている
   - 将来の統合対象として残す

3. **削除前のバックアップ推奨**
   - 削除前にGitでコミットするか、バックアップを取ることを推奨



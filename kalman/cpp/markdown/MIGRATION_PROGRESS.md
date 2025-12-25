# Migration Progress (要約)

**最終更新日**: 2025-12-25  
**現状確認日**: 2025-01-XX

## 進捗サマリー

### MEXファイルのビルド状況

| Phase | MEX関数 | ビルド状況 | ESKF.m統合状況 |
|-------|---------|-----------|---------------|
| Phase 1 | `mex_matlab_helpers` | ✅ ビルド済 | ❌ 未統合 |
| Phase 2 | `mex_sensor_filter` (拡張) | ✅ ビルド済 | ✅ 部分的に使用中 |
| Phase 3 | `mex_sensor_preprocessor` | ✅ ビルド済 | ❌ 未統合 |
| Phase 4 | `mex_adaptive_predict` | ✅ ビルド済 | ❌ 未統合 |
| Phase 5 | `mex_filter_management` | ✅ ビルド済 | ❌ 未統合 |

### 詳細状況

#### Phase 1: 基盤レイヤー（`mex_matlab_helpers`）
- **ビルド**: ✅ 完了（`kalman/cpp/bin/mex_matlab_helpers.mexw64` 存在）
- **ESKF.m統合**: ❌ **未統合**
  - `get_field_impl()`: まだMATLAB実装（379-406行）
  - `has_field_impl()`: まだMATLAB実装（408-415行）
  - `get_euler_impl()`: `mex_quaternion_lib`を使用（374-377行）← これは既にMEX使用

#### Phase 2: ユーティリティ拡張（`mex_sensor_filter`）
- **ビルド**: ✅ 完了（拡張機能含む）
- **ESKF.m統合**: ✅ **部分的に使用中**
  - `get_sensor_R()`: `mex_sensor_filter('get_R')`を使用（328-338行）
  - `estimate_noise()`: `mex_sensor_filter('noise_estimate')`を使用（340-355行）
  - `divergence_guard.regularize_covariance`: `mex_sensor_filter('divergence_regularize')`を使用（234行）
  - `divergence_guard.check_and_attenuate_update`: `mex_sensor_filter('divergence_check')`を使用（233行）
  - **未統合**: `divergence_check_velocity_impl()` はまだMATLAB実装（1068-1098行）

#### Phase 3: センサー前処理（`mex_sensor_preprocessor`）
- **ビルド**: ✅ 完了（`kalman/cpp/bin/mex_sensor_preprocessor.mexw64` 存在）
- **ESKF.m統合**: ❌ **未統合**
  - `update_sensor_impl()`: まだMATLAB実装で前処理ロジックを含む（600-720行）
  - 変更検知、異常検知、GPS座標変換などがMATLAB側で実装されている

#### Phase 4: 予測ステップ（`mex_adaptive_predict`）
- **ビルド**: ✅ 完了（`kalman/cpp/bin/mex_adaptive_predict.mexw64` 存在）
- **ESKF.m統合**: ❌ **未統合**
  - `predict()`: まだMATLAB実装（417-553行）
  - ジャイロ/加速度フィルタ適用、Adaptive Q計算、速度クリップなどがMATLAB側で実装
  - `call_meukf_step()`経由で`mex_meukf_step_v2`を呼び出しているが、前処理はMATLAB

#### Phase 5: フィルタ管理（`mex_filter_management`）
- **ビルド**: ✅ 完了（`kalman/cpp/bin/mex_filter_management.mexw64` 存在）
- **ESKF.m統合**: ❌ **未統合**
  - `check_and_reset_impl()`: まだMATLAB実装（927-952行）
  - `reset_filter_impl()`: まだMATLAB実装（955-977行）
  - `check_stationary_impl()`: まだMATLAB実装（995-1008行）
  - `update_zupt_impl()`: まだMATLAB実装（1011-1057行）

## 現状の問題点

1. **MEXファイルはビルド済みだが、ESKF.mに統合されていない**
   - Phase 1, 3, 4, 5のMEX関数がESKF.mで使用されていない
   - これにより、パフォーマンス向上の効果が得られていない

2. **統合作業が未完了**
   - 各フェーズのMEX関数をESKF.mの対応するメソッドから呼び出すように変更する必要がある

## 次の推奨アクション

### 優先度: 高
1. **Phase 1統合**: `get_field_impl()`と`has_field_impl()`を`mex_matlab_helpers`に置き換え
2. **Phase 4統合**: `predict()`を`mex_adaptive_predict`に置き換え（最重要・最大効果）
3. **Phase 3統合**: `update_sensor_impl()`の前処理を`mex_sensor_preprocessor`に置き換え

### 優先度: 中
4. **Phase 5統合**: `check_and_reset_impl()`、`reset_filter_impl()`、ZUPT系を`mex_filter_management`に置き換え
5. **Phase 2完了**: `divergence_check_velocity_impl()`を`mex_sensor_filter`に統合

### 検証
- 各統合後に`run_simulation(42, true)`で動作確認
- `run_batch_10sets()`で回帰テスト
- 数値精度の確認（特にPhase 4）

## 実装スケジュール（再計画）

```
Week 1
├─ Day 1-2: Phase 1統合（get_field_impl, has_field_impl）
│  └─ テスト: run_simulation(42, true)
├─ Day 3-4: Phase 4統合（predict()）⭐最重要⭐
│  └─ テスト: run_simulation + run_batch_10sets + 数値精度確認
└─ Day 5: Phase 3統合（update_sensor_impl前処理）
   └─ テスト: run_simulation + run_batch_10sets

Week 2
├─ Day 1: Phase 5統合（reset系、ZUPT系）
│  └─ テスト: run_batch_10sets
└─ Day 2: Phase 2完了（divergence_check_velocity_impl）
   └─ 最終検証: run_batch_10sets + パフォーマンス測定
```

## 注意事項

- 各統合時には、MATLAB実装をフォールバックとして残すことを推奨（`exist('mex_xxx', 'file') == 3`でチェック）
- Phase 4（predict）は最も影響が大きいため、慎重に検証が必要
- 数値精度の差異が許容範囲内か確認（特にfloat精度による誤差）

担当者: 要実装・統合作業

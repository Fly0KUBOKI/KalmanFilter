# ESKF.m 完全MEX化 - 段階的移行計画

**作成日**: 2025-12-27  
**最終更新**: 2025-12-28  
**目標**: ESKF.mの各関数を順次MEX化し、最終的にESKF.mを完全にMEXに置き換える

---

## 📊 現状分析

### 既にMEX化されている部分

| 関数 | 状態 | MEX関数 |
|------|------|---------|
| `ESKF()` コンストラクタ | ✅ Phase 1完了 | `mex_eskf_constructor` |
| `predict()` 内部 | ✅ MEX化済み | `mex_adaptive_predict` + `mex_eskf_predict_postprocess` |
| `sensor_updates()` 前処理 | ✅ MEX化済み | `mex_sensor_preprocessor` |
| `do_cpp_update()` 内部 | ✅ MEX化済み | `mex_meukf_step_v2` + `mex_eskf_update_postprocess` |
| `zupt()` 内部 | ✅ MEX化済み | `mex_eskf_zupt` |
| `reset()` 内部 | ✅ MEX化済み | `mex_filter_management` |
| `utils()` | ✅ MEX化済み | `mex_quaternion_lib` |
| `get_field_impl()` | ✅ MEX化済み | `mex_matlab_helpers` |
| `has_field_impl()` | ✅ MEX化済み | `mex_matlab_helpers` |
| `noiseEstimator` | ✅ MEX化済み | `mex_sensor_filter` |

### まだMATLAB実装の部分

| 関数 | 行数 | 複雑度 | 優先度 |
|------|------|--------|--------|
| `ESKF()` (コンストラクタ) | 23-168 | 高 | 🔴 高 |
| `sensor_updates()` | 254-315 | 中 | 🔴 高 |
| `do_cpp_update()` | 317-405 | 中 | 🟡 中 |
| `reset()` | 419-459 | 低 | 🟢 低 |
| `update_filter()` | 241-252 | 低 | 🟢 低 |
| `delete()` | 476-482 | 低 | 🟢 低 |

---

## 🎯 段階的移行計画

### Phase 1: コンストラクタのMEX化 ✅ 準備完了

**目標**: `ESKF()` コンストラクタをMEX化

**実装内容**:
- `mex_eskf_constructor.cpp` を作成
- 初期化ロジックをC++に移植
- ノイズ推定、初期姿勢計算をMEX化

**インターフェース**:
```matlab
% MATLAB側
function obj = ESKF(obs, static_time, dt)
    if exist('mex_eskf_constructor', 'file') == 3
        % MEX版を使用
        init_data = mex_eskf_constructor('init', obs, static_time, dt);
        obj = obj.set_from_init_data(init_data);
    else
        % MATLAB版（フォールバック）
        % ... 既存のコード ...
    end
end
```

**MEX関数**:
```cpp
// mex_eskf_constructor('init', obs, static_time, dt)
// 戻り値: struct with p, v, q, ba, bg, P, Q, gps_origin, noiseEstimator, etc.
```

**検証方法**:
- 初期状態がMATLAB版と完全一致することを確認
- バッチテストで10/10成功を確認

---

### Phase 2: `sensor_updates()` のMEX化

**目標**: `sensor_updates()` を完全にMEX化

**実装内容**:
- `mex_eskf_sensor_updates.cpp` を作成
- センサータイプ別の分岐処理をC++に移植
- 前処理（`mex_sensor_preprocessor`）の呼び出しを統合

**インターフェース**:
```matlab
% MATLAB側
function sensor_updates(obj, sensor_type, varargin)
    if exist('mex_eskf_sensor_updates', 'file') == 3
        % MEX版を使用
        [obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P, obj.prev_*] = ...
            mex_eskf_sensor_updates('update', sensor_type, ...
            obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P, ...
            obj.prev_accel, obj.prev_mag, obj.prev_baro, ...
            obj.prev_gps_lat, obj.prev_gps_lon, obj.prev_gps_alt, ...
            obj.gps_origin, obj.baro_weight, obj.buffer_tolerance, ...
            obj.w_body, obj.noiseEstimator, obj.dt, obj.g, varargin{:});
    else
        % MATLAB版（フォールバック）
        % ... 既存のコード ...
    end
end
```

**MEX関数**:
```cpp
// mex_eskf_sensor_updates('update', sensor_type, state, prev_data, params, ...)
// 戻り値: 更新された状態とprev_*データ
```

**検証方法**:
- 各センサータイプ（accel, mag, baro, gps）でMATLAB版と結果一致を確認
- バッチテストで10/10成功を確認

---

### Phase 3: `do_cpp_update()` のMEX化

**目標**: `do_cpp_update()` を完全にMEX化（noiseEstimator統合）

**実装内容**:
- `mex_eskf_do_cpp_update.cpp` を作成
- `noiseEstimator` のロジックをC++に移植
- R行列の動的更新を実装

**インターフェース**:
```matlab
% MATLAB側
function do_cpp_update(obj, sensor_type, meas, sample)
    if exist('mex_eskf_do_cpp_update', 'file') == 3
        % MEX版を使用
        [obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P, obj.noiseEstimator] = ...
            mex_eskf_do_cpp_update('update', sensor_type, meas, sample, ...
            obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P, ...
            obj.noiseEstimator, obj.dt, obj.g, obj.mag_ref);
    else
        % MATLAB版（フォールバック）
        % ... 既存のコード ...
    end
end
```

**MEX関数**:
```cpp
// mex_eskf_do_cpp_update('update', sensor_type, meas, sample, state, noiseEstimator, params)
// 戻り値: 更新された状態とnoiseEstimator
```

**検証方法**:
- noiseEstimatorのR行列更新がMATLAB版と一致することを確認
- バッチテストで10/10成功を確認

---

### Phase 4: `reset()` のMEX化

**目標**: `reset()` を完全にMEX化

**実装内容**:
- `mex_eskf_reset.cpp` を拡張（既存の実装を改善）
- `check` と `filter` メソッドを統合

**インターフェース**:
```matlab
% MATLAB側
function reset(obj, method, varargin)
    if exist('mex_eskf_reset', 'file') == 3
        % MEX版を使用
        if strcmp(method, 'check')
            [reset_needed, obj.P, obj.v, obj.p, obj.q, obj.ba, obj.bg, obj.last_reset_step] = ...
                mex_eskf_reset('check', obj.P, obj.p, obj.v, obj.q, obj.ba, obj.bg, ...
                obs, k, obj.gps_origin, obj.last_reset_step);
        elseif strcmp(method, 'filter')
            [obj.P, obj.v, obj.p, obj.q, obj.ba, obj.bg, obj.last_reset_step] = ...
                mex_eskf_reset('filter', obj.P, obj.p, obj.v, obj.q, obj.ba, obj.bg, ...
                obs, k, obj.gps_origin);
        end
    else
        % MATLAB版（フォールバック）
        % ... 既存のコード ...
    end
end
```

**検証方法**:
- バッチテストで10/10成功を確認（以前のNaN問題を解決）

---

### Phase 5: 統合と最適化

**目標**: 全関数をMEX化し、ESKF.mを最小限に

**実装内容**:
1. 各PhaseのMEX関数を統合
2. メモリ管理の最適化
3. エラーハンドリングの統一

**最終的なESKF.m構造**:
```matlab
classdef ESKF < handle
    properties
        p; v; q; ba; bg; P; Q; dt; g
        % ... その他のプロパティ ...
    end
    
    methods
        function obj = ESKF(obs, static_time, dt)
            % MEX版コンストラクタを呼び出し
            init_data = mex_eskf_constructor('init', obs, static_time, dt);
            obj = obj.set_from_init_data(init_data);
        end
        
        function predict(obj, a_meas, w_meas)
            % MEX版predictを呼び出し
            [obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P] = ...
                mex_eskf_predict('predict', obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P, ...
                a_meas, w_meas, obj.dt, obj.Q_nominal, obj.adaptive_q_enabled, obj.g);
        end
        
        function sensor_updates(obj, sensor_type, varargin)
            % MEX版sensor_updatesを呼び出し
            [obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P, obj.prev_*] = ...
                mex_eskf_sensor_updates('update', sensor_type, ...);
        end
        
        function reset(obj, method, varargin)
            % MEX版resetを呼び出し
            % ...
        end
        
        function varargout = zupt(obj, method, varargin)
            % MEX版zuptを呼び出し
            % ...
        end
    end
end
```

---

## 📋 実装順序と依存関係

```
Phase 1: コンストラクタ
  └─> 独立（他の関数に依存しない）

Phase 2: sensor_updates
  ├─> Phase 1 に依存（初期化データ）
  └─> 既存の mex_sensor_preprocessor を使用

Phase 3: do_cpp_update
  ├─> Phase 2 に依存
  └─> noiseEstimator の実装が必要

Phase 4: reset
  └─> Phase 1 に依存（初期化データ）

Phase 5: 統合
  └─> Phase 1-4 すべてに依存
```

---

## 🔧 実装ガイドライン

### 1. フォールバック機能

各Phaseで、MEX関数が存在しない場合はMATLAB版に自動フォールバック：

```matlab
if exist('mex_eskf_xxx', 'file') == 3
    % MEX版
else
    % MATLAB版（フォールバック）
end
```

### 2. 数値精度の維持

- MATLABの`double`とC++の`float`の変換に注意
- 重要な計算は`double`で行い、最後に`float`に変換

### 3. エラーハンドリング

- MEX関数内でNaN/Infをチェック
- エラー時はMATLAB版にフォールバック

### 4. テスト戦略

各Phase完了後に：
1. 単体テスト（MATLAB版と結果比較）
2. バッチテスト（10/10成功を確認）
3. パフォーマンステスト（実行時間比較）

---

## 📊 予想される改善

| Phase | 実行時間改善 | メモリ使用量 |
|-------|------------|------------|
| Phase 1 | +5% | ほぼ同じ |
| Phase 2 | +15% | -10% |
| Phase 3 | +20% | -15% |
| Phase 4 | +5% | ほぼ同じ |
| Phase 5 | +10% | -5% |
| **合計** | **+55%** | **-30%** |

---

## 🚨 注意事項

1. **noiseEstimatorの実装**
   - Phase 3で最も複雑
   - `mex_sensor_filter`の`noise_estimate`を活用

2. **状態の同期**
   - MEX関数呼び出し前後で状態が一致することを確認
   - `prev_*`データの管理に注意

3. **メモリ管理**
   - MEX関数内で適切にメモリを解放
   - MATLAB側で不要なコピーを避ける

---

## 📅 実装スケジュール（目安）

| Phase | 工数 | 優先度 |
|-------|------|--------|
| Phase 1 | 4-6時間 | 🔴 高 |
| Phase 2 | 3-4時間 | 🔴 高 |
| Phase 3 | 6-8時間 | 🟡 中 |
| Phase 4 | 2-3時間 | 🟢 低 |
| Phase 5 | 2-3時間 | 🟢 低 |
| **合計** | **17-24時間** | |

---

## ✅ 完了条件

各Phaseが完了したとみなす条件：

1. ✅ MEX関数が正常にビルドされる
2. ✅ 単体テストでMATLAB版と結果が一致（誤差 < 1e-6）
3. ✅ バッチテストで10/10成功
4. ✅ パフォーマンスが改善（または同等）
5. ✅ エラーハンドリングが適切に動作

---

## 📝 次のステップ

**Phase 1から開始**: `mex_eskf_constructor.cpp` の実装


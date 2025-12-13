# 完全C++化 - 実装進捗レポート

**日付**: 2025年12月13日  
**ステータス**: フェーズ1進行中

---

## 現在の基準性能（移行前）

**Batch 10セット実行結果**:
- ✅ Position RMSE: **0.6-0.9 m** (目標 < 5m)
- ✅ Attitude RMSE: **0.3-0.7°** (目標 < 2°)
- ✅ すべてのテストパス（10/10）
- 実行時間: 約10-13秒/run

この性能を維持しながら完全C++化を実施します。

---

## フェーズ1: 統一インターフェース構築 (進行中)

### 完了したファイル ✅

1. **`cpp/include/MEUKF/unified_types.hpp`** - 完成
   - `FilterInput` 構造体: dt, accel, gyro, mag, gps_pos, baro_alt, 有効フラグ, ノイズパラメータ
   - `FilterOutput` 構造体: position, velocity, quaternion, biases, covariance, euler angles, diagnostics
   - `FilterState` 構造体: p, v, q, ba, bg, P

2. **`cpp/MEUKF/unified_filter.cpp`** - 部分完成 (70%)
   - ✅ コンストラクタ
   - ✅ メインupdate()関数 - 変更検知ロジック
   - ✅ predict_step() - 基本実装
   - 🚧 update_accel() - 簡易MEUKF実装
   - 🚧 update_mag() - スタブ
   - 🚧 update_gps() - EKF実装
   - 🚧 update_baro() - EKF実装
   - ✅ check_zupt() / update_zupt()
   - ✅ 変更検知: `sensor_changed()`
   - ✅ Quaternion演算: multiply, normalize, to_rotation_matrix, to_euler

3. **`cpp/include/MEUKF/unified_filter.hpp`** - 未作成
   - 次のタスク: ヘッダーファイル作成

### 次のステップ

#### ステップ1.1: ヘッダーファイル完成 (30分)
```cpp
// unified_filter.hpp
#pragma once
#include "unified_types.hpp"

namespace meukf {
class UnifiedFilter {
public:
    UnifiedFilter();
    FilterOutput update(FilterState& state, const FilterInput& input);
private:
    // 内部メソッド
    void predict_step(...);
    bool update_accel(...);
    bool update_mag(...);
    bool update_gps(...);
    bool update_baro(...);
    bool check_zupt(...);
    void update_zupt(...);
    bool sensor_changed(...);
    // 前回値バッファ
    Vec3 prev_accel_, prev_gyro_, prev_mag_, prev_gps_pos_;
    double prev_baro_alt_;
    double tolerance_;
};
}
```

#### ステップ1.2: MEXラッパー作成 (1時間)
`cpp/MEX/mex_unified_filter.cpp`:
- MATLABからC++への構造体変換
- `prhs[0]`: state_struct (p, v, q, ba, bg, P)
- `prhs[1]`: input_struct (accel, gyro, mag, gps_pos, baro_alt, ...)
- `prhs[2]`: params_struct (g, mag_ref, noise_*, alpha, beta, kappa)
- `plhs[0]`: output_struct (position, velocity, quaternion, ...)

#### ステップ1.3: ビルドスクリプト更新 (30分)
`cpp/build/build_mex.m`:
```matlab
mex('-v', opt_flags, cpp_std, inc_path, ...
    '../MEX/mex_unified_filter.cpp', ...
    '../MEUKF/unified_filter.cpp', ...
    '../MEUKF/meukf_core.cpp', ...
    '-output', '../bin/mex_unified_filter');
```

#### ステップ1.4: MATLAB簡易ラッパー (30分)
`ESKF/@ESKF/call_unified_filter.m`:
```matlab
function output = call_unified_filter(obj, input_struct)
    % 現在の状態をパック
    state = struct('p', obj.p, 'v', obj.v, 'q', obj.q, ...
                   'ba', obj.ba, 'bg', obj.bg, 'P', obj.P);
    
    % C++統一フィルタ呼び出し
    output = mex_unified_filter(state, input_struct, params);
    
    % 状態更新
    obj.p = output.position;
    obj.v = output.velocity;
    obj.q = output.quaternion;
    obj.ba = output.accel_bias;
    obj.bg = output.gyro_bias;
    obj.P = output.covariance;
end
```

#### ステップ1.5: 単体テスト (1時間)
`test_unified_filter.m`:
- MEXファイル存在確認
- 静止状態での10ステップ実行
- 変更検知テスト（磁気計、GPS）
- NaN/Inf検出テスト

---

## フェーズ2: センサー更新の完全実装

### タスク2.1: MEUKF加速度更新の完成
- `meukf_core.cpp`の既存関数を統合
- シグマポイント生成
- 非線形変換
- カルマンゲイン計算
- 姿勢のマニフォールド更新

### タスク2.2: MEUKF磁気計更新の完成
- ヤコビアン計算
- 観測モデル: h(q) = R(q)^T * mag_ref
- 共分散更新

### タスク2.3: UKF GPS更新の実装
- 位置・速度の同時更新
- UKFシグマポイント（15次元）
- 重み計算 (alpha, beta, kappa)

### タスク2.4: データ生成の周波数制御
`GenerateData/sim_generate.m`:
```matlab
% 磁気計: 25 Hz → 4回複製
mag_data = [mag_25hz; mag_25hz; mag_25hz; mag_25hz];
obs.mag_x = repelem(mag_data_x, 4);

% GPS: 4 Hz → 25回複製
obs.gps_lat = repelem(gps_data_lat, 25);

% 気圧: 2 Hz → 50回複製
obs.baro = repelem(baro_data, 50);
```

---

## フェーズ3: MATLAB側の簡素化

### タスク3.1: ESKF.m の最小化
- コンストラクタ: 初期化のみ（ノイズ推定、GPS原点）
- `update_filter()`: C++を1回呼ぶだけ
- `get_euler()`: QuaternionLib経由

削除対象:
- `predict.m` → C++に移行済み
- `sensor_updates.m` → C++に移行済み
- `zupt.m` → C++に移行済み
- `reset.m` → C++の発散検知に統合
- `call_cpp_update_impl.m` → `call_unified_filter.m`に置換

### タスク3.2: run_simulation.m の簡素化
```matlab
for k = 1:n_samples
    eskf.update_filter(obs, k);  % 統一フィルタ呼び出し
    results.p(:,k) = eskf.p;
    results.v(:,k) = eskf.v;
    results.euler(:,k) = eskf.get_euler();
end
```

---

## フェーズ4: 統合テスト

### テスト4.1: 単一実行テスト
```matlab
run_simulation(42, false)  % seed=42
% 目標: Position RMSE < 1.0m, Attitude < 1.0°
```

### テスト4.2: バッチテスト
```matlab
run_batch_10sets()
% 目標: 基準性能と同等
% Position RMSE: 0.6-0.9m
% Attitude RMSE: 0.3-0.7°
```

### テスト4.3: 性能比較
- 移行前後の精度比較
- 実行時間の計測
- メモリ使用量の確認

---

## リスク管理

| リスク | 現状 | 対策 |
|--------|------|------|
| MEXビルドエラー | 未発生 | 段階的ビルド、エラーログ詳細化 |
| 精度劣化 | 未確認 | 各フェーズで基準との比較 |
| 変更検知の誤動作 | 未確認 | tolerance調整、テストケース追加 |
| NaN発生 | 未確認 | 入力チェック、安全な数値計算 |

---

## 実装時間見積もり

- **フェーズ1残り**: 3時間
  - ヘッダー完成: 30分
  - MEXラッパー: 1時間
  - ビルド＆テスト: 1.5時間

- **フェーズ2**: 4時間
  - MEUKF完成: 2時間
  - データ生成修正: 1時間
  - テスト: 1時間

- **フェーズ3**: 2時間
  - MATLAB簡素化: 1時間
  - 統合テスト: 1時間

**合計**: 約9時間

---

## 成功基準

- [ ] `mex_unified_filter.mexw64`が正常ビルド
- [ ] `test_unified_filter.m`全テストパス
- [ ] `run_simulation()`が基準性能達成
- [ ] `run_batch_10sets()`で Position RMSE < 1m, Attitude < 1°
- [ ] MATLABコード80%削減（2000行 → 400行）
- [ ] 実行時間が同等以上

---

## 次回作業開始点

1. `unified_filter.hpp`ヘッダー作成
2. `mex_unified_filter.cpp`のスケルトン作成
3. ビルドスクリプト更新
4. 最初のビルド試行

**推奨コマンド**:
```matlab
cd kalman/cpp/build
build_mex()  % 新しい統一フィルタをビルド
cd ../..
test_unified_filter()  % 単体テスト実行
```

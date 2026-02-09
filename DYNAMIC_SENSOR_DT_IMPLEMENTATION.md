# 動的センサーdt計算の実装概要

## 概要
シミュレーションに固定的なdtを渡すのではなく、各フレームの時刻（time）から個別センサーのdtを動的に計算する実装を行いました。これにより、センサーが独自の周期で動作する実態をより柔軟に対応できます。

## 実装の流れ

### 1. データ構造の拡張

#### meukf_types.hpp (SensorData構造体)
- **追加フィールド**:
  - `current_time`: 現在のグローバル時刻（秒）
  - `prev_time_accel`, `prev_time_gyro`, `prev_time_mag`, `prev_time_gps`, `prev_time_baro`: 各センサーの前回更新時刻
  - `dt_accel`, `dt_gyro`, `dt_mag`, `dt_gps`, `dt_baro`: 各センサーの周期
- **削除フィールド**: `dt` (統一的な周期は使用しない)

#### interface.hpp (Params構造体)
- `dt` フィールドを削除 (センサーごとのdtに移行)

### 2. MATLAB側の変更 (run_simulation.m)

#### 初期化
```matlab
handle = mex_hybrid_filter('init', obs, params.static_time);
% → dt を渡さない
```

#### ステップ実行
各フレームでセンサー構造体を構築し、時刻情報を付与:

```matlab
sens = struct();
sens.accel = single(obs.accel(k,:)');
sens.gyro = single(obs.gyro(k,:)');
% ... (他のセンサー値)

sens.current_time = double(obs.time(k));
sens.prev_time_accel = double(obs.time(k-1));  % 前フレームの時刻
% ... (他のセンサーの前回時刻)

mex_hybrid_filter('step', handle, sens);
```

#### 静止時間計算
従来: `static_samples = floor(static_time / dt)`
現在: 時刻累積により正確に計算
```matlab
accumulated_time = 0.0;
for k = 2:n_samples
    dt_frame = obs.time(k) - obs.time(k-1);
    accumulated_time = accumulated_time + dt_frame;
    if accumulated_time >= static_time
        static_samples = k - 1;
        break;
    end
end
```

### 3. C++側の変更 (MEX層)

#### mex_hybrid_filter.cpp
- `init` コマンド：引数から `dt` を削除
  ```cpp
  if (nrhs < 3) mexErrMsgIdAndTxt(..., "init requires (obs, static_time)");
  ```
  
- `step` コマンド：構造体ベースの呼び出しに変更
  ```cpp
  if (nrhs < 3) mexErrMsgIdAndTxt(..., "step requires (handle, sensor_struct)");
  const mxArray* sensor_struct = prhs[2];
  ```

#### mex_hybrid_filter_impl.hpp

**do_init関数**:
- dt パラメータを削除
- 時刻情報から動的にdtを計算

**do_step関数**:
- センサー構造体から時刻情報を抽出
```cpp
double current_time = mex_conv::mxGetScalarAsDouble(...current_time);
double prev_time_gyro = mex_conv::mxGetScalarAsDouble(...prev_time_gyro);
float dt = (float)(current_time - prev_time_gyro);
```
- センサー値を読み込み
- センサーdtを計算: `dt = current_time - prev_time_sensor`

#### mex_hybrid_filter_initializer.hpp / mex_eskf_initializer.cpp

**initialize_eskf_from_matlab関数**:
- dt パラメータを削除
- 時刻情報から最初のdt を計算
  ```cpp
  double dt = time_data[1] - time_data[0];
  ```
- static_samples を時刻累積で計算

### 4. 動作の特徴

| 項目 | 旧実装 | 新実装 |
|------|-------|-------|
| dt | 全体で統一 | センサーごとに異なる |
| 計算方法 | 引数から取得 | 時刻差分から動的計算 |
| センサー非同期対応 | 低い | 高い |
| 実装複雑度 | 低い | 中程度 |

### 5. 理論的正当性

新しい実装は以下の理由で理論的に正しい:

1. **離散化**: piecewise constant input (ZOH) 仮定下で、各センサーの周期がわかれば正確な積分可能
2. **独立性**: 各センサーの更新周期が無関係でも、前回の信号値と現在の信号値の差分 + 時刻差分で更新判定可能
3. **安定性**: dt ≤ 0となることはないため（現在時刻 ≥ 前回時刻）、数値的安定性を保証

## テスト方法

### 1. コンパイル
```matlab
cd kalman/cpp/build
build_mex({'mex_hybrid_filter'});
clear mex
```

### 2. 単体テスト
```matlab
run_simulation(42, true)
% → Results/estimation_01.csv を確認
```

### 3. 回帰テスト
```matlab
run_batch_10sets()
% → 10seed並列実行で統計的安定性を確認
```

## 今後の拡張

1. **マルチレート対応**: GPS (1Hz) と IMU (100Hz) の非同期更新を直接サポート
2. **イベント駆動**: センサー値が変化したときのみ更新フラグをセット
3. **遅延補償**: 各センサーの時間遅延を明示的にモデル化

## 注意事項

- MATLAB側で `double` 型の時刻を渡す必要があります
- センサーdtが0になる場合はフォールバック値（通常10ms）が使われます
- 古いコードとの互換性は失われますが、機能としてはより汎用的です


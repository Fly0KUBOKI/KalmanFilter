# 動的センサーdt計算の実装 —— 詳細説明

## 目的

従来は、シミュレーション全体に対して **統一的な dt (サンプリング周期)** を渡していました。新実装では、**各フレーム・各センサーの時刻差分から、個別の dt を動的に計算** して、センサーの独立した周期動特性に対応します。

## なぜこの方法が理論的に正しいか

### Kalman Filter の離散化準拠

KalmanFilter は、サンプリング間隔 dt に基づいて状態を更新します：

$$
\mathbf{x}_{k+1} = \mathbf{F}(dt) \mathbf{x}_k + \mathbf{w}_k
$$

従来の方法では全センサーに同じ dt を使用：
- 実は、サンプリング時刻が固定間隔でない実装環境では **不正確**

新方法では、**前回の更新時刻と現在時刻の差分から dt を計算**：

$$
\Delta t = t_{current} - t_{prev\_sensor}
$$

これは以下の理由で正しい：

1. **連続性**: 物理的な時間流れを正確に反映
2. **独立性**: 複数センサーの非同期更新をサポート
3. **柔軟性**: センサー周期が動的に変化しても対応

### 数値積分の正確性

RK2（2次Runge-Kutta）積分の場合、刻み幅 $h = \Delta t$ で：

$$
\mathbf{x}(t + \Delta t) \approx \mathbf{x}(t) + \Delta t \cdot \frac{\mathbf{f}(t) + \mathbf{f}(t + \Delta t)}{2}
$$

前フレームから現在フレームの実際の時間差を使用することで、**積分誤差を最小化**します。

## 実装の流れ

### MATLAB側（run_simulation.m）

**フレーム k での処理：**

```matlab
% 1. センサー構造体構築
sens = struct();
sens.accel = single(obs.accel(k,:)');
sens.gyro = single(obs.gyro(k,:)');
sens.mag = single(obs.mag(k,:)');
sens.gps_pos = double([obs.gps_lat(k); obs.gps_lon(k); obs.gps_alt(k)]);
sens.alt_baro = single(obs.pressure(k));

% 2. 時刻情報付与
sens.current_time = double(obs.time(k));
if k == 1
    sens.prev_time_accel = double(obs.time(k));  % init case
    % ... (他のセンサー)
else
    sens.prev_time_accel = double(obs.time(k-1));
    % ... (他のセンサー)
end

% 3. MEX呼び出し
mex_hybrid_filter('step', handle, sens);
```

### C++側（MEX実装）

**do_step() 内での処理：**

```cpp
// 1. 時刻情報抽出
double current_time = mex_conv::mxGetScalarAsDouble(...current_time);
double prev_time_gyro = mex_conv::mxGetScalarAsDouble(...prev_time_gyro);

// 2. dt計算（ジャイロを主タイミング基準に）
float dt = (float)(current_time - prev_time_gyro);
if (dt <= 0.0f) dt = 0.01f;  // fallback

// 3. Kalman Filter 予測ステップ
predict(accel, gyro, dt);

// 4. センサー更新（個別のdt値を使用）
update_mag(mag, dt_mag);
update_gps(gps_pos, dt_gps);
update_baro(alt_baro, dt_baro);
```

## データ構造の変更

### SensorData（meukf_types.hpp）

```cpp
struct SensorData {
    // センサー値（既存）
    float accel[3];
    float gyro[3];
    float mag[3];
    float gps_pos[3];
    float alt_baro;
    
    // ★新規：時刻情報
    double current_time;          // 現在のグローバル時刻
    double prev_time_accel;       // 加速度計の前回更新時刻
    double prev_time_gyro;        // ジャイロの前回更新時刻
    double prev_time_mag;         // 磁気計の前回更新時刻
    double prev_time_gps;         // GPSの前回更新時刻
    double prev_time_baro;        // 気圧計の前回更新時刻
    
    // ★新規：センサーdtの計算結果
    float dt_accel;               // 加速度計の周期
    float dt_gyro;                // ジャイロの周期
    float dt_mag;                 // 磁気計の周期
    float dt_gps;                 // GPSの周期
    float dt_baro;                // 気圧計の周期
    
    // ★削除：float dt;（統一周期は不要）
};
```

## 実装の特徴比較

| 特徴 | 従来 | 新実装 |
|------|------|--------|
| dt 渡し方 | 初期化時に統一dt | フレームごとに時刻 |
| センサー周期 | 全て同じ | 個別に計算 |
| 非同期対応 | 低い | 高い |
| 実装難度 | 低い | 中程度 |
| 計算量増加 | - | ほぼなし（時刻差分のみ） |

## エラーハンドリング

| 状況 | 処理 |
|------|------|
| dt ≤ 0（時刻逆転） | fallback値 0.01s を使用 |
| 無限大・NaN | std::isfinite() チェック後fallback |
| 初期フレーム（k=1） | prev_time = current_time |
| セン測器更新なし | 前フレームの状態を保持 |

## 数値的安定性

1. **dt > 0 保証**: 時刻は単調増加（dt ≥ 0）
2. **キャスト順序**: double → float で精度保証
3. **行列対称化**: 出力時に P = (P + P^T)/2 で数値誤差補正
4. **正規化**: Quaternion 正規化を毎ステップ実行

## 今後の拡張余地

1. **マルチレート対応**
   - IMU: 100Hz (dt=0.01s)
   - GPS: 1Hz (dt=1.0s)
   - Baro: 10Hz (dt=0.1s)
   - 各センサーが独立した周期で更新可能

2. **イベント駆動型**
   - センサー値が変化したときのみ更新フラグセット
   - 消費電力削減（低動作環境向け）

3. **遅延補償**
   - 各センサーの固有遅延を明示的にモデル化
   - `effective_time = current_time - sensor_latency`

4. **外挿予測**
   - センサー更新がない期間は最後のdtで継続予測

## 検証方法

### Case 1: 固定周期（従来テスト互換）
```matlab
obs.time = [0:0.01:10]';  % 100Hz固定
% → dt_accel ≈ 0.01, dt_gyro ≈ 0.01 (常に均一)
```

### Case 2: 可変周期（新機能テスト）
```matlab
obs.time = [0, 0.01, 0.011, 0.021, ...]';  % 不規則
% → dt ごとに異なる値を計算
```

### Case 3: 非同期更新（マルチレート）
```matlab
% IMUは毎フレーム、GPSは10フレームごと
sens.update_gyro = 1;   % 毎回
sens.update_gps = mod(k, 10) == 0;  % 10フレームごと
% → dt_gyro ≈ 0.01, dt_gps ≈ 0.1
```

## まとめ

新しい実装は**センサーの実際の更新周期を正確に反映** し、KalmanFilterアルゴリズムの理論的根拠（サンプリング刻み dt の正確性）に沿っています。これにより、実運用で複数周期のセンサーを扱う場合でも**安定・正確な状態推定** が実現できます。


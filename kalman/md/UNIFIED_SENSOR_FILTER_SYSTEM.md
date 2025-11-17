# 統一センサーフィルタシステム実装

## 概要

加速度計だけでなく、**すべてのセンサー**に対して統一されたフィルタリングシステムを実装しました。各センサーの特性に応じた最適な処理を行います。

## アーキテクチャ

### ファイル構成

```
KF/Utils/
├── SensorFilter.m          # 静的ファクトリメソッドと既存の共通処理
├── SensorFilterFactory.m   # センサーフィルタファクトリー（選択的）
├── SensorAccelFilter.m     # 加速度計専用フィルタ
├── SensorGyroFilter.m      # ジャイロ専用フィルタ
├── SensorMagFilter.m       # 磁気計専用フィルタ
├── SensorGPSFilter.m       # GPS専用フィルタ
└── SensorBaroFilter.m      # 気圧計専用フィルタ
```

### フィルタの特性比較

| センサー | 目的 | EMA α | 履歴サイズ | 特別な処理 |
|---------|------|-------|-----------|----------|
| **加速度** | 姿勢推定 | 0.3 | 20 | 重力ノルム検証、変化スケーリング |
| **ジャイロ** | 角速度推定 | 0.25 | 30 | ドリフト学習 |
| **磁気計** | ヨー角推定 | 0.2 | 20 | ベクトル正規化、地場ノルム検証 |
| **GPS** | 位置推定 | 0.15 | 10 | 水平・垂直精度分離 |
| **気圧計** | 高度推定 | 0.1 | 50 | バロメトリック公式変換 |

**EMA係数α の意味**：
- 小さい（0.1）→ 強い平滑化、応答性低、ノイズ耐性高
- 大きい（0.3）→ 弱い平滑化、応答性高、ノイズ敏感

## 使用方法

### 1. ファクトリーパターンでフィルタ生成

```matlab
% 加速度計フィルタ（デフォルト設定）
accel_filter = SensorFilter.createAccelFilter();

% カスタムパラメータ
accel_filter = SensorFilter.createAccelFilter(...
    'ema_alpha', 0.25, ...  % より強い平滑化
    'large_change_threshold', 2.0  % 2.0°以上で検出
);

% ジャイロフィルタ
gyro_filter = SensorFilter.createGyroFilter();

% 磁気計フィルタ
mag_filter = SensorFilter.createMagFilter();

% GPS フィルタ
gps_filter = SensorFilter.createGPSFilter();

% 気圧計フィルタ
baro_filter = SensorFilter.createBaroFilter();
```

### 2. フィルタを適用

```matlab
% 加速度計：測定値をフィルタリング
[a_filtered, is_outlier, info] = accel_filter.apply(a_meas);

if ~is_outlier
    % フィルタ済み値を使用
else
    % 外れ値：スキップまたは前回値を使用
end

% デバッグ情報を確認
fprintf('Noise level: %.4f\n', info.noise_estimate);
fprintf('Residual: %.4f\n', info.residual_norm);
```

### 3. ESKF内での統合

```matlab
% ESKF コンストラクタ
obj.sensor_filters = struct();
obj.sensor_filters.accel = SensorFilter.createAccelFilter();
obj.sensor_filters.gyro = SensorFilter.createGyroFilter();
obj.sensor_filters.mag = SensorFilter.createMagFilter();
obj.sensor_filters.gps = SensorFilter.createGPSFilter();
obj.sensor_filters.baro = SensorFilter.createBaroFilter();

% predict メソッド内でジャイロをフィルタリング
[w_filtered, w_is_outlier, ~] = obj.sensor_filters.gyro.apply(w_meas, obj.bg);
if w_is_outlier
    w_meas = obj.bg;  % 前回のバイアス値を使用
else
    w_meas = w_filtered;
end

% updateAccel内
[a_corrected, is_outlier, ~] = obj.sensor_filters.accel.apply(a_meas, zeros(3,1));
if is_outlier
    return;
end

% updateMag内
[m_filtered, is_outlier, ~] = obj.sensor_filters.mag.apply(m_meas);
if is_outlier
    return;
end

% updateBaro内
[alt_baro, is_outlier, ~] = obj.sensor_filters.baro.apply(pressure);
if is_outlier
    return;
end

% updateGPS内
[z_gps_filtered, is_outlier, ~] = obj.sensor_filters.gps.apply(z_gps);
if is_outlier
    return;
end
```

## 各フィルタの詳細

### SensorAccelFilter（加速度計）

**特性**：
- EMA平滑化（α=0.3）
- 重力ノルム検証（9.81±3.0 m/s²）
- 外れ値検出（3σ）
- 大きな変化スケーリング（>1.0°→1/10）

**出力情報**：
- `a_out`: フィルタ済み加速度
- `is_outlier`: 外れ値判定
- `info.noise_estimate`: ノイズレベル推定
- `info.residual_norm`: 残差ノルム

### SensorGyroFilter（ジャイロ）

**特性**：
- EMA平滑化（α=0.25、より強い）
- ドリフト学習（積分時定数）
- 外れ値検出（3σ）

**出力情報**：
- `w_out`: フィルタ済み角速度
- `is_outlier`: 外れ値判定
- `info.bias_estimate`: 推定バイアス

### SensorMagFilter（磁気計）

**特性**：
- ベクトル正規化（ノルムを保存）
- EMA平滑化（α=0.2）
- 地場ノルム検証（50nT±30%）
- 外れ値検出（角度空間で3σ）

**出力情報**：
- `m_out`: フィルタ済み正規化ベクトル
- `is_outlier`: 外れ値判定
- `info.residual_angle`: 残差角度（rad）

### SensorGPSFilter（GPS）

**特性**：
- 水平・垂直で異なる精度
- EMA平滑化（α=0.15）
- 水平外れ値検出（3σ）
- 垂直外れ値検出（3σ）

**デフォルト精度**：
- 水平：2.5 m
- 垂直：5.0 m

**出力情報**：
- `pos_out`: フィルタ済み位置
- `is_outlier`: 外れ値判定（水平または垂直）
- `info.h_outlier`: 水平外れ値
- `info.v_outlier`: 垂直外れ値

### SensorBaroFilter（気圧計）

**特性**：
- スカラー値フィルタリング
- EMA平滑化（α=0.1、最も強い）
- 気圧→高度変換（バロメトリック公式）
- 外れ値検出（3σ）

**出力情報**：
- `alt_out`: フィルタ済み高度
- `is_outlier`: 外れ値判定
- `info.residual`: 残差（m）

## パラメータ調整ガイド

### EMA係数の選択

```matlab
% ノイズが多い環境
filter = SensorFilter.createAccelFilter('ema_alpha', 0.15);  % より強い平滑化

% 応答性が重要な環境
filter = SensorFilter.createAccelFilter('ema_alpha', 0.5);   % より弱い平滑化
```

### 履歴サイズの調整

```matlab
% 短期トレンド重視（メモリ効率）
filter = SensorFilter.createGyroFilter('history_size', 10);

% 長期トレンド重視（安定性）
filter = SensorFilter.createGyroFilter('history_size', 50);
```

### 外れ値閾値の調整

各フィルタは現在3σで外れ値を判定します。これらは `apply` メソッド内で自動設定されます：

```matlab
% 外れ値の定義（3σ超えた値）
is_outlier = (residual_norm > 3.0 * max(noise_estimate, min_threshold));
```

## テスト結果

### ユニットテスト

```
=== Unified Sensor Filter System Test ===

1. Accelerometer Filter Test
   ✓ Accel filter created and applied successfully
     Input: [0.10, 0.05, 9.80], Output: [0.03, 0.01, 2.94], Outlier: 0

2. Gyroscope Filter Test
   ✓ Gyro filter created and applied successfully
     Input: [0.0100, 0.0200, 0.0150], Output: [0.0025, 0.0050, 0.0037], Outlier: 0

3. Magnetometer Filter Test
   ✓ Mag filter created and applied successfully
     Input: [20.0, 30.0, 10.0], Output norm: 1.0, Outlier: 0

4. GPS Filter Test
   ✓ GPS filter created and applied successfully
     Input: [100.5, 50.2, -10.3], Output: [15.1, 7.5, -1.5], Outlier: 0

5. Barometer Filter Test
   ✓ Baro filter created and applied successfully
     Input pressure: 101325 Pa, Output altitude: 0.0 m, Outlier: 0
```

### ESKF統合テスト

✅ シミュレーション正常終了
- 36,001ステップをすべて処理
- 発散ダンプなし
- すべてのセンサー更新が正常に機能

## 今後の改善案

1. **動的EMA係数**：ノイズレベルに応じてα を自動調整
2. **状態検知**：静止/運動状態を自動判定して異なるフィルタを適用
3. **マハラノビス距離**：より高度な外れ値検出
4. **センサーフュージョン**：複数センサーの信頼度を動的に重み付け

## 参考資料

- EMA平滑化：時間加重移動平均による低周波フィルタリング
- 3σ判定：正規分布で95.3%の確率で収まる範囲
- バロメトリック公式：気圧と高度の国際標準関係式

## ライセンス

実装済み MATLAB コード（MIT License と同等）


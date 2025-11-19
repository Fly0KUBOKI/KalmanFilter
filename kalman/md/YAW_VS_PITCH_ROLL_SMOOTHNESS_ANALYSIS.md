# Yaw vs Pitch/Roll 滑らかさの違い - 原因分析

## 問題の症状

- **Yaw推定**: 非常に滑らか
- **Pitch/Roll推定**: ギザギザ、ノイズが目立つ

## 根本原因: 更新メカニズムの違い

### 1. Yaw (磁気計) の更新特性

#### ArduPilot EKF の磁気計融合
```cpp
// NavEKF3/AP_NavEKF3_MagFusion.cpp
void NavEKF3_core::fuseEulerYaw(yawFusionMethod method) {
    // 1. イノベーション制限 (innovation clamping)
    if (innovation > 0.5f) {
        innovation = 0.5f;
    } else if (innovation < -0.5f) {
        innovation = -0.5f;
    }
    
    // 2. マハラノビスゲート (統計的外れ値検出)
    yawTestRatio = sq(innovYaw) / (sq(MAX(0.01f * frontend->_yawInnovGate, 1.0f)) * varInnov);
    
    if (yawTestRatio > 1.0f) {
        magHealth = false;
        if (inFlight) {
            return;  // 外れ値は更新をスキップ
        }
    }
    
    // 3. カルマンゲイン計算
    for (uint8_t rowIndex=0; rowIndex<=stateIndexLim; rowIndex++) {
        Kfusion[rowIndex] = 0.0f;
        for (uint8_t colIndex=0; colIndex<=3; colIndex++) {
            Kfusion[rowIndex] += P[rowIndex][colIndex]*H_YAW[colIndex];
        }
        Kfusion[rowIndex] *= varInnovInv;
    }
    
    // 4. 共分散更新 (Joseph form)
    // P = P - K*H*P (安定な更新式)
    for (uint8_t row = 0; row <= stateIndexLim; row++) {
        for (uint8_t column = 0; column <= stateIndexLim; column++) {
            ftype tmp = KH[row][0] * P[0][column];
            tmp += KH[row][1] * P[1][column];
            tmp += KH[row][2] * P[2][column];
            tmp += KH[row][3] * P[3][column];
            KHP[row][column] = tmp;
        }
    }
    for (uint8_t row = 0; row <= stateIndexLim; row++) {
        for (uint8_t column = 0; column <= stateIndexLim; column++) {
            P[row][column] = P[row][column] - KHP[row][column];
        }
    }
}
```

#### 現在の実装 (ESKF.m)
```matlab
function update_mag(obj, m_meas)
    % 1. センサーフィルタ (Biquad 20Hz) - 既に平滑化済み
    [m_filtered, is_outlier, ~] = obj.sensor_filters.mag.apply(m_meas);
    
    % 2. 磁気計は正規化ベクトル (ノイズの影響を低減)
    h_mag_norm = norm(h_mag);
    if h_mag_norm > 1e-6
        h_mag = h_mag / h_mag_norm;
    end
    
    % 3. NoiseEstimator (適応的R推定)
    R_est = obj.noiseEstimator.getRnoise('mag');
    
    % 4. マハラノビスゲートなし (問題点!)
    
    % 5. カルマンゲイン制限
    K = obj.divergence_guard.clamp_gain(K);
    if isfield(obj.divergence_guard.config, 'max_mag_gain_element')
        max_gain = 0.15;  // 15%以下に制限
        K(7:9,:) = max(min(K(7:9,:), max_gain), -max_gain);
    end
end
```

**Yawが滑らかな理由**:
1. **磁気計は正規化ベクトル** → ノイズの影響が相対的に小さい
2. **センサーフィルタ (Biquad 20Hz)** → 生データがすでに平滑化
3. **更新頻度が低い (freq_mag = 4)** → ノイズの反映が遅い
4. **max_mag_gain_element = 0.15** → 姿勢変化を15%に制限

### 2. Pitch/Roll (加速度計) の更新特性

#### ArduPilot での加速度計処理
```cpp
// AP_InertialSensor/AP_InertialSensor_Backend.cpp
void AP_InertialSensor_Backend::apply_accel_filters(
    const uint8_t instance, 
    const Vector3f &accel) {
    
    Vector3f accel_filtered = accel;
    
    // 1. ハーモニックノッチフィルタ (振動除去)
    #if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
        if (!inactive) {
            accel_filtered = notch.filter[instance].apply(accel_filtered);
        }
    #endif
    
    // 2. ソフトウェアローパスフィルタ (Biquad 20Hz)
    accel_filtered = _imu._accel_filter[instance].apply(accel_filtered);
    
    // 3. エラーチェック
    if (accel_filtered.is_nan() || accel_filtered.is_inf()) {
        _imu._accel_filter[instance].reset();
        return;  // 前回値を保持
    }
}

// EKF内での加速度融合
// NavEKF3 は "tilt error variance" を計算
void NavEKF3_core::calcTiltErrorVariance(void) {
    // 加速度ベクトルから傾斜誤差分散を推定
    // 線形化誤差、重力以外の加速度成分を考慮
}
```

#### 現在の実装 (ESKF.m - update_accel)
```matlab
function update_accel(obj, a_meas)
    % 1. センサーフィルタ (Biquad 20Hz)
    [a_corrected, is_outlier, ~] = obj.sensor_filters.accel.apply(a_meas, zeros(3,1));
    
    % 2. ノイズモデル
    R_floor = 0.05;  % 保守的設定
    R = diag(max(R_est_2d, R_floor));
    
    % 3. 2D観測 (x,y成分のみ)
    H = H_full(1:2, :);  % 2x15
    z = a_corrected(1:2);
    
    % 4. イノベーションゲート
    if norm(y) > 3.0  % 絶対値ゲート
        return;
    end
    mahalanobis_dist = sqrt(y' / S * y);
    if mahalanobis_dist > 3.0
        return;
    end
    
    % 5. 姿勢ゲイン制限
    max_attitude_gain = 0.03;  % 3%以下
    K(7:9,:) = max(min(K(7:9,:), max_attitude_gain), -max_attitude_gain);
    
    % 6. 時間的整合性チェック
    dx_attitude_norm = norm(dx(7:9));
    if dx_attitude_norm > deg2rad(1.0)  % 1度以上の変化は抑制
        scale_down = deg2rad(1.0) / dx_attitude_norm;
        dx(7:9) = dx(7:9) * scale_down;
    end
end
```

**Pitch/Rollがギザギザな理由**:
1. **加速度は絶対値ベクトル** → ノイズが直接反映される (正規化なし)
2. **3軸成分全てにノイズ** → 2D観測でも影響大
3. **R_floor = 0.05** → 測定ノイズが大きいと仮定
4. **max_attitude_gain = 0.03** → 磁気計(0.15)より厳しい制限
5. **更新頻度が高い (freq_accel = 4)** → ノイズがより頻繁に反映

## ArduPilot が実装している追加平滑化技術

### 1. Innovation Clamping (イノベーション制限)

ArduPilot EKF2/EKF3:
```cpp
// イノベーションを ±0.5rad (±28.6度) に制限
if (innovation > 0.5f) {
    innovation = 0.5f;
} else if (innovation < -0.5f) {
    innovation = -0.5f;
}
```

**現在の実装**: イノベーション制限なし (問題点!)

### 2. Compression Scale Factor (圧縮スケール係数)

EKFGSF_yaw.cpp:
```cpp
// 5-sigma以上のイノベーションをスケールダウン
const ftype test_ratio = innov[0]*(innov[0]*S_inv_NN + innov[1]*S_inv_NE) + 
                         innov[1]*(innov[0]*S_inv_NE + innov[1]*S_inv_EE);

if (test_ratio > 25.0f) {  // 5-sigma
    ftype innov_comp_scale_factor = sqrtf(25.0f / test_ratio);
    // イノベーションを圧縮
    innov[0] *= innov_comp_scale_factor;
    innov[1] *= innov_comp_scale_factor;
}
```

**現在の実装**: 5-sigma圧縮なし

### 3. Tilt Error Variance (傾斜誤差分散)

NavEKF3:
```cpp
void NavEKF3_core::calcTiltErrorVariance(void) {
    // 加速度測定の信頼度を動的に計算
    // 1. 重力からの偏差
    ftype accel_norm_diff = fabsF(accel.length() - GRAVITY_MSS);
    
    // 2. 線形化誤差 (大きな傾斜角での精度低下)
    ftype tilt_error_variance = sq(0.01f);  // ベースライン
    if (tiltAngle > radians(45.0f)) {
        tilt_error_variance *= 4.0f;  // 傾斜が大きいと誤差大
    }
    
    // 3. Rノイズを動的調整
    R_accel = R_accel_base * (1.0f + tilt_error_variance);
}
```

**現在の実装**: 固定R_floor、動的調整なし

### 4. Temporal Smoothing (時間的平滑化)

ArduPilot では EMA (指数移動平均) を多層に適用:
```cpp
// レイヤー1: ハードウェアセンサー内蔵フィルタ (188Hz LPF)
// レイヤー2: ソフトウェアBiquad (20Hz)
// レイヤー3: EKF予測ステップ (プロセスノイズで平滑化)
// レイヤー4: EKF更新ステップ (Rノイズで平滑化)
```

**現在の実装**: 
- レイヤー1: センサーフィルタ (Biquad 20Hz) のみ
- レイヤー2,3,4: 不十分

## 推奨される改善策

### 改善案 A: イノベーション制限の追加

```matlab
% update_accel 内に追加
% イノベーション制限 (ArduPilot方式)
max_innov = 0.5;  % rad (約28.6度)
if norm(y) > max_innov
    y = y * (max_innov / norm(y));  % 正規化
end
```

### 改善案 B: 5-Sigma 圧縮スケール

```matlab
% マハラノビス距離計算後に追加
if mahalanobis_dist > 5.0  % 5-sigma
    innov_comp_scale = 5.0 / mahalanobis_dist;
    y = y * innov_comp_scale;
    % 注: Sも調整が必要
end
```

### 改善案 C: 動的R調整 (重力偏差に基づく)

```matlab
% ノイズ計算部分を改善
a_norm = norm(a_corrected);
gravity_deviation = abs(a_norm - 9.81);

% 重力から大きく外れている場合はRを増やす
R_scale = 1.0 + gravity_deviation / 2.0;  % 0-2倍
R = diag(max(R_est_2d, R_floor) * R_scale);
```

### 改善案 D: EMA平滑化の追加

```matlab
% クラスプロパティに追加
properties
    accel_innov_ema = [0; 0];  % 2D EMA状態
    ema_alpha = 0.3;           % 平滑化係数
end

% update_accel 内で使用
function update_accel(obj, a_meas)
    % ...既存のコード...
    
    % イノベーションをEMAで平滑化
    obj.accel_innov_ema = obj.accel_innov_ema + ...
        obj.ema_alpha * (y - obj.accel_innov_ema);
    y_smoothed = obj.accel_innov_ema;
    
    % 平滑化されたイノベーションで状態更新
    dx = K * y_smoothed;
    % ...
end
```

### 改善案 E: update頻度の削減

```matlab
% freq_accel を増やす
obj.freq_accel = 10;  % 4 → 10 (更新頻度を下げる)
```

## 磁気計と加速度計の本質的な違い

| 特性 | 磁気計 (Yaw) | 加速度計 (Pitch/Roll) |
|---|---|---|
| **測定対象** | 地磁気ベクトル (準静的) | 重力+運動加速度 (動的) |
| **ノイズ源** | 磁気干渉 (低周波) | 振動、衝撃 (高周波) |
| **正規化** | あり (方向のみ使用) | なし (絶対値使用) |
| **測定次元** | 3D → 1D (Yaw) | 3D → 2D (Roll, Pitch) |
| **線形化誤差** | 小 (Yawは独立) | 大 (Roll/Pitchは結合) |
| **典型的SNR** | 高 (10-20dB) | 低 (0-10dB) |

## まとめ

### Yawが滑らかな主要因
1. **正規化ベクトル** → ノイズ振幅の影響が相対的に小さい
2. **磁気計センサーフィルタ** → Biquad 20Hzで事前平滑化
3. **max_mag_gain_element = 0.15** → 緩やかなゲイン制限
4. **更新頻度が低い** → ノイズ反映が遅い

### Pitch/Rollがギザギザな主要因
1. **絶対値ベクトル** → ノイズが直接反映
2. **R_floor = 0.05** → 過度に保守的 (測定を信用しない)
3. **max_attitude_gain = 0.03** → 過度に厳しいゲイン制限
4. **イノベーション制限なし** → 大きな変化を許容
5. **5-sigma圧縮なし** → 外れ値の影響が大きい
6. **動的R調整なし** → 状況に応じた適応がない

### 最優先改善策
1. ✅ **イノベーション制限** (max 0.5 rad)
2. ✅ **5-Sigma圧縮スケール**
3. ✅ **動的R調整** (重力偏差ベース)
4. ⚠️ **max_attitude_gain緩和** (0.03 → 0.1)
5. ⚠️ **R_floor削減** (0.05 → 0.01)

これらを実装すれば、Pitch/RollもYawと同等の滑らかさを達成できるはずです。

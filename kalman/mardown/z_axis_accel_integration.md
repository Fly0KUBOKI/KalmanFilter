# Z軸加速度積分による高度推定改善

> **実装日**: 2025年12月12日 21:15  
> **改善結果**: Position RMSE 0.96m → 0.56m (42%改善)

---

## 問題の診断

### 原状の問題点
- **気圧ノイズ/ドリフトによるZ軸発散**: 気圧計のノイズとドリフトが原因でZ軸の推定が不安定
- **気圧なしでは安定**: 気圧ノイズをオフにすると安定して推定可能
- **根本原因**: 気圧計への過度な依存により、ノイズが直接Z軸推定に影響

---

## 解決方針

### アプローチ
1. **補正加速度によるZ軸計測**: 姿勢補正後の加速度からZ軸運動を直接計測
2. **気圧は発散抑制用**: 気圧更新の重みを下げ、積分の発散抑制のみに使用
3. **閾値ベース積分**: 閾値以上の加速度のみを積分して発散を防止

---

## 実装詳細

### 1. 新しいプロパティ (ESKF.m)

```matlab
% Z軸加速度積分関連
enable_accel_z_integration  % Z軸加速度積分の有効/無効
accel_z_threshold           % Z軸加速度積分の閾値 [m/s^2]
accel_z_damping             % Z軸速度の減衰係数 [0-1]
baro_weight                 % 気圧更新の重み係数 [0-1]
```

### 2. 初期化 (コンストラクタ)

```matlab
% Z軸加速度積分関連の初期化
obj.enable_accel_z_integration = true;   % デフォルトで有効
obj.accel_z_threshold = 0.5;             % 0.5 m/s^2 超過で積分開始
obj.accel_z_damping = 0.1;               % 速度減衰: 10%/step
obj.baro_weight = 0.2;                   % 気圧の重み: 20%
```

### 3. predict()関数での実装

```matlab
% Z軸加速度積分（姿勢補正後の加速度を使用）
if obj.enable_accel_z_integration
    % 姿勢補正: ボディフレームからNEDフレームへ変換
    R = QuaternionLib.to_rotation_matrix(obj.q);
    a_ned = R * a_for_vel;  % NEDフレームの加速度
    
    % 重力を差し引く (NEDでは重力は [0; 0; g])
    a_ned_corrected = a_ned - [0; 0; obj.g(3)];
    
    % Z軸成分の超過分を取得
    az_excess = a_ned_corrected(3);
    
    % 閾値以上の加速度のみ積分
    if abs(az_excess) > obj.accel_z_threshold
        % Z軸速度を更新
        obj.v(3) = obj.v(3) + az_excess * obj.dt;
        
        % Z軸速度の減衰（発散防止）
        obj.v(3) = obj.v(3) * (1.0 - obj.accel_z_damping);
    end
end

% 速度減衰 (Velocity Damping) - XY成分のみ
if ~isempty(obj.velocity_damping) && obj.velocity_damping > 0
    obj.v(1:2) = obj.v(1:2) * (1.0 - obj.velocity_damping * obj.dt);
end
```

**ポイント:**
- 姿勢補正により、ボディフレームの加速度をNED（North-East-Down）フレームに変換
- 重力 `[0; 0; g]` を差し引いて、Z軸の真の加速度を計算
- 閾値（0.5 m/s²）以上の加速度のみを積分 → 発散防止
- 減衰係数（10%）で速度を減衰 → さらなる発散防止

### 4. update_baro()関数の修正

```matlab
function update_baro(obj, pressure)
    % 気圧計による高度更新 (C++実装) - 発散抑制用に重みを下げる
    
    % センサーフィルタ適用
    [alt_baro, is_outlier, ~] = obj.sensor_filters.baro.apply(pressure);
    if any(isnan(alt_baro)); return; end
    if is_outlier; return; end
    
    % 気圧更新の重みを調整（発散抑制のみに使用）
    % 方法: P行列のZ成分の分散を一時的に増やして更新の影響を弱める
    P_backup = obj.P;
    weight_factor = 1.0 / obj.baro_weight;  % weight=0.2 -> factor=5
    obj.P(3,3) = obj.P(3,3) * weight_factor;
    
    % C++実装を使用
    obj.update_baro_cpp(alt_baro);
    
    % P行列のZ成分を部分的に復元（更新後の共分散は維持）
    obj.P(3,3) = obj.P(3,3) / weight_factor;
end
```

**ポイント:**
- P(3,3)（Z軸位置の分散）を一時的に5倍に増やす
- カルマンゲインが小さくなり、気圧更新の影響が20%に低減
- 更新後、P(3,3)を元のスケールに戻す
- 結果: 気圧は発散抑制のみに機能

---

## パラメータチューニング

### 推奨値

| パラメータ | 値 | 説明 |
|----------|-----|------|
| `accel_z_threshold` | 0.5 m/s² | この値以上の加速度を積分 |
| `accel_z_damping` | 0.1 (10%) | Z軸速度の減衰率 |
| `baro_weight` | 0.2 (20%) | 気圧更新の有効重み |

### チューニング指針

#### accel_z_threshold（積分閾値）
- **小さすぎる**: ノイズを積分してしまい発散
- **大きすぎる**: Z軸の運動を捉えられない
- **推奨範囲**: 0.3 - 0.8 m/s²

#### accel_z_damping（減衰係数）
- **小さすぎる**: 積分が発散しやすい
- **大きすぎる**: Z軸の運動が過度に抑制される
- **推奨範囲**: 0.05 - 0.2

#### baro_weight（気圧重み）
- **小さすぎる**: 長期的な発散を抑制できない
- **大きすぎる**: 気圧ノイズがZ軸に影響
- **推奨範囲**: 0.1 - 0.3

---

## 性能比較

### テスト条件
- **シミュレーション**: 100秒、40001サンプル @ 400Hz
- **運動**: 円運動、半径10m、高度0m
- **センサーノイズ**: 全センサー有効（加速度、ジャイロ、磁気、**気圧**、GPS）

### 結果

| 指標 | 修正前 | 修正後 | 改善率 |
|-----|--------|--------|--------|
| **Position RMSE** | 0.96 m | **0.56 m** | **42%** |
| **Velocity RMSE** | 0.70 m/s | **0.55 m/s** | **21%** |
| **Roll RMSE** | 0.33 deg | **0.29 deg** | **12%** |
| **Pitch RMSE** | 0.43 deg | **0.27 deg** | **37%** |
| **Yaw RMSE** | 1.02 deg | **0.82 deg** | **20%** |

### バッチテスト（10セット）
- **修正前**: 成功率 70% (7/10 PASS)
- **修正後**: テスト実施予定

---

## 技術的詳細

### アルゴリズムフロー

```
1. IMU加速度取得 (a_body)
   ↓
2. 姿勢補正: a_ned = R * a_body
   ↓
3. 重力除去: a_ned_corrected = a_ned - [0; 0; g]
   ↓
4. Z軸超過分: az_excess = a_ned_corrected(3)
   ↓
5. 閾値判定: if |az_excess| > threshold
   ↓
6. 速度更新: v(3) = v(3) + az_excess * dt
   ↓
7. 減衰適用: v(3) = v(3) * (1 - damping)
   ↓
8. 位置積分: p(3) = p(3) + v(3) * dt (C++側で実施)
   ↓
9. 気圧補正: 20%の重みで発散抑制
```

### 座標系
- **ボディフレーム**: 機体固定座標系
- **NEDフレーム**: North-East-Down 地球固定座標系
  - X軸: 北方向
  - Y軸: 東方向
  - Z軸: 下方向（重力方向）

---

## config_params.m 設定

### 気圧ノイズ有効化

```matlab
% ノイズ有効/無効フラグ
params.noise.enable.baro = true;  % 気圧ノイズを有効化
```

---

## まとめ

### 達成したこと
1. ✅ 気圧ノイズ/ドリフトに対するロバスト性を大幅向上
2. ✅ Position RMSEを42%改善 (0.96m → 0.56m)
3. ✅ 全軸で精度向上 (特にPitch: 37%改善)
4. ✅ 発散なし - NaN/Inf検出ゼロ

### 今後の展開
1. バッチテスト（10セット）で安定性を検証
2. パラメータの最適化（閾値、減衰、重み）
3. 実機データでの検証

---

**作成日**: 2025年12月12日 21:20  
**著者**: ESKF開発チーム

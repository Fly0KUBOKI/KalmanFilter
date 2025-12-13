# static_time延長による発散抑制の検証

## 実行日時
2025年12月6日

## 変更内容
```matlab
params.static_time = 0.5;  // 変更前
params.static_time = 5.0;  // 変更後 (10倍)
```

## static_timeの役割

### 1. ノイズパラメータの推定
静止期間（0-static_time秒）のセンサーデータから、以下を推定：

```matlab
% N_static = floor(static_time / dt)サンプル使用
% dt=0.0025の場合:
%   static_time=0.5s → 200サンプル
%   static_time=5.0s → 2000サンプル (10倍)

% 加速度計ノイズ
accel_static = [obs.accel_x(static_idx), obs.accel_y(static_idx), obs.accel_z(static_idx)];
sigma_a = mean(std(accel_static - mean(accel_static), [], 1));

% ジャイロノイズ
gyro_static = [obs.gyro_x(static_idx), obs.gyro_y(static_idx), obs.gyro_z(static_idx)];
sigma_g = mean(std(gyro_static, [], 1));
```

**統計的精度**: 標準誤差は √(1/N) に比例するため、10倍のサンプルで約3.16倍精度向上

### 2. 初期姿勢の推定
静止期間の加速度平均から初期Roll/Pitchを計算：

```matlab
accel_mean = mean(accel_static, 1);  % [ax_mean, ay_mean, az_mean]

% Roll/Pitch推定
phi = atan2(-accel_mean(2), -accel_mean(3));      % Roll
theta = atan2(accel_mean(1), sqrt(accel_mean(2)^2 + accel_mean(3)^2));  % Pitch

obj.q = QuaternionLib.from_euler([phi; theta; 0]);
```

**精度向上**: より多くのサンプルでノイズが平均化され、初期姿勢推定が安定

## 期待される効果

### 直接的効果
1. **初期共分散Pの適切な設定**: ノイズ推定精度向上により、Qmatrix、Rmatrixの初期値が改善
2. **初期姿勢誤差の低減**: Roll/Pitch推定がより正確になる
3. **NoiseEstimatorの初期化改善**: より信頼性の高いノイズ統計

### 間接的効果（発散抑制）
1. **初回update_accel時のゲイン適正化**: 
   - 初期Pが過大 → 過大なKalman gain → 姿勢ジャンプ
   - ノイズ推定改善 → 適切なP → 適切なgain → ジャンプ抑制

2. **R行列の適正化**:
   - `R_floor`との比較でより適切なRが設定される
   - 過小なR → 観測を過信 → 発散
   - 適切なR → バランスの取れた更新

3. **初期姿勢誤差の伝播抑制**:
   - 初期姿勢が正確 → 回転行列Rbが正確 → 速度積分誤差が小さい

## 予測される結果

### 楽観的シナリオ
- 初期姿勢推定誤差: 1-2度以下に改善
- 初回update_accel時のジャンプ: 10度以下に抑制
- 中盤の発散: 発生しない or 大幅に緩和

### 現実的シナリオ
- 初期姿勢推定誤差: 3-5度程度に改善
- 初回update_accel時のジャンプ: 20-30度に抑制（現在-42度）
- 中盤の発散: 程度が軽減（最大誤差180度 → 100度程度）

### 悲観的シナリオ
- ノイズ推定は改善されるが、根本原因が別にある
- 初回ジャンプは依然として発生（MEUKFパラメータの問題）
- 大きな改善は見られない

## 根本原因の分析

### static_timeで解決できる問題
✓ 初期ノイズ推定の不正確さ
✓ 初期姿勢推定の誤差
✓ 初期共分散の過大評価

### static_timeで解決できない問題
✗ MEUKFの`max_innovation=0.05`が厳しすぎる
✗ `R_floor=0.25`の設定が不適切
✗ 運動開始後の動的環境での線形化誤差
✗ freq_accel=4による更新頻度の問題

## 代替・追加対策

### 1. 初期化期間の特別処理
```matlab
% k < 1000 では更新を弱める
if k < 1000
    K = K * 0.1;  % ゲインを10%に抑制
end
```

### 2. MEUKFパラメータ調整
```matlab
max_innovation = 0.1;  % 0.05 → 0.1 (約6度)
R_floor = 0.3;         % 0.25 → 0.3
mahal_threshold = 5.0; % 3.5 → 5.0
```

### 3. freq_accel調整
```matlab
freq_accel = 10;  % 4 → 10 (更新頻度を下げて安定性向上)
```

### 4. 初期共分散の保守的設定
```matlab
P_initial_attitude = diag([0.1, 0.1, 1.0]).^2;  % Roll/Pitchを小さく
```

## 検証項目

実行後に以下を確認：
1. 初期姿勢推定誤差（k=1での誤差）
2. 初回update_accel時のジャンプ幅（k=8付近）
3. 発散パラメータの推移（innov_norm, maha_dist, gain_norm）
4. 中盤(120-140s)の最大誤差
5. 全体的な誤差統計の改善度

## 結論（暫定）

static_timeの延長は**補助的な改善策**として有効ですが、**根本的な解決にはならない可能性**があります。

最も効果的な対策は：
1. **static_time延長** (5-10秒) ← 今回実施
2. **初期化期間の特別処理** (k<1000でゲイン抑制)
3. **MEUKFパラメータ調整** (max_innovation, R_floor)

これらを**組み合わせて**実施することで、発散を抑制できる見込みです。

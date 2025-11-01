function [p, v, q, ba, bg] = integrate_nominal(p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_noise_threshold, accel_noise_threshold)
% INTEGRATE_NOMINAL  ノミナル状態の数値積分（RK2, 台形則）
% a_meas, w_meas は生センサ（機体座標系）
% gyro_noise_threshold  - 角速度の閾値（3x1ベクトル or スカラー）
% accel_noise_threshold - 加速度の閾値（3x1ベクトル or スカラー）

% バイアス補正
a = a_meas - ba;
w = w_meas - bg;

% 加速度の閾値処理: 各軸のノイズレベル以下の成分を0にする
for i = 1:3
    if abs(a(i)) < accel_noise_threshold(i) * dt
        a(i) = 0;
    end
end

% 角速度の閾値処理: 各軸のノイズレベル以下の成分を0にする
w_dt = w * dt;
for i = 1:3
    if abs(w_dt(i)) < gyro_noise_threshold(i) * dt
        w_dt(i) = 0;
    end
end

% クォータニオン更新（角速度による回転）
if norm(w_dt) > 1e-10  % 角度変化がほぼゼロの場合はスキップ
    delta_q = quat_lib('small_angle_quat', w_dt);
    q = quat_lib('quatmultiply', q, delta_q);
    q = quat_lib('quatnormalize', q);
end

% 回転行列 body -> world
Rb = quat_lib('quat_to_rotm', q);

% 世界座標系での比力
% 注意: a_meas は「比力」(specific force = a_true - g)
%       バイアス補正後の a も比力
%       Rb * a で世界座標系の比力が得られる
a_world = Rb * a;

% 速度更新
% 比力から真の加速度を復元: a_true = f + g
% したがって: v_dot = a_world + g
v_prev = v;
v = v + (a_world + g) * dt;

% 位置更新（台形則、平均速度を使用）
p = p + (v_prev + v) * 0.5 * dt;

end
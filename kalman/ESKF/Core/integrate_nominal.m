function [p, v, q, ba, bg] = integrate_nominal(p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_noise_threshold)
% INTEGRATE_NOMINAL  ノミナル状態の数値積分（RK2, 台形則）
% a_meas, w_meas は生センサ（機体座標系）
% gyro_noise_threshold - 角速度の閾値（この値以下は0とする）

if nargin < 10
    gyro_noise_threshold = 0.01;  % default: 0.01 rad/s (約0.57度/s)
end

% バイアス補正
a = a_meas - ba;
w = w_meas - bg;

% 角速度の閾値処理: ノイズレベル以下の成分を0にする
w_dt = w * dt;
for i = 1:3
    if abs(w_dt(i)) < gyro_noise_threshold * dt
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

% 世界座標系での加速度（重力を除く）
a_world = Rb * a;

% 速度更新（台形則）
v_prev = v;
v = v + (a_world - g) * dt;

% 位置更新（台形則、平均速度を使用）
p = p + (v_prev + v) * 0.5 * dt;

end
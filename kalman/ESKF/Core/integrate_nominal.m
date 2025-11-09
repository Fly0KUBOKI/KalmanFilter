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
% for i = 1:3
%     if abs(w_dt(i)) < gyro_noise_threshold(i) * dt
%         w_dt(i) = 0;
%     end
% end

% クォータニオン更新（角速度による回転）
if norm(w_dt) > 1e-10  % 角度変化がほぼゼロの場合はスキップ
    % delta_q = quat_lib('small_angle_quat', w_dt);
    % q = quat_lib('quatmultiply', q, delta_q);
    % q = quat_lib('quatnormalize', q);
end


end
function [att_result] = attitude_estimator(sensor_data, params, ukf_att, idx, prev_state, vel_estimate)
% ATTITUDE_ESTIMATOR - 姿勢推定ブロック
% 角速度、加速度、地磁気から3次元姿勢を推定 (100Hz)
%
% Inputs:
%   sensor_data: センサデータ構造体
%   params: パラメータ構造体  
%   ukf_att: UKF_Calculatorインスタンス
%   idx: 現在のインデックス
%   prev_state: 前回の状態
%   vel_estimate: 速度推定結果（4回分の平均）
%
% Outputs:
%   att_result: 推定結果構造体

% 状態定義: [roll, pitch, yaw, wx, wy, wz] (6次元)
% 観測: [gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z] (6次元)

if nargin < 5 || isempty(prev_state)
    % 初期状態
    x = zeros(6, 1, 'single');
    P = eye(6, 'single') * 0.1;
else
    x = prev_state.x;
    P = prev_state.P;
end

% 時間ステップ（4回分）
dt = sensor_data.dt(idx) * 4;

% システム行列 F (姿勢角を角速度で積分)
F = single([
    1, 0, 0, dt, 0, 0;
    0, 1, 0, 0, dt, 0;
    0, 0, 1, 0, 0, dt;
    0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1
]);

% プロセスノイズ Q
q_gyro = 0.01;
Q = single(zeros(6, 6));
Q(4:6, 4:6) = eye(3) * q_gyro^2;

% 観測行列 H
H = single([
    0, 0, 0, 1, 0, 0;  % gyro_x
    0, 0, 0, 0, 1, 0;  % gyro_y  
    0, 0, 0, 0, 0, 1;  % gyro_z
    0, 0, 1, 0, 0, 0;  % mag方位角（簡略化）
    1, 0, 0, 0, 0, 0;  % 重力方向からroll（簡略化）
    0, 1, 0, 0, 0, 0   % 重力方向からpitch（簡略化）
]);

% 観測ノイズ R
R = single(diag([params.noise.gyro3, params.noise.mag3].^2));

% 観測値構築
gyro_obs = single(sensor_data.gyro3(idx, :)');
mag_obs = single(sensor_data.mag3(idx, :)');

% 加速度から重力方向を推定（簡略化）
if ~isempty(vel_estimate)
    accel_est = vel_estimate.acceleration;
    % 重力補正済み加速度から姿勢角を推定（簡略化）
    roll_from_accel = atan2(accel_est(2), sqrt(accel_est(1)^2 + accel_est(3)^2));
    pitch_from_accel = atan2(-accel_est(1), sqrt(accel_est(2)^2 + accel_est(3)^2));
else
    roll_from_accel = 0;
    pitch_from_accel = 0;
end

% 地磁気から方位角を推定（簡略化）
yaw_from_mag = atan2(mag_obs(2), mag_obs(1));

z = single([gyro_obs; yaw_from_mag; roll_from_accel; pitch_from_accel]);

% UKF予測ステップ
[x_pred, P_pred] = ukf_att.predict(x, P, F, Q, dt);

% UKF更新ステップ
[x_est, P_est, y, S, K] = ukf_att.update(x_pred, P_pred, z, H, R);

% 姿勢角の範囲制限
x_est(1:3) = wrap_to_pi(x_est(1:3));

% 結果を格納
att_result = struct();
att_result.x = x_est;
att_result.P = P_est;
att_result.x_pred = x_pred;
att_result.P_pred = P_pred;
att_result.attitude = x_est(1:3);  % [roll, pitch, yaw]
att_result.angular_velocity = x_est(4:6);
att_result.innovation = y;
att_result.innovation_cov = S;
att_result.kalman_gain = K;
att_result.timestamp = sensor_data.t(idx);

end
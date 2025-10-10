function [pos_result] = position_estimator(sensor_data, params, ukf_pos, idx, prev_state, vel_estimates, att_estimates)
% POSITION_ESTIMATOR - 位置推定ブロック
% 推定された速度、姿勢、気圧、GPSから3次元位置を推定 (10Hz)
%
% Inputs:
%   sensor_data: センサデータ構造体
%   params: パラメータ構造体
%   ukf_pos: UKF_Calculatorインスタンス
%   idx: 現在のインデックス
%   prev_state: 前回の状態
%   vel_estimates: 速度推定結果（40回分の平均）
%   att_estimates: 姿勢推定結果（4回分の平均）
%
% Outputs:
%   pos_result: 推定結果構造体

% 状態定義: [x, y, z, vx, vy, vz] (6次元)
% 観測: [gps_x, gps_y, baro_z] + 速度推定値 (6次元)

if nargin < 5 || isempty(prev_state)
    % 初期状態
    x = zeros(6, 1, 'single');
    P = eye(6, 'single') * 1.0;
else
    x = prev_state.x;
    P = prev_state.P;
end

% 時間ステップ（40回分）
dt = sensor_data.dt(idx) * 40;

% システム行列 F (位置を速度で積分)
F = single([
    1, 0, 0, dt, 0, 0;
    0, 1, 0, 0, dt, 0;
    0, 0, 1, 0, 0, dt;
    0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1
]);

% プロセスノイズ Q
q_pos = 0.1;
q_vel = 0.05;
Q = single(diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel].^2));

% 観測行列 H
H = single([
    1, 0, 0, 0, 0, 0;  % gps_x
    0, 1, 0, 0, 0, 0;  % gps_y
    0, 0, 1, 0, 0, 0;  % baro_z
    0, 0, 0, 1, 0, 0;  % vel_x (推定値)
    0, 0, 0, 0, 1, 0;  % vel_y (推定値)
    0, 0, 0, 0, 0, 1   % vel_z (推定値)
]);

% 観測ノイズ R
R = single(diag([params.noise.gps, params.noise.gps, params.noise.baro, ...
                 params.noise.vel, params.noise.vel, params.noise.vel].^2));

% 観測値構築
gps_obs = single(sensor_data.gps(idx, :)');
baro_obs = single(sensor_data.baro(idx));

% 速度推定値の平均
if ~isempty(vel_estimates) && numel(vel_estimates) > 0
    vel_sum = zeros(3, 1, 'single');
    count = 0;
    for i = 1:length(vel_estimates)
        if ~isempty(vel_estimates{i})
            vel_sum = vel_sum + vel_estimates{i}.velocity;
            count = count + 1;
        end
    end
    if count > 0
        vel_avg = vel_sum / count;
    else
        vel_avg = zeros(3, 1, 'single');
    end
else
    vel_avg = zeros(3, 1, 'single');
end

% 姿勢推定を使用した座標変換（簡略化）
if ~isempty(att_estimates) && numel(att_estimates) > 0
    % 姿勢の平均
    att_sum = zeros(3, 1, 'single');
    count = 0;
    for i = 1:length(att_estimates)
        if ~isempty(att_estimates{i})
            att_sum = att_sum + att_estimates{i}.attitude;
            count = count + 1;
        end
    end
    if count > 0
        att_avg = att_sum / count;
        % 簡略化された座標変換
        yaw = att_avg(3);
        R_yaw = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
        vel_avg = R_yaw * vel_avg;
    end
end

z = single([gps_obs; baro_obs; vel_avg]);

% UKF予測ステップ
[x_pred, P_pred] = ukf_pos.predict(x, P, F, Q, dt);

% UKF更新ステップ
[x_est, P_est, y, S, K] = ukf_pos.update(x_pred, P_pred, z, H, R);

% 結果を格納
pos_result = struct();
pos_result.x = x_est;
pos_result.P = P_est;
pos_result.x_pred = x_pred;
pos_result.P_pred = P_pred;
pos_result.position = x_est(1:3);
pos_result.velocity = x_est(4:6);
pos_result.innovation = y;
pos_result.innovation_cov = S;
pos_result.kalman_gain = K;
pos_result.timestamp = sensor_data.t(idx);

end
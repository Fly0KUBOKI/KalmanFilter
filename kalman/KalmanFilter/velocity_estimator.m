function [vel_result] = velocity_estimator(sensor_data, params, ukf_vel, idx, prev_state)
% VELOCITY_ESTIMATOR - 速度推定ブロック
% 3軸加速度から3次元速度を推定 (400Hz)
%
% Inputs:
%   sensor_data: センサデータ構造体
%   params: パラメータ構造体
%   ukf_vel: UKF_Calculatorインスタンス
%   idx: 現在のインデックス
%   prev_state: 前回の状態 [vx; vy; vz; ax; ay; az]
%
% Outputs:
%   vel_result: 推定結果構造体

% 状態定義: [vx, vy, vz, ax, ay, az] (6次元)
% 観測: 加速度 [ax, ay, az] (3次元)

if nargin < 5 || isempty(prev_state)
    % 初期状態
    x = zeros(6, 1, 'single');
    P = eye(6, 'single') * 0.1;
else
    x = prev_state.x;
    P = prev_state.P;
end

% 時間ステップ
dt = sensor_data.dt(idx);

% システム行列 F
F = single([
    1, 0, 0, dt, 0, 0;
    0, 1, 0, 0, dt, 0;
    0, 0, 1, 0, 0, dt;
    0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1
]);

% プロセスノイズ Q
q_accel = params.kf.process_noise_accel;
Q = single(zeros(6, 6));
Q(4:6, 4:6) = eye(3) * q_accel^2;

% 観測行列 H (加速度を直接観測)
H = single([
    0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1
]);

% 観測ノイズ R
R = single(diag([params.noise.accel3].^2));

% 観測値
z = single(sensor_data.accel3(idx, :)');

% UKF予測ステップ
[x_pred, P_pred] = ukf_vel.predict(x, P, F, Q, dt);

% UKF更新ステップ
[x_est, P_est, y, S, K] = ukf_vel.update(x_pred, P_pred, z, H, R);

% 結果を格納
vel_result = struct();
vel_result.x = x_est;
vel_result.P = P_est;
vel_result.x_pred = x_pred;
vel_result.P_pred = P_pred;
vel_result.velocity = x_est(1:3);
vel_result.acceleration = x_est(4:6);
vel_result.innovation = y;
vel_result.innovation_cov = S;
vel_result.kalman_gain = K;
vel_result.timestamp = sensor_data.t(idx);

end
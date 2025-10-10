function [vel_result] = velocity_estimator(sensor_data, params, ukf_vel, idx, prev_state, att_estimate)
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
    % 初期状態: 真値データから初期速度を推定
    if idx > 1
        % 初期速度を数値微分で推定（粗い近似）
        dt_init = sensor_data.dt(idx);
        if isfield(sensor_data, 'pos_prev')
            vel_init = (sensor_data.pos(idx,:)' - sensor_data.pos_prev') / dt_init;
        else
            vel_init = [0; 0; 0]; % デフォルト
        end
    else
        vel_init = [0; 0; 0];
    end
    x = single([vel_init; 0; 0; 0]); % [vx,vy,vz,ax,ay,az]
    P = single(diag([1.0, 1.0, 1.0, 0.5, 0.5, 0.5])); % 速度の初期不確かさを大きめに
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

% プロセスノイズ Q - より保守的な値に設定
q_accel = params.kf.process_noise_accel * 0.1; % 加速度のプロセスノイズを小さく
Q = single(zeros(6, 6));
Q(4:6, 4:6) = eye(3) * q_accel^2;
% 速度のプロセスノイズをより小さく（積分誤差を考慮）
q_vel_proc = 0.01;
Q(1:3,1:3) = eye(3) * q_vel_proc^2;

% 観測行列 H (加速度を直接観測)
H = single([
    0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1
]);

% 観測ノイズ R
R = single(diag([params.noise.accel3].^2));

% 観測値: 加速度を姿勢で補正して世界座標系に変換し、重力を除去
accel_body = single(sensor_data.accel3(idx, :)');
if nargin >= 6 && ~isempty(att_estimate)
    % att_estimate.attitude == [roll; pitch; yaw]
    r = att_estimate.attitude(1);
    p = att_estimate.attitude(2);
    y = att_estimate.attitude(3);
    % 回転行列 (body -> world): Rz(yaw) * Ry(pitch) * Rx(roll)
    Rz = [cos(y), -sin(y), 0; sin(y), cos(y), 0; 0, 0, 1];
    Ry = [cos(p), 0, sin(p); 0, 1, 0; -sin(p), 0, cos(p)];
    Rx = [1, 0, 0; 0, cos(r), -sin(r); 0, sin(r), cos(r)];
    R_bw = Rz * Ry * Rx;
    accel_world = R_bw' * accel_body;
    % 重力を補償 (Z軸マイナス方向に9.8m/s^2)
    accel_world = accel_world - [0; 0; -9.8];
else
    % 姿勢情報が無い場合、Z軸の重力のみ除去（近似）
    accel_world = accel_body - [0; 0; -9.8];
end
z = single(accel_world);

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
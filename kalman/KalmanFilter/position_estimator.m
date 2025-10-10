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
    % 初期状態: GPSとバロメータから初期位置を推定
    gps_init = single(sensor_data.gps(idx, :)');
    baro_init = single(sensor_data.baro(idx));
    
    % 速度推定値があれば使用
    if nargin >= 6 && ~isempty(vel_estimates) && numel(vel_estimates) > 0
        vel_init = vel_estimates{1}.velocity;
    else
        vel_init = [0; 0; 0];
    end
    
    x = single([gps_init; baro_init; vel_init]); % [x,y,z,vx,vy,vz]
    P = single(diag([10.0, 10.0, 5.0, 2.0, 2.0, 1.0])); % 位置の不確かさを大きめに
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

% 観測行列 H: [gps_x; gps_y; baro_z; vx; vy; vz]
H = single([
    1, 0, 0, 0, 0, 0;
    0, 1, 0, 0, 0, 0;
    0, 0, 1, 0, 0, 0;
    0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1
]);

% R will be constructed after vel_avg is computed below

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
        vel_avg = struct();
        vel_avg.velocity = vel_sum / count;
        % vel_avg.P may exist in vel_estimates; average if present
        Psum = zeros(3,3,'single'); pcount = 0;
        for k=1:length(vel_estimates)
            if ~isempty(vel_estimates{k}) && isfield(vel_estimates{k}, 'P')
                Psum = Psum + single(vel_estimates{k}.P(1:3,1:3));
                pcount = pcount + 1;
            end
        end
        if pcount>0
            vel_avg.P = Psum / pcount;
        else
            vel_avg.P = [];
        end
    else
        vel_avg = struct(); vel_avg.velocity = zeros(3, 1, 'single'); vel_avg.P = [];
    end
else
    vel_avg = struct(); vel_avg.velocity = zeros(3, 1, 'single'); vel_avg.P = [];
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
        % 今は vel_avg を world frame と仮定してそのまま使う
    end
end

% 観測ノイズ R をここで構築（vel_avg が利用可能になった後）
R = single(zeros(6,6));
R(1,1) = params.noise.gps.^2; R(2,2) = params.noise.gps.^2;
R(3,3) = params.noise.baro.^2;
if isfield(vel_avg,'P') && ~isempty(vel_avg.P)
    R(4:6,4:6) = diag(params.noise.vel.^2 * ones(1,3)) + vel_avg.P;
else
    R(4:6,4:6) = diag(params.noise.vel.^2 * ones(1,3));
end

% もし速度が機体座標系(body)で与えられているなら姿勢で world-frame に変換して使う
vel_for_z = vel_avg.velocity;
if isfield(params, 'data') && isfield(params.data, 'vel_in_body') && params.data.vel_in_body
    if exist('att_avg','var') && ~isempty(att_avg)
        r = att_avg(1); p = att_avg(2); y = att_avg(3);
        Rx = [1, 0, 0; 0, cos(r), -sin(r); 0, sin(r), cos(r)];
        Ry = [cos(p), 0, sin(p); 0, 1, 0; -sin(p), 0, cos(p)];
        Rz = [cos(y), -sin(y), 0; sin(y), cos(y), 0; 0, 0, 1];
        R_bw = Rz * Ry * Rx; % body -> world
        vel_for_z = R_bw * vel_for_z;
    end
end

z = single([gps_obs; baro_obs; vel_for_z]);

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
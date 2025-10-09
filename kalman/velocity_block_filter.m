function [x_est, P_est, diagnostics] = velocity_block_filter(x_est, P_est, meas, params, k, last_update_step)
% velocity_block_filter.m
% 速度ブロックのフィルタ処理（高頻度更新）
%
% Inputs:
%   x_est           - 現在の状態推定値
%   P_est           - 現在の共分散行列
%   meas            - 測定値構造体
%   params          - パラメータ構造体
%   k               - 現在のステップ
%   last_update_step - 前回の速度ブロック更新ステップ
%
% Outputs:
%   x_est       - 更新された状態推定値
%   P_est       - 更新された共分散行列
%   diagnostics - 診断情報構造体

diagnostics = struct('updated', false, 'y', [], 'S', [], 'K', []);

% 更新頻度の決定
high_dt = params.dt;
vel_dt = high_dt; % 最高頻度（毎ステップ）
vel_factor = 1;   % 毎ステップ更新

% パラメータオーバーライド
if isfield(params,'kf') && isfield(params.kf,'update_factors') && isfield(params.kf.update_factors,'vel')
    vel_factor = params.kf.update_factors.vel;
end

% 速度測定が存在し、かつ更新タイミングの場合のみ実行
include_vel = (mod(k-1, vel_factor) == 0) && isfield(meas, 'vel') && ~isempty(meas.vel);

if ~include_vel
    return; % 更新しない
end

% 実際の経過時間の計算
if last_update_step == 0
    actual_dt = vel_dt;
else
    actual_dt = (k - last_update_step) * params.dt;
end

% 予測ステップ
[F_vel, Q_vel] = build_process_model(numel(x_est), actual_dt, params);
params_vel = params;
params_vel.dt = actual_dt;
params_vel.kf.F = F_vel;
params_vel.kf.Q = Q_vel;

[x_pred, P_pred] = predict_state_covariance(x_est, P_est, params_vel);

% 速度測定ブロックの組み立て
blocks_vel = {};
if isfield(meas, 'vel') && ~isempty(meas.vel)
    % 速度測定ブロック 
    block_vel = struct();
    block_vel.name = 'vel';
    block_vel.z = meas.vel(:);
    block_vel.h = x_pred(3:4); % 予測速度
    block_vel.H = zeros(2, numel(x_pred));
    block_vel.H(1,3) = 1; % vx
    block_vel.H(2,4) = 1; % vy
    
    % 測定ノイズ
    if isfield(params,'sensors') && isfield(params.sensors,'vel_noise')
        vel_noise = params.sensors.vel_noise;
    else
        vel_noise = 0.5; % デフォルト
    end
    block_vel.R = eye(2) * vel_noise^2;
    
    blocks_vel{end+1} = block_vel;
end

% 測定ブロックが存在する場合のみ更新
if ~isempty(blocks_vel)
    % 測定値の統合
    z_vel = []; h_vel = []; H_vel = zeros(0, numel(x_pred)); R_vel = []; meas_tags_vel = {};
    start_idx = 1;
    
    for ib = 1:numel(blocks_vel)
        b = blocks_vel{ib};
        z_vel = [z_vel; b.z(:)];
        h_vel = [h_vel; b.h(:)];
        H_vel = [H_vel; b.H];
        R_vel = blkdiag(R_vel, b.R);
        idx_range = start_idx:(start_idx + numel(b.z) - 1);
        meas_tags_vel{end+1} = struct('name', b.name, 'range', idx_range);
        start_idx = start_idx + numel(b.z);
    end
    
    if ~isempty(z_vel)
        meas_vel = struct('z', z_vel, 'h', h_vel, 'H', H_vel, 'R', R_vel, 'meas_tags', {meas_tags_vel});
        
        % UKF測定更新（予測は既に実行済み）
        params_vel.kf.skip_predict = true;
        [~, ~, x_est, P_est, y, S, K, ~] = ukf_filter_step(x_pred, P_pred, meas_vel, params_vel);
        
        diagnostics.updated = true;
        diagnostics.y = y;
        diagnostics.S = S;
        diagnostics.K = K;
    else
        % 測定更新なし、予測値をそのまま使用
        x_est = x_pred;
        P_est = P_pred;
    end
else
    % 測定ブロックなし、予測値をそのまま使用
    x_est = x_pred;
    P_est = P_pred;
end

end
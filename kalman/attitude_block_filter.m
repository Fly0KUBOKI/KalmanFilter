function [x_est, P_est, diagnostics] = attitude_block_filter(x_est, P_est, meas, params, k, last_update_step)
% attitude_block_filter.m
% 姿勢ブロックのフィルタ処理（中頻度更新）
%
% Inputs:
%   x_est           - 現在の状態推定値
%   P_est           - 現在の共分散行列
%   meas            - 測定値構造体
%   params          - パラメータ構造体
%   k               - 現在のステップ
%   last_update_step - 前回の姿勢ブロック更新ステップ
%
% Outputs:
%   x_est       - 更新された状態推定値
%   P_est       - 更新された共分散行列
%   diagnostics - 診断情報構造体

diagnostics = struct('updated', false, 'y', [], 'S', [], 'K', []);

% 更新頻度の決定
high_dt = params.dt;
att_dt = 4 * high_dt; % 中頻度（4×dt）
att_factor = max(1, round(att_dt / params.dt));

% パラメータオーバーライド
if isfield(params,'kf') && isfield(params.kf,'update_factors') && isfield(params.kf.update_factors,'att')
    att_factor = params.kf.update_factors.att;
end

% 更新タイミングの判定（測定値が存在する場合のみ）
has_att_measurement = (isfield(meas, 'gyro3') && ~isempty(meas.gyro3)) || ...
                      (isfield(meas, 'accel3') && ~isempty(meas.accel3));
include_att = (mod(k-1, att_factor) == 0) && has_att_measurement;

if ~include_att
    return; % 更新しない
end

% 実際の経過時間の計算
if last_update_step == 0
    actual_dt = att_dt;
else
    actual_dt = (k - last_update_step) * params.dt;
end

% 予測ステップ
[F_att, Q_att] = build_process_model(numel(x_est), actual_dt, params);
params_att = params;
params_att.dt = actual_dt;
params_att.kf.F = F_att;
params_att.kf.Q = Q_att;

[x_pred, P_pred] = predict_state_covariance(x_est, P_est, params_att);

% 姿勢測定ブロックの組み立て
blocks_att = {};

% ジャイロスコープ測定
if isfield(meas, 'gyro3') && ~isempty(meas.gyro3)
    block_gyro = struct();
    block_gyro.name = 'gyro3';
    block_gyro.z = meas.gyro3(:);
    
    % 角速度の予測値（状態ベクトルのomega成分）
    if numel(x_pred) >= 8
        block_gyro.h = [0; 0; x_pred(8)]; % omega_z のみ
    else
        block_gyro.h = [0; 0; 0];
    end
    
    block_gyro.H = zeros(3, numel(x_pred));
    if numel(x_pred) >= 8
        block_gyro.H(3,8) = 1; % domega_z/domega
    end
    
    % 測定ノイズ
    if isfield(params,'sensors') && isfield(params.sensors,'gyro_noise')
        gyro_noise = params.sensors.gyro_noise;
    else
        gyro_noise = 0.1; % デフォルト
    end
    block_gyro.R = eye(3) * gyro_noise^2;
    
    blocks_att{end+1} = block_gyro;
end

% 加速度計測定
if isfield(meas, 'accel3') && ~isempty(meas.accel3)
    block_accel = struct();
    block_accel.name = 'accel3';
    block_accel.z = meas.accel3(:);
    
    % 加速度の予測値（状態ベクトルの加速度成分＋重力）
    if numel(x_pred) >= 7
        block_accel.h = [x_pred(6); x_pred(7); -9.81]; % ax, ay, gravity
    else
        block_accel.h = [0; 0; -9.81];
    end
    
    block_accel.H = zeros(3, numel(x_pred));
    if numel(x_pred) >= 6
        block_accel.H(1,6) = 1; % dax/dax
    end
    if numel(x_pred) >= 7
        block_accel.H(2,7) = 1; % day/day
    end
    
    % 測定ノイズ
    if isfield(params,'sensors') && isfield(params.sensors,'accel_noise')
        accel_noise = params.sensors.accel_noise;
    else
        accel_noise = 0.2; % デフォルト
    end
    block_accel.R = eye(3) * accel_noise^2;
    
    blocks_att{end+1} = block_accel;
end

% 測定ブロックが存在する場合のみ更新
if ~isempty(blocks_att)
    % 測定値の統合
    z_att = []; h_att = []; H_att = zeros(0, numel(x_pred)); R_att = []; meas_tags_att = {};
    start_idx = 1;
    
    for ib = 1:numel(blocks_att)
        b = blocks_att{ib};
        z_att = [z_att; b.z(:)];
        h_att = [h_att; b.h(:)];
        H_att = [H_att; b.H];
        R_att = blkdiag(R_att, b.R);
        idx_range = start_idx:(start_idx + numel(b.z) - 1);
        meas_tags_att{end+1} = struct('name', b.name, 'range', idx_range);
        start_idx = start_idx + numel(b.z);
    end
    
    if ~isempty(z_att)
        meas_att = struct('z', z_att, 'h', h_att, 'H', H_att, 'R', R_att, 'meas_tags', {meas_tags_att});
        
        % UKF測定更新（予測は既に実行済み）
        params_att.kf.skip_predict = true;
        [~, ~, x_est, P_est, y, S, K, ~] = ukf_filter_step(x_pred, P_pred, meas_att, params_att);
        
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
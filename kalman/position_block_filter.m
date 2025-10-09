function [x_est, P_est, diagnostics] = position_block_filter(x_est, P_est, meas, params, k, last_update_step, att_history)
% position_block_filter.m
% 位置ブロックのフィルタ処理（低頻度更新）
%
% Inputs:
%   x_est           - 現在の状態推定値
%   P_est           - 現在の共分散行列
%   meas            - 測定値構造体
%   params          - パラメータ構造体
%   k               - 現在のステップ
%   last_update_step - 前回の位置ブロック更新ステップ
%   att_history     - 姿勢履歴構造体配列
%
% Outputs:
%   x_est       - 更新された状態推定値
%   P_est       - 更新された共分散行列
%   diagnostics - 診断情報構造体

diagnostics = struct('updated', false, 'y', [], 'S', [], 'K', []);

% 更新頻度の決定
high_dt = params.dt;
pos_dt = 40 * high_dt; % 低頻度（40×dt）
pos_factor = max(1, round(pos_dt / params.dt));

% パラメータオーバーライド
if isfield(params,'kf') && isfield(params.kf,'update_factors') && isfield(params.kf.update_factors,'pos')
    pos_factor = params.kf.update_factors.pos;
end

% 更新タイミングの判定（測定値が存在する場合のみ）
has_pos_measurement = (isfield(meas, 'gps') && ~isempty(meas.gps)) || ...
                      (isfield(meas, 'vel') && ~isempty(meas.vel)) || ...
                      (isfield(meas, 'heading') && ~isempty(meas.heading)) || ...
                      (isfield(meas, 'mag3') && ~isempty(meas.mag3));
include_pos = (mod(k-1, pos_factor) == 0) && has_pos_measurement;

if ~include_pos
    return; % 更新しない
end

% 実際の経過時間の計算
if last_update_step == 0
    actual_dt = pos_dt;
else
    actual_dt = (k - last_update_step) * params.dt;
end

% 姿勢履歴の統合と予測
if ~isempty(att_history)
    % 最新の姿勢推定値を現在時刻まで予測（最も近い情報を使用）
    latest_att = att_history(end);  % 最新の姿勢推定
    dt_from_latest = (k - latest_att.step) * params.dt;
    
    % 最新姿勢推定値を現在時刻まで予測
    [F_pred, Q_pred] = build_process_model(numel(latest_att.x), dt_from_latest, params);
    params_pred = params;
    params_pred.dt = dt_from_latest;
    params_pred.kf.F = F_pred;
    params_pred.kf.Q = Q_pred;
    
    [x_pred, P_pred] = predict_state_covariance(latest_att.x, latest_att.P, params_pred);
    
    % 複数の姿勢履歴がある場合、信頼性向上のため共分散を調整
    if numel(att_history) > 1
        % 履歴の数に応じて不確実性を若干減少（情報の蓄積効果）
        confidence_factor = 1.0 - 0.1 * min(numel(att_history)-1, 3) / 3;
        P_pred = P_pred * confidence_factor;
    end
else
    % 姿勢履歴がない場合は現在状態から通常予測
    [F_pos, Q_pos] = build_process_model(numel(x_est), actual_dt, params);
    params_pos_pred = params;
    params_pos_pred.dt = actual_dt;
    params_pos_pred.kf.F = F_pos;
    params_pos_pred.kf.Q = Q_pos;
    
    [x_pred, P_pred] = predict_state_covariance(x_est, P_est, params_pos_pred);
end

% 位置測定ブロックの組み立て
blocks_pos = {};

% GPS測定
if isfield(meas, 'gps') && ~isempty(meas.gps)
    block_gps = struct();
    block_gps.name = 'gps';
    block_gps.z = meas.gps(:);
    block_gps.h = x_pred(1:2); % 位置の予測値
    block_gps.H = zeros(2, numel(x_pred));
    block_gps.H(1,1) = 1; % dx/dx
    block_gps.H(2,2) = 1; % dy/dy
    
    % 測定ノイズ
    if isfield(params,'sensors') && isfield(params.sensors,'gps_noise')
        gps_noise = params.sensors.gps_noise;
    else
        gps_noise = 2.0; % デフォルト
    end
    block_gps.R = eye(2) * gps_noise^2;
    
    blocks_pos{end+1} = block_gps;
end

% 速度測定（位置ブロックに含めて位置-速度相関を保持）
if isfield(meas, 'vel') && ~isempty(meas.vel)
    block_vel = struct();
    block_vel.name = 'vel';
    block_vel.z = meas.vel(:);
    block_vel.h = x_pred(3:4); % 速度の予測値
    block_vel.H = zeros(2, numel(x_pred));
    block_vel.H(1,3) = 1; % dvx/dvx
    block_vel.H(2,4) = 1; % dvy/dvy
    
    % 測定ノイズ
    if isfield(params,'sensors') && isfield(params.sensors,'vel_noise')
        vel_noise = params.sensors.vel_noise;
    else
        vel_noise = 0.5; % デフォルト
    end
    block_vel.R = eye(2) * vel_noise^2;
    
    blocks_pos{end+1} = block_vel;
end

% ヘディング測定
if isfield(meas, 'heading') && ~isempty(meas.heading)
    block_heading = struct();
    block_heading.name = 'heading';
    block_heading.z = meas.heading(:);
    
    % ヘディング角thetaから単位ベクトルへの変換
    if numel(x_pred) >= 5
        th = x_pred(5);
        block_heading.h = [cos(th); sin(th)];
        block_heading.H = zeros(2, numel(x_pred));
        block_heading.H(:,5) = [-sin(th); cos(th)]; % d/dtheta
    else
        block_heading.h = [1; 0];
        block_heading.H = zeros(2, numel(x_pred));
    end
    
    % 測定ノイズ
    if isfield(params,'sensors') && isfield(params.sensors,'heading_noise')
        heading_noise = params.sensors.heading_noise;
    else
        heading_noise = 0.1; % デフォルト
    end
    block_heading.R = eye(2) * heading_noise^2;
    
    blocks_pos{end+1} = block_heading;
end

% 磁力計測定
if isfield(meas, 'mag3') && ~isempty(meas.mag3)
    block_mag = struct();
    block_mag.name = 'mag3';
    block_mag.z = meas.mag3(:);
    
    % 磁場ベクトルの回転変換
    if numel(x_pred) >= 5
        th = x_pred(5);
        mag_field = [1; 0; 0]; % デフォルト磁場
        if isfield(params,'sensors') && isfield(params.sensors,'mag_field')
            mag_field = params.sensors.mag_field(:);
        end
        
        % Z軸回転行列
        Rz = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
        block_mag.h = Rz * mag_field;
        
        % ヤコビアン
        dRz = [-sin(th) -cos(th) 0; cos(th) -sin(th) 0; 0 0 0];
        block_mag.H = zeros(3, numel(x_pred));
        block_mag.H(:,5) = dRz * mag_field;
    else
        block_mag.h = [1; 0; 0];
        block_mag.H = zeros(3, numel(x_pred));
    end
    
    % 測定ノイズ
    if isfield(params,'sensors') && isfield(params.sensors,'mag_noise')
        mag_noise = params.sensors.mag_noise;
    else
        mag_noise = 0.1; % デフォルト
    end
    block_mag.R = eye(3) * mag_noise^2;
    
    blocks_pos{end+1} = block_mag;
end

% 測定ブロックが存在する場合のみ更新
if ~isempty(blocks_pos)
    % 測定値の統合
    z_pos = []; h_pos = []; H_pos = zeros(0, numel(x_pred)); R_pos = []; meas_tags_pos = {};
    start_idx = 1;
    
    for ib = 1:numel(blocks_pos)
        b = blocks_pos{ib};
        z_pos = [z_pos; b.z(:)];
        h_pos = [h_pos; b.h(:)];
        H_pos = [H_pos; b.H];
        R_pos = blkdiag(R_pos, b.R);
        idx_range = start_idx:(start_idx + numel(b.z) - 1);
        meas_tags_pos{end+1} = struct('name', b.name, 'range', idx_range);
        start_idx = start_idx + numel(b.z);
    end
    
    if ~isempty(z_pos)
        meas_pos = struct('z', z_pos, 'h', h_pos, 'H', H_pos, 'R', R_pos, 'meas_tags', {meas_tags_pos});
        
        % UKF測定更新（予測は既に実行済み）
        params_pos = params;
        params_pos.kf.skip_predict = true;
        [~, ~, x_est, P_est, y, S, K, ~] = ukf_filter_step(x_pred, P_pred, meas_pos, params_pos);
        
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
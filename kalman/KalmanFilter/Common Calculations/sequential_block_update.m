function [x_upd, P_upd, y_all, S_all, K_all, params] = sequential_block_update(x_prev, P_prev, blocks, params)
% Perform one predict (caller should have already predicted if needed) and then
% apply sequential Kalman updates for each block in order.
% blocks: cell array of structs with fields z,h,H,R,name

% initialize
x_curr = x_prev;
P_curr = P_prev;
y_all = {};
S_all = {};
K_all = {};

for i=1:numel(blocks)
    b = blocks{i};
    if isempty(b) || isempty(b.z)
        continue;
    end
    % compute innovation and S for this block
    [y, S, R_used] = compute_innovation_and_S(b.z, b.h, b.H, P_curr, b.R, params);
    % compute gain
    K = compute_kalman_gain(P_curr, b.H, S);
    % update
    [x_next, P_next] = update_state_covariance(x_curr, P_curr, K, b.H, y, R_used);

    % store
    y_all{end+1} = y;
    S_all{end+1} = S;
    K_all{end+1} = K;

    % advance
    x_curr = x_next;
    P_curr = P_next;

    % optional adaptive R update per block
    if isfield(params,'kf') && isfield(params.kf,'adaptive_R_enabled') && params.kf.adaptive_R_enabled
        % build a meas_tags-like struct for this block
        meas_tag = struct('name', b.name, 'range', 1:numel(b.z));
        params = adaptive_R_update(params, y, b.H, P_curr, R_used, {meas_tag});
    end
end

x_upd = x_curr;
P_upd = P_curr;

end

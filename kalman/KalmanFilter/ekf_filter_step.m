function [x_pred, P_pred, x_upd, P_upd, y, S, K, params] = ekf_filter_step(x_prev, P_prev, meas, params)
% EKF filter step that uses shared helper modules for common operations.
% Returns the same outputs as the original implementation.

% ensure Common Calculations on path
root = fileparts(mfilename('fullpath'));
commonDir = fullfile(root, 'Common Calculations');
if exist(commonDir, 'dir') && ~contains(path, commonDir)
    addpath(commonDir);
end

% 1) prediction
[x_pred, P_pred] = predict_state_covariance(x_prev, P_prev, params);

% 2) assemble measurements: try to use preassembled fields, otherwise build from blocks
if isfield(meas, 'z') && isfield(meas, 'H') && isfield(meas, 'R')
    % Read preassembled fields using getfield; if any read fails, treat as no measurements
    meas_tags = {};
    try
        z = getfield(meas, 'z');
        h = getfield(meas, 'h');
        H = getfield(meas, 'H');
        R = getfield(meas, 'R');
        if isfield(meas, 'meas_tags')
            meas_tags = getfield(meas, 'meas_tags');
        end
    catch
        % if fields are not accessible or cause issues, fallback to empty measurement
        z = [];
        h = [];
        H = zeros(0, numel(x_pred));
        R = [];
        meas_tags = {};
    end
else
    blocks = assemble_measurement_blocks(meas, x_pred, params);
    z = []; h = []; H = zeros(0, numel(x_pred)); R = []; meas_tags = {};
    start = 1;
    for i=1:numel(blocks)
        b = blocks{i};
        z = [z; b.z];
        h = [h; b.h];
        H = [H; b.H];
        R = blkdiag(R, b.R);
        idx = start:(start+numel(b.z)-1);
        meas_tags{end+1} = struct('name', b.name, 'range', idx);
        start = start + numel(b.z);
    end
end

if isempty(z)
    x_upd = x_pred; P_upd = P_pred; y = []; S = []; K = [];
    return;
end

% 3) innovation and S (handles NaN/Inf in z/h and regularizes S)
[y, S, R] = compute_innovation_and_S(z, h, H, P_pred, R, params);

% 4) Kalman gain
K = compute_kalman_gain(P_pred, H, S);

% 5) update state and covariance (use Joseph form with R)
[x_upd, P_upd] = update_state_covariance(x_pred, P_pred, K, H, y, R);

% 6) adaptive R estimation (EMA) (optional: controlled via params.kf.adaptive_R_enabled)
if isfield(params,'kf') && isfield(params.kf,'adaptive_R_enabled') && params.kf.adaptive_R_enabled
    params = adaptive_R_update(params, y, H, P_pred, R, meas_tags);
end

end

function [x_pred, P_pred, x_upd, P_upd, y, S, K, params] = kf_filter_step(x_prev, P_prev, meas, params)
% Simple 1-step linear Kalman filter using shared helper modules

% ensure Common Calculations on path
root = fileparts(mfilename('fullpath'));
commonDir = fullfile(root, 'Common Calculations');
if exist(commonDir, 'dir') && ~contains(path, commonDir)
    addpath(commonDir);
end

% 1) prediction
[x_pred, P_pred] = predict_state_covariance(x_prev, P_prev, params);

% 2) assemble measurements (uses same mapping as ekf/ukf helpers)
[z, h, H, R, meas_tags] = assemble_measurements(meas, x_pred, params);

if isempty(z)
    x_upd = x_pred; P_upd = P_pred; y = []; S = []; K = [];
    return;
end

% 3) innovation and S (handles NaN/Inf and regularizes S)
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

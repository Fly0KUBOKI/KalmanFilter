function [x_pred, P_pred, x_upd, P_upd, y, S, K, params] = ukf_filter_step(x_prev, P_prev, meas, params)
% UKF_FILTER_STEP - Generic Unscented Kalman Filter (one step)
%
% Inputs:
%   x_prev: previous state (n×1)
%   P_prev: previous covariance (n×n)
%   meas: measurement struct
%   params: parameter struct
%           params.dt - time step
%           params.kf.F (optional) - state transition matrix, default eye(n)
%           params.kf.Q (optional) - process noise, default zeros(n)
%           params.kf.skip_predict (optional) - if true, skip prediction
%
% Outputs:
%   x_pred, P_pred: predicted state and covariance
%   x_upd, P_upd: updated state and covariance
%   y, S, K: innovation, innovation covariance, Kalman gain
%   params: updated parameters

% ensure Common Calculations on path
root = fileparts(mfilename('fullpath'));
commonDir = fullfile(root, 'Common Calculations');
if exist(commonDir, 'dir') && ~contains(path, commonDir)
    addpath(commonDir);
end

n = numel(x_prev);

% Check if caller wants to skip prediction (already done externally)
skip_predict = false;
if isfield(params, 'kf') && isfield(params.kf, 'skip_predict') && params.kf.skip_predict
    skip_predict = true;
end

if skip_predict
    % Use input as prediction (caller already predicted)
    x_pred = x_prev;
    P_pred = P_prev;
else
    % Perform UKF prediction step
    alpha_ukf = 1e-3; beta = 2; kappa = 0;
    lambda = alpha_ukf^2*(n+kappa)-n;
    wm = [lambda/(n+lambda); repmat(1/(2*(n+lambda)), 2*n,1)];
    wc = wm;
    wc(1) = wc(1) + (1-alpha_ukf^2+beta);

    % Generate sigma points
    sqrtP = chol((n+lambda)*P_prev, 'lower');
    sig = [x_prev, x_prev + sqrtP, x_prev - sqrtP];

    % Get F and Q from params or use defaults
    if isfield(params, 'kf') && isfield(params.kf, 'F')
        F = params.kf.F;
    else
        F = eye(n);
    end
    
    if isfield(params, 'kf') && isfield(params.kf, 'Q')
        Q = params.kf.Q;
    else
        Q = zeros(n);
    end

    % Propagate sigma points
    for i = 1:size(sig, 2)
        sig(:, i) = F * sig(:, i);
    end
    
    % Predicted mean and covariance
    x_pred = sig * wm;
    P_pred = Q;
    for i = 1:size(sig, 2)
        d = sig(:, i) - x_pred;
        P_pred = P_pred + wc(i) * (d * d');
    end
end

% Measurement update (always performed)
[z, h, H, R, meas_tags] = assemble_measurements(meas, x_pred, params);

if isempty(z)
    x_upd = x_pred; 
    P_upd = P_pred; 
    y = []; 
    S = []; 
    K = []; 
    return;
end

% Innovation and innovation covariance
[y, S, R] = compute_innovation_and_S(z, h, H, P_pred, R, params);

% Kalman gain
K = compute_kalman_gain(P_pred, H, S);

% State and covariance update
[x_upd, P_upd] = update_state_covariance(x_pred, P_pred, K, H, y, R);

% Optional adaptive R update
if isfield(params, 'kf') && isfield(params.kf, 'adaptive_R_enabled') && params.kf.adaptive_R_enabled
    params = adaptive_R_update(params, y, H, P_pred, R, meas_tags);
end
end
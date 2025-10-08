function [x_pred, P_pred, x_upd, P_upd, y, S, K, params] = ukf_filter_step(x_prev, P_prev, meas, params)
% Minimal UKF 1-step implementation (for demonstration).
% This is a simplified UKF: it uses standard unscented transform for process
% propagation and then a measurement update using linearized H similar to EKF
% for sensors already implemented in ekf_filter_step.

% ensure Common Calculations on path
root = fileparts(mfilename('fullpath'));
commonDir = fullfile(root, 'Common Calculations');
if exist(commonDir, 'dir') && ~contains(path, commonDir)
    addpath(commonDir);
end

n = numel(x_prev);
alpha = 1e-3; beta = 2; kappa = 0;
lambda = alpha^2*(n+kappa)-n;
wm = [lambda/(n+lambda); repmat(1/(2*(n+lambda)), 2*n,1)];
wc = wm;
wc(1) = wc(1) + (1-alpha^2+beta);

% sigma points
sqrtP = chol((n+lambda)*P_prev, 'lower');
sig0 = x_prev;
sig = [sig0, x_prev + sqrtP, x_prev - sqrtP];

% process model: use same F and Q as ekf
dt = params.dt;
F = eye(n);
if n>=10
    F(1,3) = dt; F(1,6) = 0.5*dt^2;
    F(2,4) = dt; F(2,7) = 0.5*dt^2;
        F(3,6) = dt; F(4,7) = dt; F(5,8) = dt;
end
q_a = params.kf.process_noise_accel;
    Q = zeros(n); if n>=8, Q(6,6) = (q_a)^2; Q(7,7) = (q_a)^2; Q(8,8) = (q_a*0.1)^2; end

% propagate sigma points
for i=1:size(sig,2)
    sig(:,i) = F*sig(:,i); % no process nonlinearity here
end
x_pred = sig*wm;
P_pred = Q;
for i=1:size(sig,2)
    d = sig(:,i) - x_pred;
    P_pred = P_pred + wc(i)*(d*d');
end
[z, h, H, R, meas_tags] = assemble_measurements(meas, x_pred, params);
[z, h, H, R, meas_tags] = assemble_measurements(meas, x_pred, params);

if isempty(z)
    x_upd = x_pred; P_upd = P_pred; y=[]; S=[]; K=[]; return;
end

% innovation and S (handles NaN/Inf and regularizes S)
[y, S, R] = compute_innovation_and_S(z, h, H, P_pred, R, params);

% Kalman gain and update
K = compute_kalman_gain(P_pred, H, S);
[x_upd, P_upd] = update_state_covariance(x_pred, P_pred, K, H, y, R);

% optional adaptive R update
if isfield(params,'kf') && isfield(params.kf,'adaptive_R_enabled') && params.kf.adaptive_R_enabled
    params = adaptive_R_update(params, y, H, P_pred, R, meas_tags);
end
end
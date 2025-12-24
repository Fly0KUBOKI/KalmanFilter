function [x_pred, v_pred, x_upd, v_upd] = alpha_beta_step(x, v, z, dt, alpha, beta)
% ALPHA_BETA_STEP  alpha-beta tracker for 1D position (MEX required)
% x,v: previous estimates, z: measurement (scalar, NaN to skip)

if nargin < 5 || isempty(alpha), alpha = 0.85; end
if nargin < 6 || isempty(beta), beta = 0.1; end

% Require MEX implementation (no MATLAB fallback)
if exist('mex_filter_utils', 'file') ~= 3
	error('alpha_beta_step:MissingMEX', 'Required MEX ''mex_filter_utils'' not found. Run build_mex().');
end

[x_pred, v_pred, x_upd, v_upd] = mex_filter_utils('alpha_beta_step', x, v, z, dt, alpha, beta);
end

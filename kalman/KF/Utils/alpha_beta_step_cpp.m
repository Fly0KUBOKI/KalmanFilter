function [x_pred, v_pred, x_upd, v_upd] = alpha_beta_step_cpp(x, v, z, dt, alpha, beta)
% ALPHA_BETA_STEP_CPP  C++実装版 alpha-beta tracker for 1D position
% x,v: previous estimates, z: measurement (scalar, NaN to skip)
% 
% This is a wrapper for the C++ MEX implementation.
% Falls back to MATLAB implementation if MEX is not available.

if nargin < 5 || isempty(alpha), alpha = 0.85; end
if nargin < 6 || isempty(beta), beta = 0.1; end

% Check if MEX function is available
persistent use_mex;
if isempty(use_mex)
    use_mex = exist('mex_filter_utils', 'file') == 3;
end

if use_mex
    [x_pred, v_pred, x_upd, v_upd] = mex_filter_utils('alpha_beta_step', x, v, z, dt, alpha, beta);
else
    % Fallback to MATLAB implementation
    x_pred = x + v*dt;
    v_pred = v;
    if isnan(z)
        x_upd = x_pred; v_upd = v_pred;
    else
        r = z - x_pred;
        x_upd = x_pred + alpha * r;
        v_upd = v_pred + (beta/dt) * r;
    end
end
end

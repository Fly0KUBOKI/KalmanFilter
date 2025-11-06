function [x_pred, v_pred, x_upd, v_upd] = alpha_beta_step(x, v, z, dt, alpha, beta)
% ALPHA_BETA_STEP  simple alpha-beta tracker for 1D position
% x,v: previous estimates, z: measurement (scalar, NaN to skip)
if nargin < 5 || isempty(alpha), alpha = 0.85; end
if nargin < 6 || isempty(beta), beta = 0.1; end
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

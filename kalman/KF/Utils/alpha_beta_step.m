function [x_pred, v_pred, x_upd, v_upd] = alpha_beta_step(x, v, z, dt, alpha, beta)
% ALPHA_BETA_STEP  C++実装版 alpha-beta tracker for 1D position
% x,v: previous estimates, z: measurement (scalar, NaN to skip)
% 
% This function now uses the C++ implementation via alpha_beta_step_cpp.
% The original MATLAB implementation has been replaced.

[x_pred, v_pred, x_upd, v_upd] = alpha_beta_step_cpp(x, v, z, dt, alpha, beta);
end

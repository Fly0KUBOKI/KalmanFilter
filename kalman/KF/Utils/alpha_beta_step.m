function [x_pred, v_pred, x_upd, v_upd] = alpha_beta_step(x, v, z, dt, alpha, beta)
% ALPHA_BETA_STEP  alpha-beta tracker for 1D position with MEX delegation
% Usage: [x_pred,v_pred,x_upd,v_upd] = alpha_beta_step(x,v,z,dt,alpha,beta)
if nargin < 5 || isempty(alpha), alpha = 0.85; end
if nargin < 6 || isempty(beta), beta = 0.1; end
% Try to use compiled MEX `mex_filter_utils` if available
if exist('mex_filter_utils','file')==3 || exist('mex_filter_utils','file')==2
	try
		[x_pred, v_pred, x_upd, v_upd] = mex_filter_utils('alpha_beta_step', x, v, z, dt, alpha, beta);
		return;
	catch
		% fall through to MATLAB fallback
	end
end

% MATLAB fallback implementation
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

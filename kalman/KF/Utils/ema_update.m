function y = ema_update(x, y_prev, alpha)
% EMA_UPDATE  exponential moving average update (MEX required)
%   y = ema_update(x, y_prev, alpha)
%   x, y_prev can be scalar or vector of same size.

if nargin < 3 || isempty(alpha), alpha = 0.1; end

% Require MEX implementation (no MATLAB fallback)
if exist('mex_filter_utils', 'file') ~= 3
	error('ema_update:MissingMEX', 'Required MEX ''mex_filter_utils'' not found. Run build_mex().');
end

if isempty(y_prev)
	y = mex_filter_utils('ema_update', x, [], alpha);
else
	y = mex_filter_utils('ema_update', x, y_prev, alpha);
end
end

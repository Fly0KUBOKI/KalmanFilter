function out = ema_update(x, y_prev, alpha)
% EMA_UPDATE  exponential moving average update with optional MEX delegate
if nargin < 3 || isempty(alpha), alpha = 0.1; end
if exist('mex_filter_utils','file')==3 || exist('mex_filter_utils','file')==2
	try
		if isempty(y_prev)
			out = mex_filter_utils('ema_update', x, [], alpha);
		else
			out = mex_filter_utils('ema_update', x, y_prev, alpha);
		end
		return
	catch
		% fall back to MATLAB
	end
end
% MATLAB fallback
if isempty(y_prev)
	out = x;
else
	out = alpha .* x + (1-alpha) .* y_prev;
end
end

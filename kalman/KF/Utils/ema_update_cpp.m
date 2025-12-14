function out = ema_update_cpp(x, y_prev, alpha)
% EMA_UPDATE_CPP  C++実装版 exponential moving average update
%   out = ema_update_cpp(x, y_prev, alpha)
%   x, y_prev can be scalar or vector of same size.
%
% This is a wrapper for the C++ MEX implementation.
% Falls back to MATLAB implementation if MEX is not available.

if nargin < 3 || isempty(alpha), alpha = 0.1; end

% Check if MEX function is available
persistent use_mex;
if isempty(use_mex)
    use_mex = exist('mex_filter_utils', 'file') == 3;
end

if use_mex
    if isempty(y_prev)
        out = mex_filter_utils('ema_update', x, [], alpha);
    else
        out = mex_filter_utils('ema_update', x, y_prev, alpha);
    end
else
    % Fallback to MATLAB implementation
    if isempty(y_prev)
        out = x;
    else
        out = alpha .* x + (1-alpha) .* y_prev;
    end
end
end

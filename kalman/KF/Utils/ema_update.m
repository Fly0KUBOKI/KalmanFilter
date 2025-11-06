function y = ema_update(x, y_prev, alpha)
% EMA_UPDATE  Simple exponential moving average update
%   y = ema_update(x, y_prev, alpha)
%   x, y_prev can be scalar or vector of same size.
if nargin < 3 || isempty(alpha), alpha = 0.1; end
if isempty(y_prev)
    y = x;
else
    y = alpha .* x + (1-alpha) .* y_prev;
end
end

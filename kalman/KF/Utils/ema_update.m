function y = ema_update(x, y_prev, alpha)
% EMA_UPDATE  C++実装版 exponential moving average update
%   y = ema_update(x, y_prev, alpha)
%   x, y_prev can be scalar or vector of same size.
%
% This function now uses the C++ implementation via ema_update_cpp.
% The original MATLAB implementation has been replaced.

y = ema_update_cpp(x, y_prev, alpha);
end

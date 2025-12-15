function [a_out, is_outlier] = AccelFilter_cpp(a_meas, a_expected, ema_alpha, history_size)
% AccelFilter_cpp  MATLAB wrapper that prefers mex_sensor_filter('accel',...)
%   a_meas: 3x1
%   a_expected: 3x1 (optional)
%   ema_alpha/history_size: optional; used for MATLAB fallback

if nargin < 2 || isempty(a_expected)
    a_expected = zeros(3,1);
end

use_mex = (exist('mex_sensor_filter','file') == 3);
if use_mex
    try
        if nargout >= 2
            [a_out, is_outlier] = mex_sensor_filter('accel', a_meas, a_expected);
        else
            a_out = mex_sensor_filter('accel', a_meas, a_expected);
            is_outlier = false;
        end
        return;
    catch ME
        warning('[AccelFilter_cpp] mex_sensor_filter failed: %s — falling back to MATLAB', ME.message);
    end
end

% MATLAB fallback: simple EMA with outlier detection (compatible with AccelFilter.m)
persistent fallback_filter;
if isempty(fallback_filter) || ~isstruct(fallback_filter)
    fallback_filter.a_filtered = [0;0;0];
    if nargin >= 3 && ~isempty(ema_alpha)
        fallback_filter.ema_alpha = ema_alpha;
    else
        fallback_filter.ema_alpha = 0.3;
    end
    if nargin >=4 && ~isempty(history_size)
        fallback_filter.history_size = history_size;
    else
        fallback_filter.history_size = 20;
    end
    fallback_filter.noise_history = [];
end

% Compute residual
residual = a_meas - a_expected;
residual_norm = norm(residual);

% Estimate noise
if isempty(fallback_filter.noise_history)
    noise_estimate = residual_norm;
else
    noise_std = std(fallback_filter.noise_history);
    noise_estimate = max(noise_std, residual_norm/3.0);
end

outlier_threshold = 3.0;
is_outlier = (residual_norm > outlier_threshold * max(noise_estimate, 0.1));
if is_outlier
    a_out = fallback_filter.a_filtered;
    return;
end

% EMA
alpha = fallback_filter.ema_alpha;
a_out = alpha * a_meas + (1 - alpha) * fallback_filter.a_filtered;
fallback_filter.a_filtered = a_out;

% update history
fallback_filter.noise_history = [fallback_filter.noise_history; residual_norm];
if length(fallback_filter.noise_history) > fallback_filter.history_size
    fallback_filter.noise_history = fallback_filter.noise_history(2:end);
end
end

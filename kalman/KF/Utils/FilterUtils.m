function out = ema_update(x, y_prev, alpha)
% EMA_UPDATE  Simple exponential moving average update
%   out = ema_update(x, y_prev, alpha)
%   x, y_prev can be scalar or vector of same size.
if nargin < 3 || isempty(alpha), alpha = 0.1; end
if isempty(y_prev)
    out = x;
else
    out = alpha .* x + (1-alpha) .* y_prev;
end
end

function x_out = hampel_causal(buffer, new_x, window, n_sigma)
% HAMPEL_CAUSAL  Simple causal Hampel-style outlier replacer
%   buffer: column vector of up to (window-1) previous values
%   new_x: scalar (or vector) new sample
%   window: window length (odd recommended)
%   n_sigma: threshold in MAD-equivalent (default 3)
if nargin < 3 || isempty(window), window = 5; end
if nargin < 4 || isempty(n_sigma), n_sigma = 3; end
% Build local window including new sample
buf = [buffer(:); new_x(:)];
if numel(buf) > window
    buf = buf(end-window+1:end);
end
med = median(buf);
madv = median(abs(buf - med));
if madv == 0, madv = 1e-9; end
threshold = n_sigma * 1.4826 * madv; % approx sigma
if numel(new_x) == 1
    if abs(new_x - med) > threshold
        x_out = med;
    else
        x_out = new_x;
    end
else
    % vectorized: elementwise compare
    x_out = new_x;
    for k=1:numel(new_x)
        if abs(new_x(k)-med(k)) > threshold(k)
            x_out(k) = med(k);
        end
    end
end
end

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

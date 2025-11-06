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

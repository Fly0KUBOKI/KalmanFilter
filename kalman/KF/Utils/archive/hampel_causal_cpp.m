function x_out = hampel_causal_cpp(buffer, new_x, window, n_sigma)
% HAMPEL_CAUSAL_CPP  C++実装版 causal Hampel-style outlier replacer
%   buffer: column vector of up to (window-1) previous values
%   new_x: scalar (or vector) new sample
%   window: window length (odd recommended)
%   n_sigma: threshold in MAD-equivalent (default 3)
%
% This is a wrapper for the C++ MEX implementation.
% Falls back to MATLAB implementation if MEX is not available.

if nargin < 3 || isempty(window), window = 5; end
if nargin < 4 || isempty(n_sigma), n_sigma = 3; end

% Check if MEX function is available
persistent use_mex;
if isempty(use_mex)
    use_mex = exist('mex_filter_utils', 'file') == 3;
end

if use_mex
    x_out = mex_filter_utils('hampel_causal', buffer, new_x, window, n_sigma);
else
    % Fallback to MATLAB implementation
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
            if abs(new_x(k)-med) > threshold
                x_out(k) = med;
            end
        end
    end
end
end

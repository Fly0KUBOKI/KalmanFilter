function x_out = hampel_causal(buffer, new_x, window, n_sigma)
% HAMPEL_CAUSAL  C++実装版 causal Hampel-style outlier replacer
%   buffer: column vector of up to (window-1) previous values
%   new_x: scalar (or vector) new sample
%   window: window length (odd recommended)
%   n_sigma: threshold in MAD-equivalent (default 3)
%
% This function now uses the C++ implementation via hampel_causal_cpp.
% The original MATLAB implementation has been replaced.

x_out = hampel_causal_cpp(buffer, new_x, window, n_sigma);
end

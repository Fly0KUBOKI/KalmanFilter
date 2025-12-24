function x_out = hampel_causal(buffer, new_x, window, n_sigma)
% HAMPEL_CAUSAL  causal Hampel-style outlier replacer (MEX required)
%   buffer: column vector of up to (window-1) previous values
%   new_x: scalar (or vector) new sample
%   window: window length (odd recommended)
%   n_sigma: threshold in MAD-equivalent (default 3)

if nargin < 3 || isempty(window), window = 5; end
if nargin < 4 || isempty(n_sigma), n_sigma = 3; end

% Require MEX implementation (no MATLAB fallback)
if exist('mex_filter_utils', 'file') ~= 3
	error('hampel_causal:MissingMEX', 'Required MEX ''mex_filter_utils'' not found. Run build_mex().');
end

x_out = mex_filter_utils('hampel_causal', buffer, new_x, window, n_sigma);
end

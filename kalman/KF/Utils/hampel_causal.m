function x_out = hampel_causal(buffer, new_x, window, n_sigma)
% HAMPEL_CAUSAL  causal Hampel-style outlier replacer with MEX delegate
if nargin < 3 || isempty(window), window = 5; end
if nargin < 4 || isempty(n_sigma), n_sigma = 3; end
if exist('mex_filter_utils','file')==3 || exist('mex_filter_utils','file')==2
	try
		x_out = mex_filter_utils('hampel_causal', buffer, new_x, window, n_sigma);
		return
	catch
		% fallback to MATLAB
	end
end
% MATLAB fallback implementation
buf = [buffer(:); new_x(:)];
if numel(buf) > window
	buf = buf(end-window+1:end);
end
med = median(buf);
madv = median(abs(buf - med));
if madv == 0, madv = 1e-9; end
threshold = n_sigma * 1.4826 * madv;
if numel(new_x) == 1
	if abs(new_x - med) > threshold
		x_out = med;
	else
		x_out = new_x;
	end
else
	x_out = new_x;
	for k=1:numel(new_x)
		if abs(new_x(k)-med(k)) > threshold(k)
			x_out(k) = med(k);
		end
	end
end
end

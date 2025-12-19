function [sig, wm, wc] = ukf_sigma_points(x, P, alpha, beta, kappa)
    % UKF_SIGMA_POINTS - MEX専用実装
    % C++ MEX実装(mex_ukf_sigma_points)が利用可能なことを前提とします。

    if nargin < 3, alpha = 1e-3; end
    if nargin < 4, beta = 2; end
    if nargin < 5, kappa = 0; end

    if exist('mex_ukf_sigma_points', 'file') ~= 3
        error('ukf_sigma_points:noMEX', 'MEX implementation mex_ukf_sigma_points not found. Build the MEX to use UKF.');
    end

    try
        [sig, wm, wc] = mex_ukf_sigma_points(x, P, alpha, beta, kappa);
    catch ME
        error('ukf_sigma_points:mexFailed', 'Call to mex_ukf_sigma_points failed: %s', ME.message);
    end
end

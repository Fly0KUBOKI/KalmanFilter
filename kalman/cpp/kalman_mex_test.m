% kalman_mex_test.m
% Build the kalman mex and run a simple test using simulated data.
% Usage: open MATLAB, cd to this folder (or anywhere) and run: kalman_mex_test

fprintf('kalman_mex_test: building MEX (if needed) and running a simple filter test...\n');
try
    kalman_mex_build();
catch ME
    fprintf('Build failed: %s\n', ME.message);
    error('Cannot continue without successful MEX build.');
end

% Create C++ Kalman handle
h = kalman_mex('new');
% 2-state (pos, vel), 1-observation (pos)
kalman_mex('init', h, 2, 1);

dt = 1.0;
F = [1 dt; 0 1];
H = [1 0];
kalman_mex('setSystem', h, F);
kalman_mex('setObservationMatrix', h, H);

% initial estimate
est = [0; 0];
kalman_mex('setPrediction', h, est);

% simulate truth and noisy measurements
N = 200;
true = zeros(2, N);
true(:,1) = [0; 0.5];
for k = 2:N
    true(:,k) = F * true(:,k-1);
end

sigma_meas = 0.5; % measurement noise std
rng(0); % for reproducibility
meas = true(1,:) + sigma_meas * randn(1, N);

est_hist = zeros(2, N);
est_curr = est;

for k = 1:N
    pred = F * est_curr; % simple prediction step (user provides prediction)
    kalman_mex('setPrediction', h, pred);
    kalman_mex('setObservation', h, meas(k));
    kalman_mex('update', h);
    out = kalman_mex('get', h);
    % ensure we use exactly 2 elements for the state (trim or pad as needed)
    outv = out(:);
    if numel(outv) < 2
        outv(end+1:2) = 0; % pad
    elseif numel(outv) > 2
        outv = outv(1:2);   % trim
    end
    est_curr = outv;
    est_hist(:,k) = est_curr;
end

pos_err = est_hist(1,:) - true(1,:);
rmse = sqrt(mean(pos_err.^2));
fprintf('Position RMSE = %.4f (measurement sigma = %.4f)\n', rmse, sigma_meas);

% pass criterion: RMSE less than 1.5 * measurement sigma
if rmse < 1.5 * sigma_meas
    fprintf('kalman_mex_test: PASS\n');
else
    fprintf('kalman_mex_test: FAIL\n');
end

% cleanup
kalman_mex('delete', h);

% kalman_mex_test.m
% Build the kalman mex and run a simple filter test. Put this in KalmanFilter/mex and run.

fprintf('kalman_mex_test: building MEX (if needed) and running a simple filter test...\n');
try
    kalman_mex_build();
catch ME
    fprintf('Build failed: %s\n', ME.message);
    error('Cannot continue without successful MEX build.');
end

% Create C++ Kalman handle

h = kalman_mex('new');
% 10-state (as in run_simulation_realtime). Observation size determined from CSV.
state_n = 10;
% default observation size (will be overridden after reading CSV)
obs_n = 1;
kalman_mex('init', h, state_n, obs_n);

% build F similar to run_simulation_realtime
dt = single(1.0);
F = single(eye(state_n));
if state_n >= 10
    F(1,3) = dt; F(1,6) = 0.5*dt^2;
    F(2,4) = dt; F(2,7) = 0.5*dt^2;
    F(3,6) = dt; F(4,7) = dt; F(5,8) = dt; F(9,10) = dt;
end
kalman_mex('setSystem', h, F);

% initial estimate (10-dim)
est = single(zeros(state_n,1));
% default forward velocity similar to run_simulation_realtime
if numel(est) >= 3
    est(3) = single(1);
end
kalman_mex('setPrediction', h, est);

% Read observations from CSV and run up to 10 filter updates
root = fileparts(mfilename('fullpath'));
csv_candidate = fullfile(root, '..', '..', 'sim_data.csv');
if ~exist(csv_candidate, 'file')
    fprintf('CSV file not found at %s\n', csv_candidate);
    fprintf('Falling back to synthetic test (no CSV available).\n');
    % fallback to original synthetic test for compatibility
    N = 100;
    true = single(zeros(2, N));
    true(:,1) = single([0; 0.5]);
    for k = 2:N
        true(:,k) = F * true(:,k-1);
    end
    sigma_meas = single(0.5);
    rng(0);
    meas = true(1,:) + sigma_meas * single(randn(1, N));
    est_hist = single(zeros(2, N));
    est_curr = est;
    for k = 1:N
        pred = F * est_curr;
        kalman_mex('setPrediction', h, pred);
        kalman_mex('setObservation', h, meas(k));
        kalman_mex('estimateNoise', h, meas(k));
        kalman_mex('update', h);
        out = kalman_mex('get', h);
        outv = out(:);
        if numel(outv) < 2, outv(end+1:2) = 0; end
        if numel(outv) > 2, outv = outv(1:2); end
        est_curr = outv;
        est_hist(:,k) = est_curr;
    end
    pos_err = est_hist(1,:) - true(1,:);
    rmse = sqrt(mean(pos_err.^2));
    fprintf('Position RMSE = %.4f (measurement sigma = %.4f)\n', rmse, sigma_meas);
    if rmse < 1.5 * sigma_meas
        fprintf('kalman_mex_test: PASS\n');
    else
        fprintf('kalman_mex_test: FAIL\n');
    end
    kalman_mex('delete', h);
    return;
end

T = readtable(csv_candidate);
rows = min(height(T), 5000);
vars = T.Properties.VariableNames;

% determine observation columns: prefer pos (x,y)
obs_cols = {};
if all(ismember({'meas_pos_x','meas_pos_y'}, vars))
    obs_cols = {'meas_pos_x','meas_pos_y'};
elseif all(ismember({'gps_x','gps_y'}, vars))
    obs_cols = {'gps_x','gps_y'};
elseif all(ismember({'pos_x','pos_y'}, vars))
    obs_cols = {'pos_x','pos_y'};
else
    % fallback to first numeric column
    for i=1:numel(vars)
        v = T.(vars{i});
        if isnumeric(v) || islogical(v)
            obs_cols = {vars{i}}; break;
        end
    end
end
if isempty(obs_cols)
    error('No suitable observation columns found in %s', csv_candidate);
end

obs_n = numel(obs_cols);
% re-init kalman with correct obs size
kalman_mex('delete', h);
h = kalman_mex('new');
kalman_mex('init', h, state_n, obs_n);
kalman_mex('setSystem', h, F);
H = single(zeros(obs_n, state_n));
% if pos measurements (2D), map to state positions (1,2)
if obs_n == 2
    H(1,1) = 1; H(2,2) = 1;
else
    % scalar measurement map to x position
    H(1,1) = 1;
end
kalman_mex('setObservationMatrix', h, H);

fprintf('Using observation columns: %s (obs_n=%d), running up to %d updates\n', strjoin(obs_cols,','), obs_n, rows);

est_hist = single(zeros(state_n, rows));
est_curr = est;
 % prepare plotting like run_simulation_realtime
fig = figure('Name','kalman_mex_test');
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis equal;
% initialize plots
true_traj = [];
meas_traj = [];
est_traj = [];
h_true = plot(ax, 0,0, '-k','LineWidth',1.5, 'DisplayName','True');
h_meas = plot(ax, 0,0, '.r', 'DisplayName','Meas');
h_est = plot(ax, 0,0, '-b','LineWidth',1.5, 'DisplayName','Est');
h_true_head = quiver(ax,0,0,0,0,0,'Color','k','MaxHeadSize',0.5,'DisplayName','True heading');
h_est_head = quiver(ax,0,0,0,0,0,'Color','b','MaxHeadSize',0.5,'DisplayName','Est heading');
legend;

for k = 1:rows
    pred = F * est_curr;
    kalman_mex('setPrediction', h, pred);
    % build observation vector
    z = single(zeros(obs_n,1));
    for j=1:obs_n
        z(j) = single(T.(obs_cols{j})(k));
    end
    kalman_mex('setObservation', h, z);
    % estimateNoise expects scalar or vector; pass z
    kalman_mex('estimateNoise', h, z);
    kalman_mex('update', h);
    out = kalman_mex('get', h);
    outv = out(:);
    if numel(outv) < state_n, outv(end+1:state_n) = 0; end
    if numel(outv) > state_n, outv = outv(1:state_n); end
    est_curr = outv;
    est_hist(:,k) = est_curr;
    if obs_n == 2
        fprintf('k=%d obs=[%.4g,%.4g] pos_est=[%.4g, %.4g]\n', k, double(z(1)), double(z(2)), double(est_curr(1)), double(est_curr(2)));
        % update trajs
        true_traj(end+1,:) = [double(T.(obs_cols{1})(k)), double(T.(obs_cols{2})(k))];
        meas_traj(end+1,:) = [double(z(1)), double(z(2))];
    else
        fprintf('k=%d obs=%.4g pos_est=[%.4g, %.4g]\n', k, double(z(1)), double(est_curr(1)), double(est_curr(2)));
        meas_traj(end+1,:) = [double(z(1)), 0];
    end
    est_traj(end+1,:) = [double(est_curr(1)), double(est_curr(2))];
    % update plots
    set(h_true, 'XData', true_traj(:,1), 'YData', true_traj(:,2));
    set(h_meas, 'XData', meas_traj(:,1), 'YData', meas_traj(:,2));
    set(h_est, 'XData', est_traj(:,1), 'YData', est_traj(:,2));
    % update heading arrows
    if size(true_traj,1) >= 1
        th_true = 0;
        if size(true_traj,1) >= 2
            v_true = true_traj(end,:) - true_traj(max(1,end-1),:);
            if norm(v_true) > 1e-6, th_true = atan2(v_true(2), v_true(1)); end
        end
        set(h_true_head, 'XData', true_traj(end,1), 'YData', true_traj(end,2), 'UData', cos(th_true), 'VData', sin(th_true));
    end
    if size(est_traj,1) >= 1
        th_est = 0;
        if size(est_traj,1) >= 2
            v_est = est_traj(end,:) - est_traj(max(1,end-1),:);
            if norm(v_est) > 1e-6, th_est = atan2(v_est(2), v_est(1)); end
        end
        set(h_est_head, 'XData', est_traj(end,1), 'YData', est_traj(end,2), 'UData', cos(th_est), 'VData', sin(th_est));
    end
    drawnow limitrate;
end

fprintf('kalman_mex_test: finished %d updates from CSV.\n', rows);

% cleanup
kalman_mex('delete', h);

% run_simulation_realtime.m (minimal)
% EKF-only realtime loop with simple x-y plot. Keeps CSV or sim-step data source.
clear; close all; clc

% ensure subfolders are on path
root_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(root_dir,'GenerateData'));
addpath(fullfile(root_dir,'KalmanFilter'));

% read params
params = config_params();
N = floor(params.T/params.dt)+1;

% set defaults for display update frequency
if ~isfield(params,'display') || ~isfield(params.display,'update_rate')
	params.display.update_rate = max(1, round(0.05/params.dt)); % ~20 Hz
end

% initialize state (coerce to 10-dim if provided)
if isfield(params,'initial_state') && numel(params.initial_state) >= 10
	state_prev = params.initial_state(:);
else
	if isfield(params,'initial_state')
		tmp = params.initial_state(:)'; tmp = [tmp, zeros(1, max(0,10-numel(tmp)))]; state_prev = tmp(1:10)';
	else
		state_prev = zeros(10,1); state_prev(3)=1;
	end
end

% EKF initial state and covariance
if isfield(params,'kf') && isfield(params.kf,'x0')
	tmp = params.kf.x0(:)'; tmp = [tmp, zeros(1, 10-numel(tmp))]; x_est = tmp(1:10)';
else
	x_est = zeros(10,1); x_est(3)=1;
end
if isfield(params,'kf') && isfield(params.kf,'P0') && all(size(params.kf.P0)==[10 10])
	P = params.kf.P0;
else
	P = diag([10,10,5,5,1,1,1,1,1,1]);
end

% force EKF (minimal script)
filter_step = @ekf_filter_step;

% simple plot
fig = figure('Name','Realtime EKF (minimal)');
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis equal;
h_true = plot(ax, state_prev(1), state_prev(2), '-k','LineWidth',1.5,'DisplayName','True');
h_meas = plot(ax, state_prev(1), state_prev(2), '.r','DisplayName','Meas');
h_est = plot(ax, x_est(1), x_est(2), '-b','LineWidth',1.5,'DisplayName','EKF');
h_true_head = quiver(ax, state_prev(1), state_prev(2), 0,0,0,'Color','k');
h_est_head = quiver(ax, x_est(1), x_est(2), 0,0,0,'Color','b');
legend('Location','best');

true_traj = state_prev(1:2)'; meas_traj = state_prev(1:2)'; est_traj = x_est(1:2)';

% main loop
for k = 1:N
	if ~isvalid(fig)
		fprintf('Figure closed. Exiting loop.\n'); break;
	end

	% get data: CSV or sim step
	if isfield(params,'data') && isfield(params.data,'source') && strcmpi(params.data.source,'csv')
		if k==1
			T = readtable(params.data.file);
			csvN = height(T);
		end
		if k>csvN, break; end
		state_curr = [T.x(k); T.y(k); T.vx(k); T.vy(k)];
		% measurement: prefer gps fields if present
		if ismember('gps_x', T.Properties.VariableNames)
			meas.pos = [T.gps_x(k), T.gps_y(k)];
		else
			meas.pos = [T.meas_pos_x(k), T.meas_pos_y(k)];
		end
		if ismember('meas_vel_x', T.Properties.VariableNames)
			meas.vel = [T.meas_vel_x(k), T.meas_vel_y(k)];
		end
	else
		[state_curr, meas] = sim_step(state_prev, params, k);
	end

	% EKF step
	[x_pred, P_pred, x_upd, P_upd, y, S, K, params] = filter_step(x_est, P, meas, params);
	x_est = x_upd; P = P_upd;

	% update trajectories
	true_traj(end+1,:) = state_curr(1:2)';
	if isfield(meas,'gps') && all(~isnan(meas.gps)), meas_traj(end+1,:) = meas.gps(:)'; else meas_traj(end+1,:) = meas.pos; end
	est_traj(end+1,:) = x_est(1:2)';

	% update plot at configured rate
	if mod(k-1, params.display.update_rate) == 0
		set(h_true, 'XData', true_traj(:,1), 'YData', true_traj(:,2));
		set(h_meas, 'XData', meas_traj(:,1), 'YData', meas_traj(:,2));
		set(h_est, 'XData', est_traj(:,1), 'YData', est_traj(:,2));
		% update heading arrows (based on velocity)
		th_true = atan2(state_curr(4), state_curr(3));
		th_est = atan2(x_est(4), x_est(3));
		set(h_true_head,'XData',state_curr(1),'YData',state_curr(2),'UData',cos(th_true),'VData',sin(th_true));
		set(h_est_head,'XData',x_est(1),'YData',x_est(2),'UData',cos(th_est),'VData',sin(th_est));
		drawnow limitrate
	end

	state_prev = state_curr;
end

fprintf('run_simulation_realtime (minimal) finished.\n');
 

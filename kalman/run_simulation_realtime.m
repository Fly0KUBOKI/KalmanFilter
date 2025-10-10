% run_simulation_realtime.m
% CSVベースのリアルタイム風シミュレーション + カルマンフィルタ推定（10次元固定）
clear; close all; clc

% ensure subfolders are on path so helper functions are found
% determine repository root robustly
% prefer the script location, but if MATLAB is running an editor temp copy (Editor_pptmd)
% fall back to locating the installed file via which(), and finally pwd()
detected = fileparts(mfilename('fullpath'));
if isempty(detected) || contains(detected, fullfile(getenv('LOCALAPPDATA'),'Temp')) || contains(detected, 'Editor_pptmd')
	w = which('run_simulation_realtime.m');
	if ~isempty(w)
		root_dir = fileparts(w);
	else
		% last resort: current working directory
		root_dir = pwd;
	end
else
	root_dir = detected;
end

addpath(fullfile(root_dir,'GenerateData'));
addpath(fullfile(root_dir,'Graph'));
addpath(fullfile(root_dir,'KalmanFilter'));
% add ESKF implementation path (minimal integration)
addpath(fullfile(root_dir,'ESKF'));

% パラメータは設定ファイルから読み込む
params = config_params();
N = floor(params.T/params.dt)+1;

% initialize adaptive R warmup containers to avoid accumulation across runs
if ~isfield(params,'kf'), params.kf = struct(); end
if ~isfield(params.kf,'R_warmup_count') || isempty(params.kf.R_warmup_count)
	params.kf.R_warmup_count = struct();
end
if ~isfield(params.kf,'R_warmup_sum') || isempty(params.kf.R_warmup_sum)
	params.kf.R_warmup_sum = struct();
end

% require CSV data source
if ~(isfield(params,'data') && isfield(params.data,'source') && strcmpi(params.data.source,'csv') && isfield(params.data,'file'))
	error('This simplified script only supports CSV data. Set params.data.source=''csv'' and params.data.file.');
end

% --- 10次元固定: initial_state, kf.x0, kf.P0 ---
if isfield(params,'initial_state') && numel(params.initial_state) == 10
	state_prev = params.initial_state(:);
else
	state_prev = zeros(10,1);
	state_prev(3) = 1; % default vx=1
end

if isfield(params,'kf') && isfield(params.kf,'x0') && numel(params.kf.x0) == 10
	x_est = params.kf.x0(:);
else
	x_est = zeros(10,1);
	x_est(3) = 1;
end

if isfield(params,'kf') && isfield(params.kf,'P0') && all(size(params.kf.P0) == [10 10])
	P = params.kf.P0;
else
	P = diag([10,10,5,5,1,1,1,1,1,1]);
end

% run_simulation_realtime.m (clean)
clear; close all; clc

% ensure subfolders are on path so helper functions are found
detected = fileparts(mfilename('fullpath'));
if isempty(detected) || contains(detected, fullfile(getenv('LOCALAPPDATA'),'Temp')) || contains(detected, 'Editor_pptmd')
	w = which('run_simulation_realtime.m');
	if ~isempty(w)
		root_dir = fileparts(w);
	else
		root_dir = pwd;
	end
else
	root_dir = detected;
end

addpath(fullfile(root_dir,'GenerateData'));
addpath(fullfile(root_dir,'Graph'));
addpath(fullfile(root_dir,'KalmanFilter'));

% load config and compute nominal iteration count
params = config_params();
N = floor(params.T/params.dt)+1;

% require CSV data source
if ~(isfield(params,'data') && isfield(params.data,'source') && strcmpi(params.data.source,'csv') && isfield(params.data,'file'))
	error('This simplified script only supports CSV data. Set params.data.source=''csv'' and params.data.file.');
end

% --- 10次元固定: initial_state, kf.x0, kf.P0 ---
if isfield(params,'initial_state') && numel(params.initial_state) == 10
	state_prev = params.initial_state(:);
else
	state_prev = zeros(10,1);
	state_prev(3) = 1; % default vx=1
end

if isfield(params,'kf') && isfield(params.kf,'x0') && numel(params.kf.x0) == 10
	x_est = params.kf.x0(:);
else
	x_est = zeros(10,1);
	x_est(3) = 1;
end

if isfield(params,'kf') && isfield(params.kf,'P0') && all(size(params.kf.P0) == [10 10])
	P = params.kf.P0;
else
	P = diag([10,10,5,5,1,1,1,1,1,1]);
end

% prepare minimal ESKF nominal state and covariance (15x15 error-state)
nominal_eskf.pos = [x_est(1); x_est(2); 0];
nominal_eskf.vel = [x_est(3); x_est(4); 0];
nominal_eskf.quat = [1;0;0;0];
nominal_eskf.bg = zeros(3,1);
nominal_eskf.ba = zeros(3,1);
P_eskf = eye(15) * 0.1;

% choose filter implementation (support eskf as special case)
if ~isfield(params,'kf') || ~isfield(params.kf,'type'), params.kf.type = 'ekf'; end
switch lower(params.kf.type)
	case 'ekf', filter_step = @ekf_filter_step;
	case 'kf',  filter_step = @kf_filter_step;
	case 'ukf', filter_step = @ukf_filter_step;
	case 'eskf', filter_step = @eskf_filter_step;
	otherwise
		warning('Unknown params.kf.type="%s"; defaulting to ekf', params.kf.type);
		filter_step = @ekf_filter_step;
end

% load and pre-extract CSV into compact arrays/structs
[true_state_full, meas_full, csvN] = load_sim_data(params.data.file);
% determine number of iterations: limited by requested N and available CSV rows
maxIter = min(N, csvN);

% batch mode: preallocate histories (include initial state as first row)
true_traj = zeros(maxIter+1,2);
meas_traj = zeros(maxIter+1,2);
est_traj = zeros(maxIter+1,2);
P_hist = zeros(10,10,maxIter+1);
true_traj(1,:) = state_prev(1:2)';
meas_traj(1,:) = state_prev(1:2)';
est_traj(1,:) = x_est(1:2)';
P_hist(:,:,1) = P;

for k=1:maxIter
	% batch mode loop: use pre-extracted arrays/structs
	state_curr = true_state_full(k,:)';
	meas.pos = meas_full.pos(k,:);
	meas.vel = meas_full.vel(k,:);
	if isfield(meas_full,'accel3'), meas.accel3 = meas_full.accel3(k,:); end
	if isfield(meas_full,'gyro3'), meas.gyro3 = meas_full.gyro3(k,:); end
	if isfield(meas_full,'mag3'), meas.mag3 = meas_full.mag3(k,:); end
	if isfield(meas_full,'gps'), meas.gps = meas_full.gps(k,:); end
	if isfield(meas_full,'baro'), meas.baro = meas_full.baro(k); end
	if isfield(meas_full,'heading'), meas.heading = meas_full.heading(k,:); end

	% call appropriate filter
	if exist('filter_step','var') && isa(filter_step,'function_handle') && ~strcmpi(params.kf.type,'eskf')
		[x_pred, P_pred, x_upd, P_upd, y, S, K, params] = filter_step(x_est, P, meas, params);
	elseif strcmpi(params.kf.type,'eskf')
		meas_eskf = struct();
		if isfield(meas,'accel3'), meas_eskf.imu.accel = meas.accel3(:); end
		if isfield(meas,'gyro3'), meas_eskf.imu.gyro = meas.gyro3(:); end
		if isfield(meas,'mag3'), meas_eskf.mag3 = meas.mag3(:); end
		if isfield(meas,'baro'), meas_eskf.baro = meas.baro; end
		if isfield(meas,'gps')
			g = meas.gps(:);
			if numel(g) < 3, g(3) = nominal_eskf.pos(3); end
			meas_eskf.gps = g;
		end
		if isfield(meas,'vel')
			v = meas.vel(:);
			if numel(v) < 3, v(3) = nominal_eskf.vel(3); end
			meas_eskf.vel = v;
		end
		[nominal_eskf, P_eskf, innovations] = eskf_filter_step(nominal_eskf, P_eskf, meas_eskf, params);
		x_upd = zeros(10,1); x_upd(1:2) = nominal_eskf.pos(1:2); x_upd(3:4) = nominal_eskf.vel(1:2);
		P_upd = P; y = []; S = []; K = [];
	else
		error('Unknown filter configuration');
	end

	% log into histories
	idx = k+1;
	true_traj(idx,:) = state_curr(1:2)';
	if isfield(meas,'gps') && numel(meas.gps) >= 2 && all(~isnan(meas.gps))
		meas_traj(idx,:) = meas.gps(:)';
	else
		meas_traj(idx,:) = meas.pos;
	end
	est_traj(idx,:) = x_upd(1:2)';
	P_hist(:,:,idx) = P_upd;

	% update internal state for next iteration
	state_prev = state_curr;
	x_est = x_upd;
	P = P_upd;
end

fprintf('Batch simulation finished.\n');

% prepare outputs for visualization
L = size(true_traj,1);
t = (0:(L-1))' * params.dt;
meas_struct.pos = meas_traj;
if exist('P_hist','var') && size(P_hist,3) >= L
	P_est = P_hist(:,:,1:L);
else
	P_est = repmat(P,1,1,L);
end

% determine update_step for visualization (n: draw every n-th sample)
if isfield(params,'visualize') && isfield(params.visualize,'update_step') && ~isempty(params.visualize.update_step)
	update_step = max(1, floor(params.visualize.update_step));
else
	update_step = max(1, round(size(true_traj,1)/200));
end

visualize_sim(t, true_traj, meas_struct, est_traj, P_est, update_step);

% end of script
					x_upd = zeros(10,1); x_upd(1:2) = nominal_eskf.pos(1:2); x_upd(3:4) = nominal_eskf.vel(1:2);
					P_upd = P; y = []; S = []; K = [];
				else
					error('Unknown filter configuration');
				end

				% log into histories
				idx = k+1;
				true_traj(idx,:) = state_curr(1:2)';
				if isfield(meas,'gps') && numel(meas.gps) >= 2 && all(~isnan(meas.gps))
					meas_traj(idx,:) = meas.gps(:)';
				else
					meas_traj(idx,:) = meas.pos;
				end
				est_traj(idx,:) = x_upd(1:2)';
				P_hist(:,:,idx) = P_upd;

				% update internal state for next iteration
				state_prev = state_curr;
				x_est = x_upd;
				P = P_upd;
			end

			fprintf('Batch simulation finished.\n');

			% prepare outputs for visualization
			L = size(true_traj,1);
			t = (0:(L-1))' * params.dt;
			meas_struct.pos = meas_traj;
			if exist('P_hist','var') && size(P_hist,3) >= L
				P_est = P_hist(:,:,1:L);
			else
				P_est = repmat(P,1,1,L);
			end

			% determine update_step for visualization (n: draw every n-th sample)
			if isfield(params,'visualize') && isfield(params.visualize,'update_step') && ~isempty(params.visualize.update_step)
				update_step = max(1, floor(params.visualize.update_step));
			else
				update_step = max(1, round(size(true_traj,1)/200));
			end

			visualize_sim(t, true_traj, meas_struct, est_traj, P_est, update_step);

			% end of script

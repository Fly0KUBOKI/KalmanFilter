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
% ensure ESKF implementation is on path so eskf_filter_step is callable
if exist(fullfile(root_dir,'ESKF'),'dir')
	addpath(fullfile(root_dir,'ESKF'));
end

% パラメータは設定ファイルから読み込む
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

% choose filter implementation
if ~isfield(params,'kf') || ~isfield(params.kf,'type'), params.kf.type = 'ekf'; end
switch lower(params.kf.type)
		case 'ekf', filter_step = @ekf_filter_step;
		case 'kf',  filter_step = @kf_filter_step;
	case 'eskf', filter_step = @eskf_filter_step;
		case 'ukf', filter_step = @ukf_filter_step;
		otherwise
				warning('Unknown params.kf.type="%s"; defaulting to ekf', params.kf.type);
				filter_step = @ekf_filter_step;
end

% prepare minimal ESKF nominal state and covariance (15x15 error-state) if needed
if strcmpi(params.kf.type,'eskf')
    nominal_eskf.pos = [x_est(1); x_est(2); 0];
    nominal_eskf.vel = [x_est(3); x_est(4); 0];
    nominal_eskf.quat = [1;0;0;0];
    nominal_eskf.bg = zeros(3,1);
    nominal_eskf.ba = zeros(3,1);
    P_eskf = eye(15) * 0.1;
end

% load and pre-extract CSV into compact arrays/structs
[true_state_full, meas_full, csvN] = load_sim_data(params.data.file);
% determine number of iterations: limited by requested N and available CSV rows
maxIter = min(N, csvN);

% initialize streaming visualization (minimal)
% use first sample as initial if available
if maxIter >= 1
	t0 = 0;
	true0 = true_state_full(1,1:2);
	meas0 = meas_full.pos(1,:);
	est0 = x_est(1:2)';
else
	t0 = 0; true0 = [0 0]; meas0 = [NaN NaN]; est0 = [0 0];
end
h_stream = [];
% initialize streaming visualization if helper exists
if exist('visualize_sim_stream_init','file') == 2
	h_stream = visualize_sim_stream_init(t0, true0, meas0, est0);
end

% --- Pause control: Spaceキーで一時停止/再開 ---
% h_stream の figure を親に使う。なければ制御用 figure を作成する。
stop_created_new = false;
if ~isempty(h_stream) && isstruct(h_stream) && isfield(h_stream,'fig') && ishandle(h_stream.fig)
	parentFig = h_stream.fig;
else
	parentFig = figure('Name','Simulation Control','NumberTitle','off','MenuBar','none','ToolBar','none');
	stop_created_new = true;
end
% isPaused flag: false = running, true = paused
setappdata(parentFig,'isPaused',false);
% attach key handler: Space キーで切り替え (use WindowKeyPressFcn for reliability)
set(parentFig,'WindowKeyPressFcn',@keypress_toggle_pause);
% bring control figure to front to help receive key events
try
	figure(parentFig);
catch
end

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
		% first-iteration debug prints removed
		% batch mode loop: no realtime UI
		% CSV mode: use pre-extracted arrays/structs
		state_curr = true_state_full(k,:)';
		meas.pos = meas_full.pos(k,:);
		meas.vel = meas_full.vel(k,:);
		if isfield(meas_full,'accel3'), meas.accel3 = meas_full.accel3(k,:); end
		if isfield(meas_full,'gyro3'), meas.gyro3 = meas_full.gyro3(k,:); end
		if isfield(meas_full,'mag3'), meas.mag3 = meas_full.mag3(k,:); end
		if isfield(meas_full,'gps'), meas.gps = meas_full.gps(k,:); end
		if isfield(meas_full,'baro'), meas.baro = meas_full.baro(k); end
		if isfield(meas_full,'heading'), meas.heading = meas_full.heading(k,:); end

		% call filter with raw measurement (support ESKF as special case)
		if exist('filter_step','var') && isa(filter_step,'function_handle') && ~strcmpi(params.kf.type,'eskf')
			[x_pred, P_pred, x_upd, P_upd, y, S, K, params] = filter_step(x_est, P, meas, params);
		elseif strcmpi(params.kf.type,'eskf')
			% assemble meas struct for eskf_filter_step
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

		% ログ更新 into preallocated histories
		idx = k+1; % preallocated arrays store initial at 1
		true_traj(idx,:) = state_curr(1:2)';
		if isfield(meas,'gps') && numel(meas.gps) >= 2 && all(~isnan(meas.gps))
				meas_traj(idx,:) = meas.gps(:)';
		else
				meas_traj(idx,:) = meas.pos;
		end
		est_traj(idx,:) = x_upd(1:2)';
		P_hist(:,:,idx) = P_upd;

		% streaming update (if available)
		if ~isempty(h_stream) && isstruct(h_stream) && isfield(h_stream,'fig') && ishandle(h_stream.fig)
			visualize_sim_stream_update(h_stream, (k-1)*params.dt, state_curr(1:2), meas.pos, x_upd(1:2)');
		else
			% if the streaming figure was closed by the user, ensure Stop control remains available
			if ~(exist('parentFig','var') && ishandle(parentFig))
				parentFig = figure('Name','Simulation Control','NumberTitle','off','MenuBar','none','ToolBar','none');
				stop_created_new = true;
				% use isPaused flag and key handler for pause/resume
				setappdata(parentFig,'isPaused',false);
				set(parentFig,'WindowKeyPressFcn',@keypress_toggle_pause);
				try
					figure(parentFig);
				catch
				end
			end
		end

		% ユーザによる停止リクエストをチェック
		% まず UI イベントを処理してコールバックを実行させる
		drawnow limitrate;
		if exist('parentFig','var') && ishandle(parentFig)
			isPaused = getappdata(parentFig,'isPaused');
			if ~isempty(isPaused) && all(isPaused)
				% 一時停止ループ: スペースで再開されるまで待つ
				uiwait_backoff = 0.05; % sec
				while true
					drawnow limitrate;
					isPaused = getappdata(parentFig,'isPaused');
					if isempty(isPaused) || ~all(isPaused)
						break; % 再開
					end
					pause(uiwait_backoff);
				end
			end
		end

		% プロット更新
		% プロット更新: adaptive_R のウォームアップ中（最初の N サンプル）はグラフ表示しない
		warmupActive = false;
		if isfield(params,'kf') && isfield(params.kf,'ema_warmup') && isfield(params.kf,'R_warmup_count')
				rc = params.kf.R_warmup_count;
				fn = fieldnames(rc);
				if ~isempty(fn)
						% try quick vectorized check, fall back to loop
						try
								vals = struct2array(rc);
								warmupActive = any(vals < params.kf.ema_warmup);
						catch
								for jj = 1:numel(fn)
										if rc.(fn{jj}) < params.kf.ema_warmup
												warmupActive = true; break;
										end
								end
						end
				end
		end

		% 状態更新
		state_prev = state_curr;
		x_est = x_upd;
		P = P_upd;

		% realtime-like delay: wait for one time step duration
		if isfield(params,'dt') && ~isempty(params.dt)
			pause(params.dt);
		else
			pause(0);
		end
end


% if batch mode, call visualize_sim with collected histories
% after batch run: determine length (could be maxIter+1 or less if loop broke early)
L = size(true_traj,1);
t = (0:(L-1))'*params.dt;
meas_struct.pos = meas_traj;
% ensure P_hist has correct length; if not, fall back to last P
if exist('P_hist','var') && size(P_hist,3) >= L
		P_est = P_hist(:,:,1:L);
else
		% replicate final P
		P_est = repmat(P,1,1,L);
end


update_step = 4;
visualize_sim(t, true_traj, meas_struct, est_traj, P_est, update_step);

% end of script

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

% パラメータは設定ファイルから読み込む
params = config_params();
N = floor(params.T/params.dt)+1;

% require CSV data source
if ~(isfield(params,'data') && isfield(params.data,'source') && strcmpi(params.data.source,'csv') && isfield(params.data,'file'))
	error('This simplified script only supports CSV data. Set params.data.source=''csv'' and params.data.file.');
end

% --- 8次元固定: initial_state, kf.x0, kf.P0 ---
if isfield(params,'initial_state') && numel(params.initial_state) == 8
	state_prev = params.initial_state(:);
else
	state_prev = zeros(8,1);
	state_prev(1) = 1; % default v=1
	state_prev(2:5) = [1;0;0;0]; % identity quaternion
end

if isfield(params,'kf') && isfield(params.kf,'x0') && numel(params.kf.x0) == 8
	x_est = params.kf.x0(:);
else
	x_est = zeros(8,1);
	x_est(1) = 1;
	x_est(2:5) = [1;0;0;0];
end

if isfield(params,'kf') && isfield(params.kf,'P0') && all(size(params.kf.P0) == [8 8])
	P = params.kf.P0;
else
	P = diag([5,0.1,0.1,0.1,0.1,10,10,10]);
end

% Force ESKF only (other filters deprecated)
params.kf.type = 'eskf';
filter_step = @eskf_filter_step;

% prepare minimal ESKF nominal state and covariance (15x15 error-state) if needed
if strcmpi(params.kf.type,'eskf')
	% Map 8-dim state to ESKF nominal fields: pos, vel, quat
	nominal_eskf.pos = [x_est(6); x_est(7); x_est(8)];
	% compute vel from v and quaternion: world_vel = R * [v;0;0]
	v = x_est(1); q = x_est(2:5); qw=q(1); qx=q(2); qy=q(3); qz=q(4);
	R = [1-2*(qy^2+qz^2), 2*(qx*qy- qz*qw), 2*(qx*qz+ qy*qw);
		 2*(qx*qy+ qz*qw), 1-2*(qx^2+qz^2), 2*(qy*qz- qx*qw);
		 2*(qx*qz- qy*qw), 2*(qy*qz+ qx*qw), 1-2*(qx^2+qy^2)];
	nominal_eskf.vel = R * [v;0;0];
	nominal_eskf.quat = x_est(2:5);
	nominal_eskf.bg = zeros(3,1);
	nominal_eskf.ba = zeros(3,1);
	P_eskf = eye(15) * 0.1;
end

% heading estimated by integrating gyro z (fallback / simple estimator)
% initialize from initial state theta if available, otherwise NaN
heading_gyro = NaN;
if exist('true_state_full','var') && ~isempty(true_state_full) && size(true_state_full,2) >= 6
	% true_state_full columns: t,x,y,vx,vy,theta,... -> theta at col 6
	heading_gyro = true_state_full(1,6);
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
true_traj = zeros(maxIter+1,3);
meas_traj = zeros(maxIter+1,3);
est_traj = zeros(maxIter+1,3);
P_hist = zeros(8,8,maxIter+1);
true_traj(1,:) = state_prev(6:8)';
if isfield(meas_full,'pos') && ~isempty(meas_full.pos)
	% meas_full.pos may be Nx2; pad to 3 columns if needed
	if size(meas_full.pos,2) >= 3
		meas_traj(1,:) = meas_full.pos(1,1:3);
	else
		meas_traj(1,:) = [meas_full.pos(1,1:2) 0];
	end
else
	meas_traj(1,:) = [NaN NaN NaN];
end
est_traj(1,:) = x_est(6:8)';
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
			% intentionally skip passing magnetometer measurements to ESKF
			% if isfield(meas,'mag3'), meas_eskf.mag3 = meas.mag3(:); end
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
					% route through 8-dim wrapper to centralize mapping logic
					% wrapper follows generic filter_step signature
					[~,~,x_upd_tmp, P_upd_tmp, ~,~,~,~] = eskf_filter_step_wrapper([norm(nominal_eskf.vel); nominal_eskf.quat; nominal_eskf.pos], P, meas, params);
					% wrapper returns x_upd as 8x1; unpack into nominal_eskf and P_eskf
					if ~isempty(x_upd_tmp) && numel(x_upd_tmp) >= 8
						nominal_eskf.pos = x_upd_tmp(6:8);
						nominal_eskf.quat = x_upd_tmp(2:5);
						% rebuild vel from speed+quat
						qw=x_upd_tmp(2); qx=x_upd_tmp(3); qy=x_upd_tmp(4); qz=x_upd_tmp(5);
						R = [1-2*(qy^2+qz^2), 2*(qx*qy- qz*qw), 2*(qx*qz+ qy*qw);
							 2*(qx*qy+ qz*qw), 1-2*(qx^2+qz^2), 2*(qy*qz- qx*qw);
							 2*(qx*qz- qy*qw), 2*(qy*qz+ qx*qw), 1-2*(qx^2+qy^2)];
						nominal_eskf.vel = R * [x_upd_tmp(1);0;0];
					end
					% P_upd_tmp placeholder holds 8x8 covariance; preserve per-loop P/P_eskf
					P_upd = P_upd_tmp;
					P_eskf = P_eskf; innovations = [];
			% convert ESKF output back to 8-dim state [v q1 q2 q3 q4 x y z]
			x_upd = zeros(8,1);
			% position
			x_upd(6:8) = nominal_eskf.pos(:);
			% quaternion (ensure column)
			if isfield(nominal_eskf,'quat') && numel(nominal_eskf.quat) >= 4
				x_upd(2:5) = nominal_eskf.quat(:);
			else
				x_upd(2:5) = [1;0;0;0];
			end
			% speed norm from vel
			x_upd(1) = norm(nominal_eskf.vel(:));
			P_upd = P; y = []; S = []; K = [];
		else
			error('Unknown filter configuration');
		end

		% integrate gyro z (angular rate about z) into heading_gyro for simple heading estimate
		try
			if isfield(meas,'gyro3') && numel(meas.gyro3) >= 3 && isfinite(meas.gyro3(3))
				dt_local = params.dt;
				if isempty(dt_local) || ~isfinite(dt_local) || dt_local <= 0
					dt_local = 1;
				end
				% meas.gyro3 assumed in rad/s
				if isnan(heading_gyro)
					% initialize to estimated heading via velocity if possible
					try
						heading_gyro = atan2(state_curr(4), state_curr(3));
					catch
						heading_gyro = 0;
					end
				end
				heading_gyro = heading_gyro + meas.gyro3(3) * dt_local;
			end
		catch
			% ignore gyro integration errors
		end

		% ログ更新 into preallocated histories
		idx = k+1; % preallocated arrays store initial at 1
		% state_curr from CSV may contain columns: t,x,y,...; prefer position columns if present
		try
			% attempt to use columns 2:3 or 6:8 depending on CSV layout
			if size(state_curr,1) >= 3
				true_pos = state_curr(2:3)';
			else
				true_pos = state_curr(1:2)';
			end
		catch
			true_pos = [NaN NaN NaN];
		end
		true_traj(idx,:) = [true_pos(1:2) 0];
		if isfield(meas,'gps') && numel(meas.gps) >= 2 && all(~isnan(meas.gps))
				if numel(meas.gps) >= 3
					meas_traj(idx,:) = meas.gps(:)';
				else
					meas_traj(idx,:) = [meas.gps(:)' 0];
				end
		else
				if numel(meas.pos) >= 2
					meas_traj(idx,:) = [meas.pos(1:2) 0];
				else
					meas_traj(idx,:) = [NaN NaN NaN];
				end
		end
		est_traj(idx,:) = x_upd(6:8)';
		P_hist(:,:,idx) = P_upd;

		% streaming update (if available)
		if ~isempty(h_stream) && isstruct(h_stream) && isfield(h_stream,'fig')
			if ~ishandle(h_stream.fig)
				% streaming figure closed by user -> stop simulation loop
				break;
			end
			% compute true yaw from true state if velocity columns are available in CSV
			try
				if size(state_curr,1) >= 5
					% assume columns: t,x,y,vx,vy,...
					tvx = state_curr(4); tvy = state_curr(5);
					true_yaw = atan2(tvy, tvx);
				else
					true_yaw = [];
				end
			catch
				true_yaw = [];
			end
			% compute est yaw: prefer gyro-integration estimate when available
			est_yaw = [];
			if ~isnan(heading_gyro)
				est_yaw = heading_gyro;
			else
				try
					if exist('nominal_eskf','var') && isstruct(nominal_eskf) && isfield(nominal_eskf,'quat')
						q = nominal_eskf.quat(:);
						if numel(q) == 4
							% quaternion assumed [q0; q1; q2; q3] (scalar-first)
							q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
							% yaw (z) extraction
							est_yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
						end
					end
				catch
					est_yaw = [];
				end
				% fallback: estimate yaw from ESKF velocity or quaternion
				if isempty(est_yaw)
					try
						if isfield(nominal_eskf,'vel')
							ve = nominal_eskf.vel(:);
							est_yaw = atan2(ve(2), ve(1));
						else
							est_yaw = [];
						end
					catch
						est_yaw = [];
					end
				end
			end

			% prepare points (use x,y positions)
			try
				if numel(state_curr) >= 3
					t_pt = state_curr(2:3);
				else
					t_pt = state_curr(1:2);
				end
			catch
				t_pt = [NaN NaN];
			end
			if numel(meas.pos) >= 2
				m_pt = meas.pos(1:2);
			else
				m_pt = [NaN NaN];
			end
			e_pt = x_upd(6:7)';
			visualize_sim_stream_update(h_stream, (k-1)*params.dt, t_pt, m_pt, e_pt, true_yaw, est_yaw);
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

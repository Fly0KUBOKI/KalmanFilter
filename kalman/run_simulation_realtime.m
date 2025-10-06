% run_simulation_realtime.m
% リアルタイムでデータを生成、EKFにより逐次推定、可視化を行う
clear; close all; clc

% ensure subfolders are on path so helper functions are found
root_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(root_dir,'GenerateData'));
addpath(fullfile(root_dir,'Graph'));
addpath(fullfile(root_dir,'KalmanFilter'));

% パラメータは設定ファイルから読み込む
params = config_params();
N = floor(params.T/params.dt)+1;

% display performance options (can be set in config_params as params.display.update_rate and pause_factor)
if ~isfield(params,'display')
	params.display = struct();
end
if ~isfield(params.display,'update_rate')
	% default GUI updates every ~0.05s (lower frequency to reduce CPU/GUI load)
	params.display.update_rate = max(1, round(0.05/params.dt)); % ~20 Hz
end

if ~isfield(params.display,'pause_factor')
	params.display.pause_factor = 0.0; % not used in normal mode; kept for backward compatibility
end

% minimum pause threshold (seconds). If computed pause < min_pause, we skip pausing.
if ~isfield(params.display,'min_pause')
	params.display.min_pause = 0.002; % 2 ms default
end

% which figures to show: default only EKF (user can set params.display.show.* in config)
if ~isfield(params.display,'show') || isempty(params.display.show)
	params.display.show.ekf = true;
	params.display.show.accel = false;
	params.display.show.gyro = false;
	params.display.show.mag = false;
	params.display.show.heading = false;
	params.display.show.baro = false;
	params.display.show.gps = false;
else
	% ensure missing fields default to false except ekf
	if ~isfield(params.display.show,'ekf'), params.display.show.ekf = true; end
	if ~isfield(params.display.show,'accel'), params.display.show.accel = false; end
	if ~isfield(params.display.show,'gyro'), params.display.show.gyro = false; end
	if ~isfield(params.display.show,'mag'), params.display.show.mag = false; end
	if ~isfield(params.display.show,'heading'), params.display.show.heading = false; end
	if ~isfield(params.display.show,'baro'), params.display.show.baro = false; end
	if ~isfield(params.display.show,'gps'), params.display.show.gps = false; end
end

% 初期化: params の設定が拡張されている (例: 10 要素) 場合でも安全に動作するように調整します。
% 現在の sim_step / ekf_filter_step は 4 状態 [x;y;vx;vy] を期待しているため、
% 必要に応じて先頭 4 要素を使用します。
% initialize for 10-state model
if isfield(params,'initial_state') && numel(params.initial_state) >= 10
	state_prev = params.initial_state(:);
else
	% pad or truncate to 10
	if isfield(params,'initial_state')
		tmp = params.initial_state(:)';
		tmp = [tmp, zeros(1, max(0,10-numel(tmp)))];
		state_prev = tmp(1:10)';
	else
		state_prev = zeros(10,1);
		state_prev(3) = 1; % default vx=1
	end
end

% EKF initial state x0 / P0: coerce to 10-dim if necessary
if isfield(params,'kf') && isfield(params.kf,'x0') && numel(params.kf.x0) >= 10
	x_est = params.kf.x0(:);
elseif isfield(params,'kf') && isfield(params.kf,'x0')
	tmp = params.kf.x0(:)'; tmp = [tmp, zeros(1, 10-numel(tmp))]; x_est = tmp(1:10)';
else
	x_est = zeros(10,1);
	x_est(3) = 1;
end

if isfield(params,'kf') && isfield(params.kf,'P0') && all(size(params.kf.P0) == [10 10])
	P = params.kf.P0;
else
	P = diag([10,10,5,5,1,1,1,1,1,1]);
end

% choose filter implementation: 'ekf' (default), 'kf', or 'ukf'
if ~isfield(params,'kf') || ~isfield(params.kf,'type')
	params.kf.type = 'ekf';
end
switch lower(params.kf.type)
	case 'ekf'
		filter_step = @ekf_filter_step;
	case 'kf'
		filter_step = @kf_filter_step;
	case 'ukf'
		filter_step = @ukf_filter_step;
	otherwise
		warning('Unknown params.kf.type="%s"; defaulting to ekf', params.kf.type);
		filter_step = @ekf_filter_step;
end


% 警告: config の次元を切り詰めた場合はユーザーに知らせる（10 状態モデル）
if isfield(params,'initial_state') && numel(params.initial_state) ~= 10
	warning('params.initial_state has %d elements; using first 10 elements for simulation/filtering.', numel(params.initial_state));
end
if isfield(params,'kf') && isfield(params.kf,'x0') && numel(params.kf.x0) ~= 10
	warning('params.kf.x0 has %d elements; coerced to 10 elements for EKF.', numel(params.kf.x0));
end


% filter options: 'none', 'avg10', or 'ema'
if ~isfield(params,'filter'), params.filter.method = 'none'; end
if ~isfield(params.filter,'method'), params.filter.method = 'none'; end
if ~isfield(params.filter,'alpha'), params.filter.alpha = 0.2; end % EMA の係数

% バッファ初期化（avg10用）
buffer_len = 10;
meas_buf_pos = nan(buffer_len,2);
meas_buf_vel = nan(buffer_len,2);
buf_idx = 0;
% prebuffer: collect initial samples to initialize EMA (and skip EKF/plots)
prebuf_target = 100; % target number of initial samples
prebuf_len = prebuf_target;
prebuf_pos = nan(prebuf_target,2);
prebuf_vel = nan(prebuf_target,2);
prebuf_idx = 0;
prebuf_done = false;

% 準備プロット
fig = figure('Name','Realtime EKF');
ax = axes(fig);
hold(ax,'on'); grid(ax,'on'); axis equal;
h_true = plot(ax, state_prev(1), state_prev(2), '-k','LineWidth',1.5, 'DisplayName','True');
h_meas = plot(ax, state_prev(1), state_prev(2), '.r', 'DisplayName','GPS');
% create avg plot only if filter is not 'none'
show_avg = ~strcmpi(params.filter.method,'none');
% create the average plot handle as a green line. If averaging is disabled, hide it.
h_meas_avg = plot(ax, state_prev(1), state_prev(2), '-', 'Color', [0 0.6 0], 'LineWidth', 1.2, 'DisplayName', 'GPS(avg)');
if ~show_avg
    set(h_meas_avg, 'Visible', 'off');
end
h_est = plot(ax, x_est(1), x_est(2), '-b','LineWidth',1.5, 'DisplayName','EKF');
% heading vectors: true and estimated (quiver: X,Y,U,V)
heading_scale = 1.0; % visual length of heading arrow
% initialize heading from velocity direction (progress direction)
th_true = atan2(state_prev(4), state_prev(3));
th_est = atan2(x_est(4), x_est(3));
h_true_head = quiver(ax, state_prev(1), state_prev(2), heading_scale*cos(th_true), heading_scale*sin(th_true), 0, 'Color','k', 'MaxHeadSize',0.5, 'DisplayName','True heading');
h_est_head = quiver(ax, x_est(1), x_est(2), heading_scale*cos(th_est), heading_scale*sin(th_est), 0, 'Color','b', 'MaxHeadSize',0.5, 'DisplayName','Est heading');
legend;

% --- Running toggle button: appdata 'running' controls pause/resume ---
setappdata(fig,'running',true);
btn_run = uicontrol('Parent',fig,'Style','pushbutton','String','Stop','Units','normalized',... 
	'Position',[0.88 0.95 0.1 0.045], 'Callback',@(src,ev) toggleRunning(src,fig));

% filter type popup (runtime selection)
uicontrol('Parent',fig,'Style','text','Units','normalized','Position',[0.70 0.95 0.08 0.03], 'String','Filter:','HorizontalAlignment','right');
popup_filter = uicontrol('Parent',fig,'Style','popupmenu','Units','normalized','Position',[0.79 0.95 0.08 0.04],...
	'String',{'ekf','kf','ukf'}, 'Value',1, 'Callback',@(src,ev) setappdata(fig,'filter_type',src.String{src.Value}));
% set initial filter type in appdata
if isfield(params,'kf') && isfield(params.kf,'type')
	setappdata(fig,'filter_type',params.kf.type);
	% set popup value to match
	vals = get(popup_filter,'String');
	idx = find(strcmpi(vals, params.kf.type),1);
	if ~isempty(idx), set(popup_filter,'Value',idx); end
else
	setappdata(fig,'filter_type','ekf');
end

% add speed control button (cycles through x1,x2,x4,x8)
% (speed control removed) no speed button; updates use configured update_rate and no pause

true_traj = state_prev(1:2)';
meas_traj = state_prev(1:2)';
est_traj = x_est(1:2)';
meas_avg_traj = state_prev(1:2)';

% (time-series x/y figure removed by user request)

% --- measurement-only figures: create only if enabled to reduce load ---
if params.display.show.accel
	fig_acc = figure('Name','accel3 (body)'); ax_ma = axes(fig_acc); hold(ax_ma,'on'); grid(ax_ma,'on'); ylabel(ax_ma,'m/s^2'); title(ax_ma,'accel3 (body)');
end
if params.display.show.gyro
	fig_gyro = figure('Name','gyro3'); ax_mg = axes(fig_gyro); hold(ax_mg,'on'); grid(ax_mg,'on'); ylabel(ax_mg,'rad/s'); title(ax_mg,'gyro3');
end
if params.display.show.mag
	fig_mag = figure('Name','mag3'); ax_mm = axes(fig_mag); hold(ax_mm,'on'); grid(ax_mm,'on'); ylabel(ax_mm,'uT'); title(ax_mm,'mag3');
end
if params.display.show.heading
	fig_head = figure('Name','heading (angle)'); ax_mh = axes(fig_head); hold(ax_mh,'on'); grid(ax_mh,'on'); ylabel(ax_mh,'rad'); title(ax_mh,'heading (angle)');
end
if params.display.show.baro
	fig_baro = figure('Name','barometer'); ax_mb = axes(fig_baro); hold(ax_mb,'on'); grid(ax_mb,'on'); ylabel(ax_mb,'unit'); title(ax_mb,'barometer');
end
% GPS figure: show x-y plane points (not time-series)
if params.display.show.gps
	fig_gps = figure('Name','GPS x-y'); ax_mgps = axes(fig_gps); hold(ax_mgps,'on'); grid(ax_mgps,'on'); xlabel(ax_mgps,'x [m]'); ylabel(ax_mgps,'y [m]'); title(ax_mgps,'GPS x-y (points)');
end

% initialize empty logs (they will be appended in loop) - only for enabled displays
meas_time = [];
if params.display.show.accel, meas_accel_log = nan(0,3); end
if params.display.show.gyro, meas_gyro_log = nan(0,3); end
if params.display.show.mag, meas_mag_log = nan(0,3); end
if params.display.show.heading, meas_heading_log = nan(0,1); end
if params.display.show.baro, meas_baro_log = nan(0,1); end
if params.display.show.gps, meas_gps_log = nan(0,2); end

% create initial plot handles conditionally so handles exist only when used
if params.display.show.accel
	h_acc = plot(ax_ma, NaN, NaN, '-r', NaN, NaN, '-g', NaN, NaN, '-b');
	legend(ax_ma, {'ax','ay','az'});
end
if params.display.show.gyro
	h_gyro = plot(ax_mg, NaN, NaN, '-r', NaN, NaN, '-g', NaN, NaN, '-b');
	legend(ax_mg, {'gx','gy','gz'});
end
if params.display.show.mag
	h_mag = plot(ax_mm, NaN, NaN, '-r', NaN, NaN, '-g', NaN, NaN, '-b');
	legend(ax_mm, {'mx','my','mz'});
end
if params.display.show.heading
	h_head = plot(ax_mh, NaN, NaN, '-k');
end
if params.display.show.baro
	h_baro = plot(ax_mb, NaN, NaN, '-m');
end
% line handle for GPS x-y trajectory
if params.display.show.gps
	h_gps_line = plot(ax_mgps, NaN, NaN, '-','Color',[0.85 0.33 0.1], 'LineWidth',1.0, 'DisplayName','GPS (x,y)');
	legend(ax_mgps, {'GPS (x,y)'});
end


for k=1:N
	% check figure closed
	if ~isvalid(fig)
		fprintf('Figure closed. Exiting loop.\n');
		break;
	end
	% if running flag is false, wait (pause) until resumed or figure closed
	while ~getappdata(fig,'running')
		if ~isvalid(fig)
			fprintf('Figure closed during pause. Exiting loop.\n');
			break;
		end
		pause(0.1);
	end
	% 1ステップ生成またはCSVから読み出し
	if isfield(params,'data') && isfield(params.data,'source') && strcmpi(params.data.source,'csv')
		% CSVモード: params.data.file を読み、行 k を使う
		if k==1
			T = readtable(params.data.file);
			csvN = height(T);
		end
		if k>csvN
			break; % CSV の終端
		end
		state_curr = [T.x(k); T.y(k); T.vx(k); T.vy(k)];
		meas.pos = [T.meas_pos_x(k), T.meas_pos_y(k)];
		meas.vel = [T.meas_vel_x(k), T.meas_vel_y(k)];
		% optional advanced sensors
		if ismember('meas_accel', T.Properties.VariableNames)
			meas.accel = T.meas_accel(k);
		end
		if ismember('accel3_x', T.Properties.VariableNames)
			meas.accel3 = [T.accel3_x(k), T.accel3_y(k), T.accel3_z(k)];
		end
		if ismember('gyro3_x', T.Properties.VariableNames)
			meas.gyro3 = [T.gyro3_x(k), T.gyro3_y(k), T.gyro3_z(k)];
		end
		if ismember('mag3_x', T.Properties.VariableNames)
			meas.mag3 = [T.mag3_x(k), T.mag3_y(k), T.mag3_z(k)];
		end
		if ismember('gps_x', T.Properties.VariableNames)
			meas.gps = [T.gps_x(k), T.gps_y(k)];
		end
		if ismember('baro', T.Properties.VariableNames)
			meas.baro = T.baro(k);
		end
		meas.heading = [T.meas_heading_x(k), T.meas_heading_y(k)];
	else
		[state_curr, meas] = sim_step(state_prev, params, k);
	end

	% --------------------------
	% prebuffer: collect initial samples and skip EKF/plot until filled
	% --------------------------
	if ~prebuf_done
		prebuf_idx = prebuf_idx + 1;
		if prebuf_idx <= prebuf_target
			% store pos/vel (may contain NaN)
			prebuf_pos(prebuf_idx,:) = meas.pos;
			prebuf_vel(prebuf_idx,:) = meas.vel;
		end
		if prebuf_idx >= prebuf_target
			% compute mean ignoring NaN (compatible implementation without nanmean)
			init_pos = zeros(1,size(prebuf_pos,2));
			for ii = 1:size(prebuf_pos,2)
				vals = prebuf_pos(:,ii);
				valid = ~isnan(vals);
				if any(valid)
					init_pos(ii) = sum(vals(valid)) / sum(valid);
				else
					init_pos(ii) = 0;
				end
			end
			init_vel = zeros(1,size(prebuf_vel,2));
			for ii = 1:size(prebuf_vel,2)
				vals = prebuf_vel(:,ii);
				valid = ~isnan(vals);
				if any(valid)
					init_vel(ii) = sum(vals(valid)) / sum(valid);
				else
					init_vel(ii) = 0;
				end
			end
			% initialize EMA buffers
			ema_pos = init_pos;
			ema_vel = init_vel;
			prebuf_done = true;
			fprintf('Prebuffer complete: initialized EMA from %d samples.\n', prebuf_target);
		end
		% skip EKF update and plotting for prebuffer samples
		meas_avg = meas; % keep for logs
		% advance true state so sim_step progresses
		state_prev = state_curr;
		continue;
	end

	% check runtime filter selection from UI (allow switching while running)
	if isappdata(fig,'filter_type')
		ft = getappdata(fig,'filter_type');
		switch lower(ft)
			case 'ekf', filter_step = @ekf_filter_step;
			case 'kf', filter_step = @kf_filter_step;
			case 'ukf', filter_step = @ukf_filter_step;
			otherwise, filter_step = @ekf_filter_step;
		end
	end
	% 平均モードの処理: バッファに追加して平均を計算
	buf_idx = buf_idx + 1;
	if buf_idx>buffer_len, buf_idx = 1; end
	meas_buf_pos(buf_idx,:) = meas.pos;
	meas_buf_vel(buf_idx,:) = meas.vel;
	if strcmpi(params.filter.method,'avg10')
		valid_pos = meas_buf_pos(~isnan(meas_buf_pos(:,1)),:);
		valid_vel = meas_buf_vel(~isnan(meas_buf_vel(:,1)),:);
		meas_avg = meas;
		meas_avg.pos = mean(valid_pos,1);
		meas_avg.vel = mean(valid_vel,1);
		% call selected filter step implementation with averaged measurement
		[x_pred, P_pred, x_upd, P_upd, y, S, K, params] = filter_step(x_est, P, meas_avg, params);
	elseif strcmpi(params.filter.method,'ema')
		% EMA を適用
		if ~exist('ema_pos','var') || any(isnan(ema_pos))
			ema_pos = meas.pos;
			ema_vel = meas.vel;
		else
			ema_pos = params.filter.alpha.*meas.pos + (1-params.filter.alpha).*ema_pos;
			ema_vel = params.filter.alpha.*meas.vel + (1-params.filter.alpha).*ema_vel;
		end
		meas_avg = meas;
		meas_avg.pos = ema_pos;
		meas_avg.vel = ema_vel;
		[x_pred, P_pred, x_upd, P_upd, y, S, K, params] = filter_step(x_est, P, meas_avg, params);
	else
	[x_pred, P_pred, x_upd, P_upd, y, S, K, params] = filter_step(x_est, P, meas, params);
		meas_avg = meas; % for plotting
	end

	% ログ更新
	true_traj(end+1,:) = state_curr(1:2)';
	% prefer GPS measurement if present, otherwise fall back to measured pos
	if isfield(meas,'gps') && numel(meas.gps) >= 2 && all(~isnan(meas.gps))
		meas_traj(end+1,:) = meas.gps(:)';
	else
		meas_traj(end+1,:) = meas.pos;
	end
	if isfield(meas_avg,'gps') && numel(meas_avg.gps) >= 2 && all(~isnan(meas_avg.gps))
		meas_avg_traj(end+1,:) = meas_avg.gps(:)';
	else
		meas_avg_traj(end+1,:) = meas_avg.pos;
	end
	est_traj(end+1,:) = x_upd(1:2)';

	% (time-series x/y logging removed)

	% --- append raw measurements and update sensor plots only if enabled ---
	if params.display.show.accel || params.display.show.gyro || params.display.show.mag || params.display.show.heading || params.display.show.baro || params.display.show.gps
		meas_time(end+1,1) = (k-1)*params.dt;
		% accel3
		if params.display.show.accel
			if isfield(meas,'accel3') && numel(meas.accel3)>=3
				meas_accel_log(end+1,:) = meas.accel3(:)';
			else
				meas_accel_log(end+1,:) = [NaN NaN NaN];
			end
		end
		% gyro3
		if params.display.show.gyro
			if isfield(meas,'gyro3') && numel(meas.gyro3)>=3
				meas_gyro_log(end+1,:) = meas.gyro3(:)';
			else
				meas_gyro_log(end+1,:) = [NaN NaN NaN];
			end
		end
		% mag3
		if params.display.show.mag
			if isfield(meas,'mag3') && numel(meas.mag3)>=3
				meas_mag_log(end+1,:) = meas.mag3(:)';
			else
				meas_mag_log(end+1,:) = [NaN NaN NaN];
			end
		end
		% heading (angle) - if heading vector provided, compute angle
		if params.display.show.heading
			if isfield(meas,'heading') && numel(meas.heading)>=2
				meas_heading_log(end+1,1) = atan2(meas.heading(2), meas.heading(1));
			else
				meas_heading_log(end+1,1) = NaN;
			end
		end
		% baro
		if params.display.show.baro
			if isfield(meas,'baro')
				meas_baro_log(end+1,1) = meas.baro;
			else
				meas_baro_log(end+1,1) = NaN;
			end
		end
		% gps
		if params.display.show.gps
			if isfield(meas,'gps') && numel(meas.gps)>=2
				meas_gps_log(end+1,:) = meas.gps(:)';
			else
				meas_gps_log(end+1,:) = [NaN NaN];
			end
		end

		% update measurement-only plots
		if params.display.show.accel
			set(h_acc(1), 'XData', meas_time, 'YData', meas_accel_log(:,1));
			set(h_acc(2), 'XData', meas_time, 'YData', meas_accel_log(:,2));
			set(h_acc(3), 'XData', meas_time, 'YData', meas_accel_log(:,3));
		end
		if params.display.show.gyro
			set(h_gyro(1), 'XData', meas_time, 'YData', meas_gyro_log(:,1));
			set(h_gyro(2), 'XData', meas_time, 'YData', meas_gyro_log(:,2));
			set(h_gyro(3), 'XData', meas_time, 'YData', meas_gyro_log(:,3));
		end
		if params.display.show.mag
			set(h_mag(1), 'XData', meas_time, 'YData', meas_mag_log(:,1));
			set(h_mag(2), 'XData', meas_time, 'YData', meas_mag_log(:,2));
			set(h_mag(3), 'XData', meas_time, 'YData', meas_mag_log(:,3));
		end
		if params.display.show.heading
			set(h_head, 'XData', meas_time, 'YData', meas_heading_log);
		end
		if params.display.show.baro
			set(h_baro, 'XData', meas_time, 'YData', meas_baro_log);
		end
		% update GPS x-y scatter (plot all measured gps points so far)
		if params.display.show.gps && exist('meas_gps_log','var') && size(meas_gps_log,1)>0
			set(h_gps_line, 'XData', meas_gps_log(:,1), 'YData', meas_gps_log(:,2));
		end
	end

	% プロット更新
	set(h_true, 'XData', true_traj(:,1), 'YData', true_traj(:,2));
	set(h_meas, 'XData', meas_traj(:,1), 'YData', meas_traj(:,2));
	if ~isempty(h_meas_avg)
		set(h_meas_avg, 'XData', meas_avg_traj(:,1), 'YData', meas_avg_traj(:,2));
	end
	set(h_est, 'XData', est_traj(:,1), 'YData', est_traj(:,2));
	% update heading arrows using latest true/estimated theta
	% compute heading from velocity direction (vx,vy)
	if numel(state_curr) >= 4
		v_true = state_curr(3:4);
		if norm(v_true) > 1e-6
			th_true = atan2(v_true(2), v_true(1));
		end
	else
		th_true = 0;
	end
	v_est = x_upd(3:4);
	if norm(v_est) > 1e-6
		th_est = atan2(v_est(2), v_est(1));
	end
	set(h_true_head, 'XData', true_traj(end,1), 'YData', true_traj(end,2), 'UData', heading_scale*cos(th_true), 'VData', heading_scale*sin(th_true));
	set(h_est_head, 'XData', est_traj(end,1), 'YData', est_traj(end,2), 'UData', heading_scale*cos(th_est), 'VData', heading_scale*sin(th_est));
	% throttle GUI updates to reduce overhead using configured update_rate
	effective_update_rate = max(1, round(params.display.update_rate));
	if mod(k, effective_update_rate) == 0
		drawnow;
	end

	% (time-series plots removed)


	% 状態更新
	state_prev = state_curr;
	x_est = x_upd;
	P = P_upd;

	% normal mode: do not pause; yield briefly to UI event loop
	drawnow limitrate;
end

fprintf('Realtime simulation finished.\n');

% local callback to toggle running state (Start <-> Stop)
function toggleRunning(btn, fig)
	% btn : uicontrol handle
	% fig : figure handle where appdata 'running' is stored
	if ~isvalid(fig) || ~isvalid(btn)
		return;
	end
	running = getappdata(fig,'running');
	if isempty(running)
		running = true;
	end
	new_state = ~running;
	setappdata(fig,'running', new_state);
	if new_state
		set(btn, 'String', 'Stop');
	else
		set(btn, 'String', 'Start');
	end
end

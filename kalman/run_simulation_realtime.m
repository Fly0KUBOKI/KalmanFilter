% run_simulation_realtime.m
% CSVベースのリアルタイム風シミュレーション + カルマンフィルタ推定（10次元固定）
clear; close all; clc

% ensure subfolders are on path so helper functions are found
root_dir = fileparts(mfilename('fullpath'));
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
	case 'ukf', filter_step = @ukf_filter_step;
	otherwise
		warning('Unknown params.kf.type="%s"; defaulting to ekf', params.kf.type);
		filter_step = @ekf_filter_step;
end

% 準備プロット
fig = figure('Name','Realtime EKF');
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis equal;
h_true = plot(ax, state_prev(1), state_prev(2), '-k','LineWidth',1.5, 'DisplayName','True');
h_meas = plot(ax, state_prev(1), state_prev(2), '.r', 'DisplayName','GPS');
h_est = plot(ax, x_est(1), x_est(2), '-b','LineWidth',1.5, 'DisplayName','EKF');
% heading vectors
heading_scale = 1.0;
th_true = atan2(state_prev(4), state_prev(3));
th_est = atan2(x_est(4), x_est(3));
h_true_head = quiver(ax, state_prev(1), state_prev(2), heading_scale*cos(th_true), heading_scale*sin(th_true), 0, 'Color','k', 'MaxHeadSize',0.5, 'DisplayName','True heading');
h_est_head = quiver(ax, x_est(1), x_est(2), heading_scale*cos(th_est), heading_scale*sin(th_est), 0, 'Color','b', 'MaxHeadSize',0.5, 'DisplayName','Est heading');
legend;

% running toggle
setappdata(fig,'running',true);
btn_run = uicontrol('Parent',fig,'Style','pushbutton','String','Stop','Units','normalized',... 
	'Position',[0.88 0.95 0.1 0.045], 'Callback',@(src,ev) toggleRunning(src,fig));

% filter type popup
uicontrol('Parent',fig,'Style','text','Units','normalized','Position',[0.70 0.95 0.08 0.03], 'String','Filter:','HorizontalAlignment','right');
popup_filter = uicontrol('Parent',fig,'Style','popupmenu','Units','normalized','Position',[0.79 0.95 0.08 0.04], 'String',{'ekf','kf','ukf'}, 'Value',1, 'Callback',@(src,ev) setappdata(fig,'filter_type',src.String{src.Value}));
if isfield(params,'kf') && isfield(params.kf,'type')
	setappdata(fig,'filter_type',params.kf.type);
	vals = get(popup_filter,'String'); idx = find(strcmpi(vals, params.kf.type),1); if ~isempty(idx), set(popup_filter,'Value',idx); end
else
	setappdata(fig,'filter_type','ekf');
end

true_traj = state_prev(1:2)';
meas_traj = state_prev(1:2)';
est_traj = x_est(1:2)';

% --- read CSV once ---
T = readtable(params.data.file);
csvN = height(T);

for k=1:N
	% check figure closed
	if ~isvalid(fig)
		fprintf('Figure closed. Exiting loop.\n');
		break;
	end
	% if running flag is false, wait (pause) until resumed or figure closed
	while ~getappdata(fig,'running')
		if ~isvalid(fig), fprintf('Figure closed during pause. Exiting loop.\n'); break; end
		pause(0.1);
	end

	if k>csvN, break; end % CSV の終端
	% CSVモード: params.data.file の行 k を使う
	state_curr = [T.x(k); T.y(k); T.vx(k); T.vy(k)];
	meas.pos = [T.meas_pos_x(k), T.meas_pos_y(k)];
	meas.vel = [T.meas_vel_x(k), T.meas_vel_y(k)];
	% optional advanced sensors
	if ismember('accel3_x', T.Properties.VariableNames), meas.accel3 = [T.accel3_x(k), T.accel3_y(k), T.accel3_z(k)]; end
	if ismember('gyro3_x', T.Properties.VariableNames), meas.gyro3 = [T.gyro3_x(k), T.gyro3_y(k), T.gyro3_z(k)]; end
	if ismember('mag3_x', T.Properties.VariableNames), meas.mag3 = [T.mag3_x(k), T.mag3_y(k), T.mag3_z(k)]; end
	if ismember('gps_x', T.Properties.VariableNames), meas.gps = [T.gps_x(k), T.gps_y(k)]; end
	if ismember('baro', T.Properties.VariableNames), meas.baro = T.baro(k); end
	if ismember('meas_heading_x', T.Properties.VariableNames), meas.heading = [T.meas_heading_x(k), T.meas_heading_y(k)]; end

	% check runtime filter selection from UI
	if isappdata(fig,'filter_type')
		ft = getappdata(fig,'filter_type');
		switch lower(ft)
			case 'ekf', filter_step = @ekf_filter_step;
			case 'kf',  filter_step = @kf_filter_step;
			case 'ukf', filter_step = @ukf_filter_step;
			otherwise,  filter_step = @ekf_filter_step;
		end
	end

	% call filter with raw measurement
	[x_pred, P_pred, x_upd, P_upd, y, S, K, params] = filter_step(x_est, P, meas, params);

	% ログ更新
	true_traj(end+1,:) = state_curr(1:2)';
	if isfield(meas,'gps') && numel(meas.gps) >= 2 && all(~isnan(meas.gps))
		meas_traj(end+1,:) = meas.gps(:)';
	else
		meas_traj(end+1,:) = meas.pos;
	end
	est_traj(end+1,:) = x_upd(1:2)';

	% プロット更新
	set(h_true, 'XData', true_traj(:,1), 'YData', true_traj(:,2));
	set(h_meas, 'XData', meas_traj(:,1), 'YData', meas_traj(:,2));
	set(h_est, 'XData', est_traj(:,1), 'YData', est_traj(:,2));
	% update heading arrows
	if numel(state_curr) >= 4
		v_true = state_curr(3:4);
		if norm(v_true) > 1e-6, th_true = atan2(v_true(2), v_true(1)); end
	else
		th_true = 0;
	end
	v_est = x_upd(3:4);
	if norm(v_est) > 1e-6, th_est = atan2(v_est(2), v_est(1)); end
	set(h_true_head, 'XData', true_traj(end,1), 'YData', true_traj(end,2), 'UData', heading_scale*cos(th_true), 'VData', heading_scale*sin(th_true));
	set(h_est_head, 'XData', est_traj(end,1), 'YData', est_traj(end,2), 'UData', heading_scale*cos(th_est), 'VData', heading_scale*sin(th_est));

	% 状態更新
	state_prev = state_curr;
	x_est = x_upd;
	P = P_upd;

	drawnow limitrate;
end

fprintf('Realtime simulation finished.\n');

% local callback to toggle running state (Start <-> Stop)
function toggleRunning(btn, fig)
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

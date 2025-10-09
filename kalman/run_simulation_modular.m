% run_simulation_realtime.m
% CSVベースのリアルタイム風シミュレーション + 階層ブロック型カルマンフィルタ
clear; close all; clc

% ensure subfolders are on path so helper functions are found
root_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(root_dir,'GenerateData'));
addpath(fullfile(root_dir,'Graph'));
addpath(fullfile(root_dir,'KalmanFilter'));
% ensure the Common Calculations helpers are on the path
addpath(fullfile(root_dir,'KalmanFilter','Common Calculations'));

% パラメータは設定ファイルから読み込む
params = config_params();
N = floor(params.T/params.dt)+1;

% If user has not enabled split_mode (or set to 'none'), force 'by_rate'
if ~isfield(params,'kf')
	params.kf = struct();
end
if ~isfield(params.kf,'split_mode') || strcmpi(params.kf.split_mode,'none')
	params.kf.split_mode = 'by_rate';
end

% require CSV data source
if ~(isfield(params,'data') && isfield(params.data,'source') && strcmpi(params.data.source,'csv') && isfield(params.data,'file'))
	error('This simplified script only supports CSV data. Set params.data.source=''csv'' and params.data.file.');
end

% --- variable-dim state: infer n from params (fallback to 10 for compat) ---
% Determine state dimension n
if isfield(params,'kf') && isfield(params.kf,'x0') && ~isempty(params.kf.x0)
	n = numel(params.kf.x0);
elseif isfield(params,'initial_state') && ~isempty(params.initial_state)
	n = numel(params.initial_state);
else
	n = 10; % backward compatibility default
end

% initial state (true) and initial estimate x_est
if isfield(params,'initial_state') && numel(params.initial_state) >= 1
	state_prev = params.initial_state(:);
	if numel(state_prev) < n, state_prev(end+1:n) = 0; end
	if numel(state_prev) > n, state_prev = state_prev(1:n); end
else
	state_prev = zeros(n,1);
	if n >= 3, state_prev(3) = 1; end % default vx=1 if present
end

if isfield(params,'kf') && isfield(params.kf,'x0') && ~isempty(params.kf.x0)
	x_est = params.kf.x0(:);
	if numel(x_est) < n, x_est(end+1:n) = 0; end
	if numel(x_est) > n, x_est = x_est(1:n); end
else
	x_est = zeros(n,1);
	if n >= 3, x_est(3) = 1; end
end

% initial covariance P
if isfield(params,'kf') && isfield(params.kf,'P0') && isequal(size(params.kf.P0), [n n])
	P = params.kf.P0;
else
	% sensible default diagonal: [10,10,5,5,1,...]
	d = ones(n,1);
	if n >= 1, d(1) = 10; end
	if n >= 2, d(2) = 10; end
	if n >= 3, d(3) = 5; end
	if n >= 4, d(4) = 5; end
	if n > 4, d(5:end) = 1; end
	P = diag(d);
end

% 準備プロット
fig = figure('Name','Hierarchical Block Kalman Filter');
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis equal;
h_true = plot(ax, state_prev(1), state_prev(2), '-k','LineWidth',1.5, 'DisplayName','True');
% initialize GPS plot with NaNs; we will only append GPS points when position block runs
h_meas = plot(ax, NaN, NaN, '.r', 'DisplayName','GPS');
h_est = plot(ax, x_est(1), x_est(2), '-b','LineWidth',1.5, 'DisplayName','Block KF');
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

true_traj = state_prev(1:2)';
meas_traj = [NaN, NaN]; % will store GPS only when position block updates
est_traj = x_est(1:2)';

% --- ブロック更新履歴の初期化 ---
att_history = struct('x', {}, 'P', {}, 'step', {});  % 姿勢履歴
last_vel_update_step = 0;  % 速度ブロック最終更新ステップ
last_att_update_step = 0;  % 姿勢ブロック最終更新ステップ
last_pos_update_step = 0;  % 位置ブロック最終更新ステップ

% 診断情報
diagnostics = struct('vel', [], 'att', [], 'pos', []);

fprintf('Starting hierarchical block Kalman filter simulation...\n');

for k=1:N
	% check figure closed
	if ~isvalid(fig)
		break;
	end
	% if running flag is false, wait (pause) until resumed or figure closed
	while ~getappdata(fig,'running')
		if ~isvalid(fig), fprintf('Figure closed during pause. Exiting loop.\n'); break; end
		pause(0.1);
	end

	% === CSV読み取り ===
	[meas, state_curr, is_end] = csv_reader(params, k);
	if is_end
		break;
	end

	% === 階層ブロック更新（論理的順序: 姿勢→速度→位置） ===
	
	% 現在の状態をコピー
	x_curr = x_est;
	P_curr = P;
	
	% 1. 姿勢ブロック（中頻度）- 角速度・加速度による姿勢更新
	[x_curr, P_curr, diag_att] = attitude_block_filter(x_curr, P_curr, meas, params, k, last_att_update_step);
	if diag_att.updated
		last_att_update_step = k;
		diagnostics.att = [diagnostics.att, diag_att];
		
		% 姿勢履歴に追加（位置更新まで蓄積）
		att_count = numel(att_history);
		att_history(att_count+1).x = x_curr;
		att_history(att_count+1).P = P_curr;
		att_history(att_count+1).step = k;
	end
	
	% 2. 速度ブロック（高頻度）- 姿勢更新後の状態で速度を更新
	[x_curr, P_curr, diag_vel] = velocity_block_filter(x_curr, P_curr, meas, params, k, last_vel_update_step);
	if diag_vel.updated
		last_vel_update_step = k;
		diagnostics.vel = [diagnostics.vel, diag_vel];
	end
	
	% 3. 位置ブロック（低頻度）- 姿勢履歴と現在状態を統合して位置更新
	[x_curr, P_curr, diag_pos] = position_block_filter(x_curr, P_curr, meas, params, k, last_pos_update_step, att_history);
	if diag_pos.updated
		last_pos_update_step = k;
		diagnostics.pos = [diagnostics.pos, diag_pos];
		
		% 位置更新後は姿勢履歴をクリア
		att_history = struct('x', {}, 'P', {}, 'step', {});
	end
	
	% 最終状態を更新
	x_est = x_curr;
	P = P_curr;

	% === 軌跡記録 ===
	true_traj(end+1,:) = state_curr(1:2)';
	% GPS軌跡は「位置ブロックが更新されたときのみ」記録
	if exist('diag_pos','var') && isfield(diag_pos,'updated') && diag_pos.updated
		if isfield(meas,'gps') && numel(meas.gps) >= 2 && all(~isnan(meas.gps))
			meas_traj(end+1,:) = meas.gps(:)';
		else
			meas_traj(end+1,:) = meas.pos;
		end
	else
		% 位置更新でないステップではGPSはプロットしない（NaNで埋める）
		meas_traj(end+1,:) = [NaN, NaN];
	end
	est_traj(end+1,:) = x_est(1:2)';

	% === プロット更新 ===
	% warmup中はプロット更新をスキップ
	warmupActive = false;
	if isfield(params,'kf') && isfield(params.kf,'ema_warmup') && isfield(params.kf,'R_warmup_count')
		rc = params.kf.R_warmup_count;
		fn = fieldnames(rc);
		if ~isempty(fn)
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

	% グラフ更新は「位置ブロックが実行された時のみ」行う
	if ~warmupActive && exist('diag_pos','var') && isfield(diag_pos,'updated') && diag_pos.updated
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
		v_est = x_est(3:4);
		if norm(v_est) > 1e-6, th_est = atan2(v_est(2), v_est(1)); end
		set(h_true_head, 'XData', true_traj(end,1), 'YData', true_traj(end,2), 'UData', heading_scale*cos(th_true), 'VData', heading_scale*sin(th_true));
		set(h_est_head, 'XData', est_traj(end,1), 'YData', est_traj(end,2), 'UData', heading_scale*cos(th_est), 'VData', heading_scale*sin(th_est));
		drawnow limitrate;
	end
	
	% update state for next iteration
	state_prev = state_curr;
end

fprintf('Hierarchical block Kalman filter simulation finished.\n');
fprintf('Velocity updates: %d\n', numel(diagnostics.vel));
fprintf('Attitude updates: %d\n', numel(diagnostics.att));
fprintf('Position updates: %d\n', numel(diagnostics.pos));

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
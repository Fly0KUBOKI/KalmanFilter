function visualize_sim(t, true_state, meas, x_est, P_est, update_step)
% visualize_sim - Realtime-capable visualizer compatible with KF/ESKF outputs
% 引数:
%  t          - 時間ベクトル (Nx1)
%  true_state - 真値軌跡 (Nx2/Nx3 or 2xN/3xN)
%  meas       - 測定 (struct with .pos/.gps or numeric)
%  x_est      - 推定軌跡 (Nx2 or struct array of nominal ESKF states)
%  P_est      - 共分散履歴 (optional)
%  update_step- 描画ステップ（フレーム間隔）

% この実装は KF と ESKF の両方から渡される多様な形状に耐性を持ち,
% X/Y 成分のみをプロットします（3D が来ても XY を抽出）。

if nargin < 1 || isempty(t)
	error('visualize_sim: t must be provided');
end
N = numel(t);

% acknowledge optional P_est (may be unused for now) and reference it
if nargin < 5
	P_est = [];
end
% harmless reference to avoid linter complaining that the input is unused
if ~isempty(P_est)
	% P_est is provided (e.g. 10x10xN or 15x15xN). Not visualized currently.
	% Keep a benign local refererence so linters don't warn about unused input.
	tmp_P_est = P_est; %#ok<NASGU>
end

% sampling step
if nargin < 6 || isempty(update_step)
	step = max(1, round(N/200));
else
	step = max(1, floor(update_step));
end

% helper: coerce various input shapes to Nx2 double array (X,Y)
	function xy = to_xy(data, fallback)
		if nargin < 2, fallback = NaN(1,2); end
		xy = repmat(fallback, N, 1);
		if isempty(data), return; end
		if isstruct(data)
			if isfield(data,'pos')
				d = data.pos;
			elseif isfield(data,'gps')
				d = data.gps;
			else
				d = [];
			end
		else
			d = data;
		end
		if isempty(d), return; end
		d = double(d);
		[r,c] = size(d);
		if r == N && c >= 2
			xy = d(:,1:2);
			return;
		elseif c == N && r >= 2
			xy = d(1:2,:)';
			return;
		end
		% vector or shorter/longer matrices: try to adapt
		if isvector(d)
			v = d(:)';
			if numel(v) >= 2
				xy = repmat(v(1:2), N, 1);
			else
				xy = repmat([v(1) NaN], N, 1);
			end
			return;
		end
		if size(d,2) >= 2
			tmp = d(:,1:2);
			if size(tmp,1) == N
				xy = tmp; return;
			elseif size(tmp,1) < N
				xy = [tmp; repmat(tmp(end,:), N-size(tmp,1), 1)]; return;
			else
				xy = tmp(1:N,:); return;
			end
		end
		% fallback
		xy = repmat(fallback, N, 1);
	end

true_xy = to_xy(true_state, [0 0]);
meas_xy = to_xy(meas, nan(1,2));

% x_est may be array-like or struct array of nominal ESKF states
if isstruct(x_est) && isfield(x_est,'pos')
	% struct array: build Nx2 from .pos
	tmp = nan(N,2);
	% allow x_est to be length < N (repeat last) or length >= N
	L = numel(x_est);
	for ii = 1:N
		idx = min(ii, L);
		if isfield(x_est(idx),'pos')
			p = double(x_est(idx).pos(:))';
			if numel(p) >= 2, tmp(ii,:) = p(1:2);
			elseif numel(p) == 1, tmp(ii,:) = [p(1) NaN];
			end
		end
	end
	est_xy = tmp;
else
	est_xy = to_xy(x_est, [NaN NaN]);
end

% create figure and axes
fig = figure('Name','Realtime');
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
cla(ax);

% determine axis limits (guard NaN)
all_x = [true_xy(:,1); meas_xy(:,1); est_xy(:,1)];
all_y = [true_xy(:,2); meas_xy(:,2); est_xy(:,2)];
if all(isnan(all_x)), all_x = [0;1]; end
if all(isnan(all_y)), all_y = [0;1]; end
minx = min(all_x(~isnan(all_x))) - 5;
maxx = max(all_x(~isnan(all_x))) + 5;
miny = min(all_y(~isnan(all_y))) - 5;
maxy = max(all_y(~isnan(all_y))) + 5;
xlim(ax,[minx maxx]); ylim(ax,[miny maxy]);

% initial plot handles (use first non-NaN true point)
firstIdx = find(~isnan(true_xy(:,1)),1);
if isempty(firstIdx), firstIdx = 1; end
h_true_line = plot(ax, true_xy(firstIdx,1), true_xy(firstIdx,2), '-k','LineWidth',1.5);
h_meas_line = plot(ax, meas_xy(firstIdx,1), meas_xy(firstIdx,2), '.r');
h_est_line = plot(ax, est_xy(firstIdx,1), est_xy(firstIdx,2), '-b','LineWidth',1.5);

h_true = plot(ax, true_xy(firstIdx,1), true_xy(firstIdx,2), 'ok','MarkerFaceColor','k','MarkerSize',8);
h_meas = plot(ax, meas_xy(firstIdx,1), meas_xy(firstIdx,2), '.r','MarkerSize',8);
h_est = plot(ax, est_xy(firstIdx,1), est_xy(firstIdx,2), 'ob','MarkerFaceColor','b','MarkerSize',6);

% heading arrows
heading_scale = 1.0;
if N>1
	dt0 = t(min(2,end)) - t(1);
	if isempty(dt0) || dt0 <= 0, dt0 = 1; end
	v_true0 = (true_xy(min(2,size(true_xy,1)),:) - true_xy(1,:)) / dt0;
	v_est0 = (est_xy(min(2,size(est_xy,1)),:) - est_xy(1,:)) / dt0;
else
	v_true0 = [1 0]; v_est0 = [1 0];
end
if any(~isfinite(v_true0)), v_true0 = [1 0]; end
if any(~isfinite(v_est0)), v_est0 = [1 0]; end
th_true0 = atan2(v_true0(2), v_true0(1));
th_est0 = atan2(v_est0(2), v_est0(1));
h_true_head = quiver(ax, true_xy(firstIdx,1), true_xy(firstIdx,2), heading_scale*cos(th_true0), heading_scale*sin(th_true0), 0, 'Color','k', 'MaxHeadSize',0.5);
h_est_head = quiver(ax, est_xy(firstIdx,1), est_xy(firstIdx,2), heading_scale*cos(th_est0), heading_scale*sin(th_est0), 0, 'Color','b', 'MaxHeadSize',0.5);

legend(ax, [h_true_line, h_meas_line, h_est_line], {'True','Measured','Filter'}, 'Location','best');

% running toggle
setappdata(fig,'running',true);
uicontrol('Parent',fig,'Style','pushbutton','String','Stop','Units','normalized',... 
	'Position',[0.88 0.95 0.1 0.045], 'Callback',@(src,ev) toggleRunning(src,fig));

h_speed_text = uicontrol('Parent',fig,'Style','text','Units','normalized',... 
	'Position',[0.02 0.95 0.4 0.045],'HorizontalAlignment','left','String','');

% optional velocity figure
if N>1
	vel_fig = figure('Name','Velocity vectors');
	ax_vel = axes(vel_fig); hold(ax_vel,'on'); grid(ax_vel,'on'); axis(ax_vel,'equal');
	dt_all = diff(t); dt_all(dt_all<=0) = eps;
	v_true_all = sqrt(sum(diff(true_xy(:,1:2)).^2,2)) ./ dt_all;
	v_est_all = sqrt(sum(diff(est_xy(:,1:2)).^2,2)) ./ dt_all;
	maxv = max([v_true_all; v_est_all]);
	MAX_VEL_LIMIT = 50;
	vel_lim = max( min(maxv*1.2, MAX_VEL_LIMIT), 0.1 );
	xlim(ax_vel,[-vel_lim vel_lim]); ylim(ax_vel,[-vel_lim vel_lim]);
	title(ax_vel,'Velocity (origin)');
	h_vel_true = quiver(ax_vel, 0,0, v_true0(1), v_true0(2), 0, 'Color','k', 'MaxHeadSize',0.5, 'LineWidth',1.2);
	h_vel_est = quiver(ax_vel, 0,0, v_est0(1), v_est0(2), 0, 'Color','b', 'MaxHeadSize',0.5, 'LineWidth',1.2);
else
	h_vel_true = []; h_vel_est = [];
end

% animation loop
for k = 1:step:N
	if ~isvalid(fig), break; end
	while ~getappdata(fig,'running')
		if ~isvalid(fig), break; end
		pause(0.05);
	end

	tx = true_xy(k,1); ty = true_xy(k,2);
	mx = meas_xy(k,1); my = meas_xy(k,2);
	ex = est_xy(k,1); ey = est_xy(k,2);

	set(h_true_line, 'XData', [get(h_true_line,'XData') tx], 'YData', [get(h_true_line,'YData') ty]);
	set(h_meas_line, 'XData', [get(h_meas_line,'XData') mx], 'YData', [get(h_meas_line,'YData') my]);
	set(h_est_line, 'XData', [get(h_est_line,'XData') ex], 'YData', [get(h_est_line,'YData') ey]);

	set(h_true, 'XData', tx, 'YData', ty);
	set(h_meas, 'XData', mx, 'YData', my);
	set(h_est, 'XData', ex, 'YData', ey);

	if k > 1
		v_true = true_xy(k,1:2) - true_xy(k-1,1:2);
		v_est = est_xy(k,1:2) - est_xy(k-1,1:2);
		dt = t(k) - t(k-1);
	elseif N > 1
		v_true = true_xy(min(N,k+1),1:2) - true_xy(k,1:2);
		v_est = est_xy(min(N,k+1),1:2) - est_xy(k,1:2);
		dt = t(2) - t(1);
	else
		v_true = [1 0]; v_est = [1 0]; dt = 1;
	end
	if isempty(dt) || dt <= 0, dt = 1; end
	if norm(v_true) < 1e-6, v_true = [1 0]; end
	if norm(v_est) < 1e-6, v_est = [1 0]; end
	th_true = atan2(v_true(2), v_true(1));
	th_est = atan2(v_est(2), v_est(1));
	set(h_true_head, 'XData', true_xy(k,1), 'YData', true_xy(k,2), 'UData', heading_scale*cos(th_true), 'VData', heading_scale*sin(th_true));
	set(h_est_head, 'XData', est_xy(k,1), 'YData', est_xy(k,2), 'UData', heading_scale*cos(th_est), 'VData', heading_scale*sin(th_est));

	v_true_mag = norm(v_true) / dt;
	v_est_mag = norm(v_est) / dt;
	set(h_speed_text, 'String', sprintf('v_{true}: %.2f  v_{est}: %.2f', v_true_mag, v_est_mag));

	if ~isempty(h_vel_true) && ishandle(h_vel_true)
		v_true_vec = v_true / dt; v_est_vec = v_est / dt;
		if any(~isfinite(v_true_vec)), v_true_vec = [0 0]; end
		if any(~isfinite(v_est_vec)), v_est_vec = [0 0]; end
		set(h_vel_true, 'UData', v_true_vec(1), 'VData', v_true_vec(2));
		set(h_vel_est, 'UData', v_est_vec(1), 'VData', v_est_vec(2));
	end

	drawnow limitrate;
end

% nested callback
function toggleRunning(~, fig)
	if ~isvalid(fig), return; end
	running = getappdata(fig,'running');
	if isempty(running), running = true; end
	setappdata(fig,'running', ~running);
	% button label is not stored; rely on user to see state
end

end


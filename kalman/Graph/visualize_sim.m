function visualize_sim(t, true_state, meas, x_est, P_est, update_step)
% visualize_sim (ESKF-compatible)
% Robust visualization that accepts inputs from both the original KF demo and
% the ESKF demo. Handles meas.pos or meas.gps (3D), and x_est as NxM or as
% transposed 3xN arrays. Only the X/Y components are used for plotting.

% normalize inputs to Nx2 matrices (x,y)
N = numel(t);

% determine sampling step
if nargin < 6 || isempty(update_step)
    step = max(1, round(N/200));
else
    step = max(1, floor(update_step));
end

% helper: extract 2D trajectory from various possible shapes
    function xy = to_xy(data, fallback)
        % data can be: Nx2 numeric, Nx3 numeric, 3xN numeric, struct with .pos or .gps
        if nargin < 2, fallback = NaN(1,2); end
        xy = [];
        if isempty(data)
            xy = repmat(fallback, N, 1);
            return;
        end
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
        if isempty(d)
            xy = repmat(fallback, N, 1);
            return;
        end
        d = double(d);
        [r,c] = size(d);
        if r == N && c >= 2
            xy = d(:,1:2);
        elseif c == N && r >= 2
            % possibly 2xN or 3xN
            xy = d(1:2,:)';
        elseif r >= 2 && c >= 2 && r ~= N && c ~= N
            % ambiguous; try first two columns
            xy = d(:,1:2);
            % if lengths mismatch, pad/trim
            if size(xy,1) < N
                xy = [xy; repmat(xy(end,:), N-size(xy,1), 1)];
            elseif size(xy,1) > N
                xy = xy(1:N,:);
            end
        else
            xy = repmat(fallback, N, 1);
        end
    end

true_xy = to_xy(true_state, [0 0]);
meas_xy = to_xy(meas, nan(1,2));
est_xy = to_xy(x_est, [NaN NaN]);

% create realtime figure
fig = figure('Name','Realtime');
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis equal;
cla(ax);

% compute axis limits with margin and guard against all-NaN
all_x = [true_xy(:,1); meas_xy(:,1); est_xy(:,1)];
all_y = [true_xy(:,2); meas_xy(:,2); est_xy(:,2)];
if all(isnan(all_x)), all_x = [0;1]; end
if all(isnan(all_y)), all_y = [0;1]; end
minx = min(all_x(~isnan(all_x)))-5;
maxx = max(all_x(~isnan(all_x)))+5;
miny = min(all_y(~isnan(all_y)))-5;
maxy = max(all_y(~isnan(all_y)))+5;
xlim(ax,[minx maxx]); ylim(ax,[miny maxy]);

% initial plotted objects (use first valid point if possible)
firstIdx = find(~isnan(true_xy(:,1)),1);
if isempty(firstIdx), firstIdx = 1; end
h_true_line = plot(ax, true_xy(firstIdx,1), true_xy(firstIdx,2), '-k','LineWidth',1.5);
h_meas_line = plot(ax, meas_xy(firstIdx,1), meas_xy(firstIdx,2), '.r');
h_est_line = plot(ax, est_xy(firstIdx,1), est_xy(firstIdx,2), '-b','LineWidth',1.5);

% markers for current point
h_true = plot(ax, true_xy(firstIdx,1), true_xy(firstIdx,2), 'ok','MarkerFaceColor','k','MarkerSize',8);
h_meas = plot(ax, meas_xy(firstIdx,1), meas_xy(firstIdx,2), '.r','MarkerSize',8);
h_est = plot(ax, est_xy(firstIdx,1), est_xy(firstIdx,2), 'ob','MarkerFaceColor','b','MarkerSize',6);

% heading arrows
heading_scale = 1.0;
if N>1
    dt0 = t(min(2,end)) - t(1);
    if isempty(dt0) || dt0<=0, dt0 = 1; end
    v_true0 = (true_xy(min(2,size(true_xy,1)),:) - true_xy(1,:)) / dt0;
    v_est0 = (est_xy(min(2,size(est_xy,1)),:) - est_xy(1,:)) / dt0;
else
    v_true0 = [1 0]; v_est0 = [1 0];
end
if any(~isfinite(v_true0)), v_true0 = [1 0]; end
if any(~isfinite(v_est0)), v_est0 = [1 0]; end
th_true0 = atan2(v_true0(2), v_true0(1));
th_est0 = atan2(v_est0(2), v_est0(1));
h_true_head = quiver(ax, true_xy(firstIdx,1), true_xy(firstIdx,2), heading_scale*cos(th_true0), heading_scale*sin(th_true0), 0, 'Color','k', 'MaxHeadSize',0.5, 'DisplayName','True heading');
h_est_head = quiver(ax, est_xy(firstIdx,1), est_xy(firstIdx,2), heading_scale*cos(th_est0), heading_scale*sin(th_est0), 0, 'Color','b', 'MaxHeadSize',0.5, 'DisplayName','Est heading');

% explicit legend
legend(ax, [h_true_line, h_meas_line, h_est_line], {'True','Measured','Filter'}, 'Location','best');

% running toggle (Start/Stop)
setappdata(fig,'running',true);
btn_run = uicontrol('Parent',fig,'Style','pushbutton','String','Stop','Units','normalized',... 
    'Position',[0.88 0.95 0.1 0.045], 'Callback',@(src,ev) toggleRunning(src,fig));

% speed display (minimal UI)
h_speed_text = uicontrol('Parent',fig,'Style','text','Units','normalized',... 
    'Position',[0.02 0.95 0.4 0.045],'HorizontalAlignment','left','String','');

% Animation loop
for k=1:step:N
    if ~isvalid(fig), break; end
    while ~getappdata(fig,'running')
        if ~isvalid(fig), break; end
        pause(0.05);
    end

    % append to trajectory lines (protect NaNs)
    tx = true_xy(k,1); ty = true_xy(k,2);
    mx = meas_xy(k,1); my = meas_xy(k,2);
    ex = est_xy(k,1); ey = est_xy(k,2);

    xdata = get(h_true_line,'XData'); ydata = get(h_true_line,'YData');
    set(h_true_line, 'XData', [xdata tx], 'YData', [ydata ty]);
    xm = get(h_meas_line,'XData'); ym = get(h_meas_line,'YData');
    set(h_meas_line, 'XData', [xm mx], 'YData', [ym my]);
    xe = get(h_est_line,'XData'); ye = get(h_est_line,'YData');
    set(h_est_line, 'XData', [xe ex], 'YData', [ye ey]);

    set(h_true, 'XData', tx, 'YData', ty);
    set(h_meas, 'XData', mx, 'YData', my);
    set(h_est, 'XData', ex, 'YData', ey);

    % headings
    if k>1
        v_true = true_xy(k,1:2) - true_xy(k-1,1:2);
        v_est = est_xy(k,1:2) - est_xy(k-1,1:2);
        dt = t(k) - t(k-1);
    elseif N>1
        v_true = true_xy(min(N,k+1),1:2) - true_xy(k,1:2);
        v_est = est_xy(min(N,k+1),1:2) - est_xy(k,1:2);
        dt = t(2) - t(1);
    else
        v_true = [1 0]; v_est = [1 0]; dt = 1;
    end
    if isempty(dt) || dt<=0, dt = 1; end
    if norm(v_true) < 1e-6, v_true = [1 0]; end
    if norm(v_est) < 1e-6, v_est = [1 0]; end
    th_true = atan2(v_true(2), v_true(1));
    th_est = atan2(v_est(2), v_est(1));
    set(h_true_head, 'XData', true_xy(k,1), 'YData', true_xy(k,2), 'UData', heading_scale*cos(th_true), 'VData', heading_scale*sin(th_true));
    set(h_est_head, 'XData', est_xy(k,1), 'YData', est_xy(k,2), 'UData', heading_scale*cos(th_est), 'VData', heading_scale*sin(th_est));

    % speeds
    v_true_mag = norm(v_true)/dt;
    v_est_mag = norm(v_est)/dt;
    set(h_speed_text, 'String', sprintf('v_{true}: %.2f  v_{est}: %.2f', v_true_mag, v_est_mag));

    drawnow limitrate;
end

% local callback
function toggleRunning(btn, fig)
    if ~isvalid(fig) || ~isvalid(btn), return; end
    running = getappdata(fig,'running');
    if isempty(running), running = true; end
    new_state = ~running;
    setappdata(fig,'running', new_state);
    if new_state
        set(btn, 'String', 'Stop');
    else
        set(btn, 'String', 'Start');
    end
end
end
function visualize_sim(t, true_state, meas, x_est, P_est, update_step)
% Realtime-only visualization: trajectory lines + measurements + estimated path
N = numel(t);

% determine sampling step: if update_step provided and positive use it,
% otherwise fallback to previous behavior (~200 frames max)
if nargin < 6 || isempty(update_step)
    step = max(1, round(N/200));
else
    step = max(1, floor(update_step));
end

% create realtime figure
fig = figure('Name','Realtime');
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis equal;
cla(ax);

% compute reasonable axis limits with margin
minx = min([true_state(:,1); meas.pos(:,1); x_est(:,1)])-5;
maxx = max([true_state(:,1); meas.pos(:,1); x_est(:,1)])+5;
miny = min([true_state(:,2); meas.pos(:,2); x_est(:,2)])-5;
maxy = max([true_state(:,2); meas.pos(:,2); x_est(:,2)])+5;
xlim(ax,[minx maxx]); ylim(ax,[miny maxy]);

% initial plotted objects: lines for trajectories (start at first point)
h_true_line = plot(ax, true_state(1,1), true_state(1,2), '-k','LineWidth',1.5);
h_meas_line = plot(ax, meas.pos(1,1), meas.pos(1,2), '.r');
h_est_line = plot(ax, x_est(1,1), x_est(1,2), '-b','LineWidth',1.5);

% markers for current point
h_true = plot(ax, true_state(1,1), true_state(1,2), 'ok','MarkerFaceColor','k','MarkerSize',8);
h_meas = plot(ax, meas.pos(1,1), meas.pos(1,2), '.r','MarkerSize',8);
h_est = plot(ax, x_est(1,1), x_est(1,2), 'ob','MarkerFaceColor','b','MarkerSize',6);

% heading arrows
heading_scale = 1.0;
if N>1
    % use time delta to convert position difference to velocity (units/sec)
    dt0 = t(2) - t(1);
    if isempty(dt0) || dt0<=0, dt0 = 1; end
    v_true0 = (true_state(2,1:2) - true_state(1,1:2)) / dt0;
    v_est0 = (x_est(2,1:2) - x_est(1,1:2)) / dt0;
else
    dt0 = 1;
    v_true0 = [1 0]; v_est0 = [1 0];
end
th_true0 = atan2(v_true0(2), v_true0(1));
th_est0 = atan2(v_est0(2), v_est0(1));
h_true_head = quiver(ax, true_state(1,1), true_state(1,2), heading_scale*cos(th_true0), heading_scale*sin(th_true0), 0, 'Color','k', 'MaxHeadSize',0.5, 'DisplayName','True heading');
h_est_head = quiver(ax, x_est(1,1), x_est(1,2), heading_scale*cos(th_est0), heading_scale*sin(th_est0), 0, 'Color','b', 'MaxHeadSize',0.5, 'DisplayName','Est heading');

% explicit legend for the main three series to avoid default 'data 1/2/3' labels
legend(ax, [h_true_line, h_meas_line, h_est_line], {'True','Measured','Filter'}, 'Location','best');

% running toggle (Start/Stop)
setappdata(fig,'running',true);
btn_run = uicontrol('Parent',fig,'Style','pushbutton','String','Stop','Units','normalized',... 
    'Position',[0.88 0.95 0.1 0.045], 'Callback',@(src,ev) toggleRunning(src,fig));

% speed display (minimal UI): will show true and estimated speed (units per second)
h_speed_text = uicontrol('Parent',fig,'Style','text','Units','normalized',... 
    'Position',[0.02 0.95 0.4 0.045],'HorizontalAlignment','left','String','');

% velocity-vector: create a separate figure to show origin-based velocity vectors
vel_fig = figure('Name','Velocity vectors');
ax_vel = axes(vel_fig); hold(ax_vel,'on'); grid(ax_vel,'on'); axis(ax_vel,'equal');
% compute reasonable limit from data
if N>1
    dt_all = diff(t);
    dt_all(dt_all<=0) = eps;
    v_true_all = sqrt(sum(diff(true_state(:,1:2)).^2,2)) ./ dt_all;
    v_est_all = sqrt(sum(diff(x_est(:,1:2)).^2,2)) ./ dt_all;
    maxv = max([v_true_all; v_est_all]);
else
    maxv = 1;
end
% prevent extremely large velocity axes (e.g., numeric outliers or unit mismatch)
% choose a reasonable maximum display limit (m/s). Keep small floor too.
MAX_VEL_LIMIT = 50; % max axis range magnitude (change if your data uses different units)
vel_lim = max( min(maxv*1.2, MAX_VEL_LIMIT), 0.1 );
xlim(ax_vel,[-vel_lim vel_lim]); ylim(ax_vel,[-vel_lim vel_lim]);
title(ax_vel,'Velocity (origin)');
% initial velocity quivers from origin (on the new figure)
h_vel_true = quiver(ax_vel, 0,0, v_true0(1), v_true0(2), 0, 'Color','k', 'MaxHeadSize',0.5, 'LineWidth',1.2);
h_vel_est = quiver(ax_vel, 0,0, v_est0(1), v_est0(2), 0, 'Color','b', 'MaxHeadSize',0.5, 'LineWidth',1.2);

% Animation loop: step through data and update lines/arrows
for k=1:step:N
    if ~isvalid(fig), break; end
    % if running flag is false, wait until resumed
    while ~getappdata(fig,'running')
        if ~isvalid(fig), break; end
        pause(0.05);
    end

    % append to trajectory lines
    xdata = get(h_true_line,'XData'); ydata = get(h_true_line,'YData');
    set(h_true_line, 'XData', [xdata true_state(k,1)], 'YData', [ydata true_state(k,2)]);
    xm = get(h_meas_line,'XData'); ym = get(h_meas_line,'YData');
    set(h_meas_line, 'XData', [xm meas.pos(k,1)], 'YData', [ym meas.pos(k,2)]);
    xe = get(h_est_line,'XData'); ye = get(h_est_line,'YData');
    set(h_est_line, 'XData', [xe x_est(k,1)], 'YData', [ye x_est(k,2)]);

    % update current-point markers
    set(h_true, 'XData', true_state(k,1), 'YData', true_state(k,2));
    set(h_meas, 'XData', meas.pos(k,1), 'YData', meas.pos(k,2));
    set(h_est, 'XData', x_est(k,1), 'YData', x_est(k,2));

    % compute and update headings
    if k>1
        v_true = true_state(k,1:2) - true_state(k-1,1:2);
        v_est = x_est(k,1:2) - x_est(k-1,1:2);
    elseif N>1
        v_true = true_state(min(N,k+1),1:2) - true_state(k,1:2);
        v_est = x_est(min(N,k+1),1:2) - x_est(k,1:2);
    else
        v_true = [1 0]; v_est = [1 0];
    end
    if norm(v_true) < 1e-6, v_true = [1 0]; end
    if norm(v_est) < 1e-6, v_est = [1 0]; end
    th_true = atan2(v_true(2), v_true(1));
    th_est = atan2(v_est(2), v_est(1));
    set(h_true_head, 'XData', true_state(k,1), 'YData', true_state(k,2), 'UData', heading_scale*cos(th_true), 'VData', heading_scale*sin(th_true));
    set(h_est_head, 'XData', x_est(k,1), 'YData', x_est(k,2), 'UData', heading_scale*cos(th_est), 'VData', heading_scale*sin(th_est));

    % compute and display speeds (convert position delta to per-second using t)
    if k>1
        dt = t(k) - t(k-1);
    elseif N>1
        dt = t(2) - t(1);
    else
        dt = 1;
    end
    if isempty(dt) || dt <= 0, dt = 1; end
    v_true_mag = norm(v_true) / dt;
    v_est_mag = norm(v_est) / dt;
    set(h_speed_text, 'String', sprintf('v_{true}: %.2f  v_{est}: %.2f', v_true_mag, v_est_mag));

    % update origin-based velocity quivers (show vector components)
    v_true_vec = v_true / dt;
    v_est_vec = v_est / dt;
    % protect against NaN/Inf
    if any(~isfinite(v_true_vec)), v_true_vec = [0 0]; end
    if any(~isfinite(v_est_vec)), v_est_vec = [0 0]; end
    set(h_vel_true, 'UData', v_true_vec(1), 'VData', v_true_vec(2));
    set(h_vel_est, 'UData', v_est_vec(1), 'VData', v_est_vec(2));

    drawnow limitrate;
end

% local callback to toggle running state (Start <-> Stop)
function toggleRunning(btn, fig)
    if ~isvalid(fig) || ~isvalid(btn), return; end
    running = getappdata(fig,'running');
    if isempty(running), running = true; end
    new_state = ~running;
    setappdata(fig,'running', new_state);
    if new_state
        set(btn, 'String', 'Stop');
    else
        set(btn, 'String', 'Start');
    end
end
end

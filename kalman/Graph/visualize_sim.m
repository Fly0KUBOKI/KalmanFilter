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
    v_true0 = true_state(2,1:2) - true_state(1,1:2);
    v_est0 = x_est(2,1:2) - x_est(1,1:2);
else
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

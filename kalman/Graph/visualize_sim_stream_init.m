function h = visualize_sim_stream_init(t0, true0, meas0, est0)
% Initialize a lightweight streaming visualizer used by run_simulation_realtime
% Returns a struct h with fields used by visualize_sim_stream_update

if nargin < 1 || isempty(t0)
    t0 = 0;
end
if nargin < 2 || isempty(true0)
    true0 = [0 0];
end
if nargin < 3 || isempty(meas0)
    meas0 = [NaN NaN];
end
if nargin < 4 || isempty(est0)
    est0 = [NaN NaN];
end


fig = figure('Name','Realtime Stream','NumberTitle','off');
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');

% initial plot handles
h_true_line = plot(ax, true0(1), true0(2), '-k','LineWidth',1.5);
h_meas_line = plot(ax, meas0(1), meas0(2), '.r');
h_est_line = plot(ax, est0(1), est0(2), '-b','LineWidth',1.5);

h_true = plot(ax, true0(1), true0(2), 'ok','MarkerFaceColor','k','MarkerSize',8);
h_meas = plot(ax, meas0(1), meas0(2), '.r','MarkerSize',8);
h_est = plot(ax, est0(1), est0(2), 'ob','MarkerFaceColor','b','MarkerSize',6);

% heading scale (used for heading vectors display)
heading_scale = 1.0; % unit length for heading arrows

% create separate figure to display heading (yaw) vectors from origin
yaw_fig = figure('Name','Heading vectors','NumberTitle','off');
ax_yaw = axes(yaw_fig); hold(ax_yaw,'on'); grid(ax_yaw,'on'); axis(ax_yaw,'equal');
% initial yaw vectors from origin (pointing along +x)
h_yaw_true = quiver(ax_yaw, 0, 0, heading_scale, 0, 0, 'Color','k', 'MaxHeadSize',0.6, 'LineWidth',1.2);
h_yaw_est = quiver(ax_yaw, 0, 0, heading_scale, 0, 0, 'Color','b', 'MaxHeadSize',0.6, 'LineWidth',1.2);
% fixed display limits for yaw plot (unit circle)
YLIM = 1;
xlim(ax_yaw,[-YLIM YLIM]); ylim(ax_yaw,[-YLIM YLIM]);
title(ax_yaw,'Heading (origin)');

% small UI control to pause/resume
% integrate with run_simulation_realtime which expects 'isPaused' appdata
setappdata(fig,'running',true);
setappdata(fig,'isPaused',false);
btn = uicontrol('Parent',fig,'Style','pushbutton','String','Stop','Units','normalized',... 
    'Position',[0.88 0.95 0.1 0.045], 'Callback',@(src,ev) toggleRunning(src,fig));
setappdata(fig,'pause_btn',btn);

% create velocity vector figure with origin-based quivers
vel_fig = figure('Name','Velocity vectors','NumberTitle','off');
ax_vel = axes(vel_fig); hold(ax_vel,'on'); grid(ax_vel,'on'); axis(ax_vel,'equal');
% initial velocity vectors from origin
v_true0 = [0 0]; v_est0 = [0 0];
h_vel_true = quiver(ax_vel, 0,0, v_true0(1), v_true0(2), 0, 'Color','k', 'MaxHeadSize',0.5, 'LineWidth',1.2);
h_vel_est = quiver(ax_vel, 0,0, v_est0(1), v_est0(2), 0, 'Color','b', 'MaxHeadSize',0.5, 'LineWidth',1.2);
% fixed velocity scale: 1 m (axis range [-1,1])
VEL_LIMIT = 1; % meters per second displayed as unit length
xlim(ax_vel,[-VEL_LIMIT VEL_LIMIT]); ylim(ax_vel,[-VEL_LIMIT VEL_LIMIT]);
title(ax_vel,'Velocity (origin)');

% return handle structure
h.fig = fig;
h.ax = ax;
h.h_true_line = h_true_line;
h.h_meas_line = h_meas_line;
h.h_est_line = h_est_line;
h.h_true = h_true;
h.h_meas = h_meas;
h.h_est = h_est;
% velocity handles
h.h_vel_true = h_vel_true;
h.h_vel_est = h_vel_est;
% yaw handles
% store yaw figure handles
h.h_yaw_true = h_yaw_true;
h.h_yaw_est = h_yaw_est;
h.ax_yaw = ax_yaw;
% store heading scale for update function
h.heading_scale = heading_scale;
% store velocity axes for auto-scaling
h.ax_vel = ax_vel;

% store last samples for velocity calc
% store only x,y for last sample (visualizer uses 2D positions)
if numel(true0) >= 2
    setappdata(fig,'last_true',double(true0(1:2)));
else
    setappdata(fig,'last_true',double([true0(:)'; NaN]));
end
if numel(est0) >= 2
    setappdata(fig,'last_est',double(est0(1:2)));
else
    setappdata(fig,'last_est',double([est0(:)'; NaN]));
end
setappdata(fig,'last_stream_t',t0);
% store initial time (benign reference to avoid lint warning)
setappdata(fig,'t0',t0);

end

function toggleRunning(~, fig)
    if ~isvalid(fig), return; end
    running = getappdata(fig,'running');
    if isempty(running), running = true; end
    new_state = ~running;
    setappdata(fig,'running', new_state);
    % align with run_simulation_realtime's pause flag
    setappdata(fig,'isPaused', ~new_state);
    % update button label if stored
    try
        btn = getappdata(fig,'pause_btn');
        if ishandle(btn)
            if new_state
                set(btn,'String','Stop');
            else
                set(btn,'String','Start');
            end
        end
    catch
    end
end

function visualize_sim_stream_update(h, t, true_pt, meas_pt, est_pt, true_yaw, est_yaw)
% Append a single sample to streaming visualizer created by visualize_sim_stream_init
% h: handle struct from init
if nargin < 1 || isempty(h) || ~isstruct(h) || ~isfield(h,'fig') || ~ishandle(h.fig)
    return;
end
if nargin < 2 || isempty(t), t = []; end
if nargin < 3, true_pt = [NaN NaN]; end
if nargin < 4, meas_pt = [NaN NaN]; end
if nargin < 5, est_pt = [NaN NaN]; end
% accept 2D or 3D points; use only x,y for plotting
if numel(true_pt) >= 3
    true_pt_xy = double(true_pt(1:2));
else
    true_pt_xy = double(true_pt(:)');
end
if numel(meas_pt) >= 3
    meas_pt_xy = double(meas_pt(1:2));
else
    meas_pt_xy = double(meas_pt(:)');
end
if numel(est_pt) >= 3
    est_pt_xy = double(est_pt(1:2));
else
    est_pt_xy = double(est_pt(:)');
end

fig = h.fig;
if ~isvalid(fig), return; end

% benignly store last timestamp so variable is 'used' (lint friendly)
if ~isempty(t)
    setappdata(fig,'last_stream_t',t);
end

% if user paused via control, do not update
% respect global pause flag used by run_simulation_realtime
isPaused = getappdata(fig,'isPaused');
if ~isempty(isPaused) && all(isPaused)
    return;
end

running = getappdata(fig,'running');
if ~isempty(running) && ~all(running)
    return;
end

% coerce to 1x2 double
true_pt = true_pt_xy; if numel(true_pt) < 2, true_pt(2) = NaN; end
meas_pt = meas_pt_xy; if numel(meas_pt) < 2, meas_pt(2) = NaN; end
est_pt = est_pt_xy; if numel(est_pt) < 2, est_pt(2) = NaN; end

% append to lines
try
    set(h.h_true_line, 'XData', [get(h.h_true_line,'XData') true_pt(1)], 'YData', [get(h.h_true_line,'YData') true_pt(2)]);
    set(h.h_meas_line, 'XData', [get(h.h_meas_line,'XData') meas_pt(1)], 'YData', [get(h.h_meas_line,'YData') meas_pt(2)]);
    set(h.h_est_line, 'XData', [get(h.h_est_line,'XData') est_pt(1)], 'YData', [get(h.h_est_line,'YData') est_pt(2)]);

    set(h.h_true, 'XData', true_pt(1), 'YData', true_pt(2));
    set(h.h_meas, 'XData', meas_pt(1), 'YData', meas_pt(2));
    set(h.h_est, 'XData', est_pt(1), 'YData', est_pt(2));

    % update origin-based velocity quivers if available
    try
        last_true = getappdata(fig,'last_true');
        last_est = getappdata(fig,'last_est');
        last_t = getappdata(fig,'last_stream_t');
        if isempty(last_true), last_true = true_pt; end
        if isempty(last_est), last_est = est_pt; end
        if isempty(last_t) || isempty(t)
            dt = 1;
        else
            dt = t - last_t;
        end
        if isempty(dt) || dt <= 0, dt = 1; end
        v_true = (true_pt - last_true) / dt;
        v_est = (est_pt - last_est) / dt;
        % protect
        if any(~isfinite(v_true)), v_true = [0 0]; end
        if any(~isfinite(v_est)), v_est = [0 0]; end
        % convert to speed-norm + heading direction when yaw available
        speed_true = norm(v_true);
        speed_est = norm(v_est);
        % true velocity vector: if true_yaw provided, use that direction
        if isfield(h,'h_vel_true') && ishandle(h.h_vel_true)
            if nargin >= 6 && ~isempty(true_yaw)
                tht = double(true_yaw);
                vtx = speed_true * cos(tht);
                vty = speed_true * sin(tht);
            else
                vtx = v_true(1);
                vty = v_true(2);
            end
            set(h.h_vel_true, 'UData', vtx, 'VData', vty);
        end
        % est velocity vector: if est_yaw provided, use that direction
        if isfield(h,'h_vel_est') && ishandle(h.h_vel_est)
            if nargin >= 7 && ~isempty(est_yaw)
                the = double(est_yaw);
                vex = speed_est * cos(the);
                vey = speed_est * sin(the);
            else
                vex = v_est(1);
                vey = v_est(2);
            end
            set(h.h_vel_est, 'UData', vex, 'VData', vey);
        end
        % update yaw quivers if provided
            % heading scale read from handle struct
            if isfield(h,'heading_scale')
                hs = h.heading_scale;
            else
                hs = 1.0;
            end
            % update yaw quivers in separate yaw figure (origin-based)
            if nargin >= 6 && ~isempty(true_yaw) && isfield(h,'h_yaw_true') && ishandle(h.h_yaw_true)
                th = double(true_yaw);
                u = hs * cos(th);
                v = hs * sin(th);
                set(h.h_yaw_true, 'UData', u, 'VData', v);
            end
            if nargin >= 7 && ~isempty(est_yaw) && isfield(h,'h_yaw_est') && ishandle(h.h_yaw_est)
                th = double(est_yaw);
                u = hs * cos(th);
                v = hs * sin(th);
                set(h.h_yaw_est, 'UData', u, 'VData', v);
            end
        % velocity axes are fixed (1 m scale), so no auto-scaling here
        % store last
        setappdata(fig,'last_true',true_pt);
        setappdata(fig,'last_est',est_pt);
        setappdata(fig,'last_stream_t',t);
    catch
        % ignore if velocity fig closed
    end
catch
    % if something failed (handles closed), ignore silently
    return;
end

drawnow limitrate;

end

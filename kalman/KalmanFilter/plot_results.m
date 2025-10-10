function plot_results(vel_results, att_results, pos_results, truth_data, current_idx, plot_opts)
% PLOT_RESULTS - リアルタイムプロット更新
% 位置推定完了時に各ブロックの結果を個別プロット

persistent fig_vel fig_att fig_pos


% 図の初期化（有効なものだけ）
if isfield(plot_opts,'enable_velocity') && plot_opts.enable_velocity
    if isempty(fig_vel) || ~any(isgraphics(fig_vel))
        fig_vel = figure('Name', '速度推定結果', 'Position', [100, 600, 800, 400], 'WindowStyle', 'docked');
    end
end
if isfield(plot_opts,'enable_attitude') && plot_opts.enable_attitude
    if isempty(fig_att) || ~any(isgraphics(fig_att))
        fig_att = figure('Name', '姿勢推定結果', 'Position', [100, 200, 800, 400], 'WindowStyle', 'docked');
    end
end
if isfield(plot_opts,'enable_position') && plot_opts.enable_position
    if isempty(fig_pos) || ~any(isgraphics(fig_pos))
        fig_pos = figure('Name', '位置推定結果', 'Position', [100, 50, 800, 600], 'WindowStyle', 'docked');
    end
end

% データ範囲の決定
max_display = min(current_idx * 40, length(truth_data.t));
t_range = 1:max_display;

% 速度プロット
if nargin < 6 || ~isstruct(plot_opts)
    plot_opts = struct('enable_velocity', true, 'enable_attitude', true, 'enable_position', true);
end


% 速度プロット
if isfield(plot_opts,'enable_velocity') && plot_opts.enable_velocity
    if any(isgraphics(fig_vel))
        set(0, 'CurrentFigure', fig_vel);
    end
    clf(fig_vel);
    plot_velocity_vectors(vel_results, truth_data, t_range);
elseif isfield(plot_opts,'enable_velocity') && ~plot_opts.enable_velocity && any(isgraphics(fig_vel))
    close(fig_vel);
    fig_vel = [];
end

% 姿勢プロット
if isfield(plot_opts,'enable_attitude') && plot_opts.enable_attitude
    if any(isgraphics(fig_att))
        set(0, 'CurrentFigure', fig_att);
    end
    clf(fig_att);
    plot_attitude_vectors(att_results, truth_data, t_range);
elseif isfield(plot_opts,'enable_attitude') && ~plot_opts.enable_attitude && any(isgraphics(fig_att))
    close(fig_att);
    fig_att = [];
end

% 位置プロット
if isfield(plot_opts,'enable_position') && plot_opts.enable_position
    if any(isgraphics(fig_pos))
        set(0, 'CurrentFigure', fig_pos);
    end
    clf(fig_pos);
    plot_position_trajectory(pos_results, truth_data, current_idx);
elseif isfield(plot_opts,'enable_position') && ~plot_opts.enable_position && any(isgraphics(fig_pos))
    close(fig_pos);
    fig_pos = [];
end

drawnow;

end

function plot_velocity_vectors(vel_results, truth_data, t_range)
% 速度ベクトルプロット（原点から）

subplot(1, 2, 1);
hold on;
title('速度ベクトル (XY平面)');
xlabel('Vx [m/s]');
ylabel('Vy [m/s]');
grid on;
axis equal;

% データ収集
truth_vel = [];
pred_vel = [];
est_vel = [];

for i = t_range
    if i <= length(truth_data.vel)
        truth_vel = [truth_vel; truth_data.vel(i, 1:2)];
    end
    
    if ~isempty(vel_results{i})
        pred_vel = [pred_vel; vel_results{i}.x_pred(1:2)'];
        est_vel = [est_vel; vel_results{i}.velocity(1:2)'];
    end
end

% プロット（最新の値のみ）
        h = gobjects(0);
        labels = {};
        if ~isempty(truth_vel)
            h(end+1) = quiver(0, 0, truth_vel(end, 1), truth_vel(end, 2), 'k-', 'LineWidth', 2, 'MaxHeadSize', 0.5);
            labels{end+1} = '真値';
        end
        if ~isempty(pred_vel)
            h(end+1) = quiver(0, 0, pred_vel(end, 1), pred_vel(end, 2), 'g--', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
            labels{end+1} = '予測値';
        end
        if ~isempty(est_vel)
            h(end+1) = quiver(0, 0, est_vel(end, 1), est_vel(end, 2), 'b-', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
            labels{end+1} = '推定値';
        end
        if ~isempty(h)
            legend(h, labels, 'Location', 'best');
        end

% Z方向速度の時系列
subplot(1, 2, 2);
hold on;
title('Z方向速度');
xlabel('時間 [s]');
ylabel('Vz [m/s]');
grid on;

if ~isempty(truth_vel) && size(truth_data.vel, 2) >= 3
    t_plot = truth_data.t(t_range);
    plot(t_plot, truth_data.vel(t_range, 3), 'k-', 'LineWidth', 2);
end

if ~isempty(est_vel)
    t_est = [];
    vz_est = [];
    for i = t_range
        if ~isempty(vel_results{i})
            t_est = [t_est; truth_data.t(i)];
            vz_est = [vz_est; vel_results{i}.velocity(3)];
        end
    end
    if ~isempty(t_est)
        plot(t_est, vz_est, 'b-', 'LineWidth', 1.5);
    end
end

    h1 = gobjects(0);
    if ~isempty(truth_vel) && size(truth_data.vel, 2) >= 3
        t_plot = truth_data.t(t_range);
        h1 = plot(t_plot, truth_data.vel(t_range, 3), 'k-', 'LineWidth', 2);
    end

    h2 = gobjects(0);
    if ~isempty(est_vel)
        t_est = [];
        vz_est = [];
        for i = t_range
            if ~isempty(vel_results{i})
                t_est = [t_est; truth_data.t(i)];
                vz_est = [vz_est; vel_results{i}.velocity(3)];
            end
        end
        if ~isempty(t_est)
            h2 = plot(t_est, vz_est, 'b-', 'LineWidth', 1.5);
        end
    end

    h_leg = gobjects(0);
    labels_leg = {};
    if ~isempty(h1)
        h_leg(end+1) = h1; labels_leg{end+1} = '真値'; end
    if ~isempty(h2)
        h_leg(end+1) = h2; labels_leg{end+1} = '推定値'; end
    if ~isempty(h_leg)
        legend(h_leg, labels_leg, 'Location', 'best');
    end

end

function plot_attitude_vectors(att_results, truth_data, t_range)
% 姿勢ベクトルプロット（原点から）

subplot(1, 3, 1:2);
hold on;
title('姿勢ベクトル (Roll-Pitch平面)');
xlabel('Roll [rad]');
ylabel('Pitch [rad]');
grid on;
axis equal;

% データ収集（簡略化）
if ~isempty(att_results)
    att_idx = min(length(att_results), ceil(max(t_range) / 4));
    if att_idx > 0 && ~isempty(att_results{att_idx})
        % 最新の姿勢推定値
        att_est = att_results{att_idx}.attitude;
        quiver(0, 0, att_est(1), att_est(2), 'b-', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
        
        % 予測値
        att_pred = att_results{att_idx}.x_pred(1:2);
        quiver(0, 0, att_pred(1), att_pred(2), 'g--', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    end
end

% 真値（簡略化）
if max(t_range) <= length(truth_data.theta)
    theta_true = truth_data.theta(max(t_range));
    quiver(0, 0, 0, theta_true, 'k-', 'LineWidth', 2, 'MaxHeadSize', 0.5);
end

        h = gobjects(0);
        labels = {};
        if ~isempty(att_results)
            att_idx = min(length(att_results), ceil(max(t_range) / 4));
            if att_idx > 0 && ~isempty(att_results{att_idx})
                % 最新の姿勢推定値
                att_est = att_results{att_idx}.attitude;
                h(end+1) = quiver(0, 0, att_est(1), att_est(2), 'b-', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
                labels{end+1} = '推定値';
                % 予測値
                att_pred = att_results{att_idx}.x_pred(1:2);
                h(end+1) = quiver(0, 0, att_pred(1), att_pred(2), 'g--', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
                labels{end+1} = '予測値';
            end
        end

        % 真値（簡略化）
        if max(t_range) <= length(truth_data.theta)
            theta_true = truth_data.theta(max(t_range));
            h(end+1) = quiver(0, 0, 0, theta_true, 'k-', 'LineWidth', 2, 'MaxHeadSize', 0.5);
            labels{end+1} = '真値';
        end

        if ~isempty(h)
            legend(h, labels, 'Location', 'best');
        end

% Yaw角の時系列
subplot(1, 3, 3);
hold on;
title('Yaw角');
xlabel('時間 [s]');
ylabel('Yaw [rad]');
grid on;

if max(t_range) <= length(truth_data.theta)
    t_plot = truth_data.t(t_range);
    plot(t_plot, truth_data.theta(t_range), 'k-', 'LineWidth', 2);
end

if ~isempty(att_results)
    t_att = [];
    yaw_att = [];
    for i = 1:length(att_results)
        if ~isempty(att_results{i})
            t_att = [t_att; att_results{i}.timestamp];
            yaw_att = [yaw_att; att_results{i}.attitude(3)];
        end
    end
    if ~isempty(t_att)
        plot(t_att, yaw_att, 'b-', 'LineWidth', 1.5);
    end
end

% 凡例（ハンドルベース）
h_leg = gobjects(0);
labels_leg = {};
if max(t_range) <= length(truth_data.theta)
    h_true = plot(NaN, NaN, 'k-', 'LineWidth', 2); % dummy handle for legend if needed
    delete(h_true);
    h_leg(end+1) = plot(NaN, NaN, 'k-', 'LineWidth', 2); labels_leg{end+1} = '真値';
end
if exist('t_att','var') && ~isempty(t_att)
    h_est = plot(NaN, NaN, 'b-', 'LineWidth', 1.5);
    delete(h_est);
    h_leg(end+1) = plot(NaN, NaN, 'b-', 'LineWidth', 1.5); labels_leg{end+1} = '推定値';
end
if ~isempty(h_leg)
    legend(h_leg, labels_leg, 'Location', 'best');
end

end

function plot_position_trajectory(pos_results, truth_data, current_idx)
% 位置軌跡プロット

% XY軌跡
subplot(2, 2, [1, 3]);
hold on;
title('位置軌跡 (XY平面)');
xlabel('X [m]');
ylabel('Y [m]');
grid on;
axis equal;

% 真値軌跡
max_display = min(current_idx * 40, length(truth_data.pos));

% 推定軌跡
pos_est_x = [];
pos_est_y = [];
gps_x = [];
gps_y = [];
pos_pred_x = [];
pos_pred_y = [];

for i = 1:length(pos_results)
    if ~isempty(pos_results{i})
        pos_est_x = [pos_est_x; pos_results{i}.position(1)];
        pos_est_y = [pos_est_y; pos_results{i}.position(2)];
        pos_pred_x = [pos_pred_x; pos_results{i}.x_pred(1)];
        pos_pred_y = [pos_pred_y; pos_results{i}.x_pred(2)];
        
        % GPS観測点
        idx = i * 40;
        if idx <= size(truth_data.pos, 1)
            % sim_data.csvのGPS列を使用
            gps_x = [gps_x; truth_data.pos(idx, 1) + 0.1 * randn()]; % ノイズ近似
            gps_y = [gps_y; truth_data.pos(idx, 2) + 0.1 * randn()];
        end
    end
end

        % 描画と凡例用ハンドル収集
        h = gobjects(0);
        labels = {};
        if max_display > 0
            h0 = plot(truth_data.pos(1:max_display, 1), truth_data.pos(1:max_display, 2), 'k-', 'LineWidth', 2);
            h(end+1) = h0; labels{end+1} = '真値';
        end
        if ~isempty(pos_est_x)
            h1 = plot(pos_est_x, pos_est_y, 'b-', 'LineWidth', 1.5);
            h(end+1) = h1; labels{end+1} = '推定値';
        end
        if ~isempty(gps_x)
            h2 = plot(gps_x, gps_y, 'ro', 'MarkerSize', 4);
            h(end+1) = h2; labels{end+1} = 'GPS';
        end
        if ~isempty(pos_pred_x)
            h3 = plot(pos_pred_x, pos_pred_y, 'g.', 'MarkerSize', 8);
            h(end+1) = h3; labels{end+1} = '予測値';
        end
        if ~isempty(h)
            legend(h, labels, 'Location', 'best');
        end

% Z方向位置
subplot(2, 2, 2);
hold on;
title('高度');
xlabel('時間 [s]');
ylabel('Z [m]');
grid on;

if max_display > 0
    t_plot = truth_data.t(1:max_display);
    h_z_true = plot(t_plot, truth_data.pos(1:max_display, 3), 'k-', 'LineWidth', 2);
end

if ~isempty(pos_results)
    t_pos = [];
    z_pos = [];
    for i = 1:length(pos_results)
        if ~isempty(pos_results{i})
            t_pos = [t_pos; pos_results{i}.timestamp];
            z_pos = [z_pos; pos_results{i}.position(3)];
        end
    end
    if ~isempty(t_pos)
        h_z_est = plot(t_pos, z_pos, 'b-', 'LineWidth', 1.5);
    end
end
% 凡例（高度）
h_leg = gobjects(0); labels_leg = {};
if exist('h_z_true','var') && ~isempty(h_z_true)
    h_leg(end+1) = h_z_true; labels_leg{end+1} = '真値';
end
if exist('h_z_est','var') && ~isempty(h_z_est)
    h_leg(end+1) = h_z_est; labels_leg{end+1} = '推定値';
end
if ~isempty(h_leg)
    legend(h_leg, labels_leg, 'Location', 'best');
end

% 推定誤差
subplot(2, 2, 4);
hold on;
title('位置推定誤差');
xlabel('時間 [s]');
ylabel('誤差 [m]');
grid on;

if ~isempty(pos_results) && max_display > 0
    t_err = [];
    err_norm = [];
    for i = 1:length(pos_results)
        if ~isempty(pos_results{i})
            idx = i * 40;
            if idx <= length(truth_data.pos)
                t_err = [t_err; pos_results{i}.timestamp];
                error_vec = pos_results{i}.position - truth_data.pos(idx, :)';
                err_norm = [err_norm; norm(error_vec)];
            end
        end
    end
    if ~isempty(t_err)
            h_err = plot(t_err, err_norm, 'r-', 'LineWidth', 1.5);
    end
end
        % 凡例（誤差）
        if exist('h_err','var') && ~isempty(h_err)
            legend(h_err, '位置誤差', 'Location', 'best');
        end

end
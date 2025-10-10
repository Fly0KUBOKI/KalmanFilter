function plot_final_results(vel_results, att_results, pos_results, truth_data, sensor_data)
% PLOT_FINAL_RESULTS - 総合結果プロット
% シミュレーション完了後の詳細な結果表示

fprintf('最終結果をプロットしています...\n');

% 新しい総合結果図を作成
figure('Name', 'カスケードUKF推定結果総合', 'Position', [50, 50, 1400, 900]);

% 位置軌跡（メイン）
subplot(2, 3, [1, 2, 4, 5]);
hold on;
title('3次元軌跡推定結果', 'FontSize', 14);
xlabel('X [m]');
ylabel('Y [m]');
grid on;
axis equal;

% 真値軌跡
plot(truth_data.pos(:, 1), truth_data.pos(:, 2), 'k-', 'LineWidth', 2);

% 推定軌跡
pos_est = [];
gps_meas = [];
pos_pred = [];

for i = 1:length(pos_results)
    if ~isempty(pos_results{i})
        pos_est = [pos_est; pos_results{i}.position(1:2)'];
        pos_pred = [pos_pred; pos_results{i}.x_pred(1:2)'];
        
        % GPS測定点
        idx = i * 40;
        if idx <= size(sensor_data.gps, 1)
            gps_meas = [gps_meas; sensor_data.gps(idx, :)];
        end
    end
end

if ~isempty(pos_est)
    plot(pos_est(:, 1), pos_est(:, 2), 'b-', 'LineWidth', 2);
end
if ~isempty(gps_meas)
    plot(gps_meas(:, 1), gps_meas(:, 2), 'ro', 'MarkerSize', 3);
end
if ~isempty(pos_pred)
    plot(pos_pred(:, 1), pos_pred(:, 2), 'g.', 'MarkerSize', 6);
end

legend('真値', '推定値', 'GPS', '予測値', 'Location', 'best');

% 速度推定精度
subplot(2, 3, 3);
hold on;
title('速度推定精度');
xlabel('時間 [s]');
ylabel('速度誤差 [m/s]');
grid on;

vel_error = [];
t_vel = [];
for i = 1:length(vel_results)
    if ~isempty(vel_results{i})
        true_vel = truth_data.vel(i, 1:3)';
        est_vel = vel_results{i}.velocity;
        vel_error = [vel_error; norm(est_vel - true_vel)];
        t_vel = [t_vel; truth_data.t(i)];
    end
end

if ~isempty(vel_error)
    plot(t_vel, vel_error, 'b-', 'LineWidth', 1);
    mean_vel_error = mean(vel_error);
    yline(mean_vel_error, 'r--', sprintf('平均誤差: %.3f m/s', mean_vel_error));
end

% 姿勢推定精度
subplot(2, 3, 6);
hold on;
title('姿勢推定精度 (Yaw角)');
xlabel('時間 [s]');
ylabel('Yaw誤差 [rad]');
grid on;

yaw_error = [];
t_att = [];
for i = 1:length(att_results)
    if ~isempty(att_results{i})
        idx = i * 4;
        if idx <= length(truth_data.theta)
            true_yaw = truth_data.theta(idx);
            est_yaw = att_results{i}.attitude(3);
            yaw_error = [yaw_error; abs(wrap_to_pi(est_yaw - true_yaw))];
            t_att = [t_att; truth_data.t(idx)];
        end
    end
end

if ~isempty(yaw_error)
    plot(t_att, yaw_error, 'b-', 'LineWidth', 1);
    mean_yaw_error = mean(yaw_error);
    yline(mean_yaw_error, 'r--', sprintf('平均誤差: %.3f rad', mean_yaw_error));
end

% 統計情報を表示
fprintf('\n=== 推定精度統計 ===\n');
if ~isempty(vel_error)
    fprintf('速度推定 - 平均誤差: %.4f m/s, 最大誤差: %.4f m/s\n', mean(vel_error), max(vel_error));
end
if ~isempty(yaw_error)
    fprintf('姿勢推定 - 平均Yaw誤差: %.4f rad (%.1f deg), 最大誤差: %.4f rad (%.1f deg)\n', ...
        mean(yaw_error), rad2deg(mean(yaw_error)), max(yaw_error), rad2deg(max(yaw_error)));
end

% 位置推定精度
pos_error = [];
for i = 1:length(pos_results)
    if ~isempty(pos_results{i})
        idx = i * 40;
        if idx <= size(truth_data.pos, 1)
            true_pos = truth_data.pos(idx, :)';
            est_pos = pos_results{i}.position;
            pos_error = [pos_error; norm(est_pos - true_pos)];
        end
    end
end

if ~isempty(pos_error)
    fprintf('位置推定 - 平均誤差: %.4f m, 最大誤差: %.4f m\n', mean(pos_error), max(pos_error));
end

fprintf('===================\n');

end
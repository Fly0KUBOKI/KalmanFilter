function plot_cascade_results()
% PLOT_CASCADE_RESULTS - カスケードUKF結果の可視化
% 保存された結果ファイルを読み込んでプロット

fprintf('カスケードUKF結果をプロット中...\n');

% 結果ファイルを読み込み
results_file = fullfile('..', 'KalmanFilter', 'optimized_simulation_results.mat');

if ~exist(results_file, 'file')
    fprintf('結果ファイルが見つかりません: %s\n', results_file);
    fprintf('まずシミュレーションを実行してください。\n');
    return;
end

load(results_file, 'vel_summary', 'att_summary', 'pos_summary', 'truth_data');
fprintf('結果ファイルを読み込みました\n');

% メイン結果図
figure('Name', 'カスケードUKF推定結果', 'Position', [50, 50, 1400, 1000]);

% 1. 位置軌跡 (XY平面)
subplot(2, 3, 1);
hold on;
title('位置軌跡 (XY平面)', 'FontSize', 12);
xlabel('X [m]');
ylabel('Y [m]');
grid on;
axis equal;

% 真値軌跡（間引き表示）
step = max(1, floor(length(truth_data.pos) / 2000));
plot(truth_data.pos(1:step:end, 1), truth_data.pos(1:step:end, 2), ...
     'k-', 'LineWidth', 1.5, 'DisplayName', '真値');

% 推定軌跡
if ~isempty(pos_summary.positions)
    plot(pos_summary.positions(:, 1), pos_summary.positions(:, 2), ...
         'b-', 'LineWidth', 2, 'DisplayName', '推定軌跡');
    
    % 開始点と終了点
    plot(pos_summary.positions(1, 1), pos_summary.positions(1, 2), ...
         'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', '開始点');
    plot(pos_summary.positions(end, 1), pos_summary.positions(end, 2), ...
         'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', '終了点');
end

legend('Location', 'best');

% 2. 3D軌跡
subplot(2, 3, 2);
hold on;
title('3次元軌跡', 'FontSize', 12);
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
grid on;

% 真値3D軌跡（間引き）
plot3(truth_data.pos(1:step:end, 1), truth_data.pos(1:step:end, 2), truth_data.pos(1:step:end, 3), ...
      'k-', 'LineWidth', 1.5, 'DisplayName', '真値');

% 推定3D軌跡
if ~isempty(pos_summary.positions)
    plot3(pos_summary.positions(:, 1), pos_summary.positions(:, 2), pos_summary.positions(:, 3), ...
          'b-', 'LineWidth', 2, 'DisplayName', '推定軌跡');
end

legend('Location', 'best');
view(3);

% 3. 速度推定誤差
subplot(2, 3, 3);
plot(vel_summary.timestamps, vel_summary.errors, 'b-', 'LineWidth', 1);
title('速度推定誤差', 'FontSize', 12);
xlabel('時間 [s]');
ylabel('誤差 [m/s]');
grid on;
ylim([0, min(10, max(vel_summary.errors))]);  % 異常値をカット

% 4. 姿勢推定誤差 (Yaw)
subplot(2, 3, 4);
plot(att_summary.timestamps, rad2deg(att_summary.errors), 'r-', 'LineWidth', 1);
title('姿勢推定誤差 (Yaw角)', 'FontSize', 12);
xlabel('時間 [s]');
ylabel('誤差 [deg]');
grid on;

% 5. 位置推定誤差
subplot(2, 3, 5);
plot(pos_summary.timestamps, pos_summary.errors, 'g-', 'LineWidth', 1);
title('位置推定誤差', 'FontSize', 12);
xlabel('時間 [s]');
ylabel('誤差 [m]');
grid on;

% 6. 統計サマリー
subplot(2, 3, 6);
axis off;
title('推定精度統計', 'FontSize', 12);

% 統計情報をテキストで表示
valid_vel = vel_summary.errors(~isnan(vel_summary.errors));
valid_att = att_summary.errors(~isnan(att_summary.errors));
valid_pos = pos_summary.errors(~isnan(pos_summary.errors));

stats_text = {};
stats_text{end+1} = '推定精度統計';
stats_text{end+1} = '================';
stats_text{end+1} = '';

if ~isempty(valid_vel)
    stats_text{end+1} = sprintf('速度推定:');
    stats_text{end+1} = sprintf('  平均誤差: %.2f m/s', mean(valid_vel));
    stats_text{end+1} = sprintf('  最大誤差: %.2f m/s', max(valid_vel));
    stats_text{end+1} = sprintf('  標準偏差: %.2f m/s', std(valid_vel));
    stats_text{end+1} = '';
end

if ~isempty(valid_att)
    stats_text{end+1} = sprintf('姿勢推定 (Yaw):');
    stats_text{end+1} = sprintf('  平均誤差: %.1f deg', rad2deg(mean(valid_att)));
    stats_text{end+1} = sprintf('  最大誤差: %.1f deg', rad2deg(max(valid_att)));
    stats_text{end+1} = sprintf('  標準偏差: %.1f deg', rad2deg(std(valid_att)));
    stats_text{end+1} = '';
end

if ~isempty(valid_pos)
    stats_text{end+1} = sprintf('位置推定:');
    stats_text{end+1} = sprintf('  平均誤差: %.1f m', mean(valid_pos));
    stats_text{end+1} = sprintf('  最大誤差: %.1f m', max(valid_pos));
    stats_text{end+1} = sprintf('  標準偏差: %.1f m', std(valid_pos));
    stats_text{end+1} = '';
end

stats_text{end+1} = sprintf('データ点数:');
stats_text{end+1} = sprintf('  速度推定: %d', length(vel_summary.timestamps));
stats_text{end+1} = sprintf('  姿勢推定: %d', length(att_summary.timestamps));
stats_text{end+1} = sprintf('  位置推定: %d', length(pos_summary.timestamps));

text(0.05, 0.95, stats_text, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
     'FontSize', 10, 'FontName', 'FixedWidth');

% 図を保存
saveas(gcf, 'cascade_ukf_results.png');
fprintf('結果図を保存しました: cascade_ukf_results.png\n');

% 個別の詳細図も作成
create_detailed_error_plots(vel_summary, att_summary, pos_summary);

fprintf('プロット完了!\n');

end

function create_detailed_error_plots(vel_summary, att_summary, pos_summary)
% 詳細誤差プロットを作成

figure('Name', '誤差分析詳細', 'Position', [100, 100, 1200, 800]);

% 速度誤差ヒストグラム
subplot(2, 3, 1);
valid_vel = vel_summary.errors(~isnan(vel_summary.errors));
valid_vel_clipped = valid_vel(valid_vel < 20);  % 異常値除去
histogram(valid_vel_clipped, 30);
title('速度誤差分布');
xlabel('誤差 [m/s]');
ylabel('頻度');
grid on;

% 姿勢誤差ヒストグラム
subplot(2, 3, 2);
valid_att = att_summary.errors(~isnan(att_summary.errors));
histogram(rad2deg(valid_att), 30);
title('姿勢誤差分布 (Yaw)');
xlabel('誤差 [deg]');
ylabel('頻度');
grid on;

% 位置誤差ヒストグラム
subplot(2, 3, 3);
valid_pos = pos_summary.errors(~isnan(pos_summary.errors));
valid_pos_clipped = valid_pos(valid_pos < 100);  % 異常値除去
histogram(valid_pos_clipped, 30);
title('位置誤差分布');
xlabel('誤差 [m]');
ylabel('頻度');
grid on;

% 誤差の時系列変化（移動平均）
subplot(2, 3, 4:6);
hold on;
title('誤差の時系列変化（移動平均）');
xlabel('時間 [s]');
ylabel('誤差');
grid on;

% 移動平均を計算してプロット
if length(valid_vel_clipped) > 10
    window = 50;
    vel_ma = movmean(valid_vel_clipped, window);
    t_vel = vel_summary.timestamps(valid_vel < 20);
    plot(t_vel(1:length(vel_ma)), vel_ma, 'b-', 'LineWidth', 2, 'DisplayName', '速度誤差 [m/s]');
end

if length(valid_att) > 10
    window = 20;
    att_ma = movmean(rad2deg(valid_att), window);
    plot(att_summary.timestamps(1:length(att_ma)), att_ma, 'r-', 'LineWidth', 2, 'DisplayName', '姿勢誤差 [deg]');
end

if length(valid_pos_clipped) > 5
    window = 10;
    pos_ma = movmean(valid_pos_clipped, window);
    t_pos = pos_summary.timestamps(valid_pos < 100);
    plot(t_pos(1:length(pos_ma)), pos_ma, 'g-', 'LineWidth', 2, 'DisplayName', '位置誤差 [m]');
end

legend('Location', 'best');

% 図を保存
saveas(gcf, 'cascade_ukf_error_analysis.png');
fprintf('誤差分析図を保存しました: cascade_ukf_error_analysis.png\n');

end
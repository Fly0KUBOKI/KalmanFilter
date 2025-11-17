%% Yaw 角速度積分の改善確認スクリプト

clear; close all; clc;

% 結果を読み込み
E = readtable('Results/estimation.csv');

% 時間とYaw角を取得
time = E.time;
yaw = E.yaw;

% パルスログがある場合は読み込み
try
    P = readtable('Results/pulse_log.csv');
    pulse_times = P.time;
catch
    pulse_times = [];
end

% 図1: Yaw 角度の時間履歴
figure('Position', [100 100 1200 400]);

subplot(1,2,1);
plot(time, rad2deg(yaw), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Yaw Angle (deg)', 'FontSize', 11);
title('Yaw Angle Estimation (After Filter Fix)', 'FontSize', 12);
if ~isempty(pulse_times)
    hold on;
    ylim_curr = ylim;
    for pt = pulse_times'
        plot([pt pt], ylim_curr, 'r--', 'LineWidth', 1);
    end
    legend('Yaw', 'Pulse Events');
end

% 図2: Yaw 角速度（数値微分）
subplot(1,2,2);
yaw_rate = gradient(yaw, time);
plot(time, rad2deg(yaw_rate), 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Yaw Rate (deg/s)', 'FontSize', 11);
title('Yaw Angular Velocity (Derived)', 'FontSize', 12);

% 統計情報
fprintf('\n========== Yaw Integration Status ==========\n');
fprintf('Simulation time: %.2f seconds\n', max(time));
fprintf('Number of steps: %d\n', length(time));
fprintf('\nYaw Angle Statistics:\n');
fprintf('  Min: %.2f deg\n', min(rad2deg(yaw)));
fprintf('  Max: %.2f deg\n', max(rad2deg(yaw)));
fprintf('  Range: %.2f deg\n', max(rad2deg(yaw)) - min(rad2deg(yaw)));
fprintf('  Mean: %.2f deg\n', mean(rad2deg(yaw)));

fprintf('\nYaw Rate Statistics:\n');
fprintf('  Min: %.2f deg/s\n', min(rad2deg(yaw_rate)));
fprintf('  Max: %.2f deg/s\n', max(rad2deg(yaw_rate)));
fprintf('  Std: %.2f deg/s\n', std(rad2deg(yaw_rate)));
fprintf('  Mean: %.2f deg/s\n', mean(rad2deg(yaw_rate)));

if ~isempty(pulse_times)
    fprintf('\nDetected %d pulse events\n', length(pulse_times));
end

fprintf('\n✓ If Yaw angle and rate show reasonable variation,\n');
fprintf('  the integration fix is working correctly.\n');
fprintf('============================================\n\n');

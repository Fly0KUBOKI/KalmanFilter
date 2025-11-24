% analyze_ukf_estimation.m - UKF推定結果の詳細分析
% 真値との比較により、観測値が正しく反映されているか検証

clear; clc; close all;
addpath(genpath(pwd));

fprintf('=== UKF GPS推定の詳細分析 ===\n\n');

%% データ読み込み
fprintf('データ読み込み中...\n');
truth = readtable('GenerateData/truth_data.csv');
obs = readtable('GenerateData/sensor_data.csv');
est = readtable('Results/estimation.csv');

fprintf('真値データ: %d行\n', height(truth));
fprintf('観測データ: %d行\n', height(obs));
fprintf('推定データ: %d行\n', height(est));

%% 時系列データ抽出
time = est.time;
N = length(time);

% 真値
true_x = truth.x;
true_y = truth.y;
true_z = truth.z;

% GPS観測値（ローカル座標に変換）
lat0 = obs.gps_lat(1);
lon0 = obs.gps_lon(1);
alt0 = obs.gps_alt(1);

gps_x = (obs.gps_lon - lon0) / (9.0e-6 / cosd(lat0));
gps_y = (obs.gps_lat - lat0) / (9.0e-6);
gps_z = obs.gps_alt - alt0;

% 推定値
est_x = est.x;
est_y = est.y;
est_z = est.z;

%% 統計分析
fprintf('\n=== 位置推定の統計 ===\n');

% X方向
err_x = est_x - true_x;
fprintf('X方向:\n');
fprintf('  真値範囲: [%.3f, %.3f] m\n', min(true_x), max(true_x));
fprintf('  推定範囲: [%.3f, %.3f] m\n', min(est_x), max(est_x));
fprintf('  誤差 RMSE: %.3f m\n', sqrt(mean(err_x.^2)));
fprintf('  誤差 Mean: %.3f m\n', mean(err_x));
fprintf('  誤差 Std:  %.3f m\n', std(err_x));

% Y方向
err_y = est_y - true_y;
fprintf('Y方向:\n');
fprintf('  真値範囲: [%.3f, %.3f] m\n', min(true_y), max(true_y));
fprintf('  推定範囲: [%.3f, %.3f] m\n', min(est_y), max(est_y));
fprintf('  誤差 RMSE: %.3f m\n', sqrt(mean(err_y.^2)));
fprintf('  誤差 Mean: %.3f m\n', mean(err_y));
fprintf('  誤差 Std:  %.3f m\n', std(err_y));

% Z方向
err_z = est_z - true_z;
fprintf('Z方向:\n');
fprintf('  真値範囲: [%.3f, %.3f] m\n', min(true_z), max(true_z));
fprintf('  推定範囲: [%.3f, %.3f] m\n', min(est_z), max(est_z));
fprintf('  誤差 RMSE: %.3f m\n', sqrt(mean(err_z.^2)));
fprintf('  誤差 Mean: %.3f m\n', mean(err_z));
fprintf('  誤差 Std:  %.3f m\n', std(err_z));

%% GPS更新の効果を確認
fprintf('\n=== GPS観測値との比較 ===\n');
gps_err_x = gps_x - true_x;
gps_err_y = gps_y - true_y;
gps_err_z = gps_z - true_z;

fprintf('GPS観測誤差:\n');
fprintf('  X RMSE: %.3f m\n', sqrt(mean(gps_err_x.^2)));
fprintf('  Y RMSE: %.3f m\n', sqrt(mean(gps_err_y.^2)));
fprintf('  Z RMSE: %.3f m\n', sqrt(mean(gps_err_z.^2)));

%% 推定値の時間変化を分析（ドリフト・振動検出）
fprintf('\n=== 時系列分析 ===\n');

% 初期値からの変化
initial_est_x = est_x(1);
initial_est_y = est_y(1);
initial_est_z = est_z(1);

drift_x = est_x - initial_est_x;
drift_y = est_y - initial_est_y;
drift_z = est_z - initial_est_z;

fprintf('初期値からのドリフト（終端値）:\n');
fprintf('  X: %.3f m\n', drift_x(end));
fprintf('  Y: %.3f m\n', drift_y(end));
fprintf('  Z: %.3f m (上昇傾向の確認)\n', drift_z(end));

% 振動分析（高周波成分）
dt = mean(diff(time));
fs = 1/dt;

% Z方向の変化率
dz_dt = diff(est_z) / dt;
fprintf('\nZ方向の変化率:\n');
fprintf('  平均: %.3f m/s\n', mean(dz_dt));
fprintf('  最大: %.3f m/s\n', max(dz_dt));
fprintf('  最小: %.3f m/s\n', min(dz_dt));

%% 詳細プロット
figure('Position', [100, 100, 1400, 900]);

% サブプロット1: X方向の時系列
subplot(3,3,1);
plot(time, true_x, 'k-', 'LineWidth', 1.5, 'DisplayName', '真値');
hold on;
plot(time, gps_x, 'b.', 'MarkerSize', 2, 'DisplayName', 'GPS観測');
plot(time, est_x, 'r-', 'LineWidth', 1, 'DisplayName', 'UKF推定');
xlabel('時間 [s]'); ylabel('X位置 [m]');
title('X方向位置');
legend('Location', 'best');
grid on;

% サブプロット2: Y方向の時系列
subplot(3,3,2);
plot(time, true_y, 'k-', 'LineWidth', 1.5, 'DisplayName', '真値');
hold on;
plot(time, gps_y, 'b.', 'MarkerSize', 2, 'DisplayName', 'GPS観測');
plot(time, est_y, 'r-', 'LineWidth', 1, 'DisplayName', 'UKF推定');
xlabel('時間 [s]'); ylabel('Y位置 [m]');
title('Y方向位置');
legend('Location', 'best');
grid on;

% サブプロット3: Z方向の時系列
subplot(3,3,3);
plot(time, true_z, 'k-', 'LineWidth', 1.5, 'DisplayName', '真値');
hold on;
plot(time, gps_z, 'b.', 'MarkerSize', 2, 'DisplayName', 'GPS観測');
plot(time, est_z, 'r-', 'LineWidth', 1, 'DisplayName', 'UKF推定');
xlabel('時間 [s]'); ylabel('Z位置 [m]');
title('Z方向位置（高度）');
legend('Location', 'best');
grid on;

% サブプロット4: X誤差
subplot(3,3,4);
plot(time, err_x, 'r-', 'LineWidth', 1);
xlabel('時間 [s]'); ylabel('誤差 [m]');
title('X方向推定誤差');
grid on;
yline(0, 'k--');

% サブプロット5: Y誤差
subplot(3,3,5);
plot(time, err_y, 'r-', 'LineWidth', 1);
xlabel('時間 [s]'); ylabel('誤差 [m]');
title('Y方向推定誤差');
grid on;
yline(0, 'k--');

% サブプロット6: Z誤差
subplot(3,3,6);
plot(time, err_z, 'r-', 'LineWidth', 1);
xlabel('時間 [s]'); ylabel('誤差 [m]');
title('Z方向推定誤差（重要）');
grid on;
yline(0, 'k--');

% サブプロット7: XY平面軌跡
subplot(3,3,7);
plot(true_x, true_y, 'k-', 'LineWidth', 1.5, 'DisplayName', '真値');
hold on;
plot(gps_x, gps_y, 'b.', 'MarkerSize', 2, 'DisplayName', 'GPS観測');
plot(est_x, est_y, 'r-', 'LineWidth', 1, 'DisplayName', 'UKF推定');
xlabel('X [m]'); ylabel('Y [m]');
title('XY平面軌跡');
legend('Location', 'best');
grid on;
axis equal;

% サブプロット8: Z方向ドリフト
subplot(3,3,8);
plot(time, drift_z, 'r-', 'LineWidth', 1.5);
xlabel('時間 [s]'); ylabel('ドリフト [m]');
title('Z方向ドリフト（初期値からの変化）');
grid on;
yline(0, 'k--');

% サブプロット9: Z変化率
subplot(3,3,9);
plot(time(1:end-1), dz_dt, 'r-', 'LineWidth', 1);
xlabel('時間 [s]'); ylabel('変化率 [m/s]');
title('Z方向変化率');
grid on;
yline(0, 'k--');

sgtitle('UKF GPS推定の詳細分析', 'FontSize', 14, 'FontWeight', 'bold');

%% 拡大表示（最初の5秒間）
figure('Position', [150, 150, 1400, 600]);

zoom_time = 5; % 秒
zoom_idx = time <= zoom_time;

subplot(1,3,1);
plot(time(zoom_idx), true_x(zoom_idx), 'k-', 'LineWidth', 1.5, 'DisplayName', '真値');
hold on;
plot(time(zoom_idx), gps_x(zoom_idx), 'b.', 'MarkerSize', 4, 'DisplayName', 'GPS観測');
plot(time(zoom_idx), est_x(zoom_idx), 'r-', 'LineWidth', 1, 'DisplayName', 'UKF推定');
xlabel('時間 [s]'); ylabel('X位置 [m]');
title(sprintf('X方向位置（最初%.1f秒）', zoom_time));
legend('Location', 'best');
grid on;

subplot(1,3,2);
plot(time(zoom_idx), true_y(zoom_idx), 'k-', 'LineWidth', 1.5, 'DisplayName', '真値');
hold on;
plot(time(zoom_idx), gps_y(zoom_idx), 'b.', 'MarkerSize', 4, 'DisplayName', 'GPS観測');
plot(time(zoom_idx), est_y(zoom_idx), 'r-', 'LineWidth', 1, 'DisplayName', 'UKF推定');
xlabel('時間 [s]'); ylabel('Y位置 [m]');
title(sprintf('Y方向位置（最初%.1f秒）', zoom_time));
legend('Location', 'best');
grid on;

subplot(1,3,3);
plot(time(zoom_idx), true_z(zoom_idx), 'k-', 'LineWidth', 1.5, 'DisplayName', '真値');
hold on;
plot(time(zoom_idx), gps_z(zoom_idx), 'b.', 'MarkerSize', 4, 'DisplayName', 'GPS観測');
plot(time(zoom_idx), est_z(zoom_idx), 'r-', 'LineWidth', 1, 'DisplayName', 'UKF推定');
xlabel('時間 [s]'); ylabel('Z位置 [m]');
title(sprintf('Z方向位置（最初%.1f秒）', zoom_time));
legend('Location', 'best');
grid on;

sgtitle('初期応答の詳細', 'FontSize', 14, 'FontWeight', 'bold');

%% 診断メッセージ
fprintf('\n=== 診断結果 ===\n');

if abs(drift_z(end)) > 10
    fprintf('⚠ 警告: Z方向に大きなドリフトが検出されました (%.2f m)\n', drift_z(end));
    fprintf('  → UKF更新が正しく適用されていない可能性があります\n');
end

if abs(mean(err_z)) > 5
    fprintf('⚠ 警告: Z方向に大きなバイアスがあります (%.2f m)\n', mean(err_z));
    fprintf('  → 観測値が反映されていない可能性があります\n');
end

if std(dz_dt) > 1.0
    fprintf('⚠ 警告: Z方向の変化率が不安定です (std=%.3f m/s)\n', std(dz_dt));
    fprintf('  → 振動が発生している可能性があります\n');
end

fprintf('\n分析プロット作成完了\n');
fprintf('図を確認して、推定値が観測値に追従しているか検証してください\n');

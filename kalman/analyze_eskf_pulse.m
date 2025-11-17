% ESKFでのパルス発生を確認
clear; clc;

fprintf('========================================\n');
fprintf('ESKFでのパルス発生確認\n');
fprintf('========================================\n\n');

addpath('GenerateData');
addpath('KF/Core');
addpath('KF/Utils');
addpath('ESKF');
addpath('ESKF/Core');

% データ読み込み
truth_data = readmatrix('GenerateData/truth_data.csv');
sensor_data = readmatrix('GenerateData/sensor_data.csv');

% 観測データ構造体を作成
obs.time = truth_data(:, 1);
obs.ax = sensor_data(:, 2);
obs.ay = sensor_data(:, 3);
obs.az = sensor_data(:, 4);
obs.wx = zeros(size(obs.ax));
obs.wy = zeros(size(obs.ax));
obs.wz = zeros(size(obs.ax));
obs.mx = zeros(size(obs.ax));
obs.my = zeros(size(obs.ax));
obs.mz = zeros(size(obs.ax));
obs.pressure = zeros(size(obs.ax));
obs.lat = NaN(size(obs.ax));
obs.lon = NaN(size(obs.ax));
obs.alt = NaN(size(obs.ax));

fprintf('ESKFを初期化中...\n');
eskf = ESKF(obs, 0.5, mean(diff(obs.time)));

N = length(obs.time);
roll_est = zeros(N, 1);
pitch_est = zeros(N, 1);

% パルス発生時刻のインデックス
pulse_times = [52.5, 67.7];
pulse_indices = zeros(size(pulse_times));
for i = 1:length(pulse_times)
    [~, pulse_indices(i)] = min(abs(obs.time - pulse_times(i)));
end

fprintf('シミュレーション開始...\n\n');

% 詳細ログ用
log_window = 100;  % 前後100ステップ

for k = 1:N
    eskf.updateFilter(obs, k);
    
    euler = eskf.getEuler();
    roll_est(k) = euler(1);
    pitch_est(k) = euler(2);
    
    % パルス付近で詳細ログ
    for i = 1:length(pulse_indices)
        if abs(k - pulse_indices(i)) <= 5
            fprintf('Step %d (t=%.3fs): Roll_est=%.2f°, Roll_truth=%.2f°, ay=%.4f, az=%.4f\n', ...
                k, obs.time(k), roll_est(k), truth_data(k, 8), obs.ay(k), obs.az(k));
        end
    end
end

fprintf('\nシミュレーション完了\n\n');

% 真値
roll_truth = truth_data(:, 8);
pitch_truth = truth_data(:, 9);

% 各パルス付近を詳細表示
for i = 1:length(pulse_times)
    t_pulse = pulse_times(i);
    idx_center = pulse_indices(i);
    
    fprintf('========================================\n');
    fprintf('パルス時刻: %.1f秒 (Step %d)\n', t_pulse, idx_center);
    fprintf('========================================\n\n');
    
    % 前後のデータ
    idx_range = max(1, idx_center-10):min(N, idx_center+10);
    
    fprintf('  Step    Time     Roll_truth  Roll_est   Error    ay       az\n');
    fprintf('----------------------------------------------------------------\n');
    for k = idx_range
        err = roll_est(k) - roll_truth(k);
        fprintf('  %5d  %7.3f   %8.2f   %8.2f  %7.2f  %7.4f  %7.4f\n', ...
            k, obs.time(k), roll_truth(k), roll_est(k), err, obs.ay(k), obs.az(k));
    end
    fprintf('\n');
    
    % Roll変化を確認
    if idx_center > 1 && idx_center < N
        roll_change = roll_est(idx_center) - roll_est(idx_center-1);
        fprintf('  Roll変化量 (step %d → %d): %.4f°\n', idx_center-1, idx_center, roll_change);
        
        % 大きな変化があれば警告
        if abs(roll_change) > 1.0
            fprintf('  ★ 警告: 1ステップで%.2f°の大きな変化！\n', roll_change);
        end
    end
    fprintf('\n');
    
    % グラフ作成
    window = 400;  % 前後1秒
    idx_start = max(1, idx_center - window);
    idx_end = min(N, idx_center + window);
    
    figure('Position', [100, 100, 1400, 900]);
    
    % Roll
    subplot(3, 1, 1);
    plot(obs.time(idx_start:idx_end), roll_truth(idx_start:idx_end), 'b-', 'LineWidth', 1.5); hold on;
    plot(obs.time(idx_start:idx_end), roll_est(idx_start:idx_end), 'r--', 'LineWidth', 1.0);
    xlabel('Time [s]'); ylabel('Roll [deg]');
    legend('Truth', 'Estimated');
    title(sprintf('Roll at %.1f秒 (Step %d)', t_pulse, idx_center));
    grid on;
    xline(obs.time(idx_center), 'g--', 'LineWidth', 2);
    
    % Roll誤差
    subplot(3, 1, 2);
    plot(obs.time(idx_start:idx_end), roll_est(idx_start:idx_end) - roll_truth(idx_start:idx_end), 'r-', 'LineWidth', 1.0);
    xlabel('Time [s]'); ylabel('Roll Error [deg]');
    title('Roll誤差');
    grid on;
    xline(obs.time(idx_center), 'g--', 'LineWidth', 2);
    yline(0, 'k--');
    
    % az
    subplot(3, 1, 3);
    plot(obs.time(idx_start:idx_end), obs.az(idx_start:idx_end), 'b-', 'LineWidth', 1.0);
    xlabel('Time [s]'); ylabel('az [m/s²]');
    title('Z軸加速度');
    grid on;
    xline(obs.time(idx_center), 'g--', 'LineWidth', 2);
    yline(0, 'r--');
    yline(9.81, 'k--');
    
    sgtitle(sprintf('ESKF推定値とパルス: %.1f秒', t_pulse));
    
    saveas(gcf, sprintf('Results/eskf_pulse_%.1fs.png', t_pulse));
    fprintf('グラフを保存: Results/eskf_pulse_%.1fs.png\n\n', t_pulse);
end

fprintf('========================================\n');
fprintf('解析完了\n');
fprintf('========================================\n');

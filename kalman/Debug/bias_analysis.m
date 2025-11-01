function bias_error = bias_analysis(obs, truth, static_samples, dt)
% bias_analysis - バイアス補正精度を分析

ba_est = [mean(obs.ax(1:static_samples)); 
          mean(obs.ay(1:static_samples)); 
          mean(obs.az(1:static_samples))] - [0; 0; 9.81];

bg_est_deg = [mean(obs.wx(1:static_samples)); 
              mean(obs.wy(1:static_samples)); 
              mean(obs.wz(1:static_samples))];
bg_est_rad = deg2rad(bg_est_deg);

fprintf('--- 推定バイアス ---\n');
fprintf('  ba = [%.6f, %.6f, %.6f] m/s²\n', ba_est(1), ba_est(2), ba_est(3));
fprintf('  bg = [%.6f, %.6f, %.6f] deg/s\n', bg_est_deg(1), bg_est_deg(2), bg_est_deg(3));
fprintf('  bg = [%.6f, %.6f, %.6f] rad/s\n\n', bg_est_rad(1), bg_est_rad(2), bg_est_rad(3));

start_idx = static_samples + 1;
n_obs = length(obs.time);
n_truth = height(truth);
end_idx = min(n_obs, n_truth);

wx_corr = obs.wx(start_idx:end_idx) - bg_est_deg(1);
wy_corr = obs.wy(start_idx:end_idx) - bg_est_deg(2);
wz_corr = obs.wz(start_idx:end_idx) - bg_est_deg(3);

fprintf('--- バイアス補正後の角速度（動作開始後）---\n');
fprintf('  wx: 平均=%.6f, 標準偏差=%.6f deg/s\n', mean(wx_corr), std(wx_corr));
fprintf('  wy: 平均=%.6f, 標準偏差=%.6f deg/s\n', mean(wy_corr), std(wy_corr));
fprintf('  wz: 平均=%.6f, 標準偏差=%.6f deg/s\n\n', mean(wz_corr), std(wz_corr));

yaw_true = truth.yaw(start_idx:end_idx);
yaw_dot_true = [0; diff(yaw_true)] / dt;
wz_error = wz_corr - yaw_dot_true;

fprintf('--- 真値との比較（yaw方向角速度）---\n');
fprintf('  真値の角速度: 平均=%.6f, 標準偏差=%.6f deg/s\n', mean(yaw_dot_true), std(yaw_dot_true));
fprintf('  角速度誤差: 平均=%.6f, 標準偏差=%.6f deg/s\n', mean(wz_error), std(wz_error));

total_time = (length(obs.time) - static_samples) * dt;
accumulated_error = mean(wz_error) * total_time;

fprintf('\n--- 誤差の累積効果 ---\n');
fprintf('  平均角速度誤差: %.6f deg/s\n', mean(wz_error));
fprintf('  全期間の長さ: %.2f 秒\n', total_time);
fprintf('  累積角度誤差: %.2f deg\n', accumulated_error);

bias_error.ba = ba_est;
bias_error.bg_deg = bg_est_deg;
bias_error.bg_rad = bg_est_rad;
bias_error.gyro_mean_error = mean(wz_error);
bias_error.gyro_std_error = std(wz_error);
bias_error.accumulated_angle_error = accumulated_error;
end

function [bias_stats, sensor_noise] = static_analysis(obs, truth, static_samples)
% static_analysis - 静止期間のセンサーデータ分析

fprintf('--- 加速度計の統計 ---\n');
ax_static = obs.ax(1:static_samples);
ay_static = obs.ay(1:static_samples);
az_static = obs.az(1:static_samples);

bias_stats.accel.mean = [mean(ax_static); mean(ay_static); mean(az_static)];
bias_stats.accel.std = [std(ax_static); std(ay_static); std(az_static)];
bias_stats.accel.norm_mean = mean(sqrt(ax_static.^2 + ay_static.^2 + az_static.^2));

fprintf('  x軸: 平均=%.6f, 標準偏差=%.6f m/s²\n', bias_stats.accel.mean(1), bias_stats.accel.std(1));
fprintf('  y軸: 平均=%.6f, 標準偏差=%.6f m/s²\n', bias_stats.accel.mean(2), bias_stats.accel.std(2));
fprintf('  z軸: 平均=%.6f, 標準偏差=%.6f m/s² (期待値: 9.81)\n', bias_stats.accel.mean(3), bias_stats.accel.std(3));
fprintf('  ノルム: 平均=%.6f m/s²\n\n', bias_stats.accel.norm_mean);

fprintf('--- ジャイロの統計 ---\n');
wx_static = obs.wx(1:static_samples);
wy_static = obs.wy(1:static_samples);
wz_static = obs.wz(1:static_samples);

bias_stats.gyro.mean_deg = [mean(wx_static); mean(wy_static); mean(wz_static)];
bias_stats.gyro.std_deg = [std(wx_static); std(wy_static); std(wz_static)];
bias_stats.gyro.mean_rad = deg2rad(bias_stats.gyro.mean_deg);

fprintf('  x軸: 平均=%.6f, 標準偏差=%.6f deg/s\n', bias_stats.gyro.mean_deg(1), bias_stats.gyro.std_deg(1));
fprintf('  y軸: 平均=%.6f, 標準偏差=%.6f deg/s\n', bias_stats.gyro.mean_deg(2), bias_stats.gyro.std_deg(2));
fprintf('  z軸: 平均=%.6f, 標準偏差=%.6f deg/s (期待値: 0)\n', bias_stats.gyro.mean_deg(3), bias_stats.gyro.std_deg(3));
fprintf('  (rad/s: [%.6f, %.6f, %.6f])\n\n', bias_stats.gyro.mean_rad(1), bias_stats.gyro.mean_rad(2), bias_stats.gyro.mean_rad(3));

sensor_noise.accel_std = bias_stats.accel.std;
sensor_noise.gyro_std_deg = bias_stats.gyro.std_deg;
sensor_noise.gyro_std_rad = deg2rad(sensor_noise.gyro_std_deg);

fprintf('--- センサーノイズレベル ---\n');
fprintf('  加速度計: [%.6f, %.6f, %.6f] m/s²\n', sensor_noise.accel_std(1), sensor_noise.accel_std(2), sensor_noise.accel_std(3));
fprintf('  ジャイロ: [%.6f, %.6f, %.6f] deg/s\n', sensor_noise.gyro_std_deg(1), sensor_noise.gyro_std_deg(2), sensor_noise.gyro_std_deg(3));
end

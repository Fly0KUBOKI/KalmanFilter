% 静止期間のジャイロデータを確認
clear; clc;

addpath('GenerateData');
obs = read_csv('GenerateData/sensor_data.csv');

static_samples = 800;  % 2.0秒 / 0.0025秒

fprintf('=== 静止期間のジャイロデータ（最初800サンプル） ===\n\n');

gyro_x = obs.wx(1:static_samples);
gyro_y = obs.wy(1:static_samples);
gyro_z = obs.wz(1:static_samples);

fprintf('ジャイロ x軸:\n');
fprintf('  mean = %.6f deg/s (%.6f rad/s)\n', mean(gyro_x), deg2rad(mean(gyro_x)));
fprintf('  std = %.6f deg/s\n', std(gyro_x));
fprintf('  min = %.6f deg/s, max = %.6f deg/s\n\n', min(gyro_x), max(gyro_x));

fprintf('ジャイロ y軸:\n');
fprintf('  mean = %.6f deg/s (%.6f rad/s)\n', mean(gyro_y), deg2rad(mean(gyro_y)));
fprintf('  std = %.6f deg/s\n', std(gyro_y));
fprintf('  min = %.6f deg/s, max = %.6f deg/s\n\n', min(gyro_y), max(gyro_y));

fprintf('ジャイロ z軸:\n');
fprintf('  mean = %.6f deg/s (%.6f rad/s)\n', mean(gyro_z), deg2rad(mean(gyro_z)));
fprintf('  std = %.6f deg/s\n', std(gyro_z));
fprintf('  min = %.6f deg/s, max = %.6f deg/s\n\n', min(gyro_z), max(gyro_z));

% 最後の10サンプルを確認
fprintf('=== 静止期間の最後10サンプル（k=791-800） ===\n');
for k = 791:800
    fprintf('k=%d: [%.4f, %.4f, %.4f] deg/s\n', k, gyro_x(k), gyro_y(k), gyro_z(k));
end

fprintf('\n=== 動き始め（k=801-810） ===\n');
for k = 801:810
    fprintf('k=%d: [%.4f, %.4f, %.4f] deg/s\n', k, obs.wx(k), obs.wy(k), obs.wz(k));
end

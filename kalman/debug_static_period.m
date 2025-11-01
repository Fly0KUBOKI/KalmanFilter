% 静止期間の加速度とバイアスを詳しく調査
clear; clc;

% データ読み込み
sensor_data = readtable('GenerateData/sensor_data.csv');
truth_data = readtable('GenerateData/truth_data.csv');

% 静止期間（0-2秒、800サンプル）
static_time = 2.0;
dt = 0.0025;
static_samples = floor(static_time / dt);

fprintf('=== 静止期間の分析 ===\n');
fprintf('静止サンプル数: %d (0 - %.4f秒)\n\n', static_samples, static_time);

% センサデータ（加速度）
accel_x = sensor_data.accel_x(1:static_samples);
accel_y = sensor_data.accel_y(1:static_samples);
accel_z = sensor_data.accel_z(1:static_samples);

% 統計量
fprintf('【加速度センサ測定値（ボディフレーム）】\n');
fprintf('  x軸: mean=%.6f, std=%.6f, range=[%.6f, %.6f]\n', ...
    mean(accel_x), std(accel_x), min(accel_x), max(accel_x));
fprintf('  y軸: mean=%.6f, std=%.6f, range=[%.6f, %.6f]\n', ...
    mean(accel_y), std(accel_y), min(accel_y), max(accel_y));
fprintf('  z軸: mean=%.6f, std=%.6f, range=[%.6f, %.6f]\n', ...
    mean(accel_z), std(accel_z), min(accel_z), max(accel_z));
fprintf('  ノルム: mean=%.6f, std=%.6f\n\n', ...
    mean(sqrt(accel_x.^2 + accel_y.^2 + accel_z.^2)), ...
    std(sqrt(accel_x.^2 + accel_y.^2 + accel_z.^2)));

% 理想的な静止時の比力
f_ideal = [0; 0; 9.81];
fprintf('【理想的な静止時の比力】: [0, 0, 9.81]\n\n');

% バイアス推定（ESKFの初期化と同じ方法）
accel_static_mean = [mean(accel_x); mean(accel_y); mean(accel_z)];
ba_estimated = accel_static_mean - f_ideal;

fprintf('【推定バイアス ba = accel_mean - [0; 0; 9.81]】\n');
fprintf('  ba = [%.6f, %.6f, %.6f]\n\n', ba_estimated(1), ba_estimated(2), ba_estimated(3));

% バイアス補正後の比力
accel_corrected_x = accel_x - ba_estimated(1);
accel_corrected_y = accel_y - ba_estimated(2);
accel_corrected_z = accel_z - ba_estimated(3);

fprintf('【バイアス補正後の比力（a = a_meas - ba）】\n');
fprintf('  x軸: mean=%.6f, std=%.6f\n', mean(accel_corrected_x), std(accel_corrected_x));
fprintf('  y軸: mean=%.6f, std=%.6f\n', mean(accel_corrected_y), std(accel_corrected_y));
fprintf('  z軸: mean=%.6f, std=%.6f (期待値: 9.81)\n', mean(accel_corrected_z), std(accel_corrected_z));
fprintf('  ノルム: mean=%.6f, std=%.6f (期待値: 9.81)\n\n', ...
    mean(sqrt(accel_corrected_x.^2 + accel_corrected_y.^2 + accel_corrected_z.^2)), ...
    std(sqrt(accel_corrected_x.^2 + accel_corrected_y.^2 + accel_corrected_z.^2)));

% 姿勢（真値）
roll = truth_data.roll(1:static_samples);
pitch = truth_data.pitch(1:static_samples);
yaw = truth_data.yaw(1:static_samples);

fprintf('【姿勢（真値）】\n');
fprintf('  roll:  mean=%.6f, std=%.6f\n', mean(roll), std(roll));
fprintf('  pitch: mean=%.6f, std=%.6f\n', mean(pitch), std(pitch));
fprintf('  yaw:   mean=%.6f, std=%.6f\n\n', mean(yaw), std(yaw));

% 速度積分シミュレーション（単純な積分）
fprintf('=== 速度積分テスト（単純積分） ===\n');
v = [0; 0; 0];
q = [1; 0; 0; 0];  % 単位クォータニオン（姿勢変化なし）
g = [0; 0; -9.81];

for i = 1:100  % 最初の100ステップ（0.25秒）
    a_meas = [accel_x(i); accel_y(i); accel_z(i)];
    a = a_meas - ba_estimated;  % バイアス補正
    
    % 姿勢変化なしと仮定（Rb = I）
    Rb = eye(3);
    a_world = Rb * a;
    
    % 速度更新
    v = v + (a_world + g) * dt;
end

fprintf('100ステップ後（t=0.25秒）の速度:\n');
fprintf('  v = [%.6f, %.6f, %.6f] m/s\n', v(1), v(2), v(3));
fprintf('  期待値: [0, 0, 0] m/s\n\n');

% a_world + g の統計
fprintf('=== (a_world + g) の分析（真の加速度） ===\n');
a_true_samples = zeros(3, 100);
for i = 1:100
    a_meas = [accel_x(i); accel_y(i); accel_z(i)];
    a = a_meas - ba_estimated;
    Rb = eye(3);  % 姿勢変化なし
    a_world = Rb * a;
    a_true_samples(:, i) = a_world + g;
end

fprintf('  x軸: mean=%.6f, std=%.6f\n', mean(a_true_samples(1,:)), std(a_true_samples(1,:)));
fprintf('  y軸: mean=%.6f, std=%.6f\n', mean(a_true_samples(2,:)), std(a_true_samples(2,:)));
fprintf('  z軸: mean=%.6f, std=%.6f\n', mean(a_true_samples(3,:)), std(a_true_samples(3,:)));
fprintf('  期待値: [0, 0, 0]\n');

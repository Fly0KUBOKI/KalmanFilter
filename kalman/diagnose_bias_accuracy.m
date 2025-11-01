% バイアス補正の精度を確認
clear; clc;

addpath('GenerateData');
obs = read_csv('GenerateData/sensor_data.csv');
truth = readtable('GenerateData/truth_data.csv');

% 静止期間でバイアスを推定
static_samples = 800;
ba = [mean(obs.ax(1:static_samples)); mean(obs.ay(1:static_samples)); mean(obs.az(1:static_samples))] - [0; 0; 9.81];
bg_deg = [mean(obs.wx(1:static_samples)); mean(obs.wy(1:static_samples)); mean(obs.wz(1:static_samples))];
bg = deg2rad(bg_deg);

fprintf('=== 推定バイアス ===\n');
fprintf('ba = [%.6f, %.6f, %.6f] m/s²\n', ba(1), ba(2), ba(3));
fprintf('bg = [%.6f, %.6f, %.6f] rad/s\n', bg(1), bg(2), bg(3));
fprintf('bg = [%.6f, %.6f, %.6f] deg/s\n\n', bg_deg(1), bg_deg(2), bg_deg(3));

% 動作期間（t=2秒以降）のバイアス補正後の角速度を確認
fprintf('=== バイアス補正後の角速度（t=2-10秒） ===\n');
samples = 801:4000;  % 2-10秒
wx_corr = obs.wx(samples) - bg_deg(1);
wy_corr = obs.wy(samples) - bg_deg(2);
wz_corr = obs.wz(samples) - bg_deg(3);

fprintf('補正後の角速度統計:\n');
fprintf('  wx: mean=%.6f, std=%.6f deg/s\n', mean(wx_corr), std(wx_corr));
fprintf('  wy: mean=%.6f, std=%.6f deg/s\n', mean(wy_corr), std(wy_corr));
fprintf('  wz: mean=%.6f, std=%.6f deg/s\n\n', mean(wz_corr), std(wz_corr));

% 真値の角速度（数値微分）
dt = 0.0025;
yaw_true = truth.yaw(samples);
yaw_dot_true = [0; diff(yaw_true)] / dt;  % deg/s

fprintf('真値の角速度（yaw方向）:\n');
fprintf('  yaw_dot: mean=%.6f, std=%.6f deg/s\n\n', mean(yaw_dot_true), std(yaw_dot_true));

% 補正後の角速度と真値を比較
fprintf('=== 角速度の誤差 ===\n');
wz_err = wz_corr - yaw_dot_true;
fprintf('  wz誤差: mean=%.6f, std=%.6f deg/s\n', mean(wz_err), std(wz_err));
fprintf('  ⇒ バイアス補正後も平均%.6f deg/sのドリフトあり\n\n', mean(wz_err));

% 積分誤差の累積
fprintf('=== 角度誤差の累積（100秒間） ===\n');
total_time = 100 - 2;  % 98秒
yaw_error_accumulated = mean(wz_err) * total_time;
fprintf('  平均角速度誤差: %.6f deg/s\n', mean(wz_err));
fprintf('  98秒後の累積誤差: %.2f deg\n\n', yaw_error_accumulated);

fprintf('【結論】\n');
fprintf('バイアス補正後も角速度に平均%.6f deg/sのドリフトがあり、\n', mean(wz_err));
fprintf('これが98秒間で%.2f度の姿勢誤差を生み出しています。\n', yaw_error_accumulated);

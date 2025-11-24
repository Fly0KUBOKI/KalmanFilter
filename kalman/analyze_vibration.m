% 振動の分析
clc;

est = readtable('Results/estimation.csv');
truth = readtable('GenerateData/truth_data.csv');

% 運動期間（t>5.5s, 静止期間0.5s+加速5s後）
motion_idx = find(est.time > 5.5);

% 位置誤差
err_x = est.px(motion_idx) - truth.x(motion_idx);
err_y = est.py(motion_idx) - truth.y(motion_idx);
err_z = est.pz(motion_idx) - truth.z(motion_idx);

% 統計
fprintf('=== 位置誤差統計（運動期間） ===\n');
fprintf('X誤差: 平均=%.3f, 標準偏差=%.3f, RMS=%.3f m\n', mean(err_x), std(err_x), rms(err_x));
fprintf('Y誤差: 平均=%.3f, 標準偏差=%.3f, RMS=%.3f m\n', mean(err_y), std(err_y), rms(err_y));
fprintf('Z誤差: 平均=%.3f, 標準偏差=%.3f, RMS=%.3f m\n', mean(err_z), std(err_z), rms(err_z));

% 振動の検出（高周波成分）
fprintf('\n=== 振動分析（標準偏差 = 振動の指標） ===\n');
fprintf('X方向の振動: %.3f m\n', std(err_x));
fprintf('Y方向の振動: %.3f m\n', std(err_y));
fprintf('Z方向の振動: %.3f m\n', std(err_z));

% 最後の1000サンプル（2.5秒分）の詳細
last_idx = motion_idx(end-1000:end);
fprintf('\n=== 最後の2.5秒間の誤差 ===\n');
fprintf('X誤差: 平均=%.3f, 標準偏差=%.3f m\n', mean(err_x(end-1000:end)), std(err_x(end-1000:end)));
fprintf('Y誤差: 平均=%.3f, 標準偏差=%.3f m\n', mean(err_y(end-1000:end)), std(err_y(end-1000:end)));
fprintf('Z誤差: 平均=%.3f, 標準偏差=%.3f m\n', mean(err_z(end-1000:end)), std(err_z(end-1000:end)));

% サンプル表示（最後の20サンプル）
fprintf('\n=== 最後の20サンプル ===\n');
for i = height(est)-19:height(est)
    fprintf('t=%.2fs: Err=[%6.3f,%6.3f,%6.3f]\n', est.time(i), ...
        est.px(i)-truth.x(i), est.py(i)-truth.y(i), est.pz(i)-truth.z(i));
end

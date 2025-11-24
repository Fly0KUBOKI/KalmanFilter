% quick_diagnosis.m - UKF問題の迅速診断
clear; clc;

fprintf('=== UKF GPS推定 迅速診断 ===\n\n');

% データ読み込み
est = readtable('Results/estimation.csv');
obs = readtable('GenerateData/sensor_data.csv');

% 基本統計
fprintf('推定データサイズ: %d行\n', height(est));
fprintf('時間範囲: %.2f ~ %.2f 秒\n', min(est.time), max(est.time));

% 位置の範囲
fprintf('\n位置範囲:\n');
fprintf('  X: [%.3f, %.3f] m\n', min(est.px), max(est.px));
fprintf('  Y: [%.3f, %.3f] m\n', min(est.py), max(est.py));
fprintf('  Z: [%.3f, %.3f] m\n', min(est.pz), max(est.pz));

% Z方向の変化を確認
z_change = est.pz(end) - est.pz(1);
fprintf('\nZ方向の総変化: %.3f m\n', z_change);

% 最初の100ステップの詳細
fprintf('\n最初の100ステップ:\n');
for i = 1:10:min(100, height(est))
    fprintf('  t=%.3fs: x=%.3f, y=%.3f, z=%.3f\n', ...
        est.time(i), est.px(i), est.py(i), est.pz(i));
end

% GPS観測値との比較（サンプル）
fprintf('\nGPS観測値サンプル（最初10個）:\n');
lat0 = obs.gps_lat(1);
lon0 = obs.gps_lon(1);
for i = 1:10:min(100, height(obs))
    gps_x = (obs.gps_lon(i) - lon0) / (9.0e-6 / cosd(lat0));
    gps_y = (obs.gps_lat(i) - lat0) / (9.0e-6);
    gps_z = obs.gps_alt(i) - obs.gps_alt(1);
    fprintf('  t=%.3fs: gps=[%.3f, %.3f, %.3f], est=[%.3f, %.3f, %.3f]\n', ...
        obs.time(i), gps_x, gps_y, gps_z, est.px(i), est.py(i), est.pz(i));
end

fprintf('\n診断完了\n');

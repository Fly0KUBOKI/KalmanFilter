% GPS原点とオフセットを確認
clc; clear;

% データ読み込み
truth = readtable('GenerateData/truth_data.csv');
obs = readtable('GenerateData/sensor_data.csv');
est = readtable('Results/estimation.csv');

% 静止期間（0.5秒 = 200ステップ）
static_idx = 1:200;

% GPS原点計算
lat0 = mean(obs.gps_lat(static_idx));
lon0 = mean(obs.gps_lon(static_idx));
alt0 = mean(obs.gps_alt(static_idx));

fprintf('=== GPS原点 ===\n');
fprintf('lat0 = %.10f°\n', lat0);
fprintf('lon0 = %.10f°\n', lon0);
fprintf('alt0 = %.3f m\n', alt0);

% 真値の静止期間平均
fprintf('\n=== 真値（静止期間平均） ===\n');
fprintf('X = %.6f m\n', mean(truth.x(static_idx)));
fprintf('Y = %.6f m\n', mean(truth.y(static_idx)));
fprintf('Z = %.6f m\n', mean(truth.z(static_idx)));

% GPS観測値をローカル座標に変換（最初のいくつか）
fprintf('\n=== GPS→ローカル座標変換（最初の5サンプル） ===\n');
for i = 1:5
    lat = obs.gps_lat(i);
    lon = obs.gps_lon(i);
    alt = obs.gps_alt(i);
    
    y_m = (lat - lat0) / (9.0e-6);
    x_m = (lon - lon0) / (9.0e-6 / cosd(lat0));
    z_m = alt - alt0;
    
    fprintf('i=%d: GPS=[%.6f, %.6f, %.3f] → Local=[%.3f, %.3f, %.3f], Truth=[%.3f, %.3f, %.3f]\n', ...
        i, lat, lon, alt, x_m, y_m, z_m, truth.x(i), truth.y(i), truth.z(i));
end

% 最終時刻の比較
i = height(truth);
lat = obs.gps_lat(i);
lon = obs.gps_lon(i);
alt = obs.gps_alt(i);

y_m = (lat - lat0) / (9.0e-6);
x_m = (lon - lon0) / (9.0e-6 / cosd(lat0));
z_m = alt - alt0;

fprintf('\n=== 最終時刻（t=90s） ===\n');
fprintf('GPS→Local: [%.3f, %.3f, %.3f] m\n', x_m, y_m, z_m);
fprintf('真値:       [%.3f, %.3f, %.3f] m\n', truth.x(i), truth.y(i), truth.z(i));
fprintf('推定値:     [%.3f, %.3f, %.3f] m\n', est.px(i), est.py(i), est.pz(i));
fprintf('推定誤差:   [%.3f, %.3f, %.3f] m\n', est.px(i)-truth.x(i), est.py(i)-truth.y(i), est.pz(i)-truth.z(i));

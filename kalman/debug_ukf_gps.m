% UKF GPS更新の詳細デバッグ
clc; clear;
addpath(genpath('ESKF'));
addpath(genpath('UKF'));
addpath(genpath('Common'));
addpath(genpath('KF'));
addpath('GenerateData');

% データ読み込み
obs_data = read_csv('GenerateData/sensor_data.csv');
params = config_params();

% ESKF初期化
dt = 0.0025;
eskf = ESKF(obs_data, params.static_time, dt);

fprintf('=== ESKF初期状態 ===\n');
fprintf('GPS Origin: [%.10f, %.10f, %.3f]\n', eskf.gps_origin(1), eskf.gps_origin(2), eskf.gps_origin(3));
fprintf('Initial position p: [%.6f, %.6f, %.6f]\n', eskf.p(1), eskf.p(2), eskf.p(3));
fprintf('Initial velocity v: [%.6f, %.6f, %.6f]\n\n', eskf.v(1), eskf.v(2), eskf.v(3));

% 最初のGPS更新（k=10）
k = 10;
lat = obs_data.lat(k);
lon = obs_data.lon(k);
alt = obs_data.alt(k);

fprintf('=== GPS観測値 (k=%d) ===\n', k);
fprintf('lat = %.10f°\n', lat);
fprintf('lon = %.10f°\n', lon);
fprintf('alt = %.3f m\n\n', alt);

% 手動で座標変換
lat0 = eskf.gps_origin(1);
lon0 = eskf.gps_origin(2);
alt0 = eskf.gps_origin(3);

y_m = (lat - lat0) / (9.0e-6);
x_m = (lon - lon0) / (9.0e-6 / cosd(lat0));
z_m = alt - alt0;

fprintf('=== 座標変換結果 ===\n');
fprintf('dlat = %.10f°\n', lat - lat0);
fprintf('dlon = %.10f°\n', lon - lon0);
fprintf('y_m (北) = %.6f m\n', y_m);
fprintf('x_m (東) = %.6f m\n', x_m);
fprintf('z_m (上) = %.6f m\n\n', z_m);

% 真値と比較
truth = readtable('GenerateData/truth_data.csv');
fprintf('=== 真値 (k=%d) ===\n', k);
fprintf('真値 [東, 北, 上] = [%.6f, %.6f, %.6f] m\n\n', truth.x(k), truth.y(k), truth.z(k));

% GPS更新を実行
p_before = eskf.p;
v_before = eskf.v;

eskf.update_gps(lat, lon, alt, k);

p_after = eskf.p;
v_after = eskf.v;

fprintf('=== GPS更新前後の比較 ===\n');
fprintf('位置 before: [%.6f, %.6f, %.6f] m\n', p_before(1), p_before(2), p_before(3));
fprintf('位置 after:  [%.6f, %.6f, %.6f] m\n', p_after(1), p_after(2), p_after(3));
fprintf('位置 delta:  [%.6f, %.6f, %.6f] m\n\n', p_after(1)-p_before(1), p_after(2)-p_before(2), p_after(3)-p_before(3));

fprintf('速度 before: [%.6f, %.6f, %.6f] m/s\n', v_before(1), v_before(2), v_before(3));
fprintf('速度 after:  [%.6f, %.6f, %.6f] m/s\n', v_after(1), v_after(2), v_after(3));

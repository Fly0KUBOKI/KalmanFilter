% GPS更新の単純なテスト
clc; clear;

% パス追加
proj_root = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(proj_root, 'ESKF')));
addpath(genpath(fullfile(proj_root, 'UKF')));
addpath(genpath(fullfile(proj_root, 'Common')));
addpath(genpath(fullfile(proj_root, 'KF')));
addpath(fullfile(proj_root, 'GenerateData'));

% データ読み込み
obs = read_csv(fullfile(proj_root, 'GenerateData', 'sensor_data.csv'));

% ESKF初期化
params = config_params();
dt = 0.0025;
eskf = ESKF(obs, params.static_time, dt);

fprintf('GPS Origin: [%.6f, %.6f, %.3f]\n', eskf.gps_origin(1), eskf.gps_origin(2), eskf.gps_origin(3));
fprintf('Initial position: [%.6f, %.6f, %.6f]\n', eskf.p(1), eskf.p(2), eskf.p(3));

% 最初のGPS更新を手動実行
k = 10;  % GPS更新タイミング
lat = obs.lat(k);
lon = obs.lon(k);
alt = obs.alt(k);

fprintf('\nGPS observation: lat=%.6f, lon=%.6f, alt=%.3f\n', lat, lon, alt);

% 更新前の状態
p_before = eskf.p;

% GPS更新実行
try
    eskf.update_gps(lat, lon, alt, k);
    fprintf('GPS update successful\n');
catch ME
    fprintf('GPS update failed: %s\n', ME.message);
    disp(getReport(ME));
end

% 更新後の状態
p_after = eskf.p;
fprintf('\nPosition before: [%.6f, %.6f, %.6f]\n', p_before(1), p_before(2), p_before(3));
fprintf('Position after:  [%.6f, %.6f, %.6f]\n', p_after(1), p_after(2), p_after(3));
fprintf('Delta: [%.6f, %.6f, %.6f]\n', p_after(1)-p_before(1), p_after(2)-p_before(2), p_after(3)-p_before(3));

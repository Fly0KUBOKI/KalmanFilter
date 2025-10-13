function run_simulation()
% RUN_SIMULATION  メイン実行スクリプト（簡易ESKFデモ）
%
% このスクリプトは、GenerateData/observation.csv を読み込み、簡易的な
% ESKF フィルタを実行して Results/estimation.csv を出力し、グラフを表示します.

projRoot = fileparts(mfilename('fullpath'));

% MATLAB パスにサブフォルダを追加
addpath(fullfile(projRoot, 'KalmanFilter'));
addpath(fullfile(projRoot, 'GenerateData'));
addpath(fullfile(projRoot, 'Graph'));
addpath(fullfile(projRoot, 'KalmanFilter', 'Common Calculations'));

dataDir = fullfile(projRoot, 'GenerateData');
obsFile = fullfile(dataDir, 'sensor_data.csv');
if ~exist(obsFile, 'file')
    error('sensor_data.csv が見つかりません: %s', obsFile);
end

obs = read_csv(obsFile);

% 推定ノイズパラメータ
[Q, R_gps, R_mag, R_baro] = estimate_noise_params(obs);

% 設定
if length(obs.time) < 2
    error('観測データが短すぎます');
end
settings.dt = mean(diff(obs.time));
settings.freq_mag = 4;   % 1/4ステップ
settings.freq_baro = 8;  % 1/8ステップ
settings.freq_gps = 40;  % 1/40ステップ
settings.Q = Q; settings.R_gps = R_gps; settings.R_mag = R_mag; settings.R_baro = R_baro;

% Delegate ESKF execution to eskf_update (keeping run_simulation for I/O & plotting)
% Sequentially call eskf_update for each timestep
N = numel(obs.time);
results.time = obs.time(:)';
results.p = zeros(3, N);
results.v = zeros(3, N);
results.euler = zeros(3, N);

% initial nominal state
p = zeros(3,1);
v = zeros(3,1);
q = quat_lib('quatnormalize', [1;0;0;0]);
ba = zeros(3,1);
bg = zeros(3,1);

P = eye(15) * 0.01;

for k = 1:N
    [p, v, q, ba, bg, P, status] = eskf_update(p, v, q, ba, bg, P, obs, settings, k);
    results.p(:,k) = p;
    results.v(:,k) = v;
    euler_angles = quat_lib('quat_to_euler', q);
    roll = euler_angles(1);
    pitch = euler_angles(2);
    yaw = euler_angles(3);
    results.euler(:,k) = [roll; pitch; yaw];
end

outDir = fullfile(projRoot, 'Results');
if ~exist(outDir,'dir'), mkdir(outDir); end
outFile = fullfile(outDir, 'estimation.csv');

% 保存
T = table(results.time(:), results.p(1,:)', results.p(2,:)', results.p(3,:)', ...
    results.v(1,:)', results.v(2,:)', results.v(3,:)', ...
    results.euler(1,:)', results.euler(2,:)', results.euler(3,:)');
T.Properties.VariableNames = {'time','px','py','pz','vx','vy','vz','roll','pitch','yaw'};
writetable(T, outFile);

% 可視化
plot_csv(outFile);

fprintf('Estimation saved to %s\n', outFile);
end

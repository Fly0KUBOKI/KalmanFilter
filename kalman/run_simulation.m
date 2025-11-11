function run_simulation()
% RUN_SIMULATION  メイン実行スクリプト（ESKF）
%
% このスクリプトは、GenerateData/sensor_data.csv を読み込み、
% ESKF フィルタを実行して Results/estimation.csv を出力し、グラフを表示します.

clc;
rehash;
projRoot = fileparts(mfilename('fullpath'));

% MATLAB パスにサブフォルダを追加
addpath(genpath(fullfile(projRoot, 'KF')));       % KF/Core, KF/Utils を含む
addpath(genpath(fullfile(projRoot, 'ESKF')));     % ESKF/@ESKF, ESKF/Core を含む
addpath(genpath(fullfile(projRoot, 'UKF')));      % UKF/Core を含む
addpath(genpath(fullfile(projRoot, 'EKF')));      % EKF を含む
addpath(fullfile(projRoot, 'Graph'));
addpath(fullfile(projRoot, 'GenerateData'));

sim_generate();  % データ生成を呼び出し

dataDir = fullfile(projRoot, 'GenerateData');
obsFile = fullfile(dataDir, 'sensor_data.csv');
if ~exist(obsFile, 'file')
    error('sensor_data.csv が見つかりません: %s', obsFile);
end

obs = read_csv(obsFile);

% 設定パラメータを読み込み
params = config_params();

% カルマンフィルタの初期化
if length(obs.time) < 2
    error('観測データが短すぎます');
end
dt = mean(diff(obs.time));
kf = ESKF(obs, params.static_time, dt);
 
% ESKFフィルタリング実行
N = numel(obs.time);
results.time = obs.time(:)';
results.p = zeros(3, N);
results.v = zeros(3, N);
results.euler = zeros(3, N);
results.ba = zeros(3, N);
results.bg = zeros(3, N);

for k = 1:N
    kf.updateFilter(obs, k);
    results.p(:,k) = kf.p;
    results.v(:,k) = kf.v;
    results.euler(:,k) = kf.getEuler();
    results.ba(:,k) = kf.ba;
    results.bg(:,k) = kf.bg;
    if mod(k, 10000) == 0
        fprintf('Step %d / %d\n', k, N);
    end
end

outDir = fullfile(projRoot, 'Results');
if ~exist(outDir,'dir'), mkdir(outDir); end
outFile = fullfile(outDir, 'estimation.csv');

% 保存
T = table(results.time(:), ...  
    results.p(1,:)', results.p(2,:)', results.p(3,:)', ...
    results.v(1,:)', results.v(2,:)', results.v(3,:)', ...
    results.euler(1,:)', results.euler(2,:)', results.euler(3,:)', ...
    results.ba(1,:)', results.ba(2,:)', results.ba(3,:)', ...
    results.bg(1,:)', results.bg(2,:)', results.bg(3,:)');

T.Properties.VariableNames = {'time','px','py','pz','vx','vy','vz','roll','pitch','yaw','ba_x','ba_y','ba_z','bg_x','bg_y','bg_z'};
writetable(T, outFile);

% 可視化
plot_csv(outFile, 'time');

fprintf('Estimation saved to %s\n', outFile);
end

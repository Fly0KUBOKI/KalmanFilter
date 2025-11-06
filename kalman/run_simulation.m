function run_simulation()
% RUN_SIMULATION  メイン実行スクリプト（ESKF）
%
% このスクリプトは、GenerateData/sensor_data.csv を読み込み、
% ESKF フィルタを実行して Results/estimation.csv を出力し、グラフを表示します.

clc;
rehash;
projRoot = fileparts(mfilename('fullpath'));
sim_generate();  % データ生成を呼び出し

% MATLAB パスにサブフォルダを追加
addpath(genpath(fullfile(projRoot, 'KF')));       % KF/Core, KF/Utils を含む
addpath(genpath(fullfile(projRoot, 'ESKF')));     % ESKF/@ESKF, ESKF/Core を含む
addpath(genpath(fullfile(projRoot, 'UKF')));      % UKF/Core を含む
addpath(genpath(fullfile(projRoot, 'EKF')));      % EKF を含む
addpath(fullfile(projRoot, 'Graph'));
addpath(fullfile(projRoot, 'GenerateData'));

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
    % --- reproducibility: fix RNG seed for deterministic runs during diagnosis ---
    seed = 0; rng(seed);

    % --- enable detailed debug dump at the suspected problematic index ---
    kf.debugDumpK = 2410;

    % --- set up per-step debug logging callback (writes debug_step_log_<ts>.csv) ---
    ts = datestr(now,'yyyymmdd_HHMMSS');
    logFile = fullfile(fileparts(mfilename('fullpath')), 'Results', sprintf('debug_step_log_%s.csv', ts));
    % prepare file and header
    fid = fopen(logFile, 'w');
    if fid > 0
        fprintf(fid, 'k,stage,traceP,rcondP,K_norm,y_norm\n');
    else
        warning('Could not open debug log file: %s', logFile);
    end
    % attach callback
    kf.debugCallback = @(info) debug_logger(info);

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

% finalize log file
try
    if exist('fid','var') && fid > 0
        fclose(fid);
        fprintf('Debug step log written to %s\n', logFile);
    end
catch
end

% nested function: debug logger uses parent fid
function debug_logger(info)
    try
        if ~exist('fid','var') || fid <= 0
            return;
        end
        kx = NaN; st = ''; traceP = NaN; rcondP = NaN; K_norm = NaN; y_norm = NaN;
        if isfield(info,'k'), kx = info.k; end
        if isfield(info,'stage'), st = info.stage; end
        if isfield(info,'P')
            try, traceP = trace(info.P); catch, traceP = NaN; end
            try, rcondP = rcond(info.P); catch, rcondP = NaN; end
        end
        if isfield(info,'K_norm'), K_norm = info.K_norm; end
        if isfield(info,'y')
            try, y_norm = norm(info.y); catch, y_norm = NaN; end
        end
    fprintf(fid, '%d,%s,%.12g,%.12g,%.12g,%.12g\n', kx, st, traceP, rcondP, K_norm, y_norm);
    catch
    end
end

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

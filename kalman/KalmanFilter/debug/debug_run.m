% debug_run.m - integrated debug harness
% Usage:
%   debug_run(); % default: in-memory sweep, summary only
%   debug_run(true, true); % sweep_write_csv=true, write_all=true

function debug_run(sweep_write_csv, write_all)
    if nargin < 1 || isempty(sweep_write_csv)
        sweep_write_csv = false;
    end
    if nargin < 2 || isempty(write_all)
        write_all = false;
    end

    clearvars -except sweep_write_csv write_all; close all; clc;

    root = fileparts(mfilename('fullpath'));
    repoRoot = fullfile(root,'..','..');

    % ensure paths
    if ~contains(path(), repoRoot)
        addpath(repoRoot);
    end
    % ensure debug folder itself is on the path so helper functions (run_trial) are visible
    if ~contains(path(), root)
        addpath(root);
    end
    addpath(fullfile(repoRoot,'GenerateData'));
    addpath(fullfile(repoRoot,'Graph'));
    addpath(fullfile(repoRoot,'KalmanFilter'));
    % also ensure KalmanFilter/debug is on the path (where run_trial.m lives)
    addpath(fullfile(repoRoot,'KalmanFilter','debug'));

    % load base params
    base_params = config_params();
    base_params.dt = 0.01;
    base_params.T = 100.0;
    base_params.kf.type = 'kf';
    base_params.kf.debug = true;
    base_params.kf.maha_gate = 9;
    base_params.kf.adaptive_R_enabled = false;
    base_params.kf.maha_disable_steps = 3;
    base_params.data.source = 'csv';
    base_params.data.file = fullfile(repoRoot,'GenerateData','sim_data.csv');

    % load CSV data
    [~, meas_full, csvN] = load_sim_data(base_params.data.file);
    maxIter = min(floor(base_params.T/base_params.dt)+1, csvN);

    % Run three trials: baseline, initGPS, initGPS+noGate
    fprintf('Starting baseline trial (maha_gate=%g)\n', base_params.kf.maha_gate);
    run_trial(root, base_params, meas_full, maxIter, 'debug_output_baseline.csv', false, false);

    fprintf('Starting trial: init from first GPS\n');
    run_trial(root, base_params, meas_full, maxIter, 'debug_output_initgps.csv', true, false);

    fprintf('Starting trial: init from first GPS + disable gating\n');
    run_trial(root, base_params, meas_full, maxIter, 'debug_output_initgps_nogate.csv', true, true);

    % Run sweep (via analyze_debug_outputs)
    fprintf('Running sweep (sweep_write_csv=%d, write_all=%d)\n', sweep_write_csv, write_all);
    try
        analyze_debug_outputs(false, true, false);
    catch ME
        warning(ME.identifier, '%s', ME.message);
    end
	
end


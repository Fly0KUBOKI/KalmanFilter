function run_simulation(seed, skip_data_gen)
    % run_simulation - ESKF simulation using MEX implementation
    % Uses mex_run_eskf for all ESKF operations
    
    clc; rehash; clear functions;
    if nargin >= 1 && ~isempty(seed); rng(seed, 'twister'); end
    if nargin < 2; skip_data_gen = false; end
    
    proj_root = fileparts(mfilename('fullpath'));
    add_paths(proj_root);
    if ~skip_data_gen; sim_generate(); end
    
    obs = read_observation(proj_root);
    params = config_params();
    dt = calculate_dt(obs);
    
    % Initialize ESKF via MEX
    handle = mex_run_eskf('init', obs, params.static_time, dt);
    
    try
        results = run_filter_mex(handle, obs, params.static_time, dt);
        save_results(proj_root, results);
    catch ME
        mex_run_eskf('free', handle);
        rethrow(ME);
    end
    
    % Cleanup
    mex_run_eskf('free', handle);
end

function add_paths(proj_root)
    addpath(fullfile(proj_root, 'Graph'));
    addpath(fullfile(proj_root, 'GenerateData'));
    addpath(fullfile(proj_root, 'cpp', 'bin'));
end

function obs = read_observation(proj_root)
    obs_file = fullfile(proj_root, 'GenerateData', 'sensor_data.csv');
    if ~exist(obs_file, 'file'); error('sensor_data.csv not found: %s', obs_file); end
    obs = read_csv(obs_file);
end

function dt = calculate_dt(obs)
    if length(obs.time) < 2; error('観測データが短すぎます'); end
    dt = mean(diff(obs.time));
end

function results = run_filter_mex(handle, obs, static_time, dt)
    % MEX mode filter execution
    n_samples = numel(obs.time);
    static_samples = floor(static_time / dt);
    fprintf('初期化期間: %.1f秒 (%d サンプル)\n', static_time, static_samples);
    
    results.time = obs.time(:)';
    results.p = zeros(3, n_samples);
    results.v = zeros(3, n_samples);
    results.euler = zeros(3, n_samples);
    results.ba = zeros(3, n_samples);
    results.bg = zeros(3, n_samples);
    results.innov_norm = zeros(1, n_samples);
    results.maha_dist = zeros(1, n_samples);
    
    for k = 1:n_samples
        if k == 1; fprintf('Start loop\n'); end
        
        if k > static_samples
            % Run one ESKF step (predict + sensor updates + reset + zupt)
            mex_run_eskf('step', handle, obs, k);
        end
        
        % Get current state
        state = mex_run_eskf('get_state', handle);
        
        results.p(:,k) = state.p;
        results.v(:,k) = state.v;
        results.euler(:,k) = state.euler;
        results.ba(:,k) = state.ba;
        results.bg(:,k) = state.bg;
        results.innov_norm(k) = 0;
        results.maha_dist(k) = 0;
        
        if mod(k, 1000) == 0; fprintf('Step %d / %d\n', k, n_samples); end
    end
    fprintf('推定完了\n');
end

function save_results(proj_root, results)
    out_dir = fullfile(proj_root, 'Results');
    if ~exist(out_dir,'dir'); mkdir(out_dir); end
    T = table(results.time(:), results.p(1,:)', results.p(2,:)', results.p(3,:)', ...
        results.v(1,:)', results.v(2,:)', results.v(3,:)', ...
        results.euler(1,:)', results.euler(2,:)', results.euler(3,:)', ...
        results.ba(1,:)', results.ba(2,:)', results.ba(3,:)', ...
        results.bg(1,:)', results.bg(2,:)', results.bg(3,:)', ...
        results.innov_norm(:), results.maha_dist(:), ...
        'VariableNames', {'time','px','py','pz','vx','vy','vz','roll','pitch','yaw',...
        'ba_x','ba_y','ba_z','bg_x','bg_y','bg_z','innov_norm','maha_dist'});
    writetable(T, fullfile(out_dir, 'estimation.csv'));
end

function plot_results(proj_root)
    plot_csv(fullfile(proj_root, 'Results', 'estimation.csv'), 'time');
end

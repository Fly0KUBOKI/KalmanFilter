function loop_elapsed = run_simulation(seed, skip_data_gen)
    % run_simulation - ESKF simulation using MEX implementation
    % Uses mex_hybrid_filter for all ESKF operations
    % Sensor dt is calculated dynamically based on timestamp changes
    
    clc; rehash; clear functions;
    if nargin >= 1 && ~isempty(seed); rng(seed, 'twister'); end
    if nargin < 2; skip_data_gen = false; end
    
    proj_root = fileparts(mfilename('fullpath'));
    add_paths(proj_root);
    if ~skip_data_gen; sim_generate(); end
    
    obs = read_observation(proj_root);
    params = config_params();
    
    % Calculate default dt for initialization
    % (until C++ is updated to accept only obs and static_time)
    dt = calculate_dt(obs);
    
    % Initialize ESKF via MEX
    handle = mex_hybrid_filter('init', obs, params.static_time, dt);
    
    % Check initial state right after initialization to verify relative yaw starts at 0
    state_init = mex_hybrid_filter('get_state', handle);
    fprintf('\n=== State immediately after initialization ===\n');
    fprintf('Initial Quaternion (q_init): [w=%.6f, x=%.6f, y=%.6f, z=%.6f]\n', ...
        state_init.q(1), state_init.q(2), state_init.q(3), state_init.q(4));
    fprintf('Initial Euler (relative to q_init): roll=%.2f°, pitch=%.2f°, yaw=%.2f°\n', ...
        state_init.euler(1), state_init.euler(2), state_init.euler(3));
    if abs(state_init.euler(3)) < 1.0
        fprintf('✓ PASS: Initial yaw is close to 0° (%.2f°)\n\n', state_init.euler(3));
    else
        fprintf('✗ WARNING: Initial yaw deviates from 0° (%.2f°)\n\n', state_init.euler(3));
    end
    
    try
        [results, loop_elapsed] = run_filter_mex(handle, obs, params.static_time);
        save_results(proj_root, results);
        % Show plots of the results
        plot_results(proj_root);
    catch ME
        mex_hybrid_filter('free', handle);
        rethrow(ME);
    end
    
    % Cleanup
    mex_hybrid_filter('free', handle);
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
    % Calculate default dt from time vector
    if length(obs.time) < 2
        error('観測データが短すぎます');
    end
    dt = mean(diff(obs.time));
end

function [results, loop_elapsed] = run_filter_mex(handle, obs, static_time)
    % MEX mode filter execution with dynamic sensor dt calculation
    % Each sensor's dt is calculated based on timestamp changes
    
    n_samples = numel(obs.time);
    
    % Find static samples by accumulating time differences
    accumulated_time = 0.0;
    static_samples = 0;
    for k = 2:n_samples
        dt_frame = obs.time(k) - obs.time(k-1);
        accumulated_time = accumulated_time + dt_frame;
        if accumulated_time >= static_time
            static_samples = k - 1;
            break;
        end
    end
    if static_samples == 0
        static_samples = min(floor(0.05 * n_samples), 100); % Fallback: 5% of samples or 100 samples
    end
    fprintf('初期化期間: %.1f秒 (%d サンプル)\n', static_time, static_samples);
    
    results.time = single(obs.time(:)');
    results.p = zeros(3, n_samples, 'single');
    results.v = zeros(3, n_samples, 'single');
    results.euler = zeros(3, n_samples, 'single');
    results.ba = zeros(3, n_samples, 'single');
    results.bg = zeros(3, n_samples, 'single');
    results.innov_norm = zeros(1, n_samples, 'single');
    results.maha_dist = zeros(1, n_samples, 'single');
    
    % initialize loop timer
    start_loop_tic = [];
    loop_elapsed = 0.0;

    for k = 1:n_samples
        if k == 1
            fprintf('Start loop\n');
            start_loop_tic = tic;
        end
        
        % Prepare sensor struct with all necessary information
        sens = struct();
        sens.accel = single([obs.ax(k); obs.ay(k); obs.az(k)]);
        sens.gyro = single([obs.wx(k); obs.wy(k); obs.wz(k)]);
        sens.mag = single([obs.mx(k); obs.my(k); obs.mz(k)]);
        sens.gps_pos = double([obs.lat(k); obs.lon(k); obs.alt(k)]);
        sens.alt_baro = single(obs.pressure(k));
        
        % Current timestamp (in seconds)
        sens.current_time = double(obs.time(k));
        
        % Previous sensor timestamp (initialized as current if first sample)
        if k == 1
            sens.prev_time_accel = double(obs.time(k));
            sens.prev_time_gyro = double(obs.time(k));
            sens.prev_time_mag = double(obs.time(k));
            sens.prev_time_gps = double(obs.time(k));
            sens.prev_time_baro = double(obs.time(k));
        else
            sens.prev_time_accel = double(obs.time(k-1));
            sens.prev_time_gyro = double(obs.time(k-1));
            sens.prev_time_mag = double(obs.time(k-1));
            sens.prev_time_gps = double(obs.time(k-1));
            sens.prev_time_baro = double(obs.time(k-1));
        end
        
        % Update flags (all sensors available in simulation)
        sens.update_accel = uint8(1);
        sens.update_gyro = uint8(1);
        sens.update_mag = uint8(1);
        sens.update_gps = uint8(1);
        sens.update_baro = uint8(1);
        sens.update_zupt = uint8(0); % No ZUPT in outdoor simulation
        
        if k > static_samples
            % Run one ESKF step (predict + sensor updates + reset + zupt)
            % NOTE: current MEX binary expects (handle, obs, k).
            % Use compatible call until MEX is rebuilt to accept sensor struct.
            mex_hybrid_filter('step', handle, obs, k);
        end
        
        % Get current state (float output)
        state = mex_hybrid_filter('get_state', handle);
        
        results.p(:,k) = single(state.p);

        results.v(:,k) = single(state.v);
        results.euler(:,k) = single(state.euler);
        results.ba(:,k) = single(state.ba);
        results.bg(:,k) = single(state.bg);
        results.innov_norm(k) = single(0);
        results.maha_dist(k) = single(0);
        
        if mod(k, 1000) == 0; fprintf('Step %d / %d\n', k, n_samples); end
    end
    fprintf('推定完了\n');

    if ~isempty(start_loop_tic)
        loop_elapsed = toc(start_loop_tic);
    else
        loop_elapsed = 0.0;
    end
end

function save_results(proj_root, results)
    out_dir = fullfile(proj_root, 'Results');
    if ~exist(out_dir,'dir'); mkdir(out_dir); end
    
    % 位置、速度、姿勢はfloatのまま出力（GPSデータのみdoubleを使用）
    % time, ba, bg, innov_norm, maha_distもfloatのまま
    T = table(single(results.time(:)), single(results.p(1,:)'), single(results.p(2,:)'), single(results.p(3,:)'), ...
        single(results.v(1,:)'), single(results.v(2,:)'), single(results.v(3,:)'), ...
        single(results.euler(1,:)'), single(results.euler(2,:)'), single(results.euler(3,:)'), ...
        single(results.ba(1,:)'), single(results.ba(2,:)'), single(results.ba(3,:)'), ...
        single(results.bg(1,:)'), single(results.bg(2,:)'), single(results.bg(3,:)'), ...
        single(results.innov_norm(:)), single(results.maha_dist(:)), ...
        'VariableNames', {'time','px','py','pz','vx','vy','vz','roll','pitch','yaw',...
        'ba_x','ba_y','ba_z','bg_x','bg_y','bg_z','innov_norm','maha_dist'});
    writetable(T, fullfile(out_dir, 'estimation.csv'));
end

function plot_results(proj_root)
    % 時系列グラフを表示（'time'モードを明示的に指定）
    plot_csv(fullfile(proj_root, 'Results', 'estimation.csv'), 'time');
end

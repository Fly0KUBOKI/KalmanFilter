function run_simulation(seed, skip_data_gen)
    clc; rehash; clear functions;
    if nargin >= 1 && ~isempty(seed); rng(seed, 'twister'); end
    if nargin < 2; skip_data_gen = false; end
    
    proj_root = fileparts(mfilename('fullpath'));
    add_paths(proj_root);
    if ~skip_data_gen; sim_generate(); end
    
    obs = read_observation(proj_root);
    params = config_params();
    dt = calculate_dt(obs);
    eskf = ESKF(obs, params.static_time, dt);
    results = run_filter(eskf, obs);
    save_results(proj_root, results);
    
    % プロット生成をスキップ（バッチ実行時のエラー回避）
    try
        plot_results(proj_root);
    catch e
        warning('プロット生成失敗: %s', e.message);
    end
    fprintf('推定完了\n');
end

function add_paths(proj_root)
    addpath(genpath(fullfile(proj_root, 'ESKF')));
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

function results = run_filter(eskf, obs)
    n_samples = numel(obs.time);
    params = config_params();
    static_samples = floor(params.static_time / eskf.dt);
    fprintf('初期化期間: %.1f秒 (%d サンプル)\n', params.static_time, static_samples);
    
    results.time = obs.time(:)';
    results.p = zeros(3, n_samples);
    results.v = zeros(3, n_samples);
    results.euler = zeros(3, n_samples);
    results.ba = zeros(3, n_samples);
    results.bg = zeros(3, n_samples);
    results.innov_norm = zeros(1, n_samples);
    results.maha_dist = zeros(1, n_samples);

    % Disable automatic saving of debug `record_*` files unless explicitly enabled
    ENABLE_SAVE_TRACES = false;

    for k = 1:n_samples
        if k == 1; fprintf('Start loop\n'); end
        a = [obs.ax(k); obs.ay(k); obs.az(k)];
        w = deg2rad([obs.wx(k); obs.wy(k); obs.wz(k)]);
        
        if k > static_samples
            eskf.predict(a, w);
            if eskf.zupt('check', a, w); eskf.zupt('update'); end

            % If TRACE_SAMPLE is set, save eskf state before/after each sensor update
            try
                trace_sample_env = getenv('TRACE_SAMPLE'); trace_sample_num = str2double(trace_sample_env);
            catch
                trace_sample_num = NaN;
            end

            if mod(k, eskf.freq_accel) == 0
                if ENABLE_SAVE_TRACES && ~isnan(trace_sample_num) && k==trace_sample_num
                    outdir_dbg = fullfile(fileparts(mfilename('fullpath')), '..', 'Results'); if ~exist(outdir_dbg,'dir'), mkdir(outdir_dbg); end
                    fname = fullfile(outdir_dbg, sprintf('record_before_accel_%d.mat', k));
                    try eskf_state.p = eskf.p; eskf_state.v = eskf.v; eskf_state.q = eskf.q; eskf_state.P = eskf.P; save(fname, 'k', 'eskf_state'); catch end
                end
                eskf.sensor_updates('accel', a, k);
                if ENABLE_SAVE_TRACES && ~isnan(trace_sample_num) && k==trace_sample_num
                    outdir_dbg = fullfile(fileparts(mfilename('fullpath')), '..', 'Results'); if ~exist(outdir_dbg,'dir'), mkdir(outdir_dbg); end
                    fname = fullfile(outdir_dbg, sprintf('record_after_accel_%d.mat', k));
                    try eskf_state.p = eskf.p; eskf_state.v = eskf.v; eskf_state.q = eskf.q; eskf_state.P = eskf.P; save(fname, 'k', 'eskf_state'); catch end
                end
            end

            if mod(k, eskf.freq_mag) == 0
                if ENABLE_SAVE_TRACES && ~isnan(trace_sample_num) && k==trace_sample_num
                    outdir_dbg = fullfile(fileparts(mfilename('fullpath')), '..', 'Results'); if ~exist(outdir_dbg,'dir'), mkdir(outdir_dbg); end
                    fname = fullfile(outdir_dbg, sprintf('record_before_mag_%d.mat', k));
                    try eskf_state.p = eskf.p; eskf_state.v = eskf.v; eskf_state.q = eskf.q; eskf_state.P = eskf.P; save(fname, 'k', 'eskf_state'); catch end
                end
                eskf.sensor_updates('mag', [obs.mx(k); obs.my(k); obs.mz(k)], k);
                if ENABLE_SAVE_TRACES && ~isnan(trace_sample_num) && k==trace_sample_num
                    outdir_dbg = fullfile(fileparts(mfilename('fullpath')), '..', 'Results'); if ~exist(outdir_dbg,'dir'), mkdir(outdir_dbg); end
                    fname = fullfile(outdir_dbg, sprintf('record_after_mag_%d.mat', k));
                    try eskf_state.p = eskf.p; eskf_state.v = eskf.v; eskf_state.q = eskf.q; eskf_state.P = eskf.P; save(fname, 'k', 'eskf_state'); catch end
                end
            end

            if mod(k, eskf.freq_baro) == 0
                if ENABLE_SAVE_TRACES && ~isnan(trace_sample_num) && k==trace_sample_num
                    outdir_dbg = fullfile(fileparts(mfilename('fullpath')), '..', 'Results'); if ~exist(outdir_dbg,'dir'), mkdir(outdir_dbg); end
                    fname = fullfile(outdir_dbg, sprintf('record_before_baro_%d.mat', k));
                    try eskf_state.p = eskf.p; eskf_state.v = eskf.v; eskf_state.q = eskf.q; eskf_state.P = eskf.P; save(fname, 'k', 'eskf_state'); catch end
                end
                eskf.sensor_updates('baro', obs.pressure(k), k);
                if ENABLE_SAVE_TRACES && ~isnan(trace_sample_num) && k==trace_sample_num
                    outdir_dbg = fullfile(fileparts(mfilename('fullpath')), '..', 'Results'); if ~exist(outdir_dbg,'dir'), mkdir(outdir_dbg); end
                    fname = fullfile(outdir_dbg, sprintf('record_after_baro_%d.mat', k));
                    try eskf_state.p = eskf.p; eskf_state.v = eskf.v; eskf_state.q = eskf.q; eskf_state.P = eskf.P; save(fname, 'k', 'eskf_state'); catch end
                end
            end

            if mod(k, eskf.freq_gps) == 0 && ~isnan(obs.lat(k))
                if ENABLE_SAVE_TRACES && ~isnan(trace_sample_num) && k==trace_sample_num
                    outdir_dbg = fullfile(fileparts(mfilename('fullpath')), '..', 'Results'); if ~exist(outdir_dbg,'dir'), mkdir(outdir_dbg); end
                    fname = fullfile(outdir_dbg, sprintf('record_before_gps_%d.mat', k));
                    try eskf_state.p = eskf.p; eskf_state.v = eskf.v; eskf_state.q = eskf.q; eskf_state.P = eskf.P; save(fname, 'k', 'eskf_state'); catch end
                end
                eskf.sensor_updates('gps', obs.lat(k), obs.lon(k), obs.alt(k), k);
                if ENABLE_SAVE_TRACES && ~isnan(trace_sample_num) && k==trace_sample_num
                    outdir_dbg = fullfile(fileparts(mfilename('fullpath')), '..', 'Results'); if ~exist(outdir_dbg,'dir'), mkdir(outdir_dbg); end
                    fname = fullfile(outdir_dbg, sprintf('record_after_gps_%d.mat', k));
                    try eskf_state.p = eskf.p; eskf_state.v = eskf.v; eskf_state.q = eskf.q; eskf_state.P = eskf.P; save(fname, 'k', 'eskf_state'); catch end
                end
            end
        end
        
        eskf.reset('check', obs, k);
        % If TRACE_SAMPLE env var set, save eskf state at that sample for debugging
            try
                trace_sample_env = getenv('TRACE_SAMPLE');
                trace_sample_num = str2double(trace_sample_env);
                if ENABLE_SAVE_TRACES && ~isnan(trace_sample_num) && k == trace_sample_num
                    outdir_dbg = fullfile(proj_root, 'Results'); if ~exist(outdir_dbg,'dir'), mkdir(outdir_dbg); end
                    % Save only serializable state (avoid saving full object)
                    eskf_state.p = eskf.p;
                    eskf_state.v = eskf.v;
                    eskf_state.q = eskf.q;
                    eskf_state.ba = eskf.ba;
                    eskf_state.bg = eskf.bg;
                    eskf_state.P = eskf.P;
                    % include dt if available
                    try, eskf_state.dt = eskf.dt; catch, eskf_state.dt = [];, end
                    try
                        % Save eskf_state under a distinct filename to avoid overwriting
                        % the immediate mex debug record (which may be saved earlier).
                        save(fullfile(outdir_dbg, sprintf('record_runfilter_sample_%d_eskfstate.mat', k)), 'k', 'eskf_state');
                    catch
                        % ignore save failures
                    end
                end
            catch
            end
        results.p(:,k) = eskf.p;
        results.v(:,k) = eskf.v;
        results.euler(:,k) = eskf.utils('get_euler');
        results.ba(:,k) = eskf.ba;
        results.bg(:,k) = eskf.bg;
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

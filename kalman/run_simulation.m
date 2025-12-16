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
    addpath(genpath(fullfile(proj_root, 'KF')));
    addpath(genpath(fullfile(proj_root, 'ESKF')));
    addpath(genpath(fullfile(proj_root, 'UKF')));
    addpath(genpath(fullfile(proj_root, 'EKF')));
    addpath(fullfile(proj_root, 'Graph'));
    addpath(fullfile(proj_root, 'GenerateData'));
    % Common削除完了: 全機能をC++/ESKF内部に統合
    addpath(genpath(fullfile(proj_root, 'cpp')));
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

    for k = 1:n_samples
        if k == 1; fprintf('Start loop\n'); end
        a = [obs.ax(k); obs.ay(k); obs.az(k)];
        w = deg2rad([obs.wx(k); obs.wy(k); obs.wz(k)]);
        
        if k > static_samples
            eskf.predict(a, w);
            if eskf.zupt('check', a, w); eskf.zupt('update'); end
            if mod(k, eskf.freq_accel) == 0; eskf.sensor_updates('accel', a); end
            if mod(k, eskf.freq_mag) == 0; eskf.sensor_updates('mag', [obs.mx(k); obs.my(k); obs.mz(k)]); end
            if mod(k, eskf.freq_baro) == 0; eskf.sensor_updates('baro', obs.pressure(k)); end
            if mod(k, eskf.freq_gps) == 0 && ~isnan(obs.lat(k))
                eskf.sensor_updates('gps', obs.lat(k), obs.lon(k), obs.alt(k), k);
            end
        end
        
        eskf.reset('check', obs, k);
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

function compare_implementations(seed)
    % 旧実装(MATLAB MEUKF)と現在実装(C++ MEX)を比較
    % seed: 乱数シード（省略可）

    clc;
    rehash;
    clear functions;
    
    if nargin >= 1 && ~isempty(seed)
        rng(seed, 'twister');
    end
    
    proj_root = fileparts(mfilename('fullpath'));
    add_paths(proj_root);

    % データ読み込み
    obs = read_observation(proj_root);
    params = config_params();
    dt = calculate_dt(obs);
    
    fprintf('\n=== 旧実装 (MATLAB MEUKF) 実行 ===\n');
    eskf_old = ESKF_Old(obs, params.static_time, dt);
    eskf_old.use_meukf = true;  % MATLAB MEUKF使用
    results_old = run_filter(eskf_old, obs);
    save_results_with_suffix(proj_root, results_old, '_old');
    
    fprintf('\n=== 現在実装 (C++ MEX) 実行 ===\n');
    eskf_new = ESKF(obs, params.static_time, dt);
    eskf_new.use_meukf = true;  % C++ MEX使用
    results_new = run_filter(eskf_new, obs);
    save_results_with_suffix(proj_root, results_new, '_new');
    
    fprintf('\n=== 結果比較 ===\n');
    compare_results(results_old, results_new);
    
    fprintf('\n完了\n');
end

function add_paths(proj_root)
    addpath(genpath(fullfile(proj_root, 'KF')));
    addpath(genpath(fullfile(proj_root, 'ESKF')));
    addpath(genpath(fullfile(proj_root, 'UKF')));
    addpath(genpath(fullfile(proj_root, 'EKF')));
    addpath(fullfile(proj_root, 'Graph'));
    addpath(fullfile(proj_root, 'GenerateData'));
    addpath(genpath(fullfile(proj_root, 'Common')));
    addpath(genpath(fullfile(proj_root, 'cpp')));
end

function obs = read_observation(proj_root)
    data_dir = fullfile(proj_root, 'GenerateData');
    obs_file = fullfile(data_dir, 'sensor_data.csv');
    if ~exist(obs_file, 'file')
        error('sensor_data.csv が見つかりません: %s', obs_file);
    end
    obs = read_csv(obs_file);
end

function dt = calculate_dt(obs)
    if length(obs.time) < 2
        error('観測データが短すぎます');
    end
    dt = mean(diff(obs.time));
end

function results = run_filter(eskf, obs)
    n_samples = numel(obs.time);
    params = config_params();
    static_samples = floor(params.static_time / eskf.dt);
    
    results.time = obs.time(:)';
    results.p = zeros(3, n_samples);
    results.v = zeros(3, n_samples);
    results.euler = zeros(3, n_samples);
    results.ba = zeros(3, n_samples);
    results.bg = zeros(3, n_samples);
    results.P_diag = zeros(15, n_samples);
    results.zupt_active = zeros(1, n_samples);
    results.a_filtered = zeros(3, n_samples);
    results.innov_norm = zeros(1, n_samples);
    results.maha_dist = zeros(1, n_samples);
    results.gain_norm = zeros(1, n_samples);
    
    tic;
    for k = 1:n_samples
        obs_k.time = obs.time(k);
        obs_k.ax = obs.ax(k);
        obs_k.ay = obs.ay(k);
        obs_k.az = obs.az(k);
        obs_k.wx = obs.wx(k);
        obs_k.wy = obs.wy(k);
        obs_k.wz = obs.wz(k);
        
        if isfield(obs, 'mx')
            obs_k.mx = obs.mx(k);
            obs_k.my = obs.my(k);
            obs_k.mz = obs.mz(k);
        end
        
        if isfield(obs, 'baro')
            obs_k.baro = obs.baro(k);
        end
        
        if isfield(obs, 'gps_lat')
            obs_k.gps_lat = obs.gps_lat(k);
            obs_k.gps_lon = obs.gps_lon(k);
            obs_k.gps_alt = obs.gps_alt(k);
        end
        
        % ESKF_Oldの場合はupdate_filter、それ以外はupdateFilter
        if isa(eskf, 'ESKF_Old')
            eskf.update_filter(obs_k, k);
        else
            eskf.updateFilter(obs_k, k);
        end
        
        results.p(:,k) = eskf.p;
        results.v(:,k) = eskf.v;
        euler = eskf.getEuler();
        results.euler(:,k) = euler;
        results.ba(:,k) = eskf.ba;
        results.bg(:,k) = eskf.bg;
        results.P_diag(:,k) = diag(eskf.P);
        results.zupt_active(k) = eskf.is_stationary;
        
        if mod(k, 1000) == 0
            fprintf('Step %d / %d\n', k, n_samples);
        end
    end
    elapsed = toc;
    fprintf('実行時間: %.2f秒\n', elapsed);
end

function save_results_with_suffix(proj_root, results, suffix)
    out_dir = fullfile(proj_root, 'Results');
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end
    
    out_file = fullfile(out_dir, ['estimation', suffix, '.csv']);
    
    data_table = table(...
        results.time(:), ...
        results.p(1,:)', results.p(2,:)', results.p(3,:)', ...
        results.v(1,:)', results.v(2,:)', results.v(3,:)', ...
        results.euler(1,:)', results.euler(2,:)', results.euler(3,:)', ...
        results.ba(1,:)', results.ba(2,:)', results.ba(3,:)', ...
        results.bg(1,:)', results.bg(2,:)', results.bg(3,:)', ...
        results.P_diag(1,:)', results.P_diag(2,:)', results.P_diag(3,:)', ...
        results.P_diag(4,:)', results.P_diag(5,:)', results.P_diag(6,:)', ...
        results.zupt_active(:), ...
        'VariableNames', {...
        'time', 'px', 'py', 'pz', 'vx', 'vy', 'vz', ...
        'roll', 'pitch', 'yaw', ...
        'ba_x', 'ba_y', 'ba_z', 'bg_x', 'bg_y', 'bg_z', ...
        'P_px', 'P_py', 'P_pz', 'P_vx', 'P_vy', 'P_vz', ...
        'zupt_active'});
    
    writetable(data_table, out_file);
    fprintf('結果保存: %s\n', out_file);
end

function compare_results(results_old, results_new)
    % 姿勢角の差分を計算
    euler_diff = results_old.euler - results_new.euler;
    
    % Yaw角は-180~180の範囲で正規化
    euler_diff(3,:) = wrapToPi(euler_diff(3,:));
    
    % RMSEを計算
    rmse_roll = sqrt(mean(euler_diff(1,:).^2));
    rmse_pitch = sqrt(mean(euler_diff(2,:).^2));
    rmse_yaw = sqrt(mean(euler_diff(3,:).^2));
    
    fprintf('姿勢角RMSE (deg):\n');
    fprintf('  Roll:  %.4f\n', rad2deg(rmse_roll));
    fprintf('  Pitch: %.4f\n', rad2deg(rmse_pitch));
    fprintf('  Yaw:   %.4f\n', rad2deg(rmse_yaw));
    
    % 最大差分
    max_roll = max(abs(euler_diff(1,:)));
    max_pitch = max(abs(euler_diff(2,:)));
    max_yaw = max(abs(euler_diff(3,:)));
    
    fprintf('\n最大差分 (deg):\n');
    fprintf('  Roll:  %.4f\n', rad2deg(max_roll));
    fprintf('  Pitch: %.4f\n', rad2deg(max_pitch));
    fprintf('  Yaw:   %.4f\n', rad2deg(max_yaw));
    
    % 位置の差分
    p_diff = results_old.p - results_new.p;
    rmse_p = sqrt(mean(sum(p_diff.^2, 1)));
    max_p = max(sqrt(sum(p_diff.^2, 1)));
    
    fprintf('\n位置RMSE: %.4f m\n', rmse_p);
    fprintf('位置最大差分: %.4f m\n', max_p);
    
    % 発散チェック
    final_euler_old = results_old.euler(:,end);
    final_euler_new = results_new.euler(:,end);
    
    fprintf('\n最終姿勢角 (deg):\n');
    fprintf('  旧実装: Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n', ...
        rad2deg(final_euler_old(1)), rad2deg(final_euler_old(2)), rad2deg(final_euler_old(3)));
    fprintf('  現在実装: Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n', ...
        rad2deg(final_euler_new(1)), rad2deg(final_euler_new(2)), rad2deg(final_euler_new(3)));
end

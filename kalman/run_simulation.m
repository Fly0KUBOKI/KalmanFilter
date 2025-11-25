function run_simulation()
    % ESKF シミュレーション実行

    clc;
    rehash;
    proj_root = fileparts(mfilename('fullpath'));

    add_paths(proj_root);

    sim_generate();

    obs = read_observation(proj_root);
    params = config_params();

    dt = calculate_dt(obs);
    eskf = ESKF(obs, params.static_time, dt);
    
    results = run_filter(eskf, obs);

    save_results(proj_root, results);
    plot_results(proj_root);

    fprintf('推定完了\n');
end

function add_paths(proj_root)
    % パス追加
    addpath(genpath(fullfile(proj_root, 'KF')));
    addpath(genpath(fullfile(proj_root, 'ESKF')));
    addpath(genpath(fullfile(proj_root, 'UKF')));
    addpath(genpath(fullfile(proj_root, 'EKF')));
    addpath(fullfile(proj_root, 'Graph'));
    addpath(fullfile(proj_root, 'GenerateData'));
    addpath(genpath(fullfile(proj_root, 'Common')));
    addpath(genpath(fullfile(proj_root, 'cpp'))); % MEXファイルのパス
end

function obs = read_observation(proj_root)
    % センサーデータ読み込み
    data_dir = fullfile(proj_root, 'GenerateData');
    obs_file = fullfile(data_dir, 'sensor_data.csv');
    if ~exist(obs_file, 'file')
        error('sensor_data.csv が見つかりません: %s', obs_file);
    end
    obs = read_csv(obs_file);
end

function dt = calculate_dt(obs)
    % サンプリング周期計算
    if length(obs.time) < 2
        error('観測データが短すぎます');
    end
    dt = mean(diff(obs.time));
end

function results = run_filter(eskf, obs)
    % フィルタ実行
    n_samples = numel(obs.time);
    results.time = obs.time(:)';
    results.p = zeros(3, n_samples);
    results.v = zeros(3, n_samples);
    results.euler = zeros(3, n_samples);
    results.ba = zeros(3, n_samples);
    results.bg = zeros(3, n_samples);
    
    % プロファイリング用タイマー
    t_predict = 0; t_accel = 0; t_mag = 0; t_baro = 0; t_gps = 0;
    n_accel = 0; n_mag = 0; n_baro = 0; n_gps = 0;

    for k = 1:n_samples
        if k == 1, fprintf('Start loop\n'); end
        % predict
        tic;
        a = [obs.ax(k); obs.ay(k); obs.az(k)];
        w = deg2rad([obs.wx(k); obs.wy(k); obs.wz(k)]);
        eskf.predict(a, w);
        t_predict = t_predict + toc;
        
        % update_accel
        if mod(k, eskf.freq_accel) == 0
            tic;
            eskf.update_accel(a);
            t_accel = t_accel + toc;
            n_accel = n_accel + 1;
        end
        
        % update_mag
        if mod(k, eskf.freq_mag) == 0
            tic;
            eskf.update_mag([obs.mx(k); obs.my(k); obs.mz(k)]);
            t_mag = t_mag + toc;
            n_mag = n_mag + 1;
        end
        
        % update_baro
        if mod(k, eskf.freq_baro) == 0
            tic;
            eskf.update_baro(obs.pressure(k));
            t_baro = t_baro + toc;
            n_baro = n_baro + 1;
        end
        
        % update_gps
        if mod(k, eskf.freq_gps) == 0 && ~isnan(obs.lat(k)) && ~isnan(obs.lon(k))
            tic;
            eskf.update_gps(obs.lat(k), obs.lon(k), obs.alt(k), k);
            t_gps = t_gps + toc;
            n_gps = n_gps + 1;
        end
        
        % 状態記録
        results.p(:,k) = eskf.p;
        results.v(:,k) = eskf.v;
        results.euler(:,k) = eskf.get_euler();
        results.ba(:,k) = eskf.ba;
        results.bg(:,k) = eskf.bg;
        
        if mod(k, 1000) == 0
            fprintf('Step %d / %d\n', k, n_samples);
        end
    end
    
    % プロファイリング結果表示
    fprintf('\n=== 計算時間プロファイル ===\n');
    fprintf('predict:      %.3f s (%.2f us/call)\n', t_predict, t_predict/n_samples*1e6);
    fprintf('update_accel: %.3f s (%.2f us/call, %d calls)\n', t_accel, t_accel/max(n_accel,1)*1e6, n_accel);
    fprintf('update_mag:   %.3f s (%.2f us/call, %d calls)\n', t_mag, t_mag/max(n_mag,1)*1e6, n_mag);
    fprintf('update_baro:  %.3f s (%.2f us/call, %d calls)\n', t_baro, t_baro/max(n_baro,1)*1e6, n_baro);
    fprintf('update_gps:   %.3f s (%.2f us/call, %d calls)\n', t_gps, t_gps/max(n_gps,1)*1e6, n_gps);
    fprintf('Total:        %.3f s\n', t_predict + t_accel + t_mag + t_baro + t_gps);
    fprintf('================================\n\n');
end

function save_results(proj_root, results)
    % 結果保存
    out_dir = fullfile(proj_root, 'Results');
    if ~exist(out_dir,'dir'), mkdir(out_dir); end
    out_file = fullfile(out_dir, 'estimation.csv');

    T = table(results.time(:), ...  
        results.p(1,:)', results.p(2,:)', results.p(3,:)', ...
        results.v(1,:)', results.v(2,:)', results.v(3,:)', ...
        results.euler(1,:)', results.euler(2,:)', results.euler(3,:)', ...
        results.ba(1,:)', results.ba(2,:)', results.ba(3,:)', ...
        results.bg(1,:)', results.bg(2,:)', results.bg(3,:)');

    T.Properties.VariableNames = {'time','px','py','pz','vx','vy','vz','roll','pitch','yaw','ba_x','ba_y','ba_z','bg_x','bg_y','bg_z'};
    writetable(T, out_file);
end

function plot_results(proj_root)
    % 結果プロット
    out_file = fullfile(proj_root, 'Results', 'estimation.csv');
    plot_csv(out_file, 'time');
end

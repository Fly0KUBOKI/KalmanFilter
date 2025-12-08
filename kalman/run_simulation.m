function run_simulation(seed, skip_data_gen)
    % ESKF シミュレーション実行
    % seed: 乱数シード（省略可）
    % skip_data_gen: データ生成をスキップ（省略可、デフォルトfalse）

    clc;
    rehash;
    clear functions; % Persistent変数のクリア
    % 乱数シードはオプション引数で与える。指定がない場合は現状の RNG を使用する。
    if nargin >= 1 && ~isempty(seed)
        rng(seed, 'twister');
    end
    
    if nargin < 2
        skip_data_gen = false;
    end
    
    proj_root = fileparts(mfilename('fullpath'));

    add_paths(proj_root);

    if ~skip_data_gen
        sim_generate();
    end

    obs = read_observation(proj_root);
    params = config_params();

    dt = calculate_dt(obs);
    eskf = ESKF(obs, params.static_time, dt);
    
    results = run_filter(eskf, obs);

    save_results(proj_root, results);
    
    try
        plot_results(proj_root);
    catch e
        warning('プロット生成中にエラーが発生しました: %s', e.message);
    end

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
    
    % 初期化期間の計算（static_time秒間は姿勢・位置更新をスキップ）
    params = config_params();
    static_samples = floor(params.static_time / eskf.dt);
    
    fprintf('初期化期間: %.1f秒 (%d サンプル)\n', params.static_time, static_samples);
    
    results.time = obs.time(:)';
    results.p = zeros(3, n_samples);
    results.v = zeros(3, n_samples);
    results.euler = zeros(3, n_samples);
    results.ba = zeros(3, n_samples);
    results.bg = zeros(3, n_samples);
    results.innov_norm = zeros(1, n_samples);  % イノベーションノルム記録用
    results.innov_norm = zeros(1, n_samples);

    for k = 1:n_samples
        if k == 1, fprintf('Start loop\n'); end
        
        a = [obs.ax(k); obs.ay(k); obs.az(k)];
        w = deg2rad([obs.wx(k); obs.wy(k); obs.wz(k)]);
        
        % 初期化期間後のみ予測・更新を実行
        if k > static_samples
            % predict
            eskf.predict(a, w);
            
            % ZUPT (静止検出と速度更新)
            is_stationary = eskf.check_stationary(a, w);
            if is_stationary
                eskf.update_zupt();
            end
        end
        
        % 初期化期間後のみ姿勢・位置更新を実行
        if k > static_samples
            % update_accel
            if mod(k, eskf.freq_accel) == 0
                eskf.update_accel(a);
            end
            
            % update_mag
            if mod(k, eskf.freq_mag) == 0
                eskf.update_mag([obs.mx(k); obs.my(k); obs.mz(k)]);
            end
            
            % update_baro
            if mod(k, eskf.freq_baro) == 0
                eskf.update_baro(obs.pressure(k));
            end
            
            % update_gps
            if mod(k, eskf.freq_gps) == 0 && ~isnan(obs.lat(k)) && ~isnan(obs.lon(k))
                eskf.update_gps(obs.lat(k), obs.lon(k), obs.alt(k), k);
            end
        end
        
        % 発散チェックとリセット（重要：各ステップで実行）
        eskf.check_and_reset_if_diverged(obs, k);
        
        % 状態記録
        results.p(:,k) = eskf.p;
        results.v(:,k) = eskf.v;
        results.euler(:,k) = eskf.get_euler();
        results.ba(:,k) = eskf.ba;
        results.bg(:,k) = eskf.bg;
        results.innov_norm(k) = eskf.innov_norm;  % イノベーションノルム記録
        results.innov_norm(k) = eskf.innov_norm;
        
        if mod(k, 1000) == 0
            fprintf('Step %d / %d\n', k, n_samples);
        end
    end
    
    fprintf('推定完了\n');
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
        results.bg(1,:)', results.bg(2,:)', results.bg(3,:)', ...
        results.innov_norm(:));

    T.Properties.VariableNames = {'time','px','py','pz','vx','vy','vz','roll','pitch','yaw',...
        'ba_x','ba_y','ba_z','bg_x','bg_y','bg_z','innov_norm'};
    writetable(T, out_file);
end

function plot_results(proj_root)
    % 結果プロット
    out_file = fullfile(proj_root, 'Results', 'estimation.csv');
    plot_csv(out_file, 'time');
end

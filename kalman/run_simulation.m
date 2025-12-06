function run_simulation(seed, skip_data_gen)
    % ESKF シミュレーション実行
    % seed: 乱数シード（省略可）
    % skip_data_gen: データ生成をスキップ（省略可、デフォルトfalse）

    clc;
    rehash;
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
    
    % 診断情報用
    results.P_diag = zeros(15, n_samples);  % 共分散の対角成分
    results.zupt_active = zeros(1, n_samples);  % ZUPTが有効か
    results.a_filtered = zeros(3, n_samples);  % フィルタ済み加速度
    
    % 姿勢発散パラメータ
    results.innov_norm = zeros(1, n_samples);       % イノベーションノルム
    results.maha_dist = zeros(1, n_samples);        % マハラノビス距離
    results.gain_norm = zeros(1, n_samples);        % ゲインノルム
    results.quat_norm = zeros(1, n_samples);        % クォータニオンノルム
    results.att_change_rate = zeros(1, n_samples);  % 姿勢変化率
    
    % プロファイリング用タイマー
    t_predict = 0; t_accel = 0; t_mag = 0; t_baro = 0; t_gps = 0;
    n_accel = 0; n_mag = 0; n_baro = 0; n_gps = 0;

    for k = 1:n_samples
        if k == 1, fprintf('Start loop\n'); end
        
        a = [obs.ax(k); obs.ay(k); obs.az(k)];
        w = deg2rad([obs.wx(k); obs.wy(k); obs.wz(k)]);
        
        % 初期化期間後のみ予測・更新を実行
        if k > static_samples
            % predict
            tic;
            eskf.predict(a, w);
            t_predict = t_predict + toc;
            
            % ZUPT (静止検出と速度更新)
            is_stationary = eskf.check_stationary(a, w);
            if is_stationary
                tic;
                eskf.update_zupt();
                t_predict = t_predict + toc;  % ZUPTの時間をpredictに含める
            end
        end
        
        % 初期化期間後のみ姿勢・位置更新を実行
        if k > static_samples
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
        end
        
        % 発散チェックとリセット（重要：各ステップで実行）
        eskf.check_and_reset_if_diverged(obs, k);
        
        % 姿勢発散パラメータのリアルタイムチェック
        if k > 1000  % 初期化期間を除外
            % クォータニオンノルムが1.0から大きく外れている
            if abs(eskf.quaternion_norm - 1.0) > 0.1
                error('エラー: クォータニオンの正規化が失敗しました (norm=%.3f, t=%.2fs)', ...
                    eskf.quaternion_norm, obs.time(k));
            end
            
            % マハラノビス距離が異常に大きい
            if eskf.accel_mahalanobis_dist > 50.0
                error('エラー: 加速度計更新のマハラノビス距離が異常です (dist=%.1f, t=%.2fs)', ...
                    eskf.accel_mahalanobis_dist, obs.time(k));
            end
            
            % ゲインノルムが異常に大きい
            if eskf.accel_gain_norm > 10.0
                error('エラー: カルマンゲインが異常に大きいです (norm=%.2f, t=%.2fs)', ...
                    eskf.accel_gain_norm, obs.time(k));
            end
            
            % イノベーションが継続的に大きい（閾値を緩和）
            if k > 1100 && mean(results.innov_norm(k-100:k)) > 5.0  % 2.0 -> 5.0 に緩和
                error('エラー: イノベーションが継続的に大きいです (avg=%.2f, t=%.2fs)', ...
                    mean(results.innov_norm(k-100:k)), obs.time(k));
            end
        end
        
        % 状態記録
        results.p(:,k) = eskf.p;
        results.v(:,k) = eskf.v;
        results.euler(:,k) = eskf.get_euler();
        results.ba(:,k) = eskf.ba;
        results.bg(:,k) = eskf.bg;
        
        % 診断情報記録
        results.P_diag(:,k) = diag(eskf.P);
        results.zupt_active(k) = eskf.is_stationary;
        if ~isempty(eskf.accel_filter)
            results.a_filtered(:,k) = eskf.accel_filter.a_filtered;
        end
        
        % 姿勢発散パラメータ記録
        results.innov_norm(k) = eskf.accel_innovation_norm;
        results.maha_dist(k) = eskf.accel_mahalanobis_dist;
        results.gain_norm(k) = eskf.accel_gain_norm;
        results.quat_norm(k) = eskf.quaternion_norm;
        results.att_change_rate(k) = eskf.attitude_change_rate;
        
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
        results.bg(1,:)', results.bg(2,:)', results.bg(3,:)', ...
        results.P_diag(1,:)', results.P_diag(2,:)', results.P_diag(3,:)', ...
        results.P_diag(4,:)', results.P_diag(5,:)', results.P_diag(6,:)', ...
        results.zupt_active', ...
        results.a_filtered(1,:)', results.a_filtered(2,:)', results.a_filtered(3,:)', ...
        results.innov_norm', results.maha_dist', results.gain_norm', ...
        results.quat_norm', results.att_change_rate');

    T.Properties.VariableNames = {'time','px','py','pz','vx','vy','vz','roll','pitch','yaw',...
        'ba_x','ba_y','ba_z','bg_x','bg_y','bg_z',...
        'P_px','P_py','P_pz','P_vx','P_vy','P_vz',...
        'zupt_active','a_filt_x','a_filt_y','a_filt_z',...
        'innov_norm','maha_dist','gain_norm','quat_norm','att_change_rate'};
    writetable(T, out_file);
end

function plot_results(proj_root)
    % 結果プロット
    out_file = fullfile(proj_root, 'Results', 'estimation.csv');
    plot_csv(out_file, 'time');
end

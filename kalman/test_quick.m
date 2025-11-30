% テスト実行スクリプト（データ生成スキップ版）
clc;
clear;

proj_root = fileparts(mfilename('fullpath'));

% パス追加
addpath(genpath(fullfile(proj_root, 'KF')));
addpath(genpath(fullfile(proj_root, 'ESKF')));
addpath(genpath(fullfile(proj_root, 'UKF')));
addpath(genpath(fullfile(proj_root, 'EKF')));
addpath(fullfile(proj_root, 'Graph'));
addpath(fullfile(proj_root, 'GenerateData'));
addpath(genpath(fullfile(proj_root, 'Common')));
addpath(genpath(fullfile(proj_root, 'cpp')));

try
    fprintf('観測データ読み込み中...\n');
    data_dir = fullfile(proj_root, 'GenerateData');
    obs_file = fullfile(data_dir, 'sensor_data.csv');
    obs = read_csv(obs_file);
    fprintf('観測データ読み込み完了 (samples=%d)\n', length(obs.time));
    
    fprintf('パラメータ読み込み中...\n');
    params = config_params();
    fprintf('パラメータ読み込み完了\n');
    
    fprintf('ESKF初期化中...\n');
    dt = mean(diff(obs.time));
    eskf = ESKF(obs, params.static_time, dt);
    fprintf('ESKF初期化完了 (use_meukf=%d)\n', eskf.use_meukf);
    
    fprintf('フィルタ実行中 (最初の1000サンプル)...\n');
    n_samples = min(1000, numel(obs.time));
    
    for k = 1:n_samples
        a = [obs.ax(k); obs.ay(k); obs.az(k)];
        w = deg2rad([obs.wx(k); obs.wy(k); obs.wz(k)]);
        
        eskf.predict(a, w);
        
        is_stationary = eskf.check_stationary(a, w);
        if is_stationary
            eskf.update_zupt();
        end
        
        if mod(k, eskf.freq_accel) == 0
            eskf.update_accel(a);
        end
        
        if mod(k, eskf.freq_mag) == 0
            eskf.update_mag([obs.mx(k); obs.my(k); obs.mz(k)]);
        end
        
        if mod(k, eskf.freq_baro) == 0
            eskf.update_baro(obs.pressure(k));
        end
        
        if mod(k, eskf.freq_gps) == 0 && ~isnan(obs.lat(k)) && ~isnan(obs.lon(k))
            eskf.update_gps(obs.lat(k), obs.lon(k), obs.alt(k), k);
        end
        
        if mod(k, 100) == 0
            fprintf('  Step %d/%d (pos=[%.2f, %.2f, %.2f])\n', k, n_samples, eskf.p(1), eskf.p(2), eskf.p(3));
        end
    end
    
    fprintf('\nテスト成功!\n');
    fprintf('最終位置: [%.3f, %.3f, %.3f]\n', eskf.p(1), eskf.p(2), eskf.p(3));
    fprintf('最終姿勢(deg): [%.1f, %.1f, %.1f]\n', rad2deg(eskf.get_euler()));
    
catch ME
    fprintf('\nエラー発生:\n');
    fprintf('  メッセージ: %s\n', ME.message);
    fprintf('  ID: %s\n', ME.identifier);
    fprintf('  スタック:\n');
    for i = 1:length(ME.stack)
        fprintf('    %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
end

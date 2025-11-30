% テスト実行スクリプト
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
    fprintf('データ生成中...\n');
    sim_generate();
    fprintf('データ生成完了\n');
    
    fprintf('観測データ読み込み中...\n');
    data_dir = fullfile(proj_root, 'GenerateData');
    obs_file = fullfile(data_dir, 'sensor_data.csv');
    obs = read_csv(obs_file);
    fprintf('観測データ読み込み完了\n');
    
    fprintf('パラメータ読み込み中...\n');
    params = config_params();
    fprintf('パラメータ読み込み完了\n');
    
    fprintf('ESKF初期化中...\n');
    dt = mean(diff(obs.time));
    eskf = ESKF(obs, params.static_time, dt);
    fprintf('ESKF初期化完了 (use_meukf=%d)\n', eskf.use_meukf);
    
    fprintf('フィルタ実行中...\n');
    n_samples = min(100, numel(obs.time));  % 最初の100サンプルのみテスト
    
    for k = 1:n_samples
        a = [obs.ax(k); obs.ay(k); obs.az(k)];
        w = deg2rad([obs.wx(k); obs.wy(k); obs.wz(k)]);
        
        eskf.predict(a, w);
        
        if mod(k, eskf.freq_accel) == 0
            eskf.update_accel(a);
        end
        
        if mod(k, 10) == 0
            fprintf('  Step %d/%d\n', k, n_samples);
        end
    end
    
    fprintf('テスト成功!\n');
    
catch ME
    fprintf('エラー発生:\n');
    fprintf('  メッセージ: %s\n', ME.message);
    fprintf('  ID: %s\n', ME.identifier);
    fprintf('  スタック:\n');
    for i = 1:length(ME.stack)
        fprintf('    %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
end

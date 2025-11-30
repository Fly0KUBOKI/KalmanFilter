% 完全なシミュレーション実行（MEUKFモード）
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

fprintf('===== ESKF (MEUKF mode) Simulation =====\n\n');

% 観測データ読み込み
data_dir = fullfile(proj_root, 'GenerateData');
obs_file = fullfile(data_dir, 'sensor_data.csv');
obs = read_csv(obs_file);
fprintf('Data loaded: %d samples\n', length(obs.time));

% パラメータ
params = config_params();
dt = mean(diff(obs.time));

% ESKF初期化
eskf = ESKF(obs, params.static_time, dt);
fprintf('ESKF initialized (use_meukf=%d)\n\n', eskf.use_meukf);

% フィルタ実行
n_samples = numel(obs.time);
results.time = obs.time(:)';
results.p = zeros(3, n_samples);
results.v = zeros(3, n_samples);
results.euler = zeros(3, n_samples);
results.ba = zeros(3, n_samples);
results.bg = zeros(3, n_samples);

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
    
    results.p(:,k) = eskf.p;
    results.v(:,k) = eskf.v;
    results.euler(:,k) = eskf.get_euler();
    results.ba(:,k) = eskf.ba;
    results.bg(:,k) = eskf.bg;
    
    if mod(k, 10000) == 0
        fprintf('Progress: %d/%d (%.1f%%)\n', k, n_samples, k/n_samples*100);
    end
end

fprintf('\nSimulation completed!\n\n');

% 結果保存
out_dir = fullfile(proj_root, 'Results');
if ~exist(out_dir,'dir'), mkdir(out_dir); end
out_file = fullfile(out_dir, 'estimation_meukf.csv');

T = table(results.time(:), ...  
    results.p(1,:)', results.p(2,:)', results.p(3,:)', ...
    results.v(1,:)', results.v(2,:)', results.v(3,:)', ...
    results.euler(1,:)', results.euler(2,:)', results.euler(3,:)', ...
    results.ba(1,:)', results.ba(2,:)', results.ba(3,:)', ...
    results.bg(1,:)', results.bg(2,:)', results.bg(3,:)');

T.Properties.VariableNames = {'time','px','py','pz','vx','vy','vz','roll','pitch','yaw','ba_x','ba_y','ba_z','bg_x','bg_y','bg_z'};
writetable(T, out_file);

fprintf('Results saved to: %s\n', out_file);
fprintf('Estimation completed successfully!\n');

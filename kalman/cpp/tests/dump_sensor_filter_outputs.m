% dump_sensor_filter_outputs.m
% MEX と MATLAB 実装のセンサーフィルタ出力を同一シーケンスでダンプし比較する

% repo root (kalman/)
repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
% do not change user's cwd; operate with full paths
obs = readtable(fullfile(repo_root,'GenerateData','sensor_data.csv'));
N = height(obs);

% 検証ウインドウ（デフォルトは全体）
window = 1:N;

% MATLAB フィルタ初期化
acc_mat = AccelFilter();
% Gyro MATLAB implementation removed — do not create gyro filter
gyro_mat = [];

sample_rate = 200; dt = 1.0/sample_rate; cutoff_freq = 30.0;

% 保存用配列
acc_mex = zeros(N,3); acc_mat_out = zeros(N,3); acc_is_out_mex = false(N,1);
% Gyro MEX handling removed — always use MATLAB-side gyro filter (NoOp)
gyro_mex = NaN(N,3); gyro_mat_out = zeros(N,3);

for k = 1:N
    a_meas = [obs.accel_x(k); obs.accel_y(k); obs.accel_z(k)];
    w_meas = [obs.gyro_x(k); obs.gyro_y(k); obs.gyro_z(k)];

    % MEX 呼び出し（存在確認）
    if exist('mex_sensor_filter','file') == 3
        try
            [a_m, is_out] = mex_sensor_filter('accel', a_meas, zeros(3,1));
            acc_mex(k,:) = a_m(:)';
            acc_is_out_mex(k) = logical(is_out);
        catch
            acc_mex(k,:) = NaN;
            acc_is_out_mex(k) = false;
        end

        % gyro MEX removed — do not call mex_sensor_filter('gyro',...)
        % gyro_mex remains NaN to indicate no MEX gyro output
    else
        acc_mex(k,:) = NaN; gyro_mex(k,:) = NaN;
    end

    % MATLAB 実装
    try
        [a_smooth, ~] = acc_mat.filter(a_meas, zeros(3,1));
        acc_mat_out(k,:) = a_smooth(:)';
    catch
        acc_mat_out(k,:) = NaN;
    end

    % Gyro MATLAB implementation removed — pass raw gyro through
    gyro_mat_out(k,:) = w_meas(:)';
end

T = table(obs.time, obs.accel_x, obs.accel_y, obs.accel_z, ...
    acc_mex(:,1), acc_mex(:,2), acc_mex(:,3), acc_is_out_mex, ...
    acc_mat_out(:,1), acc_mat_out(:,2), acc_mat_out(:,3), ...
    gyro_mex(:,1), gyro_mex(:,2), gyro_mex(:,3), ...
    gyro_mat_out(:,1), gyro_mat_out(:,2), gyro_mat_out(:,3), ...
    'VariableNames', {
    'time','accel_x','accel_y','accel_z', ...
    'accel_mex_x','accel_mex_y','accel_mex_z','accel_mex_is_outlier', ...
    'accel_mat_x','accel_mat_y','accel_mat_z', ...
    'gyro_mex_x','gyro_mex_y','gyro_mex_z', ...
    'gyro_mat_x','gyro_mat_y','gyro_mat_z'});

out_dir = fullfile(repo_root,'Results');
if ~exist(out_dir,'dir')
    mkdir(out_dir);
end
out_path = fullfile(out_dir,'debug_sensor_filter_outputs.csv');
try
    writetable(T,out_path);
    fprintf('Wrote %s\n', out_path);
catch ME
    warning('Failed to write debug CSV: %s', ME.message);
end

% Print first indices where gyro diff is non-zero (tolerance)
diffg = [T.gyro_mex_x - T.gyro_mat_x, T.gyro_mex_y - T.gyro_mat_y, T.gyro_mex_z - T.gyro_mat_z];
tol = 1e-6;
rows = find(any(abs(diffg) > tol, 2));
if isempty(rows)
    fprintf('No gyro differences above tol=%g\n', tol);
else
    firstN = rows(1:min(20,length(rows)));
    fprintf('First differing gyro rows (index, time, mex - mat):\n');
    for i = firstN'
        fprintf('%d | %.4f | %+g %+g %+g\n', i, T.time(i), diffg(i,1), diffg(i,2), diffg(i,3));
    end
end

% 追加でウインドウ要約を表示
if exist('window','var') && ~isempty(window)
    idx = intersect(window, (1:N));
    acc_diff = acc_mex(idx,:) - acc_mat_out(idx,:);
    gyro_diff = gyro_mex(idx,:) - gyro_mat_out(idx,:);
    fprintf('Window accel RMSE: %g %g %g\n', sqrt(mean(acc_diff.^2,1)));
    fprintf('Window gyro RMSE: %g %g %g\n', sqrt(mean(gyro_diff.^2,1)));
end

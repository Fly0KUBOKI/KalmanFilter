% compare_mex_matlab_detailed.m
% MEX と MATLAB の詳細な数値差を比較分析
clear all; clc;

proj_root = fileparts(mfilename('fullpath'));
results_dir = fullfile(proj_root, 'Results');

fprintf('========================================\n');
fprintf('詳細比較分析: MEX vs MATLAB\n');
fprintf('========================================\n\n');

% Run 1 の CSV を読み込み
matlab_file = fullfile(results_dir, 'estimation_matlab_01.csv');
mex_file = fullfile(results_dir, 'estimation_mex_01.csv');

if ~exist(matlab_file, 'file')
    fprintf('Error: %s not found\n', matlab_file);
    fprintf('Please run test_matlab_vs_mex.m first\n');
    return;
end

if ~exist(mex_file, 'file')
    fprintf('Error: %s not found\n', mex_file);
    fprintf('Please run test_matlab_vs_mex.m first\n');
    return;
end

fprintf('Reading files...\n');
matlab_data = readtable(matlab_file);
mex_data = readtable(mex_file);

fprintf('  MATLAB: %d rows, %d columns\n', height(matlab_data), width(matlab_data));
fprintf('  MEX:    %d rows, %d columns\n', height(mex_data), width(mex_data));

% 一致する行数を確認
min_rows = min(height(matlab_data), height(mex_data));

% 主要カラムの差分計算
fprintf('\n========================================\n');
fprintf('差分分析（最初の 100 行）\n');
fprintf('========================================\n\n');

% Position の差
if ismember('px', matlab_data.Properties.VariableNames)
    p_matlab = table2array(matlab_data(1:min(100, min_rows), {'px', 'py', 'pz'}));
    p_mex = table2array(mex_data(1:min(100, min_rows), {'px', 'py', 'pz'}));
    
    diff_p = abs(p_matlab - p_mex);
    max_diff_p = max(diff_p(:));
    mean_diff_p = mean(diff_p(:));
    
    fprintf('Position (p) difference:\n');
    fprintf('  Max:  %.6e m\n', max_diff_p);
    fprintf('  Mean: %.6e m\n', mean_diff_p);
    fprintf('  Relative error: %.2e %%\n', 100 * mean_diff_p / (mean(abs(p_matlab(:))) + 1e-10));
end

% Velocity の差
if ismember('vx', matlab_data.Properties.VariableNames)
    v_matlab = table2array(matlab_data(1:min(100, min_rows), {'vx', 'vy', 'vz'}));
    v_mex = table2array(mex_data(1:min(100, min_rows), {'vx', 'vy', 'vz'}));
    
    diff_v = abs(v_matlab - v_mex);
    max_diff_v = max(diff_v(:));
    mean_diff_v = mean(diff_v(:));
    
    fprintf('\nVelocity (v) difference:\n');
    fprintf('  Max:  %.6e m/s\n', max_diff_v);
    fprintf('  Mean: %.6e m/s\n', mean_diff_v);
    fprintf('  Relative error: %.2e %%\n', 100 * mean_diff_v / (mean(abs(v_matlab(:))) + 1e-10));
end

% Quaternion の差（正規化考慮）
if ismember('qw', matlab_data.Properties.VariableNames)
    q_matlab = table2array(matlab_data(1:min(100, min_rows), {'qw', 'qx', 'qy', 'qz'}));
    q_mex = table2array(mex_data(1:min(100, min_rows), {'qw', 'qx', 'qy', 'qz'}));
    
    % クォータニオンのノルム差
    norm_q_matlab = sqrt(sum(q_matlab.^2, 2));
    norm_q_mex = sqrt(sum(q_mex.^2, 2));
    
    fprintf('\nQuaternion (q) analysis:\n');
    fprintf('  MATLAB norm: min=%.6f, max=%.6f, mean=%.6f\n', ...
        min(norm_q_matlab), max(norm_q_matlab), mean(norm_q_matlab));
    fprintf('  MEX norm:    min=%.6f, max=%.6f, mean=%.6f\n', ...
        min(norm_q_mex), max(norm_q_mex), mean(norm_q_mex));
    
    % 要素ごとの差
    diff_q = abs(q_matlab - q_mex);
    max_diff_q = max(diff_q(:));
    mean_diff_q = mean(diff_q(:));
    
    fprintf('  Max element diff:  %.6e\n', max_diff_q);
    fprintf('  Mean element diff: %.6e\n', mean_diff_q);
end

% Bias の差
if ismember('bax', matlab_data.Properties.VariableNames)
    ba_matlab = table2array(matlab_data(1:min(100, min_rows), {'bax', 'bay', 'baz'}));
    ba_mex = table2array(mex_data(1:min(100, min_rows), {'bax', 'bay', 'baz'}));
    
    diff_ba = abs(ba_matlab - ba_mex);
    max_diff_ba = max(diff_ba(:));
    mean_diff_ba = mean(diff_ba(:));
    
    fprintf('\nAccel Bias (ba) difference:\n');
    fprintf('  Max:  %.6e m/s²\n', max_diff_ba);
    fprintf('  Mean: %.6e m/s²\n', mean_diff_ba);
end

if ismember('bgx', matlab_data.Properties.VariableNames)
    bg_matlab = table2array(matlab_data(1:min(100, min_rows), {'bgx', 'bgy', 'bgz'}));
    bg_mex = table2array(mex_data(1:min(100, min_rows), {'bgx', 'bgy', 'bgz'}));
    
    diff_bg = abs(bg_matlab - bg_mex);
    max_diff_bg = max(diff_bg(:));
    mean_diff_bg = mean(diff_bg(:));
    
    fprintf('\nGyro Bias (bg) difference:\n');
    fprintf('  Max:  %.6e rad/s\n', max_diff_bg);
    fprintf('  Mean: %.6e rad/s\n', mean_diff_bg);
end

fprintf('\n========================================\n');
fprintf('結論:\n');
fprintf('========================================\n');
fprintf('float32 vs float64 による精度差がこのレベルであれば改善が必要です。\n');
fprintf('C++ の型を float64 に変更して再ビルド・テストしてください。\n');

% test_matlab_vs_mex.m
% MEX と MATLAB 実装を比較して、どちらが正しいか調査
clear all; clc;

proj_root = fileparts(mfilename('fullpath'));

fprintf('\n============================================\n');
fprintf('TEST 1: MATLAB 実装で実行\n');
fprintf('============================================\n');
setenv('FORCE_MATLAB_FILTERS', '1');
fprintf('Environment: FORCE_MATLAB_FILTERS = 1 (MATLAB mode)\n\n');

try
    run_batch_10sets(false);  % use_mex = false
    fprintf('\n✓ MATLAB mode completed\n');
catch ME
    fprintf('\n✗ MATLAB mode FAILED: %s\n', ME.message);
end

% 結果を保存
results_dir = fullfile(proj_root, 'Results');
logs_dir = fullfile(results_dir, 'log');
matlab_log = '';
% Find the most recent MATLAB-mode log in Results/log and reference it (do not copy to Results root)
if exist(logs_dir, 'dir')
    files = dir(fullfile(logs_dir, 'batch_10sets_log_matlab_*.txt'));
    if ~isempty(files)
        [~, idx] = max([files.datenum]);
        matlab_log = fullfile(logs_dir, files(idx).name);
        fprintf('Found MATLAB mode log: %s\n', matlab_log);
    else
        fprintf('No MATLAB-mode log found in %s\n', logs_dir);
    end
else
    fprintf('Logs directory not found: %s\n', logs_dir);
end

% CSV も保存
for i = 1:10
    src = fullfile(results_dir, sprintf('estimation_%02d.csv', i));
    dst = fullfile(results_dir, sprintf('estimation_matlab_%02d.csv', i));
    if exist(src, 'file')
        copyfile(src, dst);
    end
end

fprintf('\n============================================\n');
fprintf('TEST 2: MEX 実装で実行\n');
fprintf('============================================\n');
setenv('FORCE_MATLAB_FILTERS', '0');
fprintf('Environment: FORCE_MATLAB_FILTERS = 0 (MEX mode)\n\n');

clear mex;  % キャッシュをクリア

try
    run_batch_10sets(true);  % use_mex = true
    fprintf('\n✓ MEX mode completed\n');
catch ME
    fprintf('\n✗ MEX mode FAILED: %s\n', ME.message);
end

% 結果を保存
mex_log = '';
% Find the most recent MEX-mode log in Results/log and reference it (do not copy)
if exist(logs_dir, 'dir')
    files = dir(fullfile(logs_dir, 'batch_10sets_log_mex_*.txt'));
    if ~isempty(files)
        [~, idx] = max([files.datenum]);
        mex_log = fullfile(logs_dir, files(idx).name);
        fprintf('Found MEX mode log: %s\n', mex_log);
    else
        fprintf('No MEX-mode log found in %s\n', logs_dir);
    end
else
    fprintf('Logs directory not found: %s\n', logs_dir);
end

% CSV も保存
for i = 1:10
    src = fullfile(results_dir, sprintf('estimation_%02d.csv', i));
    dst = fullfile(results_dir, sprintf('estimation_mex_%02d.csv', i));
    if exist(src, 'file')
        copyfile(src, dst);
    end
end

fprintf('\n============================================\n');
fprintf('TEST 3: 比較\n');
fprintf('============================================\n');

% ログ比較
fprintf('\n--- Log comparison ---\n');
fprintf('MATLAB mode log: %s\n', matlab_log);
fprintf('MEX mode log:    %s\n', mex_log);

% CSV サンプル比較（Run 1）
fprintf('\n--- CSV Run 1 comparison (first 5 lines) ---\n');
matlab_csv = fullfile(results_dir, 'estimation_matlab_01.csv');
mex_csv = fullfile(results_dir, 'estimation_mex_01.csv');

if exist(matlab_csv, 'file') && exist(mex_csv, 'file')
    fprintf('\nMATLAB mode:\n');
    system(sprintf('head -5 "%s"', matlab_csv));
    
    fprintf('\nMEX mode:\n');
    system(sprintf('head -5 "%s"', mex_csv));
else
    fprintf('CSV files not found\n');
end

fprintf('\n============================================\n');
fprintf('Done\n');
fprintf('============================================\n');

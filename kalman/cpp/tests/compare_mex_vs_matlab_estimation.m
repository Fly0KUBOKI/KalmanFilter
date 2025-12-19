% compare_mex_vs_matlab_estimation.m
% Run the estimator twice (MEX vs MATLAB filters) with the same seed
% and output direct differences between the resulting estimation CSVs.

% Usage: run this file from MATLAB (no GUI) or call from VSCode MATLAB terminal.

script_dir = fileparts(mfilename('fullpath'));
% project root should be the 'kalman' folder (two levels up from cpp/tests)
proj_root = fullfile(script_dir, '..', '..');
oldpwd = pwd(); cd(proj_root); proj_root = pwd(); cd(oldpwd);

results_dir = fullfile(proj_root, 'Results');
if ~exist(results_dir,'dir'), mkdir(results_dir); end

seed = 42; % fixed seed for reproducibility

fprintf('Project root: %s\n', proj_root);

% Helper to run one mode and save estimation.csv under a specific name
function run_and_save(proj_root, results_dir, seed, mode_label, force_flag)
    fprintf('\n--- Running: %s (FORCE_MATLAB_FILTERS=%s) ---\n', mode_label, force_flag);
    setenv('FORCE_MATLAB_FILTERS', force_flag);
    clear mex;
    addpath(fullfile(proj_root,'cpp','bin'));
    % Ensure run_simulation and project code are on the MATLAB path
    oldpwd = pwd();
    cd(proj_root);
    addpath(genpath(proj_root));
    % run_simulation will write Results/estimation.csv
    try
        run_simulation(seed, false);
    catch e
        % write error to a log for diagnosis and restore cwd
        err_log = fullfile(results_dir, sprintf('run_%s_error.txt', mode_label));
        fid = fopen(err_log,'w');
        if fid ~= -1
            fprintf(fid, 'run_simulation failed for mode %s\n', mode_label);
            fprintf(fid, 'Error message:\n%s\n', getReport(e,'extended'));
            fclose(fid);
        end
        cd(oldpwd);
        error('run_simulation failed for mode %s. See %s for details.', mode_label, err_log);
    end
    % restore current folder
    cd(oldpwd);
    % locate estimation.csv - handle different project layouts (kalman/Results)
    cand1 = fullfile(proj_root,'Results','estimation.csv');
    cand2 = fullfile(proj_root,'kalman','Results','estimation.csv');
    if exist(cand1,'file')
        src = cand1;
    elseif exist(cand2,'file')
        src = cand2;
    else
        err_log = fullfile(results_dir, sprintf('run_%s_missing_estimation.txt', mode_label));
        fid = fopen(err_log,'w');
        if fid ~= -1
            fprintf(fid, 'estimation.csv not found after run_simulation for mode %s\n', mode_label);
            fprintf(fid, 'Checked paths:\n  %s\n  %s\n', cand1, cand2);
            fclose(fid);
        end
        error('estimation.csv not found after run_simulation for mode %s. See %s', mode_label, err_log);
    end
    dst = fullfile(results_dir, sprintf('estimation_%s.csv', mode_label));
    copyfile(src, dst);
    fprintf('Saved: %s\n', dst);
end

% Run MEX (force_flag = '0')
run_and_save(proj_root, results_dir, seed, 'mex', '0');

% Run MATLAB filters (force_flag = '1')
run_and_save(proj_root, results_dir, seed, 'matlab', '1');

% Read both results
tbl_mex = readtable(fullfile(results_dir,'estimation_mex.csv'));
tbl_mat = readtable(fullfile(results_dir,'estimation_matlab.csv'));

% Ensure same size
if size(tbl_mex,1) ~= size(tbl_mat,1)
    error('Row count mismatch between MEX and MATLAB estimations');
end

% Compute numeric difference for all numeric columns
vars = tbl_mex.Properties.VariableNames;
num_cols = width(tbl_mex);
data_mex = table2array(tbl_mex);
data_mat = table2array(tbl_mat);
diff = data_mex - data_mat;

% Save diff CSV
diff_tbl = array2table(diff, 'VariableNames', vars);
diff_file = fullfile(results_dir,'estimation_diff.csv');
writetable(diff_tbl, diff_file);
fprintf('\nWrote diff to: %s\n', diff_file);

% Summary stats per column
fprintf('\nColumn-wise RMSE (abs) summary:\n');
for i = 1:num_cols
    col = vars{i};
    v = diff(:,i);
    rmse = sqrt(mean(v.^2));
    maxabs = max(abs(v));
    fprintf('  %s: RMSE=%.6g, MaxAbs=%.6g\n', col, rmse, maxabs);
end

% Print roll/pitch/yaw detailed stats and first 10 diffs
fprintf('\nAttitude (roll/pitch/yaw) diffs (first 10 rows):\n');
idx_roll = find(strcmp(vars,'roll'));
idx_pitch = find(strcmp(vars,'pitch'));
idx_yaw = find(strcmp(vars,'yaw'));
if isempty(idx_roll) || isempty(idx_pitch) || isempty(idx_yaw)
    warning('roll/pitch/yaw columns not found in estimation CSV');
else
    att_diff = diff(:, [idx_roll, idx_pitch, idx_yaw]);
    for r = 1:min(10,size(att_diff,1))
        fprintf('  row %3d: %+8.6f / %+8.6f / %+8.6f deg\n', r, att_diff(r,1), att_diff(r,2), att_diff(r,3));
    end
    % overall RMSE for attitude axes
    rmse_att = sqrt(mean(att_diff.^2,1));
    fprintf('\nAttitude RMSE diffs: roll=%.6g, pitch=%.6g, yaw=%.6g (MEX - MATLAB)\n', rmse_att(1), rmse_att(2), rmse_att(3));
end

fprintf('\nDone.\n');

%% FOCUSED_REBUILD.m
% Just rebuild mex_meukf_step (outputs as mex_meukf_step_v2)

% This script lives in: <proj_root>/kalman/cpp/build
build_dir = fileparts(mfilename('fullpath'));
% proj_root = three levels up from build_dir: .../MATLAB/KalmanFilter
proj_root = fileparts(fileparts(fileparts(build_dir)));

fprintf('>>> Going to build directory: %s\n', build_dir);
cd(build_dir);

fprintf('>>> Clearing MEX cache...\n');
clear mex

fprintf('>>> Building ONLY mex_meukf_step (rebuilds as mex_meukf_step_v2)...\n');
build_mex('mex_meukf_step');

fprintf('\n>>> Build complete!\n');
fprintf('>>> Now run in kalman/ folder:\n');
fprintf('cd %s\n', fullfile(proj_root, 'kalman'));
fprintf('clear all\n');
fprintf('run_simulation(42, true)\n');

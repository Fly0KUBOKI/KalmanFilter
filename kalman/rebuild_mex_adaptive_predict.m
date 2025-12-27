% rebuild_mex_adaptive_predict.m
% Rebuild mex_adaptive_predict after fixing max_accel

proj_root = fileparts(mfilename('fullpath'));
addpath(fullfile(proj_root, 'cpp', 'build'));

cd(fullfile(proj_root, 'cpp', 'build'));
build_mex({'mex_adaptive_predict'});

fprintf('Rebuild complete!\n');


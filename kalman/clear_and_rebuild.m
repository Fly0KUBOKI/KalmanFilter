% clear_and_rebuild.m
% Clear MEX files and rebuild mex_adaptive_predict and mex_eskf_predict_postprocess

proj_root = fileparts(mfilename('fullpath'));
addpath(fullfile(proj_root, 'cpp', 'build'));
addpath(fullfile(proj_root, 'cpp', 'bin'));

% Clear MEX files
clear mex;
clear functions;

% Try to clear specific MEX files
try
    clear mex_adaptive_predict;
    clear mex_eskf_predict_postprocess;
catch
end

cd(fullfile(proj_root, 'cpp', 'build'));
build_mex({'mex_adaptive_predict', 'mex_eskf_predict_postprocess'});

fprintf('Rebuild complete!\n');


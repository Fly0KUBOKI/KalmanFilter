% clear_and_rebuild.m
% Clear MEX files and rebuild Phase 1 & 2 MEX files

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
    clear mex_eskf_update_postprocess;
catch
end

cd(fullfile(proj_root, 'cpp', 'build'));
build_mex({'mex_adaptive_predict', 'mex_eskf_predict_postprocess', 'mex_eskf_update_postprocess'});

fprintf('Rebuild complete!\n');


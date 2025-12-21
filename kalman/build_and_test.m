% build_and_test.m - Build MEX and run batch test
% This script is now located inside the kalman/ folder
% Avoid "clear all" to prevent confusion with workspace state
% clear all; clc;
clc;

% Get the kalman directory (where this script is located)
kalmanDir = fileparts(mfilename('fullpath'));

buildDir = fullfile(kalmanDir, 'cpp', 'build');
if ~exist(buildDir, 'dir')
    error('Build directory not found: %s', buildDir);
end

fprintf('===== STEP 1: Build MEX =====\n');
cd(buildDir);
try
    build_mex();
    fprintf('✓ MEX build succeeded\n');
catch ME
    fprintf('✗ MEX build FAILED:\n%s\n', ME.message);
    return;
end

fprintf('\n===== STEP 2: Clear MEX cache =====\n');
clear mex;
fprintf('✓ MEX cache cleared\n');

% Ensure the built MEX binaries are on the MATLAB path (bin folder)
binDir = fullfile(kalmanDir, 'cpp', 'bin');
if exist(binDir, 'dir')
    addpath(binDir, '-begin');
    fprintf('✓ Added MEX bin to MATLAB path: %s\n', binDir);
    mexFiles = dir(fullfile(binDir, '*.mex*'));
    if isempty(mexFiles)
        warning('No MEX files found in %s', binDir);
    else
        names = {mexFiles.name};
        fprintf('Found MEX files: %s\n', strjoin(names, ', '));
    end
else
    warning('MEX bin directory not found: %s', binDir);
end

fprintf('\n===== STEP 3: Ensure all MATLAB paths are set =====\n');
cd(kalmanDir);
% Add all required subdirectories
addpath(genpath(fullfile(kalmanDir, 'KF')));
addpath(genpath(fullfile(kalmanDir, 'ESKF')));
addpath(genpath(fullfile(kalmanDir, 'UKF')));
addpath(genpath(fullfile(kalmanDir, 'EKF')));
addpath(fullfile(kalmanDir, 'Graph'));
addpath(fullfile(kalmanDir, 'GenerateData'));
fprintf('✓ Added project MATLAB paths (all required subdirs)\n');

fprintf('\n===== STEP 3.5: Configure environment for MEX mode =====\n');
% Force MEX filters to be used (same as run_batch_10sets(true))
setenv('FORCE_MATLAB_FILTERS', '0');
setenv('DEBUG_TIMING', '1');  % Enable timing debug output
setenv('DEBUG_MEX_CHECK', '1');  % Enable MEX diagnostic output
fprintf('✓ Environment: FORCE_MATLAB_FILTERS = 0 (MEX mode)\n');
fprintf('✓ DEBUG_TIMING = 1 (timing logs enabled)\n');
fprintf('✓ DEBUG_MEX_CHECK = 1 (MEX diagnostics enabled)\n');

fprintf('\n===== STEP 4: Run batch (10 sets) with MEX =====\n');

% Diagnostic: show where SensorFilters is resolved from
try
    which_sf = which('SensorFilters', '-all');
    if isempty(which_sf)
        fprintf('SensorFilters not found on MATLAB path.\n');
    else
        fprintf('SensorFilters found: %s\n', which_sf);
    end
catch
    fprintf('Unable to run which SensorFilters.\n');
end

try
    % run_batch_10sets handles additional path setup internally
    run_batch_10sets(true);  % use_mex = true
    fprintf('✓ Batch test completed\n');
catch ME
    fprintf('✗ Batch test FAILED:\n%s\n', ME.message);
    fprintf('Stack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
    return;
end

fprintf('\n===== ALL TESTS PASSED =====\n');


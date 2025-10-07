% kalman_mex_build.m
% Helper to compile the kalman MEX wrapper on Windows/MATLAB.
% Run from MATLAB in the cpp folder or provide full path.

% Ensure we run in this script's directory so relative source paths resolve
thisFile = mfilename('fullpath');
thisDir = fileparts(thisFile);
origDir = pwd;
cd(thisDir);
srcs = {'kalman_mex.cpp','src/kalman_filter.cpp'};
fprintf('Building kalman mex with sources: %s\n', strjoin(srcs, ', '));
try
    % Try to unload any loaded MEX functions to avoid mexLock preventing overwrite
    try
        clear mex
    catch
        % ignore
    end
    mex('-v', srcs{:});
    fprintf('MEX build succeeded. You can call kalman_mex from MATLAB.\n');
catch ME
    % Provide a more actionable message when the failure is due to mexLock
    msg = ME.message;
    fprintf('MEX build failed: %s\n', msg);
    if contains(msg, 'mexLock', 'IgnoreCase', true) || contains(msg, 'locked', 'IgnoreCase', true)
        fprintf('\nIt looks like the existing MEX is locked in MATLAB (mexLock).\n');
        fprintf('Try the following inside MATLAB before rebuilding:\n');
        fprintf('  1) If you have a handle variable (e.g. h), call: kalman_mex(''delete'', h); clear h\n');
        fprintf('  2) Try unloading all MEX functions: clear mex\n');
        fprintf('  3) If that does not work, restart MATLAB to release the lock, then rebuild.\n\n');
    end
    cd(origDir);
    rethrow(ME);
end
cd(origDir);

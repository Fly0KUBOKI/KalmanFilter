% kalman_mex_build.m
% Helper to compile the kalman MEX wrapper. Place this in KalmanFilter/mex and run from there.

thisFile = mfilename('fullpath');
thisDir = fileparts(thisFile);
origDir = pwd;
cd(thisDir);
srcs = {'kalman_mex.cpp','../../cpp/src/kalman_filter.cpp'};
fprintf('Building kalman mex with sources: %s\n', strjoin(srcs, ', '));
try
    try
        clear mex
    catch
    end
    mex('-v', srcs{:});
    fprintf('MEX build succeeded. You can call kalman_mex from MATLAB.\n');
catch ME
    msg = ME.message;
    fprintf('MEX build failed: %s\n', msg);
    if contains(lower(msg), 'mexlock') || contains(lower(msg), 'locked')
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

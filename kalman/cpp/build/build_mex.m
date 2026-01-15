function build_mex(verbose)
% BUILD_MEX  Build MEX files (compiler must be pre-selected).
%
% Usage:
%  build_mex()      - build with current compiler (quiet mode)
%  build_mex(true)  - build with current compiler (verbose output)
%
% Before running build_mex, select a compiler:
%  select_mex_compiler('msvc')   % or 'mingw'

if nargin < 1 || isempty(verbose)
    verbose = false;
else
    verbose = logical(verbose);
end

clc; clear mex;

fprintf('\n=== Building MEX Files ===\n\n');

% Resolve paths
build_dir = fileparts(mfilename('fullpath'));
proj_root = fileparts(build_dir);
lib_dir = fullfile(proj_root, 'Lib');
mex_src_dir = fullfile(proj_root, 'MEX');
bin_dir = fullfile(proj_root, 'bin');

if ~exist(bin_dir, 'dir'), mkdir(bin_dir); end

% Get currently selected compiler
try
    sel = mex.getCompilerConfigurations('C++','Selected');
    if isempty(sel)
        fprintf('✗ No compiler selected. Run select_mex_compiler first.\n\n');
        return;
    end
    fprintf('Compiler: %s\n\n', sel.Name);
    is_msvc = contains(sel.Name, 'Visual', 'IgnoreCase', true) || ...
              contains(sel.Name, 'Microsoft', 'IgnoreCase', true);
catch
    fprintf('✗ Error checking compiler configuration.\n\n');
    return;
end

% Set compiler-specific flags
if is_msvc
    setenv('COMPFLAGS', '/O2 /fp:precise /arch:SSE2 /MD');
    opt_flags = {};
else
    % MinGW
    opt_flags = {'CXXFLAGS=$CXXFLAGS -O2 -msse2 -mfpmath=sse -fno-fast-math -ffloat-store -frounding-math -s -std=gnu++17'};
end
% Compile options
compile_opts = [opt_flags, {'-DNDEBUG', '-DKALMAN_NO_STANDALONE'}];
if ispc
    compile_opts = [compile_opts, {'-DWIN32', '-D_CRT_SECURE_NO_WARNINGS'}];
    if is_msvc
        % Enable C++17 on MSVC
        setenv('COMPFLAGS', [getenv('COMPFLAGS') ' /std:c++17']);
    end
end

% Build include paths (explicit for refactored layout)
inc_args = {['-I' fullfile(proj_root, 'inc')]};
candidates = {
    fullfile(lib_dir, 'Core')
    fullfile(lib_dir, 'Sensor')
    fullfile(lib_dir, 'Matrix')
    fullfile(lib_dir, 'Quaternion')
    fullfile(lib_dir, 'ESKF', 'inc')
    fullfile(lib_dir, 'MEUKF', 'inc')
    fullfile(lib_dir, 'KF', 'inc')
    fullfile(lib_dir, 'EKF', 'inc')
    fullfile(lib_dir, 'UKF', 'inc')
    fullfile(lib_dir, 'Common', 'inc')
};
for ii = 1:numel(candidates)
    if exist(candidates{ii}, 'dir')
        inc_args{end+1} = ['-I' candidates{ii}];
    end
end

% Define MEX targets
mex_targets = {
    {'mex_hybrid_filter.cpp', {
        fullfile(lib_dir, 'Common', 'src', 'filter_mgmt.cpp')
        fullfile(lib_dir, 'ESKF', 'src', 'eskf_postprocess.cpp')
        fullfile(lib_dir, 'ESKF', 'src', 'eskf_core.cpp')
        fullfile(lib_dir, 'ESKF', 'src', 'eskf_math.cpp')
        fullfile(lib_dir, 'ESKF', 'src', 'eskf_sensor_updates.cpp')
        fullfile(lib_dir, 'Common', 'src', 'Sensor', 'sensor_preprocessor.cpp')
        fullfile(lib_dir, 'ESKF', 'src', 'eskf_runner.cpp')
        fullfile(lib_dir, 'ESKF', 'src', 'eskf_initializer.cpp')
        fullfile(mex_src_dir, 'mex_eskf_initializer.cpp')
        fullfile(lib_dir, 'MEUKF', 'src', 'meukf_core.cpp')
        fullfile(lib_dir, 'MEUKF', 'src', 'meukf_predict.cpp')
        fullfile(lib_dir, 'MEUKF', 'src', 'meukf_sigma_points.cpp')
        fullfile(lib_dir, 'MEUKF', 'src', 'meukf_update.cpp')
        % legacy constant-definition sources removed; constants moved to headers
        fullfile(lib_dir, 'MEUKF', 'src', 'meukf_observation_models_constants.cpp')
    }, 'mex_hybrid_filter'}
};
% NOTE: mex_meukf_step_v2 target removed (not used in run_simulation.m, 
%       MEUKF functionality is integrated into do_sensor_update_meukf within mex_hybrid_filter)

% Clean previous outputs
old_mexs = dir(fullfile(bin_dir, ['*.' mexext]));
for ii = 1:numel(old_mexs), delete(fullfile(bin_dir, old_mexs(ii).name)); end

% Prepare a basic build log file (always) and a verbose log on request
logfile = fullfile(build_dir, sprintf('build_mex_log_%s.txt', datestr(now,'yyyymmdd_HHMMSS')));
fid_log = fopen(logfile,'w');
if fid_log ~= -1
    fprintf(fid_log, 'Build started: %s\n', datestr(now));
else
    fid_log = [];
end
if verbose
    vfile = fullfile(build_dir, sprintf('build_mex_verbose_%s.txt', datestr(now,'yyyymmdd_HHMMSS')));
    fid_vlog = fopen(vfile,'w');
else
    fid_vlog = [];
end

% Build targets
built = 0;
for t = 1:numel(mex_targets)
    entry = mex_targets{t};
    mex_file = entry{1};
    extra_srcs = entry{2};
    outname = entry{3};
    
    mex_full = fullfile(mex_src_dir, mex_file);
    if ~exist(mex_full, 'file')
        fprintf('Skipping missing: %s\n', mex_file); continue;
    end
    
    % Filter existing source files
    valid_srcs = {};
    for s = 1:numel(extra_srcs)
        if exist(extra_srcs{s}, 'file'), valid_srcs{end+1} = extra_srcs{s}; end
    end
    
    all_srcs = [{mex_full}, valid_srcs];
    mex_args = [compile_opts, inc_args, {'-output', outname}, all_srcs];

    % If MSVC requested and verbose requested, enable verbose mex output
    if is_msvc && verbose
        mex_args = [{'-v'}, mex_args];
    end

    fprintf('[%d/%d] Compiling %s... ', t, numel(mex_targets), outname);
    % Capture mex output to logfile for post-mortem (always capture, but
    % only enable -v on MSVC when verbose requested)
    try
        out = evalc('mex(mex_args{:});');
        if ~isempty(fid_log), fprintf(fid_log, '--- %s ---\n%s\n', outname, out); end
        if ~isempty(fid_vlog) % also write verbose-specific log if requested
            fprintf(fid_vlog, '--- %s ---\n%s\n', outname, out);
        end
    catch e
        if ~isempty(fid_log), fprintf(fid_log, '--- %s FAILED ---\n%s\n', outname, getReport(e,'extended')); end
        if ~isempty(fid_vlog), fprintf(fid_vlog, '--- %s FAILED ---\n%s\n', outname, getReport(e,'extended')); end
        fprintf('FAILED\n  %s\n', e.message); continue;
    end
    
    % Check output and move to bin
    out_mex = [outname '.' mexext];
    if exist(out_mex, 'file')
        copyfile(out_mex, fullfile(bin_dir, out_mex), 'f'); 
        delete(out_mex);
        info = dir(fullfile(bin_dir, out_mex));
        fprintf('OK (%.1f KB)\n', info.bytes/1024);
        built = built + 1;
    else
        fprintf('FAILED (no output)\n');
    end
end

fprintf('\nBuild finished: %d/%d MEX built\n', built, numel(mex_targets));
fprintf('Output: %s\n\n', bin_dir);

if ~isempty(fid_log)
    fclose(fid_log);
    fprintf('Build log: %s\n', logfile);
end
if exist('fid_vlog','var') && ~isempty(fid_vlog)
    fclose(fid_vlog);
    fprintf('Verbose log: %s\n\n', vfile);
end

end

 
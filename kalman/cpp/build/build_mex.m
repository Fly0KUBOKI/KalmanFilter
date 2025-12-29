function build_mex(targets)
    % BUILD_MEX  Build C++/MEX libraries for Kalman Filters
    %
    % Usage:
    %   cd cpp/build
    %   build_mex()
    %
    % Requirements:
    %   - C++ compiler configured (run: mex -setup C++)
    %   - Source files in cpp/MEX/, cpp/Src/, cpp/Inc/

    % Clear command window and unload MEX files for clean build
    clc;
    clear functions;
    clear mex;
    rehash;
    
    % Usage: build_mex()          -> build all
    %        build_mex({'mex_sensor_filter','mex_meukf_step'})
    %        build_mex('mex_sensor_filter.cpp')

    % Normalize targets argument (empty => build all)
    if nargin < 1 || isempty(targets)
        targets = {};
    elseif ischar(targets) || isstring(targets)
        targets = {char(targets)};
    end
    % ensure cellstr
    if ~iscell(targets)
        targets = cellstr(targets);
    end

    function yes = wants(name)
        % Return true if 'name' should be built. If targets empty => build all.
        if isempty(targets)
            yes = true; return;
        end
        % normalize requested name to base (without extension)
        [~, name_base, ~] = fileparts(name);
        if isempty(name_base), name_base = name; end

        yes = false;
        for k = 1:numel(targets)
            t = targets{k};
            if isempty(t), continue; end
            [~, t_base, ~] = fileparts(char(t));
            if isempty(t_base), t_base = char(t); end
            if strcmpi(name_base, t_base)
                yes = true; return;
            end
        end
    end
    
    % Path configuration
    build_dir = fileparts(mfilename('fullpath'));
    
    % Setup log file (output to build folder)
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    log_file = fullfile(build_dir, sprintf('build_mex_log_%s.txt', timestamp));
    log_fid = fopen(log_file, 'w');
    if log_fid == -1
        warning('Failed to open log file: %s', log_file);
        log_fid = [];
    end
    
    % Custom log function that writes to both console and file
    function log_print(varargin)
        fprintf(varargin{:});
        if ~isempty(log_fid) && log_fid ~= -1
            fprintf(log_fid, varargin{:});
        end
    end
    
    log_print('=== MEX Build Log Started at %s ===\n', datestr(now));
    log_print('Log file: %s\n\n', log_file);
    log_print('MEX Build for Kalman Filters\n');
    cpp_root = fileparts(build_dir);
    mex_src_dir = fullfile(cpp_root, 'MEX');
    src_dir = fullfile(cpp_root, 'Src');
    inc_dir = fullfile(cpp_root, 'Inc');
    lib_dir = fullfile(cpp_root, 'Lib');
    bin_dir = fullfile(cpp_root, 'bin');
    
    % Check directories
    if ~exist(mex_src_dir, 'dir')
        error('MEX source directory not found: %s', mex_src_dir);
    end
    if ~exist(inc_dir, 'dir')
        error('Include directory not found: %s', inc_dir);
    end
    if ~exist(src_dir, 'dir')
        error('Source directory not found: %s', src_dir);
    end
    if ~exist(lib_dir, 'dir')
        warning('Lib directory not found: %s (creating if needed)', lib_dir);
    end
    if ~exist(bin_dir, 'dir')
        mkdir(bin_dir);
        log_print('Created output directory: %s\n', bin_dir);
    end
    
    log_print('Output: %s\n\n', bin_dir);
    
    % Check compiler
    try
        cc = mex.getCompilerConfigurations('C++', 'Selected');
        if isempty(cc)
            warning('C++ compiler not configured. Run: mex -setup C++');
            mex('-setup', 'C++');
        end
    catch
        warning('Failed to check compiler. Attempting build anyway...');
    end
    
    % Move to MEX source directory for compilation
    original_dir = pwd;
    cd(mex_src_dir);
    
    build_success = true;
    built_count = 0;
    
    % Compile options
    compile_opts = {'-O', '-DNDEBUG'};
    if ispc
        compile_opts = [compile_opts, {'-DWIN32', '-D_CRT_SECURE_NO_WARNINGS'}];
        % UTF-8 encoding support for Visual C++
        % Set COMPFLAGS environment variable to pass /utf-8 to compiler
        old_compflags = getenv('COMPFLAGS');
        if isempty(old_compflags)
            setenv('COMPFLAGS', '/utf-8');
        else
            setenv('COMPFLAGS', [old_compflags ' /utf-8']);
        end
    end
    
    % Include paths
    % MEX sources use relative paths like "../KF/Core/kalman_filter_core.hpp"
    % Map these to the new Inc/ structure:
    %   ../KF/Core/ -> Inc/KF/
    %   ../UKF/Core/ -> Inc/UKF/
    %   ../ESKF/ -> Inc/ESKF/
    %   ../Common/ -> Inc/Common/
    inc_include = ['-I' inc_dir];  % Inc/全体をインクルードパスに追加
    inc_kf_core = ['-I' fullfile(inc_dir, 'KF')];  % for ../KF/Core/...
    inc_ekf_core = ['-I' fullfile(inc_dir, 'EKF')];  % for ../EKF/Core/...
    inc_eskf = ['-I' fullfile(inc_dir, 'ESKF')];  % for ../ESKF/...
    inc_ukf_core = ['-I' fullfile(inc_dir, 'UKF')];  % for ../UKF/Core/...
    inc_common = ['-I' fullfile(inc_dir, 'Common')];  % for ../Common/...
    inc_meukf = ['-I' fullfile(inc_dir, 'MEUKF')];  % for MEUKF headers
    inc_lib = ['-I' lib_dir];  % for Lib headers
    inc_args = {inc_include, inc_kf_core, inc_ekf_core, inc_eskf, inc_ukf_core, inc_common, inc_meukf, inc_lib};
    
    % Build: mex_matlab_helpers (Phase 1 helper)
    if exist('mex_matlab_helpers.cpp', 'file')
        if wants('mex_matlab_helpers') && build_single_mex('mex_matlab_helpers.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end

    % mex_sensor_preprocessor (実装はSrc/Common/Sensor/sensor_preprocessor.cppに移動)
    sensor_preprocessor_cpp = fullfile(src_dir, 'Common', 'Sensor', 'sensor_preprocessor.cpp');
    if wants('mex_sensor_preprocessor') && build_single_mex('mex_sensor_preprocessor.cpp', compile_opts, inc_args, {sensor_preprocessor_cpp}, bin_dir, [], log_fid)
        built_count = built_count + 1;
    end

    % Phase 4: mex_adaptive_predict (predict() 全体の C++ 化ラッパ)
    if wants('mex_adaptive_predict') && build_single_mex('mex_adaptive_predict.cpp', compile_opts, inc_args, {fullfile(src_dir, 'ESKF', 'eskf_core.cpp')}, bin_dir, [], log_fid)
        built_count = built_count + 1;
    end

    try
        % Build: mex_kalman_filter_core
        if exist('mex_kalman_filter_core.cpp', 'file')
            if wants('mex_kalman_filter_core') && build_single_mex('mex_kalman_filter_core.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end

        % Build: mex_ekf
        ekf_linear_cpp = fullfile(src_dir, 'EKF', 'ekf_linear_update.cpp');
        if exist('mex_ekf.cpp', 'file') && exist(ekf_linear_cpp, 'file')
            if wants('mex_ekf') && build_single_mex('mex_ekf.cpp', compile_opts, inc_args, {ekf_linear_cpp}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end

        % Build: mex_ukf_sigma_points
        ukf_sigma_points_cpp = fullfile(src_dir, 'UKF', 'ukf_sigma_points.cpp');
        if exist('mex_ukf_sigma_points.cpp', 'file')
            if wants('mex_ukf_sigma_points') && build_single_mex('mex_ukf_sigma_points.cpp', compile_opts, inc_args, {ukf_sigma_points_cpp}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end

        % Build: mex_eskf_core (legacy)
        eskf_core_cpp = fullfile(cpp_root, 'ESKF', 'eskf_core.cpp');
        if exist('mex_eskf_core.cpp', 'file') && exist(eskf_core_cpp, 'file')
            % Skip locked legacy target
        else
            % ESKF sources missing -> skip
        end

        % mex_eskf_math
        eskf_math_cpp = fullfile(src_dir, 'ESKF', 'eskf_math.cpp');
        if exist('mex_eskf_math.cpp', 'file') && exist(eskf_math_cpp, 'file')
            if wants('mex_eskf_math') && build_single_mex('mex_eskf_math.cpp', compile_opts, inc_args, {eskf_math_cpp}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end

        % mex_eskf_init
        if exist('mex_eskf_init.cpp', 'file')
            if wants('mex_eskf_init') && build_single_mex('mex_eskf_init.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end

        % mex_eskf_get_state / free / set_state / step_handle
        if exist('mex_eskf_get_state.cpp', 'file') && wants('mex_eskf_get_state')
            if build_single_mex('mex_eskf_get_state.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid), built_count = built_count + 1; end
        end
        if exist('mex_eskf_free.cpp', 'file') && wants('mex_eskf_free')
            if build_single_mex('mex_eskf_free.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid), built_count = built_count + 1; end
        end
        if exist('mex_eskf_set_state.cpp', 'file') && wants('mex_eskf_set_state')
            if build_single_mex('mex_eskf_set_state.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid), built_count = built_count + 1; end
        end
        if exist('mex_eskf_step_handle.cpp', 'file') && wants('mex_eskf_step_handle')
            if build_single_mex('mex_eskf_step_handle.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid), built_count = built_count + 1; end
        end

        % mex_eskf_step (wrapper) - skipped here

        % mex_quaternion_lib
        if exist('mex_quaternion_lib.cpp', 'file')
            if wants('mex_quaternion_lib') && build_single_mex('mex_quaternion_lib.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end

        % mex_ukf (uses ekf_linear_update)
        ekf_linear_cpp = fullfile(src_dir, 'EKF', 'ekf_linear_update.cpp');
        if exist('mex_ukf.cpp', 'file')
            if wants('mex_ukf') && build_single_mex('mex_ukf.cpp', compile_opts, inc_args, {ekf_linear_cpp}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end

        % mex_ukf_update
        if exist('mex_ukf_update.cpp', 'file')
            if wants('mex_ukf_update') && build_single_mex('mex_ukf_update.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end

        % mex_meukf_step
        meukf_core_cpp = fullfile(src_dir, 'MEUKF', 'meukf_core.cpp');
        if exist('mex_meukf_step.cpp', 'file') && exist(meukf_core_cpp, 'file')
            if wants('mex_meukf_step') && build_single_mex('mex_meukf_step.cpp', compile_opts, inc_args, {meukf_core_cpp}, bin_dir, 'mex_meukf_step_v2', log_fid)
                built_count = built_count + 1;
            end
        end

        % mex_sensor_filter
        if wants('mex_sensor_filter') && build_single_mex('mex_sensor_filter.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end

        % mex_unified_filter
        unified_cpp = fullfile(src_dir, 'MEUKF', 'unified_filter.cpp');
        if exist('mex_unified_filter.cpp', 'file') && exist(unified_cpp, 'file') && exist(meukf_core_cpp, 'file')
            if wants('mex_unified_filter') && build_single_mex('mex_unified_filter.cpp', compile_opts, inc_args, {unified_cpp, meukf_core_cpp}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end

        % mex_eskf_step final
        if exist('mex_eskf_step.cpp', 'file') && exist(unified_cpp, 'file') && exist(meukf_core_cpp, 'file')
            if wants('mex_eskf_step') && build_single_mex('mex_eskf_step.cpp', compile_opts, inc_args, {unified_cpp, meukf_core_cpp}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end
        % Phase 5: mex_filter_management (実装はSrc/Common/filter_management.cppに移動)
        filter_management_cpp = fullfile(src_dir, 'Common', 'filter_management.cpp');
        if exist('mex_filter_management.cpp', 'file')
            if wants('mex_filter_management') && build_single_mex('mex_filter_management.cpp', compile_opts, inc_args, {filter_management_cpp}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end
        
        % Phase 1: mex_eskf_predict_postprocess (実装はSrc/Common/filter_management.cppとSrc/ESKF/eskf_postprocess.cppに移動)
        filter_management_cpp = fullfile(src_dir, 'Common', 'filter_management.cpp');
        eskf_postprocess_cpp = fullfile(src_dir, 'ESKF', 'eskf_postprocess.cpp');
        if exist('mex_eskf_predict_postprocess.cpp', 'file')
            if wants('mex_eskf_predict_postprocess') && build_single_mex('mex_eskf_predict_postprocess.cpp', compile_opts, inc_args, {filter_management_cpp, eskf_postprocess_cpp}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end
        
        % Phase 1: mex_eskf_update_postprocess (実装はSrc/ESKF/eskf_postprocess.cppとSrc/Common/filter_management.cppに移動)
        eskf_postprocess_cpp = fullfile(src_dir, 'ESKF', 'eskf_postprocess.cpp');
        filter_management_cpp = fullfile(src_dir, 'Common', 'filter_management.cpp');
        if exist('mex_eskf_update_postprocess.cpp', 'file')
            if wants('mex_eskf_update_postprocess') && build_single_mex('mex_eskf_update_postprocess.cpp', compile_opts, inc_args, {eskf_postprocess_cpp, filter_management_cpp}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end
        
        % Phase 1: mex_eskf_zupt
        eskf_core_cpp = fullfile(src_dir, 'ESKF', 'eskf_core.cpp');
        if exist('mex_eskf_zupt.cpp', 'file')
            if wants('mex_eskf_zupt') && build_single_mex('mex_eskf_zupt.cpp', compile_opts, inc_args, {eskf_core_cpp}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end
        
        % Full MEX: mex_eskf_full (complete ESKF in MEX)
        if exist('mex_eskf_full.cpp', 'file')
            if wants('mex_eskf_full') && build_single_mex('mex_eskf_full.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end
        
        % Phase 1: mex_eskf_constructor (ESKF constructor MEX)
        if exist('mex_eskf_constructor.cpp', 'file')
            if wants('mex_eskf_constructor') && build_single_mex('mex_eskf_constructor.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end
        
        % Phase 2: mex_eskf_sensor_updates (sensor_updates MEX)
        if exist('mex_eskf_sensor_updates.cpp', 'file')
            if wants('mex_eskf_sensor_updates') && build_single_mex('mex_eskf_sensor_updates.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end
        
        % mex_eskf_sensor_update (統合版)
        if exist('mex_eskf_sensor_update.cpp', 'file')
            if wants('mex_eskf_sensor_update') && build_single_mex('mex_eskf_sensor_update.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end
        
        % mex_eskf_do_update (do_cpp_update MEX化) (mex_eskf_update_postprocess を統合)
        eskf_postprocess_cpp = fullfile(src_dir, 'ESKF', 'eskf_postprocess.cpp');
        filter_management_cpp = fullfile(src_dir, 'Common', 'filter_management.cpp');
        if exist('mex_eskf_do_update.cpp', 'file')
            if wants('mex_eskf_do_update') && build_single_mex('mex_eskf_do_update.cpp', compile_opts, inc_args, {eskf_postprocess_cpp, filter_management_cpp}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end
        
        % mex_eskf_sensor_updates_full (sensor_updates完全MEX化)
        if exist('mex_eskf_sensor_updates_full.cpp', 'file')
            if wants('mex_eskf_sensor_updates_full') && build_single_mex('mex_eskf_sensor_updates_full.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end
        
        % mex_run_eskf (ESKF.m完全移行版) (mex_eskf_predict_postprocess を統合)
        filter_management_cpp = fullfile(src_dir, 'Common', 'filter_management.cpp');
        eskf_postprocess_cpp = fullfile(src_dir, 'ESKF', 'eskf_postprocess.cpp');
        if exist('mex_run_eskf.cpp', 'file')
            if wants('mex_run_eskf') && build_single_mex('mex_run_eskf.cpp', compile_opts, inc_args, {filter_management_cpp, eskf_postprocess_cpp}, bin_dir, [], log_fid)
                built_count = built_count + 1;
            end
        end
        
    catch ME
        build_success = false;
        log_print('\nBuild failed!\n');
        log_print('Error: %s\n', ME.message);
        if ~isempty(ME.stack)
            for k = 1:min(length(ME.stack), 5)
                log_print('  at %s (line %d)\n', ME.stack(k).file, ME.stack(k).line);
            end
        end
        log_print('\nTroubleshooting:\n');
        log_print('1. Run "mex -setup C++" to configure compiler\n');
        log_print('2. Ensure Visual Studio or MinGW is installed\n');
        log_print('3. Check that all source files exist in %s\n', mex_src_dir);
    end
    
    % Return to original directory
    cd(original_dir);
    
    if build_success
        log_print('\n=== Build Complete ===\n');
        log_print('Successfully built %d MEX file(s)\n', built_count);
        log_print('Output: %s\n\n', bin_dir);
        log_print('To use MEX files, add to MATLAB path:\n');
        log_print('  addpath(''%s'')\n', bin_dir);
    else
        log_print('\n=== Build Failed ===\n');
        log_print('MEX build failed. See error messages above.\n');
    end
    
    % Restore COMPFLAGS environment variable if it was modified
    if ispc && exist('old_compflags', 'var')
        if isempty(old_compflags)
            setenv('COMPFLAGS', '');
        else
            setenv('COMPFLAGS', old_compflags);
        end
    end
    
    % Close log file
    log_print('\n=== MEX Build Log Ended at %s ===\n', datestr(now));
    if ~isempty(log_fid)
        fclose(log_fid);
    end
    fprintf('Log saved to: %s\n', log_file);
end

function success = build_single_mex(mex_file, compile_opts, inc_args, extra_sources, output_dir, output_name, log_fid)
    % Build a single MEX file
    success = false;
    
    if nargin < 7, log_fid = []; end
    if nargin < 6 || isempty(output_name)
        [~, output_name, ~] = fileparts(mex_file);
    end
    
    if isempty(output_name)
        warning('Cannot determine output name for: %s', mex_file);
        return;
    end
    
    % .bakファイルを除外
    if length(mex_file) >= 4 && strcmp(mex_file(end-3:end), '.bak')
        return;
    end
    
    % mex_fileを絶対パスに変換（mex_src_dirからの相対パスとして扱う）
    build_dir = fileparts(mfilename('fullpath'));
    cpp_root = fileparts(build_dir);
    mex_src_dir = fullfile(cpp_root, 'MEX');
    mex_file_full = fullfile(mex_src_dir, mex_file);
    
    if ~exist(mex_file_full, 'file')
        warning('Source not found: %s', mex_file_full);
        return;
    end
    
    try
        % extra_sourcesから.bakファイルを除外
        valid_extra_sources = {};
        for i = 1:length(extra_sources)
            src = extra_sources{i};
            if length(src) < 4 || ~strcmp(src(end-3:end), '.bak')
                if exist(src, 'file')
                    valid_extra_sources{end+1} = src;
                else
                    warning('Extra source not found: %s', src);
                end
            end
        end
        
        all_sources = [{mex_file_full}, valid_extra_sources];
        mex_args = [compile_opts, inc_args, {'-output', output_name}, all_sources];
        mex_output = [output_name '.' mexext];
        
        fprintf('Compiling %s... ', output_name);
        if ~isempty(log_fid), fprintf(log_fid, 'Compiling %s... ', output_name); end
        
        cmd_out = evalc('mex(mex_args{:});');
        if ~isempty(log_fid) && ~isempty(cmd_out)
            fprintf(log_fid, '%s', cmd_out);
        end
        
            if exist(mex_output, 'file')
            copyfile(mex_output, fullfile(output_dir, mex_output), 'f');
            if exist(mex_output, 'file'), delete(mex_output); end
                fprintf('OK\n');
            if ~isempty(log_fid), fprintf(log_fid, 'OK\n'); end
                success = true;
            else
                fprintf('FAILED\n');
            if ~isempty(log_fid), fprintf(log_fid, 'FAILED\n'); end
            end
        catch ME
            fprintf('ERROR\n');
        if ~isempty(log_fid)
            fprintf(log_fid, 'ERROR\n');
            fprintf(log_fid, 'Error: %s\n', ME.message);
        end
        warning('Failed to build %s: %s', mex_file, ME.message);
    end
end


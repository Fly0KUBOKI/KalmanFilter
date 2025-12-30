function build_mex(targets)
    % BUILD_MEX  Build C++/MEX libraries for Kalman Filters

    clc;
    clear functions;
    clear mex;
    rehash;
    
    if nargin < 1 || isempty(targets)
        targets = {};
    elseif ischar(targets) || isstring(targets)
        targets = {char(targets)};
    end
    if ~iscell(targets)
        targets = cellstr(targets);
    end

    function yes = wants(name)
        if isempty(targets)
            yes = true; return;
        end
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
    
    build_dir = fileparts(mfilename('fullpath'));
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    log_file = fullfile(build_dir, sprintf('build_mex_log_%s.txt', timestamp));
    log_fid = fopen(log_file, 'w');
    if log_fid == -1
        log_fid = [];
    end
    
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
    
    if ~exist(mex_src_dir, 'dir')
        error('MEX source directory not found: %s', mex_src_dir);
    end
    if ~exist(inc_dir, 'dir')
        error('Include directory not found: %s', inc_dir);
    end
    if ~exist(src_dir, 'dir')
        error('Source directory not found: %s', src_dir);
    end
    if ~exist(bin_dir, 'dir')
        mkdir(bin_dir);
    else
        bin_files = dir(fullfile(bin_dir, '*.*'));
        for i = 1:length(bin_files)
            if ~bin_files(i).isdir && ~strcmp(bin_files(i).name, '.') && ~strcmp(bin_files(i).name, '..')
                delete(fullfile(bin_dir, bin_files(i).name));
            end
        end
    end
    
    log_print('Output: %s\n\n', bin_dir);
    
    original_dir = pwd;
    cd(mex_src_dir);
    
    built_count = 0;
    
    compile_opts = {'-O', '-DNDEBUG'};
    if ispc
        compile_opts = [compile_opts, {'-DWIN32', '-D_CRT_SECURE_NO_WARNINGS'}];
        old_compflags = getenv('COMPFLAGS');
        if isempty(old_compflags)
            setenv('COMPFLAGS', '/utf-8');
        else
            setenv('COMPFLAGS', [old_compflags ' /utf-8']);
        end
    end
    
    inc_include = ['-I' inc_dir];
    inc_kf_core = ['-I' fullfile(inc_dir, 'KF')];
    inc_ekf_core = ['-I' fullfile(inc_dir, 'EKF')];
    inc_eskf = ['-I' fullfile(inc_dir, 'ESKF')];
    inc_ukf_core = ['-I' fullfile(inc_dir, 'UKF')];
    inc_common = ['-I' fullfile(inc_dir, 'Common')];
    inc_meukf = ['-I' fullfile(inc_dir, 'MEUKF')];
    inc_lib = ['-I' lib_dir];
    inc_args = {inc_include, inc_kf_core, inc_ekf_core, inc_eskf, inc_ukf_core, inc_common, inc_meukf, inc_lib};
    
    if exist('mex_matlab_helpers.cpp', 'file')
        if wants('mex_matlab_helpers') && build_single_mex('mex_matlab_helpers.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end

    sensor_preprocessor_cpp = fullfile(src_dir, 'Common', 'Sensor', 'sensor_preprocessor.cpp');
    if wants('mex_sensor_preprocessor') && build_single_mex('mex_sensor_preprocessor.cpp', compile_opts, inc_args, {sensor_preprocessor_cpp}, bin_dir, [], log_fid)
        built_count = built_count + 1;
    end

    if exist('mex_kalman_filter_core.cpp', 'file')
        if wants('mex_kalman_filter_core') && build_single_mex('mex_kalman_filter_core.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end

    ekf_linear_cpp = fullfile(src_dir, 'EKF', 'ekf_linear_update.cpp');
    if exist('mex_ekf.cpp', 'file') && exist(ekf_linear_cpp, 'file')
        if wants('mex_ekf') && build_single_mex('mex_ekf.cpp', compile_opts, inc_args, {ekf_linear_cpp}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end

    ukf_sigma_points_cpp = fullfile(src_dir, 'UKF', 'ukf_sigma_points.cpp');
    if exist('mex_ukf_sigma_points.cpp', 'file')
        if wants('mex_ukf_sigma_points') && build_single_mex('mex_ukf_sigma_points.cpp', compile_opts, inc_args, {ukf_sigma_points_cpp}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end

    eskf_math_cpp = fullfile(src_dir, 'ESKF', 'eskf_math.cpp');
    if exist('mex_eskf_math.cpp', 'file') && exist(eskf_math_cpp, 'file')
        if wants('mex_eskf_math') && build_single_mex('mex_eskf_math.cpp', compile_opts, inc_args, {eskf_math_cpp}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end

    if exist('mex_eskf_init.cpp', 'file')
        if wants('mex_eskf_init') && build_single_mex('mex_eskf_init.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end

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

    if exist('mex_ukf.cpp', 'file')
        if wants('mex_ukf') && build_single_mex('mex_ukf.cpp', compile_opts, inc_args, {ekf_linear_cpp}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end

    if exist('mex_ukf_update.cpp', 'file')
        if wants('mex_ukf_update') && build_single_mex('mex_ukf_update.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end

    meukf_core_cpp = fullfile(src_dir, 'MEUKF', 'meukf_core.cpp');
    if exist('mex_meukf_step.cpp', 'file') && exist(meukf_core_cpp, 'file')
        if wants('mex_meukf_step') && build_single_mex('mex_meukf_step.cpp', compile_opts, inc_args, {meukf_core_cpp}, bin_dir, 'mex_meukf_step_v2', log_fid)
            built_count = built_count + 1;
        end
    end

    if wants('mex_sensor_filter') && build_single_mex('mex_sensor_filter.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
        built_count = built_count + 1;
    end

    unified_cpp = fullfile(src_dir, 'MEUKF', 'unified_filter.cpp');
    if exist('mex_unified_filter.cpp', 'file') && exist(unified_cpp, 'file') && exist(meukf_core_cpp, 'file')
        if wants('mex_unified_filter') && build_single_mex('mex_unified_filter.cpp', compile_opts, inc_args, {unified_cpp, meukf_core_cpp}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end

    if exist('mex_eskf_step.cpp', 'file') && exist(unified_cpp, 'file') && exist(meukf_core_cpp, 'file')
        if wants('mex_eskf_step') && build_single_mex('mex_eskf_step.cpp', compile_opts, inc_args, {unified_cpp, meukf_core_cpp}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end
    
    filter_management_cpp = fullfile(src_dir, 'Common', 'filter_management.cpp');
    eskf_postprocess_cpp = fullfile(src_dir, 'ESKF', 'eskf_postprocess.cpp');
    if exist('mex_eskf_predict_postprocess.cpp', 'file')
        if wants('mex_eskf_predict_postprocess') && build_single_mex('mex_eskf_predict_postprocess.cpp', compile_opts, inc_args, {filter_management_cpp, eskf_postprocess_cpp}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end
    
    if exist('mex_eskf_full.cpp', 'file')
        if wants('mex_eskf_full') && build_single_mex('mex_eskf_full.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end
    
    if exist('mex_eskf_sensor_updates.cpp', 'file')
        if wants('mex_eskf_sensor_updates') && build_single_mex('mex_eskf_sensor_updates.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end
    
    if exist('mex_eskf_sensor_update.cpp', 'file')
        if wants('mex_eskf_sensor_update') && build_single_mex('mex_eskf_sensor_update.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end
    
    if exist('mex_eskf_do_update.cpp', 'file')
        if wants('mex_eskf_do_update') && build_single_mex('mex_eskf_do_update.cpp', compile_opts, inc_args, {eskf_postprocess_cpp, filter_management_cpp}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end
    
    filter_management_cpp = fullfile(src_dir, 'Common', 'filter_management.cpp');
    eskf_postprocess_cpp = fullfile(src_dir, 'ESKF', 'eskf_postprocess.cpp');
    eskf_core_cpp = fullfile(src_dir, 'ESKF', 'eskf_core.cpp');
    eskf_math_cpp = fullfile(src_dir, 'ESKF', 'eskf_math.cpp');
    eskf_sensor_updates_cpp = fullfile(src_dir, 'ESKF', 'eskf_sensor_updates.cpp');
    sensor_preprocessor_cpp = fullfile(src_dir, 'Common', 'Sensor', 'sensor_preprocessor.cpp');
    eskf_runner_cpp = fullfile(src_dir, 'ESKF', 'eskf_runner.cpp');
    eskf_initializer_cpp = fullfile(src_dir, 'ESKF', 'eskf_initializer.cpp');
    if exist('mex_run_eskf.cpp', 'file')
        if wants('mex_run_eskf') && build_single_mex('mex_run_eskf.cpp', compile_opts, inc_args, {filter_management_cpp, eskf_postprocess_cpp, eskf_core_cpp, eskf_math_cpp, eskf_sensor_updates_cpp, sensor_preprocessor_cpp, eskf_runner_cpp, eskf_initializer_cpp}, bin_dir, [], log_fid)
            built_count = built_count + 1;
        end
    end
    
    cd(original_dir);
    
    log_print('\n=== Build Complete ===\n');
    log_print('Successfully built %d MEX file(s)\n', built_count);
    log_print('Output: %s\n\n', bin_dir);
    log_print('To use MEX files, add to MATLAB path:\n');
    log_print('  addpath(''%s'')\n', bin_dir);
    
    if ispc && exist('old_compflags', 'var')
        if isempty(old_compflags)
            setenv('COMPFLAGS', '');
        else
            setenv('COMPFLAGS', old_compflags);
        end
    end
    
    log_print('\n=== MEX Build Log Ended at %s ===\n', datestr(now));
    if ~isempty(log_fid)
        fclose(log_fid);
    end
    fprintf('Log saved to: %s\n', log_file);
end

function success = build_single_mex(mex_file, compile_opts, inc_args, extra_sources, output_dir, output_name, log_fid)
    success = false;
    
    if nargin < 7, log_fid = []; end
    if nargin < 6 || isempty(output_name)
        [~, output_name, ~] = fileparts(mex_file);
    end
    
    if isempty(output_name)
        return;
    end
    
    build_dir = fileparts(mfilename('fullpath'));
    cpp_root = fileparts(build_dir);
    mex_src_dir = fullfile(cpp_root, 'MEX');
    mex_file_full = fullfile(mex_src_dir, mex_file);
    
    if ~exist(mex_file_full, 'file')
        return;
    end
    
    valid_extra_sources = {};
    for i = 1:length(extra_sources)
        src = extra_sources{i};
        if exist(src, 'file')
            valid_extra_sources{end+1} = src;
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
end

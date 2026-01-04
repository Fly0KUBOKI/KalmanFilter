function build_mex(targets)
    % BUILD_MEX  Build C++/MEX libraries for Kalman Filters
    
    clc;
    clear mex;
    
    if nargin < 1 || isempty(targets)
        targets = {};
    elseif ischar(targets) || isstring(targets)
        targets = {char(targets)};
    end
    if ~iscell(targets)
        targets = cellstr(targets);
    end
    
    build_dir = fileparts(mfilename('fullpath'));
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    log_file = fullfile(build_dir, sprintf('build_mex_log_%s.txt', timestamp));
    log_fid = fopen(log_file, 'w');
    if log_fid == -1, log_fid = []; end
    
    log_fn = @(varargin) fprintf_both(log_fid, varargin{:});
    log_fn('=== MEX Build Log Started at %s ===\n', datestr(now));
    log_fn('Log file: %s\n\n', log_file);
    
    cpp_root = fileparts(build_dir);
    mex_src_dir = fullfile(cpp_root, 'MEX');
    lib_dir = fullfile(cpp_root, 'Lib');
    bin_dir = fullfile(cpp_root, 'bin');
    
    if ~exist(mex_src_dir, 'dir')
        error('MEX source directory not found: %s', mex_src_dir);
    end
    
    if ~exist(bin_dir, 'dir')
        mkdir(bin_dir);
    end
    
    % Clean old MEX files
    old_mexs = dir(fullfile(bin_dir, ['*.' mexext]));
    for i = 1:length(old_mexs)
        delete(fullfile(bin_dir, old_mexs(i).name));
    end
    
    log_fn('Output: %s\n\n', bin_dir);
    
    original_dir = pwd;
    cd(mex_src_dir);
    
    % Compiler options
    compile_opts = {'-O', '-DNDEBUG', '-DKALMAN_NO_STANDALONE'};
    if ispc
        compile_opts = [compile_opts, {'-DWIN32', '-D_CRT_SECURE_NO_WARNINGS'}];
        % /utf-8 flags must be passed via COMPFLAGS, not directly
        old_compflags = getenv('COMPFLAGS');
        setenv('COMPFLAGS', '/utf-8');
    end
    
    % Include paths
    inc_args = {['-I' lib_dir]};
    lib_entries = dir(lib_dir);
    for i = 1:length(lib_entries)
        if ~lib_entries(i).isdir, continue; end
        name = lib_entries(i).name;
        if strcmp(name, '.') || strcmp(name, '..'), continue; end
        lib_inc = fullfile(lib_dir, name, 'inc');
        if exist(lib_inc, 'dir')
            inc_args{end+1} = ['-I' lib_inc];
        end
    end
    
% Define MEX targets: {mex_file, extra_sources, output_name}
    mex_targets = {
        'mex_run_eskf.cpp', {
            fullfile(lib_dir, 'Common', 'src', 'filter_mgmt.cpp');
            fullfile(lib_dir, 'ESKF', 'src', 'eskf_postprocess.cpp');
            fullfile(lib_dir, 'ESKF', 'src', 'eskf_core.cpp');
            fullfile(lib_dir, 'ESKF', 'src', 'eskf_math.cpp');
            fullfile(lib_dir, 'ESKF', 'src', 'eskf_sensor_updates.cpp');
            fullfile(lib_dir, 'Common', 'src', 'Sensor', 'sensor_preprocessor.cpp');
            fullfile(lib_dir, 'ESKF', 'src', 'eskf_runner.cpp');
            fullfile(lib_dir, 'ESKF', 'src', 'eskf_initializer.cpp');
            fullfile(mex_src_dir, 'mex_eskf_initializer.cpp');
            fullfile(lib_dir, 'MEUKF', 'src', 'meukf_core.cpp');
        }, [];
        'mex_meukf_step.cpp', {fullfile(lib_dir, 'MEUKF', 'src', 'meukf_core.cpp')}, 'mex_meukf_step_v2';
    };
    
    built_count = build_mex_targets(mex_targets, compile_opts, inc_args, bin_dir, targets, log_fn, log_fid);
    
    cd(original_dir);
    
    % Restore COMPFLAGS
    if ispc && exist('old_compflags', 'var')
        if isempty(old_compflags)
            setenv('COMPFLAGS', '');
        else
            setenv('COMPFLAGS', old_compflags);
        end
    end
    
    log_fn('\n=== Build Complete ===\n');
    log_fn('Successfully built %d MEX file(s)\n', built_count);
    log_fn('Output: %s\n', bin_dir);
    
    log_fn('\n=== MEX Build Log Ended at %s ===\n', datestr(now));
    if ~isempty(log_fid)
        fclose(log_fid);
    end
    fprintf('Log saved to: %s\n', log_file);
end

function fprintf_both(fid, varargin)
    fprintf(varargin{:});
    if ~isempty(fid)
        fprintf(fid, varargin{:});
    end
end

function wants_target = should_build(name, target_list)
    wants_target = isempty(target_list);
    if ~wants_target
        [~, name_base, ~] = fileparts(name);
        if isempty(name_base), name_base = name; end
        for k = 1:numel(target_list)
            t = char(target_list{k});
            [~, t_base, ~] = fileparts(t);
            if isempty(t_base), t_base = t; end
            if strcmpi(name_base, t_base)
                wants_target = true;
                return;
            end
        end
    end
end

function built = build_mex_targets(mex_targets, compile_opts, inc_args, bin_dir, target_list, log_fn, log_fid)
    built = 0;
    
    build_dir = fileparts(mfilename('fullpath'));
    cpp_root = fileparts(build_dir);
    mex_src_dir = fullfile(cpp_root, 'MEX');
    
    for idx = 1:size(mex_targets, 1)
        mex_file = mex_targets{idx, 1};
        extra_sources = mex_targets{idx, 2};
        output_name = mex_targets{idx, 3};
        
        if isempty(output_name)
            [~, output_name, ~] = fileparts(mex_file);
        end
        
        if ~should_build(mex_file, target_list)
            continue;
        end
        
        mex_file_full = fullfile(mex_src_dir, mex_file);
        if ~exist(mex_file_full, 'file')
            continue;
        end
        
        valid_extra = {};
        for i = 1:length(extra_sources)
            src = extra_sources{i};
            if exist(src, 'file')
                valid_extra{end+1} = src;
            end
        end
        
        all_sources = [{mex_file_full}, valid_extra];
        mex_args = [compile_opts, inc_args, {'-output', output_name}, all_sources];
        mex_output = [output_name '.' mexext];
        
        fprintf('Compiling %s... ', output_name);
        fprintf(log_fid, 'Compiling %s... ', output_name);
        
        try
            evalc('mex(mex_args{:});');
        catch e
            fprintf('FAILED\n');
            fprintf(log_fid, 'FAILED\n%s\n', getReport(e, 'extended'));
            continue;
        end
        
        if exist(mex_output, 'file')
            copyfile(mex_output, fullfile(bin_dir, mex_output), 'f');
            delete(mex_output);
            fprintf('OK\n');
            fprintf(log_fid, 'OK\n');
            built = built + 1;
        else
            fprintf('FAILED\n');
            fprintf(log_fid, 'FAILED\n');
        end
    end
end
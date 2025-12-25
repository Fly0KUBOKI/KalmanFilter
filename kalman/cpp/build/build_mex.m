function build_mex(targets)
    % BUILD_MEX  Build C++/MEX libraries for Kalman Filters
    %
    % Usage:
    %   cd cpp/build
    %   build_mex()
    %
    % Requirements:
    %   - C++ compiler configured (run: mex -setup C++)
    %   - Source files in cpp/mex/, cpp/src/, cpp/include/

    fprintf('MEX Build for Kalman Filters\n');
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
    cpp_root = fileparts(build_dir);
    mex_src_dir = fullfile(cpp_root, 'mex');
    src_dir = fullfile(cpp_root, 'src');
    inc_dir = fullfile(cpp_root, 'include');
    bin_dir = fullfile(cpp_root, 'bin');
    
    % Check directories
    if ~exist(mex_src_dir, 'dir')
        error('MEX source directory not found: %s', mex_src_dir);
    end
    if ~exist(inc_dir, 'dir')
        error('Include directory not found: %s', inc_dir);
    end
    if ~exist(bin_dir, 'dir')
        mkdir(bin_dir);
        fprintf('Created output directory: %s\n', bin_dir);
    end
    
    fprintf('Output: %s\n\n', bin_dir);
    
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
    end
    
    % Include paths
    % MEX sources use relative paths like "../KF/Core/kalman_filter_core.hpp"
    % Map these to the new include/ structure:
    %   ../KF/Core/ -> include/KF/
    %   ../UKF/Core/ -> include/UKF/
    %   ../ESKF/ -> include/ESKF/
    %   ../Common/ -> include/Common/
    inc_kf_core = ['-I' fullfile(inc_dir, 'KF')];  % for ../KF/Core/...
    inc_ekf_core = ['-I' fullfile(inc_dir, 'EKF')];  % for ../EKF/Core/...
    inc_eskf = ['-I' fullfile(inc_dir, 'ESKF')];  % for ../ESKF/...
    inc_ukf_core = ['-I' fullfile(inc_dir, 'UKF')];  % for ../UKF/Core/...
    inc_common = ['-I' fullfile(inc_dir, 'Common')];  % for ../Common/...
    inc_meukf = ['-I' fullfile(cpp_root, 'MEUKF')];  % for MEUKF headers
    inc_args = {inc_kf_core, inc_ekf_core, inc_eskf, inc_ukf_core, inc_common, inc_meukf};
    
    % Build: mex_matlab_helpers (Phase 1 helper)
    if wants('mex_matlab_helpers') && build_single_mex('mex_matlab_helpers.cpp', compile_opts, inc_args, {}, bin_dir)
        built_count = built_count + 1;
    end

    if wants('mex_sensor_preprocessor') && build_single_mex('mex_sensor_preprocessor.cpp', compile_opts, inc_args, {}, bin_dir)
        built_count = built_count + 1;
    end

    % Phase 4: mex_adaptive_predict (predict() 全体の C++ 化ラッパ)
    if wants('mex_adaptive_predict') && build_single_mex('mex_adaptive_predict.cpp', compile_opts, inc_args, {fullfile(src_dir, 'ESKF', 'eskf_core.cpp')}, bin_dir)
        built_count = built_count + 1;
    end

    try
        % Build: mex_kalman_filter_core
        if wants('mex_kalman_filter_core') && build_single_mex('mex_kalman_filter_core.cpp', compile_opts, inc_args, {}, bin_dir)
            built_count = built_count + 1;
        end

        % Build: mex_ukf_sigma_points
        if wants('mex_ukf_sigma_points') && build_single_mex('mex_ukf_sigma_points.cpp', compile_opts, inc_args, {}, bin_dir)
            built_count = built_count + 1;
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
            if wants('mex_eskf_math') && build_single_mex('mex_eskf_math.cpp', compile_opts, inc_args, {eskf_math_cpp}, bin_dir)
                built_count = built_count + 1;
            end
        end

        % mex_eskf_init
        if exist('mex_eskf_init.cpp', 'file')
            if wants('mex_eskf_init') && build_single_mex('mex_eskf_init.cpp', compile_opts, inc_args, {}, bin_dir)
                built_count = built_count + 1;
            end
        end

        % mex_eskf_get_state / free / set_state / step_handle
        if exist('mex_eskf_get_state.cpp', 'file') && wants('mex_eskf_get_state')
            if build_single_mex('mex_eskf_get_state.cpp', compile_opts, inc_args, {}, bin_dir), built_count = built_count + 1; end
        end
        if exist('mex_eskf_free.cpp', 'file') && wants('mex_eskf_free')
            if build_single_mex('mex_eskf_free.cpp', compile_opts, inc_args, {}, bin_dir), built_count = built_count + 1; end
        end
        if exist('mex_eskf_set_state.cpp', 'file') && wants('mex_eskf_set_state')
            if build_single_mex('mex_eskf_set_state.cpp', compile_opts, inc_args, {}, bin_dir), built_count = built_count + 1; end
        end
        if exist('mex_eskf_step_handle.cpp', 'file') && wants('mex_eskf_step_handle')
            if build_single_mex('mex_eskf_step_handle.cpp', compile_opts, inc_args, {}, bin_dir), built_count = built_count + 1; end
        end

        % mex_eskf_step (wrapper) - skipped here

        % mex_quaternion_lib - locked/skipped

        % mex_ukf_update
        if wants('mex_ukf_update') && build_single_mex('mex_ukf_update.cpp', compile_opts, inc_args, {}, bin_dir)
            built_count = built_count + 1;
        end

        % mex_meukf_step
        meukf_core_cpp = fullfile(cpp_root, 'MEUKF', 'meukf_core.cpp');
        if exist('mex_meukf_step.cpp', 'file') && exist(meukf_core_cpp, 'file')
            if wants('mex_meukf_step') && build_single_mex('mex_meukf_step.cpp', compile_opts, inc_args, {meukf_core_cpp}, bin_dir, 'mex_meukf_step_v2')
                built_count = built_count + 1;
            end
        end

        % mex_sensor_filter
        if wants('mex_sensor_filter') && build_single_mex('mex_sensor_filter.cpp', compile_opts, inc_args, {}, bin_dir)
            built_count = built_count + 1;
        end

        % mex_unified_filter
        unified_cpp = fullfile(cpp_root, 'MEUKF', 'unified_filter.cpp');
        if exist('mex_unified_filter.cpp', 'file') && exist(unified_cpp, 'file') && exist(meukf_core_cpp, 'file')
            if wants('mex_unified_filter') && build_single_mex('mex_unified_filter.cpp', compile_opts, inc_args, {unified_cpp, meukf_core_cpp}, bin_dir)
                built_count = built_count + 1;
            end
        end

        % mex_eskf_step final
        if exist('mex_eskf_step.cpp', 'file') && exist(unified_cpp, 'file') && exist(meukf_core_cpp, 'file')
            if wants('mex_eskf_step') && build_single_mex('mex_eskf_step.cpp', compile_opts, inc_args, {unified_cpp, meukf_core_cpp}, bin_dir)
                built_count = built_count + 1;
            end
        end
        % Phase 5: mex_filter_management
        if exist('mex_filter_management.cpp', 'file')
            if wants('mex_filter_management') && build_single_mex('mex_filter_management.cpp', compile_opts, inc_args, {}, bin_dir)
                built_count = built_count + 1;
            end
        end
        
    catch ME
        build_success = false;
        fprintf('\nBuild failed!\n');
        fprintf('Error: %s\n', ME.message);
        fprintf('\nTroubleshooting:\n');
        fprintf('1. Run "mex -setup C++" to configure compiler\n');
        fprintf('2. Ensure Visual Studio or MinGW is installed\n');
        fprintf('3. Check that all source files exist in %s\n', mex_src_dir);
    end
    
    % Return to original directory
    cd(original_dir);
    
    if build_success
        fprintf('\n=== Build Complete ===\n');
        fprintf('Successfully built %d MEX file(s)\n', built_count);
        fprintf('Output: %s\n\n', bin_dir);
        fprintf('To use MEX files, add to MATLAB path:\n');
        fprintf('  addpath(''%s'')\n', bin_dir);
    else
        error('MEX build failed. See error messages above.');
    end
end

function success = build_single_mex(mex_file, compile_opts, inc_args, extra_sources, output_dir, output_name)
    % Build a single MEX file
    success = false;
    
    if nargin < 6
        [~, output_name, ~] = fileparts(mex_file);
    end
    
    if ~exist(mex_file, 'file')
        warning('Source not found: %s', mex_file);
        return;
    end
    
    try
        % Combine all arguments
        all_sources = [{mex_file}, extra_sources];
        % Specify output name
        mex_args = [compile_opts, inc_args, {'-output', output_name}, all_sources];
        % Remove previous build artifacts that can cause stale linking issues
        mex_output = [output_name '.' mexext];
        try
            if exist(mex_output, 'file')
                delete(mex_output);
            end
        catch
            % ignore
        end
        % also remove common linker artifacts on Windows
        try
            if ispc
                if exist([output_name '.lib'], 'file'), delete([output_name '.lib']); end
                if exist([output_name '.exp'], 'file'), delete([output_name '.exp']); end
                if exist([output_name '.obj'], 'file'), delete([output_name '.obj']); end
            end
        catch
        end

        % Build (suppress verbose mex output; show concise result)
        mex_output = [output_name '.' mexext];
        fprintf('Compiling %s... ', output_name);
        try
            cmd_out = evalc('mex(mex_args{:});'); %# capture mex stdout/stderr
            if exist(mex_output, 'file')
                dest_file = fullfile(output_dir, mex_output);
                copyfile(mex_output, dest_file, 'f');
                fprintf('OK\n');
                success = true;
            else
                fprintf('FAILED\n');
                warning('Output file not generated: %s', mex_output);
            end
        catch ME
            fprintf('ERROR\n');
            warning('Failed to build %s: %s', mex_file, ME.message);
        end
    catch ME
        warning('Failed to build %s: %s', mex_file, ME.message);
    end
end


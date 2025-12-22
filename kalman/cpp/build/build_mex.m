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

    fprintf('=== MEX Build for Kalman Filters ===\n');
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
    
    fprintf('C++ root: %s\n', cpp_root);
    fprintf('MEX sources: %s\n', mex_src_dir);
    fprintf('Output: %s\n\n', bin_dir);
    
    % Check compiler
    try
        cc = mex.getCompilerConfigurations('C++', 'Selected');
        if isempty(cc)
            warning('C++ compiler not configured. Run: mex -setup C++');
            mex('-setup', 'C++');
        else
            fprintf('Compiler: %s\n\n', cc.Name);
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
    
    try
        % Build: mex_kalman_filter_core
        fprintf('=== [1/7] mex_kalman_filter_core ===\n');
        if wants('mex_kalman_filter_core') && build_single_mex('mex_kalman_filter_core.cpp', compile_opts, inc_args, {}, bin_dir)
            built_count = built_count + 1;
        end
        
        % Build: mex_ukf_sigma_points
        fprintf('\n=== [2/7] mex_ukf_sigma_points ===\n');
        if wants('mex_ukf_sigma_points') && build_single_mex('mex_ukf_sigma_points.cpp', compile_opts, inc_args, {}, bin_dir)
            built_count = built_count + 1;
        end
        
        % Build: mex_eskf_core (legacy)
        fprintf('\n=== [3/7] mex_eskf_core (legacy) ===\n');
        eskf_core_cpp = fullfile(cpp_root, 'ESKF', 'eskf_core.cpp');
        if exist('mex_eskf_core.cpp', 'file') && exist(eskf_core_cpp, 'file')
            fprintf('Sources: mex_eskf_core.cpp + eskf_core.cpp\n');
            % if build_single_mex('mex_eskf_core.cpp', compile_opts, inc_args, {eskf_core_cpp}, bin_dir)
            %     built_count = built_count + 1;
            % end
            fprintf('Skipping mex_eskf_core (locked)\n');
        else
            warning('ESKF sources not found, skipping');
        end
        
        % Build: mex_eskf_math (NEW - stateless computation library)
        fprintf('\n=== [4/7] mex_eskf_math (NEW - pure computation) ===\n');
        eskf_math_cpp = fullfile(src_dir, 'ESKF', 'eskf_math.cpp');
        if exist('mex_eskf_math.cpp', 'file') && exist(eskf_math_cpp, 'file')
            fprintf('Sources: mex_eskf_math.cpp + eskf_math.cpp\n');
            if wants('mex_eskf_math') && build_single_mex('mex_eskf_math.cpp', compile_opts, inc_args, {eskf_math_cpp}, bin_dir)
                built_count = built_count + 1;
            end
        else
            warning('ESKF math sources not found, skipping');
        end

        % Build: mex_eskf_init (NEW - ESKF object initialization)
        fprintf('\n=== [5/7] mex_eskf_init (NEW - ESKF init) ===\n');
        if exist('mex_eskf_init.cpp', 'file')
            fprintf('Sources: mex_eskf_init.cpp\n');
            if wants('mex_eskf_init') && build_single_mex('mex_eskf_init.cpp', compile_opts, inc_args, {}, bin_dir)
                built_count = built_count + 1;
            end
        else
            % not an error; optional target
        end
        
        % Build: mex_eskf_get_state
        fprintf('\n=== mex_eskf_get_state ===\n');
        if exist('mex_eskf_get_state.cpp', 'file')
            if wants('mex_eskf_get_state') && build_single_mex('mex_eskf_get_state.cpp', compile_opts, inc_args, {}, bin_dir)
                built_count = built_count + 1;
            end
        end

        % Build: mex_eskf_free
        fprintf('\n=== mex_eskf_free ===\n');
        if exist('mex_eskf_free.cpp', 'file')
            if wants('mex_eskf_free') && build_single_mex('mex_eskf_free.cpp', compile_opts, inc_args, {}, bin_dir)
                built_count = built_count + 1;
            end
        end
        
        % Build: mex_eskf_set_state
        fprintf('\n=== mex_eskf_set_state ===\n');
        if exist('mex_eskf_set_state.cpp', 'file')
            if wants('mex_eskf_set_state') && build_single_mex('mex_eskf_set_state.cpp', compile_opts, inc_args, {}, bin_dir)
                built_count = built_count + 1;
            end
        end

        % Build: mex_eskf_step_handle (NEW - handle-based wrapper)
        fprintf('\n=== mex_eskf_step_handle ===\n');
        if exist('mex_eskf_step_handle.cpp', 'file')
            if wants('mex_eskf_step_handle') && build_single_mex('mex_eskf_step_handle.cpp', compile_opts, inc_args, {}, bin_dir)
                built_count = built_count + 1;
            end
        end

        % Build: mex_eskf_step (wrapper calling mex_unified_filter)
        fprintf('\n=== mex_eskf_step (wrapper) ===\n');
        if exist('mex_eskf_step.cpp', 'file')
            fprintf('Skipping intermediate mex_eskf_step build; will build with unified sources later.\n');
        end
        
        % Build: mex_quaternion_lib
        fprintf('\n=== [5/7] mex_quaternion_lib ===\n');
        % if build_single_mex('mex_quaternion_lib.cpp', compile_opts, inc_args, {}, bin_dir)
        %     built_count = built_count + 1;
        % end
        fprintf('Skipping mex_quaternion_lib (locked)\n');
        
        % Build: mex_ukf_update
        fprintf('\n=== [6/7] mex_ukf_update ===\n');
        if wants('mex_ukf_update') && build_single_mex('mex_ukf_update.cpp', compile_opts, inc_args, {}, bin_dir)
            built_count = built_count + 1;
        end

        % Build: mex_meukf_step
        fprintf('\n=== [7/9] mex_meukf_step ===\n');
        meukf_core_cpp = fullfile(cpp_root, 'MEUKF', 'meukf_core.cpp');
        if exist('mex_meukf_step.cpp', 'file') && exist(meukf_core_cpp, 'file')
            fprintf('Sources: mex_meukf_step.cpp + meukf_core.cpp\n');
            % Build with new name to avoid lock
            if wants('mex_meukf_step') && build_single_mex('mex_meukf_step.cpp', compile_opts, inc_args, {meukf_core_cpp}, bin_dir, 'mex_meukf_step_v2')
                built_count = built_count + 1;
            end
        else
            warning('MEUKF sources not found, skipping');
        end
        
        % Build: mex_sensor_filter (NEW - for incremental testing)
        fprintf('\n=== [8/9] mex_sensor_filter ===\n');
        if wants('mex_sensor_filter') && build_single_mex('mex_sensor_filter.cpp', compile_opts, inc_args, {}, bin_dir)
            built_count = built_count + 1;
        end
        
        % Build: mex_unified_filter (NEW - unified C++ interface)
        fprintf('\n=== [9/10] mex_unified_filter ===\n');
        unified_cpp = fullfile(cpp_root, 'MEUKF', 'unified_filter.cpp');
        if exist('mex_unified_filter.cpp', 'file') && exist(unified_cpp, 'file') && exist(meukf_core_cpp, 'file')
            fprintf('Sources: mex_unified_filter.cpp + unified_filter.cpp + meukf_core.cpp\n');
            if wants('mex_unified_filter') && build_single_mex('mex_unified_filter.cpp', compile_opts, inc_args, {unified_cpp, meukf_core_cpp}, bin_dir)
                built_count = built_count + 1;
            end
        else
            warning('Unified filter sources not found, skipping');
        end
        
        % Build: mex_eskf_step (NEW - ESKF統合: 予測+全センサー更新)
        fprintf('\n=== [10/10] mex_eskf_step ===\n');
        if exist('mex_eskf_step.cpp', 'file') && exist(unified_cpp, 'file') && exist(meukf_core_cpp, 'file')
            fprintf('Sources: mex_eskf_step.cpp + unified_filter.cpp + meukf_core.cpp\n');
            if wants('mex_eskf_step') && build_single_mex('mex_eskf_step.cpp', compile_opts, inc_args, {unified_cpp, meukf_core_cpp}, bin_dir)
                built_count = built_count + 1;
            end
        else
            warning('ESKF step sources not found, skipping');
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

        % Build
        fprintf('Compiling...\n');
        mex(mex_args{:});
        
        % Check output
        mex_output = [output_name '.' mexext];

        if exist(mex_output, 'file')
            % Copy to output directory
            dest_file = fullfile(output_dir, mex_output);
            copyfile(mex_output, dest_file, 'f');
            fprintf('Built: %s\n', mex_output);
            success = true;
        else
            warning('Output file not generated: %s', mex_output);
        end
    catch ME
        warning('Failed to build %s: %s', mex_file, ME.message);
    end
end


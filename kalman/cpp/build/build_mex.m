function build_mex()
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
    inc_args = {inc_kf_core, inc_ekf_core, inc_eskf, inc_ukf_core, inc_common};
    
    try
        % Build: mex_kalman_filter_core
        fprintf('=== [1/6] mex_kalman_filter_core ===\n');
        if build_single_mex('mex_kalman_filter_core.cpp', compile_opts, inc_args, {}, bin_dir)
            built_count = built_count + 1;
        end
        
        % Build: mex_ukf_sigma_points
        fprintf('\n=== [2/6] mex_ukf_sigma_points ===\n');
        if build_single_mex('mex_ukf_sigma_points.cpp', compile_opts, inc_args, {}, bin_dir)
            built_count = built_count + 1;
        end
        
        % Build: mex_eskf_core (OLD - will be deprecated)
        fprintf('\n=== [3/6] mex_eskf_core (legacy) ===\n');
        eskf_core_cpp = fullfile(src_dir, 'ESKF', 'eskf_core.cpp');
        if exist('mex_eskf_core.cpp', 'file') && exist(eskf_core_cpp, 'file')
            fprintf('Sources: mex_eskf_core.cpp + eskf_core.cpp\n');
            if build_single_mex('mex_eskf_core.cpp', compile_opts, inc_args, {eskf_core_cpp}, bin_dir)
                built_count = built_count + 1;
            end
        else
            warning('ESKF sources not found, skipping');
        end
        
        % Build: mex_eskf_math (NEW - stateless computation library)
        fprintf('\n=== [4/6] mex_eskf_math (NEW - pure computation) ===\n');
        eskf_math_cpp = fullfile(src_dir, 'ESKF', 'eskf_math.cpp');
        if exist('mex_eskf_math.cpp', 'file') && exist(eskf_math_cpp, 'file')
            fprintf('Sources: mex_eskf_math.cpp + eskf_math.cpp\n');
            if build_single_mex('mex_eskf_math.cpp', compile_opts, inc_args, {eskf_math_cpp}, bin_dir)
                built_count = built_count + 1;
            end
        else
            warning('ESKF math sources not found, skipping');
        end
        
        % Build: mex_quaternion_lib
        fprintf('\n=== [5/6] mex_quaternion_lib ===\n');
        if build_single_mex('mex_quaternion_lib.cpp', compile_opts, inc_args, {}, bin_dir)
            built_count = built_count + 1;
        end
        
        % Build: mex_ukf_update
        fprintf('\n=== [6/6] mex_ukf_update ===\n');
        if build_single_mex('mex_ukf_update.cpp', compile_opts, inc_args, {}, bin_dir)
            built_count = built_count + 1;
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

function success = build_single_mex(mex_file, compile_opts, inc_args, extra_sources, output_dir)
    % Build a single MEX file
    success = false;
    
    if ~exist(mex_file, 'file')
        warning('Source not found: %s', mex_file);
        return;
    end
    
    try
        % Combine all arguments
        all_sources = [{mex_file}, extra_sources];
        mex_args = [compile_opts, inc_args, all_sources];
        
        % Build
        fprintf('Compiling...\n');
        mex(mex_args{:});
        
        % Check output
        [~, mex_name, ~] = fileparts(mex_file);
        mex_output = [mex_name '.' mexext];
        
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


function diagnose_mex_binary()
    % diagnose_mex_binary - MEXバイナリの詳細診断
    % コンパイラ情報、バイナリの健全性、実際の動作を検証
    
    clc;
    fprintf('================================================================================\n');
    fprintf('MEX BINARY DIAGNOSTIC REPORT\n');
    fprintf('Generated at: %s\n', datestr(now));
    fprintf('================================================================================\n\n');
    
    %% 1. MATLAB & Compiler Environment
    fprintf('【TEST 1】MATLAB & Compiler Environment\n');
    fprintf('--------------------------------------------------------------------------------\n');
    fprintf('MATLAB Version: %s\n', version);
    fprintf('Architecture: %s\n', computer('arch'));
    fprintf('MEX Extension: %s\n', mexext);
    
    % Get MEX compiler info
    try
        cc = mex.getCompilerConfigurations('C++', 'Selected');
        if ~isempty(cc)
            fprintf('C++ Compiler: %s\n', cc.Name);
            fprintf('Compiler Version: %s\n', cc.Version);
            fprintf('Compiler Location: %s\n', cc.Location);
        else
            fprintf('⚠️ No C++ compiler selected\n');
        end
    catch
        fprintf('⚠️ Could not retrieve compiler info\n');
    end
    fprintf('\n');
    
    %% 2. MEX Binary Analysis
    fprintf('【TEST 2】MEX Binary Analysis\n');
    fprintf('--------------------------------------------------------------------------------\n');
    
    proj_root = fileparts(mfilename('fullpath'));
    bin_dir = fullfile(proj_root, 'cpp', 'bin');
    
    mex_files = {'mex_run_eskf.mexw64', 'mex_meukf_step_v2.mexw64'};
    
    for i = 1:length(mex_files)
        mex_path = fullfile(bin_dir, mex_files{i});
        if exist(mex_path, 'file')
            info = dir(mex_path);
            fprintf('  %s:\n', mex_files{i});
            fprintf('    Size: %d bytes (%.2f KB)\n', info.bytes, info.bytes/1024);
            fprintf('    Modified: %s\n', datestr(info.datenum));
            
            % Size analysis
            if info.bytes > 500000
                fprintf('    ⚠️ WARNING: Large binary - may contain debug symbols\n');
            elseif info.bytes < 100000
                fprintf('    ✅ Optimized binary (release mode)\n');
            else
                fprintf('    ℹ️ Standard size\n');
            end
        else
            fprintf('  ❌ %s: NOT FOUND\n', mex_files{i});
        end
    end
    fprintf('\n');
    
    %% 3. MEX Function Loading Test
    fprintf('【TEST 3】MEX Function Loading Test\n');
    fprintf('--------------------------------------------------------------------------------\n');
    
    addpath(bin_dir);
    
    try
        % Clear MEX first
        clear mex;
        
        % Try loading MEX function
        mex_run_eskf('invalid_command');
    catch ME
        if contains(ME.identifier, 'unknown') || contains(ME.message, 'Unknown')
            fprintf('✅ mex_run_eskf loaded successfully (got expected error for invalid command)\n');
        else
            fprintf('❌ mex_run_eskf failed to load: %s\n', ME.message);
        end
    end
    fprintf('\n');
    
    %% 4. Quick Functional Test
    fprintf('【TEST 4】Quick Functional Test (5 steps)\n');
    fprintf('--------------------------------------------------------------------------------\n');
    
    try
        addpath(fullfile(proj_root, 'GenerateData'));
        params = config_params();
        
        % Read sensor data
        obs_file = fullfile(proj_root, 'GenerateData', 'sensor_data.csv');
        if ~exist(obs_file, 'file')
            % Generate data first
            addpath(fullfile(proj_root, 'GenerateData'));
            sim_generate();
        end
        obs = read_csv(obs_file);
        dt = mean(diff(obs.time));
        
        % Initialize ESKF
        handle = mex_run_eskf('init', obs, params.static_time, dt);
        
        % Get initial state
        state0 = mex_run_eskf('get_state', handle);
        fprintf('Initial quaternion: [%.6f, %.6f, %.6f, %.6f]\n', ...
            state0.q(1), state0.q(2), state0.q(3), state0.q(4));
        fprintf('Initial position: [%.6f, %.6f, %.6f]\n', ...
            state0.p(1), state0.p(2), state0.p(3));
        fprintf('Initial gyro bias: [%.6f, %.6f, %.6f]\n', ...
            state0.bg(1), state0.bg(2), state0.bg(3));
        
        % Run 5 steps after static period
        static_samples = floor(params.static_time / dt);
        start_k = static_samples + 1;
        
        fprintf('\nRunning 5 steps (from k=%d)...\n', start_k);
        
        for k = start_k : min(start_k+4, numel(obs.time))
            mex_run_eskf('step', handle, obs, k);
            state = mex_run_eskf('get_state', handle);
            fprintf('  k=%d: p=[%.4f, %.4f, %.4f], bg=[%.6f, %.6f, %.6f]\n', ...
                k, state.p(1), state.p(2), state.p(3), ...
                state.bg(1), state.bg(2), state.bg(3));
        end
        
        % Get final state
        state_final = mex_run_eskf('get_state', handle);
        
        % Check if state changed
        p_changed = any(abs(state_final.p - state0.p) > 1e-10);
        bg_changed = any(abs(state_final.bg - state0.bg) > 1e-10);
        
        if p_changed && bg_changed
            fprintf('✅ State is being updated correctly\n');
        elseif p_changed && ~bg_changed
            fprintf('⚠️ Position updated but gyro bias NOT updated\n');
            fprintf('   This indicates sensor updates may not be working!\n');
        else
            fprintf('❌ State is NOT being updated - filter is broken\n');
        end
        
        mex_run_eskf('free', handle);
        
    catch ME
        fprintf('❌ Functional test failed: %s\n', ME.message);
        fprintf('   Stack:\n');
        for i = 1:length(ME.stack)
            fprintf('     %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
        end
    end
    fprintf('\n');
    
    %% 5. Covariance Matrix Check
    fprintf('【TEST 5】Covariance Matrix Check\n');
    fprintf('--------------------------------------------------------------------------------\n');
    
    try
        handle = mex_run_eskf('init', obs, params.static_time, dt);
        state = mex_run_eskf('get_state', handle);
        
        P = state.P;
        P_diag = diag(P);
        
        fprintf('P matrix size: %dx%d\n', size(P, 1), size(P, 2));
        fprintf('P diagonal (first 5): [%.6e, %.6e, %.6e, %.6e, %.6e]\n', ...
            P_diag(1), P_diag(2), P_diag(3), P_diag(4), P_diag(5));
        fprintf('P diagonal (bg, last 3): [%.6e, %.6e, %.6e]\n', ...
            P_diag(13), P_diag(14), P_diag(15));
        
        % Check if P is positive definite
        eigvals = eig(P);
        min_eig = min(eigvals);
        max_eig = max(eigvals);
        
        if min_eig > 0
            fprintf('✅ P is positive definite (eigenvalues in [%.2e, %.2e])\n', min_eig, max_eig);
        else
            fprintf('❌ P is NOT positive definite (min eigenvalue: %.2e)\n', min_eig);
        end
        
        % Check symmetry
        sym_error = max(max(abs(P - P')));
        if sym_error < 1e-10
            fprintf('✅ P is symmetric (max asymmetry: %.2e)\n', sym_error);
        else
            fprintf('⚠️ P asymmetry detected: %.2e\n', sym_error);
        end
        
        mex_run_eskf('free', handle);
        
    catch ME
        fprintf('❌ Covariance check failed: %s\n', ME.message);
    end
    fprintf('\n');
    
    %% 6. Build Environment Check
    fprintf('【TEST 6】Build Environment Check\n');
    fprintf('--------------------------------------------------------------------------------\n');
    
    % Check COMPFLAGS
    compflags = getenv('COMPFLAGS');
    if ~isempty(compflags)
        fprintf('COMPFLAGS: %s\n', compflags);
    else
        fprintf('COMPFLAGS: (not set)\n');
    end
    
    % Check PATH for compilers
    path_env = getenv('PATH');
    if contains(path_env, 'MSVC') || contains(path_env, 'Microsoft Visual Studio')
        fprintf('ℹ️ MSVC detected in PATH\n');
    end
    if contains(path_env, 'MinGW') || contains(path_env, 'mingw')
        fprintf('ℹ️ MinGW detected in PATH\n');
    end
    
    fprintf('\n');
    
    %% Summary
    fprintf('================================================================================\n');
    fprintf('SUMMARY\n');
    fprintf('================================================================================\n');
    fprintf('Run this script on BOTH PCs and compare outputs.\n');
    fprintf('Key differences to look for:\n');
    fprintf('  1. Compiler name and version\n');
    fprintf('  2. MEX binary size (should be similar)\n');
    fprintf('  3. Gyro bias updates (should NOT be zero after steps)\n');
    fprintf('  4. Covariance matrix properties\n');
    fprintf('================================================================================\n');
end

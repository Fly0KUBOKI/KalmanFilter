% environment_quick_fix.m
% 環境依存性の問題を素早く診断・修正するための統合スクリプト
% 
% 使用方法:
%   environment_quick_fix()
%   または
%   [diag, success] = environment_quick_fix();

function [diagnostics, success] = environment_quick_fix()
    success = false;
    diagnostics = struct();
    
    fprintf('\n');
    fprintf('╔════════════════════════════════════════════════════════════════════════════╗\n');
    fprintf('║  KALMANFILTER ENVIRONMENT QUICK FIX                                       ║\n');
    fprintf('║  環境依存性問題の自動診断・修正                                            ║\n');
    fprintf('╚════════════════════════════════════════════════════════════════════════════╝\n\n');
    
    % Get current directory
    current_dir = pwd;
    proj_root = fileparts(mfilename('fullpath'));
    
    % ======================== STEP 1: Initial Diagnostics ========================
    fprintf('【STEP 1】初期診断中...\n');
    fprintf('-' * 80 + "\n");
    
    % Check MATLAB environment
    ver_info = ver('MATLAB');
    arch = computer;
    ext = mexext;
    
    fprintf('MATLAB Version: %s\n', ver_info.Version);
    fprintf('Architecture: %s\n', arch);
    fprintf('MEX Extension: %s\n\n', ext);
    
    if ~strcmp(ext, 'mexw64')
        fprintf('❌ WARNING: MEX extension is %s (expected mexw64)\n', ext);
        fprintf('   Available MEX binaries may not be compatible.\n');
    end
    
    diagnostics.matlab_version = ver_info.Version;
    diagnostics.matlab_arch = arch;
    diagnostics.mex_ext = ext;
    
    % ======================== STEP 2: Check MEX Binary ========================
    fprintf('【STEP 2】MEXバイナリ確認...\n');
    fprintf('-' * 80 + "\n");
    
    mex_bin_dir = fullfile(proj_root, '..', 'kalman', 'cpp', 'bin');
    mex_files = dir(fullfile(mex_bin_dir, ['*.' mexext]));
    
    if isempty(mex_files)
        fprintf('❌ MEXバイナリが見つかりません。\n');
        fprintf('   パス: %s\n\n', mex_bin_dir);
        fprintf('修正実行中...\n');
        
        % Try to build MEX
        try
            cd(fullfile(proj_root, '..', 'kalman', 'cpp', 'build'));
            fprintf('Building MEX files...\n');
            build_mex();
            fprintf('✅ MEX build completed.\n\n');
        catch ME
            fprintf('❌ MEX build failed: %s\n', ME.message);
            cd(current_dir);
            return;
        end
    else
        fprintf('✅ Found %d MEX binary files:\n', length(mex_files));
        for i = 1:length(mex_files)
            fprintf('   - %s\n', mex_files(i).name);
        end
        fprintf('\n');
    end
    
    % Clear MEX cache
    clear functions; clear mex; rehash;
    
    % ======================== STEP 3: Verify MEX Callable ========================
    fprintf('【STEP 3】MEX関数呼び出し可能性確認...\n');
    fprintf('-' * 80 + "\n");
    
    try
        % Add paths
        addpath(fullfile(proj_root, '..', 'kalman', 'cpp', 'bin'));
        addpath(fullfile(proj_root, '..', 'kalman', 'GenerateData'));
        
        % Test MEX function
        if exist('mex_run_eskf', 'file') == 3
            fprintf('✅ mex_run_eskf is callable\n');
            
            % Try to get usage error (which indicates MEX is loaded)
            try
                mex_run_eskf();
            catch ME
                if contains(ME.message, 'Command required')
                    fprintf('✅ MEX function loaded successfully\n\n');
                else
                    fprintf('⚠️  Unexpected MEX error: %s\n\n', ME.message);
                end
            end
        else
            fprintf('❌ mex_run_eskf not found\n\n');
        end
    catch ME
        fprintf('❌ Error during MEX test: %s\n\n', ME.message);
    end
    
    % ======================== STEP 4: Check Configuration ========================
    fprintf('【STEP 4】設定パラメータ確認...\n');
    fprintf('-' * 80 + "\n");
    
    try
        params = config_params();
        
        fprintf('✅ config_params loaded\n');
        fprintf('   static_time: %.1f sec\n', params.static_time);
        fprintf('   dt: %.4f sec\n', params.dt);
        fprintf('   GPS origin: [%.6f, %.6f]\n', ...
            params.gps_origin.lat, params.gps_origin.lon);
        fprintf('   Noise: accel=%.3f, gyro=%.3f\n\n', ...
            params.noise.accel_std, params.noise.gyro_std);
        
        if params.static_time < 3
            fprintf('⚠️  WARNING: static_time=%.1f is very short\n', params.static_time);
            fprintf('   Recommended: >= 5 seconds for good initialization\n\n');
        end
        
        diagnostics.static_time = params.static_time;
        diagnostics.gps_origin = params.gps_origin;
        
    catch ME
        fprintf('❌ Failed to load config_params: %s\n\n', ME.message);
        diagnostics.config_error = ME.message;
    end
    
    % ======================== STEP 5: Check Sensor Data ========================
    fprintf('【STEP 5】センサーデータ確認...\n');
    fprintf('-' * 80 + "\n");
    
    try
        obs_file = fullfile(proj_root, '..', 'kalman', 'GenerateData', 'sensor_data.csv');
        
        if ~exist(obs_file, 'file')
            fprintf('⚠️  sensor_data.csv not found\n');
            fprintf('   Creating sensor data...\n');
            sim_generate();
        end
        
        obs = read_csv(obs_file);
        
        fprintf('✅ Sensor data loaded\n');
        fprintf('   Samples: %d\n', length(obs.time));
        fprintf('   ax type: %s\n', class(obs.ax));
        fprintf('   lat type: %s\n', class(obs.lat));
        fprintf('\n');
        
        % Check types
        type_ok = true;
        if ~strcmp(class(obs.ax), 'single')
            fprintf('⚠️  ax is not single (found: %s)\n', class(obs.ax));
            type_ok = false;
        end
        if ~strcmp(class(obs.lat), 'double')
            fprintf('⚠️  lat is not double (found: %s)\n', class(obs.lat));
            type_ok = false;
        end
        
        if type_ok
            fprintf('✅ Sensor data types OK\n\n');
        else
            fprintf('⚠️  Sensor data type mismatch detected\n\n');
        end
        
        diagnostics.sensor_data_types_ok = type_ok;
        diagnostics.n_sensor_samples = length(obs.time);
        
    catch ME
        fprintf('❌ Error checking sensor data: %s\n\n', ME.message);
        diagnostics.sensor_error = ME.message;
    end
    
    % ======================== STEP 6: Test Run ========================
    fprintf('【STEP 6】試運転実行（単一シミュレーション）...\n');
    fprintf('-' * 80 + "\n");
    
    try
        fprintf('Initializing and running filter for 1st sample...\n');
        
        cd(fullfile(proj_root, '..', 'kalman'));
        
        % Check if we have sensor data
        if ~exist('GenerateData/sensor_data.csv', 'file')
            fprintf('Generating sensor data...\n');
            sim_generate();
        end
        
        % Run a quick test
        obs = read_observation(pwd);
        params = config_params();
        dt = calculate_dt(obs);
        
        % Initialize ESKF
        handle = mex_run_eskf('init', obs, params.static_time, dt);
        
        % Run one step
        mex_run_eskf('step', handle, obs, params.static_time/dt + 1);
        
        % Get state
        state = mex_run_eskf('get_state', handle);
        
        % Cleanup
        mex_run_eskf('free', handle);
        
        fprintf('✅ Filter initialization and execution successful\n');
        fprintf('   Position: [%.2f, %.2f, %.2f] m\n', state.p(1), state.p(2), state.p(3));
        fprintf('   Velocity: [%.2f, %.2f, %.2f] m/s\n', state.v(1), state.v(2), state.v(3));
        fprintf('   Attitude: [%.2f, %.2f, %.2f] deg\n\n', ...
            state.euler(1), state.euler(2), state.euler(3));
        
        success = true;
        diagnostics.test_run_successful = true;
        
    catch ME
        fprintf('❌ Test run failed: %s\n\n', ME.message);
        diagnostics.test_run_error = ME.message;
        success = false;
    end
    
    % ======================== Summary ========================
    fprintf('╔════════════════════════════════════════════════════════════════════════════╗\n');
    fprintf('║  診断結果サマリー                                                           ║\n');
    fprintf('╚════════════════════════════════════════════════════════════════════════════╝\n\n');
    
    if success
        fprintf('✅ 環境は正常に設定されています。\n\n');
        fprintf('推奨実行コマンド:\n');
        fprintf('   run_simulation(42, false);        % 単一実行テスト\n');
        fprintf('   run_batch_10sets();               % バッチテスト\n');
    else
        fprintf('❌ 環境に問題があります。\n\n');
        fprintf('次のステップ:\n');
        fprintf('1. build_mex() で環境専用ビルドを実行\n');
        fprintf('2. diagnose_environment() で詳細診断\n');
        fprintf('3. docs/ENVIRONMENT_DEPENDENCY_GUIDE.md を参照\n');
    end
    
    cd(current_dir);
    fprintf('\n');
    
end

% Helper functions (copied from run_simulation.m)
function obs = read_observation(proj_root)
    obs_file = fullfile(proj_root, 'GenerateData', 'sensor_data.csv');
    if ~exist(obs_file, 'file'); error('sensor_data.csv not found: %s', obs_file); end
    obs = read_csv(obs_file);
end

function dt = calculate_dt(obs)
    if length(obs.time) < 2; error('Observation data too short'); end
    dt = mean(diff(obs.time));
end


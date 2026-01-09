function [report, passed_tests] = diagnose_environment()
    % diagnose_environment - 現在の環境の詳細情報を取得し、問題を特定
    % 使用方法: 
    %   [report, tests] = diagnose_environment();
    %   disp(report);
    %
    % 戻り値:
    %   report: 診断結果の文字列（出力用）
    %   passed_tests: struct(テスト結果の詳細)
    
    clc;
    clear functions; clear mex; rehash;
    
    report = '';
    test_count = 0;
    pass_count = 0;
    
    % Helper function to add line to report
    function add_line(msg)
        report = [report sprintf('%s\n', msg)];
    end
    
    % Header
    add_line('================================================================================');
    add_line('ENVIRONMENT DEPENDENCY DIAGNOSTIC REPORT');
    add_line(sprintf('Generated at: %s', datestr(now)));
    add_line('================================================================================');
    add_line('');
    
    % ======================== TEST 1: MATLAB Environment ========================
    add_line('【TEST 1】MATLAB Environment');
    add_line(repmat('-', 1, 80));
    test_count = test_count + 1;
    
    [~, result] = system('ver');
    matlab_ver = ver('MATLAB');
    comp = computer;
    ext = mexext;
    
    add_line(sprintf('MATLABVersion: %s', matlab_ver.Version));
    add_line(sprintf('Architecture: %s', comp));
    add_line(sprintf('MEX Extension: %s', ext));
    
    % 64-bit 確認
    if strcmp(ext, 'mexw64')
        add_line('✅ 64-bit MATLAB detected (correct for mex_run_eskf.mexw64)');
        pass_count = pass_count + 1;
    elseif strcmp(ext, 'mexw32')
        add_line('❌ 32-bit MATLAB detected (incompatible with .mexw64 binary)');
        add_line('   → 64-bit MATLAB をインストール、またはMEXを再ビルド (mexext=mexw32) してください');
    else
        add_line(sprintf('⚠️  Unknown architecture: %s', ext));
    end
    
    passed_tests.matlab_version = matlab_ver.Version;
    passed_tests.matlab_arch = comp;
    passed_tests.mex_ext = ext;
    add_line('');
    
    % ======================== TEST 2: MEX Binary Check ========================
    add_line('【TEST 2】MEX Binary Availability');
    add_line(repmat('-', 1, 80));
    test_count = test_count + 1;
    
    mex_bin_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'kalman', 'cpp', 'bin');
    if exist(mex_bin_dir, 'dir')
        add_line(sprintf('✅ MEX binary directory found: %s', mex_bin_dir));
        
        mex_files = dir(fullfile(mex_bin_dir, ['*.' mexext]));
        if ~isempty(mex_files)
            add_line(sprintf('Found %d MEX binaries:', length(mex_files)));
            for i = 1:length(mex_files)
                finfo = dir(fullfile(mex_bin_dir, mex_files(i).name));
                add_line(sprintf('  - %s (%d bytes, modified %s)', ...
                    mex_files(i).name, finfo(1).bytes, finfo(1).date));
            end
            pass_count = pass_count + 1;
        else
            add_line('❌ No MEX binaries found with correct extension');
            add_line(sprintf('   → Expected: *%s', ['.' mexext]));
        end
    else
        add_line(sprintf('❌ MEX binary directory not found: %s', mex_bin_dir));
    end
    add_line('');
    
    % ======================== TEST 3: config_params Check ========================
    add_line('【TEST 3】Configuration Parameters');
    add_line(repmat('-', 1, 80));
    test_count = test_count + 1;
    
    try
        % Add GenerateData to path temporarily
        proj_root = fileparts(mfilename('fullpath'));
        addpath(fullfile(proj_root, '..', 'kalman', 'GenerateData'));
        
        params = config_params();
        
        add_line('✅ config_params() loaded successfully');
        add_line(sprintf('  static_time: %.1f sec', params.static_time));
        add_line(sprintf('  dt: %.4f sec', params.dt));
        add_line(sprintf('  GPS origin: lat=%.6f, lon=%.6f, alt=%.2f', ...
            params.gps_origin.lat, params.gps_origin.lon, params.gps_origin.alt));
        
        % Validate parameters
        if params.static_time < 2
            add_line('  ⚠️  static_time < 2 sec (may cause poor roll/pitch initialization)');
        else
            pass_count = pass_count + 1;
        end
        
        add_line(sprintf('  Sensor Noise Std (σ):'));
        add_line(sprintf('    - accel: %.3f m/s²', params.noise.accel_std));
        add_line(sprintf('    - gyro: %.3f deg/s', params.noise.gyro_std));
        add_line(sprintf('    - mag: %.3f nT', params.noise.mag_std));
        
        passed_tests.static_time = params.static_time;
        passed_tests.dt = params.dt;
        passed_tests.gps_origin = params.gps_origin;
        
    catch ME
        add_line(sprintf('❌ Failed to load config_params: %s', ME.message));
        passed_tests.config_error = ME.message;
    end
    add_line('');
    
    % ======================== TEST 4: Sensor Data Format ========================
    add_line('【TEST 4】Sensor Data Format & Types');
    add_line(repmat('-', 1, 80));
    test_count = test_count + 1;
    
    try
        % Load sensor data
        obs_file = fullfile(fileparts(mfilename('fullpath')), '..', 'kalman', 'GenerateData', 'sensor_data.csv');
        if ~exist(obs_file, 'file')
            add_line(sprintf('⚠️  sensor_data.csv not found at: %s', obs_file));
            add_line('   → Run sim_generate() to create sensor data');
        else
            obs = read_csv(obs_file);
            
            add_line('✅ sensor_data.csv loaded');
            add_line(sprintf('  Samples: %d', length(obs.time)));
            
            % Check data types
            add_line('Data Types:');
            type_ok = true;
            
            % (removed complex anonymous helper - explicit checks follow below)
            
            % Sensor types
            if strcmp(class(obs.ax), 'single')
                add_line(sprintf('  ✅ ax: %s', class(obs.ax)));
            else
                add_line(sprintf('  ❌ ax: %s (expected single)', class(obs.ax)));
                type_ok = false;
            end
            if strcmp(class(obs.wx), 'single')
                add_line(sprintf('  ✅ wx: %s', class(obs.wx)));
            else
                add_line(sprintf('  ❌ wx: %s (expected single)', class(obs.wx)));
                type_ok = false;
            end
            if strcmp(class(obs.mx), 'single')
                add_line(sprintf('  ✅ mx: %s', class(obs.mx)));
            else
                add_line(sprintf('  ❌ mx: %s (expected single)', class(obs.mx)));
                type_ok = false;
            end
            
            % GPS types (must be double)
            if strcmp(class(obs.lat), 'double')
                add_line(sprintf('  ✅ lat: %s (GPS - must be double)', class(obs.lat)));
            else
                add_line(sprintf('  ❌ lat: %s (expected double for GPS)', class(obs.lat)));
                type_ok = false;
            end
            if strcmp(class(obs.lon), 'double')
                add_line(sprintf('  ✅ lon: %s (GPS - must be double)', class(obs.lon)));
            else
                add_line(sprintf('  ❌ lon: %s (expected double for GPS)', class(obs.lon)));
                type_ok = false;
            end
            if strcmp(class(obs.alt), 'double')
                add_line(sprintf('  ✅ alt: %s (GPS - must be double)', class(obs.alt)));
            else
                add_line(sprintf('  ❌ alt: %s (expected double for GPS)', class(obs.alt)));
                type_ok = false;
            end
            
            if type_ok
                pass_count = pass_count + 1;
            end
            
            add_line('');
            add_line('Data Ranges:');
            add_line(sprintf('  ax: [%.6f, %.6f] m/s²', min(obs.ax), max(obs.ax)));
            add_line(sprintf('  wx: [%.6f, %.6f] deg/s', min(obs.wx), max(obs.wx)));
            add_line(sprintf('  mx: [%.6f, %.6f] nT', min(obs.mx), max(obs.mx)));
            add_line(sprintf('  pressure: [%.2f, %.2f] Pa', min(obs.pressure), max(obs.pressure)));
            add_line(sprintf('  lat: [%.10f, %.10f] rad', min(obs.lat), max(obs.lat)));
            add_line(sprintf('  lon: [%.10f, %.10f] rad', min(obs.lon), max(obs.lon)));
            add_line(sprintf('  alt: [%.2f, %.2f] m', min(obs.alt), max(obs.alt)));
            
            passed_tests.sensor_data_types_ok = type_ok;
            passed_tests.sensor_n_samples = length(obs.time);
        end
    catch ME
        add_line(sprintf('❌ Failed to check sensor data: %s', ME.message));
        passed_tests.sensor_error = ME.message;
    end
    add_line('');
    
    % ======================== TEST 5: MEX Function Test ========================
    add_line('【TEST 5】MEX Function Callable Test');
    add_line(repmat('-', 1, 80));
    test_count = test_count + 1;
    
    try
        if exist('mex_run_eskf', 'file') == 3
            add_line('✅ mex_run_eskf is callable');
            
            % Try a simple call (will fail but checks if MEX loads)
            try
                % This should error because we're not providing proper arguments
                % but it shows the MEX is loadable
                mex_run_eskf();
            catch ME
                % Expected error
                if contains(ME.message, 'Command required') || contains(ME.message, 'usage')
                    add_line('✅ MEX loaded successfully (expected usage error)');
                    pass_count = pass_count + 1;
                else
                    add_line(sprintf('⚠️  Unexpected MEX error: %s', ME.message));
                end
            end
        else
            add_line('❌ mex_run_eskf is NOT callable');
            add_line('   → Check if mex_run_eskf.mexw64 exists in kalman/cpp/bin/');
            add_line('   → Add path with: addpath(fullfile(kalman_root, ''cpp'', ''bin''))');
        end
    catch ME
        add_line(sprintf('❌ MEX function test failed: %s', ME.message));
    end
    add_line('');
    
    % ======================== TEST 6: Quick Simulation Test ========================
    add_line('【TEST 6】Quick Simulation Test (Optional - runs actual filter)');
    add_line('-' * 80);
    add_line('⏭️  Skipping quick simulation test (may take 30+ seconds)');
    add_line('To run: [~, ~] = diagnose_environment(true);');
    add_line('');
    
    % ======================== SUMMARY ========================
    add_line('================================================================================');
    add_line('SUMMARY');
    add_line('================================================================================');
    add_line(sprintf('Tests Passed: %d/%d', pass_count, test_count));
    add_line('');
    
    if pass_count == test_count
        add_line('✅ All checks passed! Environment is properly configured.');
    elseif pass_count >= test_count - 1
        add_line('⚠️  Most checks passed. Minor issues detected.');
        add_line('   → See above for details. Simulation may still work.');
    else
        add_line('❌ Multiple issues detected. Simulation will likely fail.');
        add_line('   → Follow the recommendations above to fix issues.');
    end
    
    add_line('');
    add_line('Next Steps:');
    add_line('1. If MEX binary not found: cd kalman/cpp/build && build_mex()');
    add_line('2. If sensor data types wrong: check read_csv() function');
    add_line('3. If config_params error: verify kalman/GenerateData/config_params.m exists');
    add_line('4. Run: run_simulation(42, false) to test single simulation');
    
    add_line('================================================================================');
    
    % Save report to file
    report_file = fullfile(fileparts(mfilename('fullpath')), '..', 'kalman', 'Results', 'environment_diagnostic.txt');
    try
        if ~exist(fileparts(report_file), 'dir')
            mkdir(fileparts(report_file));
        end
        fid = fopen(report_file, 'w');
        fprintf(fid, '%s', report);
        fclose(fid);
        add_line(sprintf('Report saved to: %s', report_file));
    catch
        % Silently fail if can't save
    end
    
    % Display report
    disp(report);
end


function compare_environments(env1_csv, env2_csv)
    % compare_environments - 2つの異なる環境で実行した結果を比較
    % 使用方法:
    %   compare_environments('env1_estimation_01.csv', 'env2_estimation_01.csv')
    %
    % 前提条件:
    %   1. 環境1で実行: run_simulation(42, false); copyfile('kalman/Results/estimation_01.csv', 'env1_estimation_01.csv');
    %   2. 環境2で実行: run_simulation(42, false); copyfile('kalman/Results/estimation_01.csv', 'env2_estimation_01.csv');
    %   3. 比較実行: compare_environments('env1_estimation_01.csv', 'env2_estimation_01.csv');
    
    if nargin < 2
        error('Usage: compare_environments(env1_csv, env2_csv)');
    end
    
    fprintf('================================================================================\n');
    fprintf('ENVIRONMENT COMPARISON REPORT\n');
    fprintf('Generated at: %s\n', datestr(now));
    fprintf('================================================================================\n\n');
    
    % Load CSVs
    fprintf('Loading CSV files...\n');
    if ~exist(env1_csv, 'file')
        error('File not found: %s', env1_csv);
    end
    if ~exist(env2_csv, 'file')
        error('File not found: %s', env2_csv);
    end
    
    T1 = readtable(env1_csv);
    T2 = readtable(env2_csv);
    
    fprintf('✅ Loaded %s (%d rows)\n', env1_csv, height(T1));
    fprintf('✅ Loaded %s (%d rows)\n\n', env2_csv, height(T2));
    
    % Check row count
    if height(T1) ~= height(T2)
        fprintf('⚠️  Row counts differ: %d vs %d\n', height(T1), height(T2));
    end
    
    % Compare columns
    fprintf('Column Comparison:\n');
    fprintf('%s\n', repmat('-', 1, 80));
    
    cols_to_compare = {'p_east', 'p_north', 'p_up', 'v_east', 'v_north', 'v_up', ...
                       'roll', 'pitch', 'yaw', 'ba_x', 'ba_y', 'ba_z', 'bg_x', 'bg_y', 'bg_z'};
    
    comparison_results = struct();
    
    for i = 1:length(cols_to_compare)
        col = cols_to_compare{i};
        
        % Check if column exists in both tables
        if ~ismember(col, T1.Properties.VariableNames)
            fprintf('⚠️  %s not in env1 table\n', col);
            continue;
        end
        if ~ismember(col, T2.Properties.VariableNames)
            fprintf('⚠️  %s not in env2 table\n', col);
            continue;
        end
        
        v1 = T1.(col);
        v2 = T2.(col);
        
        % Handle length difference
        n = min(length(v1), length(v2));
        v1 = v1(1:n);
        v2 = v2(1:n);
        
        % Calculate metrics
        diff = v1 - v2;
        abs_diff = abs(diff);
        
        max_error = max(abs_diff);
        mean_error = mean(abs_diff);
        std_error = std(abs_diff);
        
        % RMSE
        rmse = sqrt(mean(diff.^2));
        
        % Store results
        comparison_results.(col) = struct('max_error', max_error, 'mean_error', mean_error, ...
                                         'std_error', std_error, 'rmse', rmse);
        
        % Print summary
        fprintf('%s:\n', col);
        fprintf('  Max Error:   %.10g\n', max_error);
        fprintf('  Mean Error:  %.10g\n', mean_error);
        fprintf('  Std Error:   %.10g\n', std_error);
        fprintf('  RMSE:        %.10g\n', rmse);
        
        % Interpretation
        if max_error < 1e-6
            fprintf('  ✅ Identical (numerical precision < 1e-6)\n');
        elseif max_error < 1e-3
            fprintf('  ✅ Very similar (error < 1e-3, floating point variance)\n');
        elseif max_error < 0.01
            fprintf('  ⚠️  Similar (error < 0.01, minor environmental difference)\n');
        else
            fprintf('  ❌ Significantly different (error > 0.01, major issue)\n');
        end
        fprintf('\n');
    end
    
    % Summary statistics
    fprintf('================================================================================\n');
    fprintf('DIAGNOSIS\n');
    fprintf('================================================================================\n\n');
    
    % Count error levels
    identical_count = 0;
    similar_count = 0;
    minor_diff_count = 0;
    major_diff_count = 0;
    
    field_names = fieldnames(comparison_results);
    for i = 1:length(field_names)
        result = comparison_results.(field_names{i});
        if result.max_error < 1e-6
            identical_count = identical_count + 1;
        elseif result.max_error < 1e-3
            similar_count = similar_count + 1;
        elseif result.max_error < 0.01
            minor_diff_count = minor_diff_count + 1;
        else
            major_diff_count = major_diff_count + 1;
        end
    end
    
    fprintf('Results Summary:\n');
    fprintf('  Identical (< 1e-6):      %d columns\n', identical_count);
    fprintf('  Similar (< 1e-3):        %d columns\n', similar_count);
    fprintf('  Minor difference (< 0.01): %d columns\n', minor_diff_count);
    fprintf('  Major difference (>= 0.01): %d columns\n', major_diff_count);
    fprintf('\n');
    
    % Overall diagnosis
    if major_diff_count == 0
        fprintf('✅ DIAGNOSIS: Environments are essentially equivalent\n');
        fprintf('   The small differences are likely due to floating-point arithmetic variance.\n');
        fprintf('   Both environments should produce acceptable results.\n');
    elseif major_diff_count <= 3
        fprintf('⚠️  DIAGNOSIS: Minor environmental differences detected\n');
        fprintf('   Some state variables show differences > 0.01 units.\n');
        fprintf('   Likely causes (in order of probability):\n');
        fprintf('   1. MEX binary was not rebuilt for this environment\n');
        fprintf('   2. config_params.m differs (static_time, dt, noise parameters)\n');
        fprintf('   3. Sensor data has different precision/formatting\n');
        fprintf('   4. Initial conditions differ (GPS origin, initial state)\n');
        fprintf('\n');
        fprintf('   Recommendations:\n');
        fprintf('   - cd kalman/cpp/build && build_mex() in problematic environment\n');
        fprintf('   - Verify config_params.m is identical in both environments\n');
        fprintf('   - Check sensor_data.csv generation with same seed\n');
    else
        fprintf('❌ DIAGNOSIS: Significant environmental differences detected\n');
        fprintf('   Multiple state variables show major errors.\n');
        fprintf('   Likely causes:\n');
        fprintf('   1. MEX binary is for wrong architecture (32-bit vs 64-bit)\n');
        fprintf('   2. MATLAB version mismatch (>5 year gap)\n');
        fprintf('   3. config_params.m values are significantly different\n');
        fprintf('   4. Sensor data type conversion is incorrect\n');
        fprintf('   5. GPS origin or initial state mismatch\n');
        fprintf('\n');
        fprintf('   Action required:\n');
        fprintf('   - Run diagnose_environment() in both environments\n');
        fprintf('   - Rebuild MEX: cd kalman/cpp/build && build_mex()\n');
        fprintf('   - Synchronize config_params.m\n');
        fprintf('   - Verify sensor data generation (same seed)\n');
    end
    
    fprintf('\n================================================================================\n');
    fprintf('DETAILED ERROR ANALYSIS\n');
    fprintf('================================================================================\n\n');
    
    % Find columns with major differences
    major_diff_cols = {};
    for i = 1:length(field_names)
        result = comparison_results.(field_names{i});
        if result.max_error >= 0.01
            major_diff_cols{end+1} = field_names{i};
        end
    end
    
    if ~isempty(major_diff_cols)
        fprintf('Columns with major differences:\n');
        for i = 1:length(major_diff_cols)
            col = major_diff_cols{i};
            result = comparison_results.(col);
            fprintf('  %s: max_error=%.6f\n', col, result.max_error);
        end
        fprintf('\n');
        
        % Pattern analysis
        fprintf('Pattern Analysis:\n');
        has_position_error = any(ismember(major_diff_cols, {'p_east', 'p_north', 'p_up'}));
        has_velocity_error = any(ismember(major_diff_cols, {'v_east', 'v_north', 'v_up'}));
        has_attitude_error = any(ismember(major_diff_cols, {'roll', 'pitch', 'yaw'}));
        has_bias_error = any(ismember(major_diff_cols, {'ba_x', 'ba_y', 'ba_z', 'bg_x', 'bg_y', 'bg_z'}));
        
        if has_position_error && ~has_attitude_error
            fprintf('  → Position errors without attitude errors\n');
            fprintf('    Likely: GPS initialization or coordinate frame differs\n');
        elseif has_attitude_error && ~has_position_error
            fprintf('  → Attitude errors without position errors\n');
            fprintf('    Likely: Gyroscope or magnetic field calibration differs\n');
        elseif has_attitude_error && has_position_error
            fprintf('  → Both attitude and position errors\n');
            fprintf('    Likely: Initialization parameters (static_time, noise) differ\n');
        elseif has_bias_error
            fprintf('  → Bias estimation errors\n');
            fprintf('    Likely: Sensor noise characteristics differ\n');
        end
    else
        fprintf('No columns with major differences detected.\n');
    end
    
    fprintf('\n================================================================================\n');
    
end


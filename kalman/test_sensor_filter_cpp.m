% test_sensor_filter_cpp.m
try
    addpath('Common/Sensor');
    addpath('cpp/bin');

    fprintf('Testing SensorFilter C++ implementation...\n');

    % Check if MEX exists
    if exist('mex_sensor_filter', 'file') ~= 3
        fprintf('Error: mex_sensor_filter not found in cpp/bin.\n');
        fprintf('Attempting to build...\n');
        current_dir = pwd;
        cd('cpp/build');
        try
            build_sensor_filter();
            cd(current_dir);
        catch ME
            cd(current_dir);
            error('Build failed: %s', ME.message);
        end
    end

    % Initialize
    matlab_filter = SensorFilterLib();
    % Reset C++ filter
    mex_sensor_filter('reset');

    % Generate data
    N = 100;
    t = linspace(0, 1, N);
    accel_true = [sin(2*pi*t); cos(2*pi*t); zeros(1,N)];
    rng(42); % Fixed seed
    noise = 0.1 * randn(3, N);
    accel_meas = accel_true + noise;

    % Add outliers
    outlier_indices = [20, 50, 80];
    accel_meas(:, outlier_indices) = accel_meas(:, outlier_indices) + 5.0;

    % Run filters
    matlab_out = zeros(3, N);
    cpp_out = zeros(3, N);
    matlab_outlier = false(1, N);
    cpp_outlier = false(1, N);

    fprintf('Running Accel loop...\n');
    for k = 1:N
        meas = accel_meas(:, k);
        
        % MATLAB
        [m_filt, m_is_out] = matlab_filter.filter_accel(meas);
        matlab_out(:, k) = m_filt;
        matlab_outlier(k) = m_is_out;
        
        % C++
        [c_filt, c_is_out] = mex_sensor_filter('accel', meas);
        cpp_out(:, k) = c_filt;
        cpp_outlier(k) = c_is_out;
    end

    % Compare Accel
    diff = matlab_out - cpp_out;
    max_diff = max(vecnorm(diff));
    fprintf('Accel Max difference: %e\n', max_diff);

    if max_diff < 1e-5
        fprintf('SUCCESS: Accel implementation matches MATLAB.\n');
    else
        fprintf('FAILURE: Accel mismatch detected.\n');
    end

    % Check outliers
    outlier_diff = sum(matlab_outlier ~= cpp_outlier);
    fprintf('Accel Outlier mismatch count: %d\n', outlier_diff);
    
    %% Gyro Test
    fprintf('\nRunning Gyro loop...\n');
    gyro_meas = accel_meas; % Reuse random data
    dt = 0.01;
    cutoff = 20;
    
    matlab_gyro = zeros(3, N);
    cpp_gyro = zeros(3, N);
    
    % Reset
    matlab_filter = SensorFilterLib();
    mex_sensor_filter('reset');
    
    for k = 1:N
        meas = gyro_meas(:, k);
        
        % MATLAB
        m_filt = matlab_filter.filter_gyro(meas, dt, cutoff);
        matlab_gyro(:, k) = m_filt;
        
        % C++
        c_filt = mex_sensor_filter('gyro', meas, dt, cutoff);
        cpp_gyro(:, k) = c_filt;
    end
    
    diff_gyro = matlab_gyro - cpp_gyro;
    max_diff_gyro = max(vecnorm(diff_gyro));
    fprintf('Gyro Max difference: %e\n', max_diff_gyro);
    if max_diff_gyro < 1e-5
        fprintf('SUCCESS: Gyro implementation matches MATLAB.\n');
    else
        fprintf('FAILURE: Gyro mismatch detected.\n');
    end
    
    %% Mag Test
    fprintf('\nRunning Mag loop...\n');
    mag_meas = accel_meas; % Reuse random data
    % Add large outliers for Mag (threshold is 5.0 * 5.0 = 25.0)
    mag_meas(:, 30) = mag_meas(:, 30) + 30.0;
    
    matlab_mag = zeros(3, N);
    cpp_mag = zeros(3, N);
    matlab_mag_out = false(1, N);
    cpp_mag_out = false(1, N);
    
    % Reset
    matlab_filter = SensorFilterLib();
    mex_sensor_filter('reset');
    
    for k = 1:N
        meas = mag_meas(:, k);
        
        % MATLAB
        [m_filt, m_is_out] = matlab_filter.filter_mag(meas);
        matlab_mag(:, k) = m_filt;
        matlab_mag_out(k) = m_is_out;
        
        % C++
        [c_filt, c_is_out] = mex_sensor_filter('mag', meas);
        cpp_mag(:, k) = c_filt;
        cpp_mag_out(k) = c_is_out;
    end
    
    diff_mag = matlab_mag - cpp_mag;
    max_diff_mag = max(vecnorm(diff_mag));
    fprintf('Mag Max difference: %e\n', max_diff_mag);
    if max_diff_mag < 1e-5
        fprintf('SUCCESS: Mag implementation matches MATLAB.\n');
    else
        fprintf('FAILURE: Mag mismatch detected.\n');
    end
    fprintf('Mag Outlier mismatch count: %d\n', sum(matlab_mag_out ~= cpp_mag_out));
    
    %% GPS Test
    fprintf('\nRunning GPS loop...\n');
    gps_meas = cumsum(randn(3, N), 2); % Random walk
    dt = 0.1;
    
    matlab_gps_p = zeros(3, N);
    matlab_gps_v = zeros(3, N);
    cpp_gps_p = zeros(3, N);
    cpp_gps_v = zeros(3, N);
    
    % Reset
    matlab_filter = SensorFilterLib();
    mex_sensor_filter('reset');
    
    for k = 1:N
        meas = gps_meas(:, k);
        
        % MATLAB
        [m_p, m_v] = matlab_filter.filter_gps(meas, dt);
        matlab_gps_p(:, k) = m_p;
        matlab_gps_v(:, k) = m_v;
        
        % C++
        [c_p, c_v] = mex_sensor_filter('gps', meas, dt);
        cpp_gps_p(:, k) = c_p;
        cpp_gps_v(:, k) = c_v;
    end
    
    diff_gps_p = matlab_gps_p - cpp_gps_p;
    diff_gps_v = matlab_gps_v - cpp_gps_v;
    max_diff_gps = max([vecnorm(diff_gps_p), vecnorm(diff_gps_v)]);
    fprintf('GPS Max difference: %e\n', max_diff_gps);
    if max_diff_gps < 1e-5
        fprintf('SUCCESS: GPS implementation matches MATLAB.\n');
    else
        fprintf('FAILURE: GPS mismatch detected.\n');
    end
    
    %% Baro Test
    fprintf('\nRunning Baro loop...\n');
    baro_meas = cumsum(randn(1, N));
    
    matlab_baro = zeros(1, N);
    cpp_baro = zeros(1, N);
    
    % Reset
    matlab_filter = SensorFilterLib();
    mex_sensor_filter('reset');
    
    for k = 1:N
        meas = baro_meas(k);
        
        % MATLAB
        m_filt = matlab_filter.filter_baro(meas);
        matlab_baro(k) = m_filt;
        
        % C++
        c_filt = mex_sensor_filter('baro', meas);
        cpp_baro(k) = c_filt;
    end
    
    diff_baro = matlab_baro - cpp_baro;
    max_diff_baro = max(abs(diff_baro));
    fprintf('Baro Max difference: %e\n', max_diff_baro);
    if max_diff_baro < 1e-5
        fprintf('SUCCESS: Baro implementation matches MATLAB.\n');
    else
        fprintf('FAILURE: Baro mismatch detected.\n');
    end
    
catch ME
    fprintf('Error: %s\n', ME.message);
    fprintf('Stack trace:\n');
    for k = 1:length(ME.stack)
        fprintf('  %s:%d\n', ME.stack(k).name, ME.stack(k).line);
    end
end

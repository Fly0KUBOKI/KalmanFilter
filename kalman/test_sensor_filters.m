%% Test unified sensor filter system
addpath(genpath(pwd));

fprintf('=== Unified Sensor Filter System Test ===\n\n');

% Test 1: Accelerometer Filter
fprintf('1. Accelerometer Filter Test\n');
try
    accel_filter = SensorFilter.createAccelFilter();
    a_test = [0.1; 0.05; 9.8];
    [a_out, is_outlier, info] = accel_filter.apply(a_test);
    fprintf('   ✓ Accel filter created and applied successfully\n');
    fprintf('     Input: [%.2f, %.2f, %.2f], Output: [%.2f, %.2f, %.2f], Outlier: %d\n', ...
        a_test(1), a_test(2), a_test(3), a_out(1), a_out(2), a_out(3), is_outlier);
catch e
    fprintf('   ✗ Error: %s\n', e.message);
end

% Test 2: Gyroscope Filter
fprintf('\n2. Gyroscope Filter Test\n');
try
    gyro_filter = SensorFilter.createGyroFilter();
    w_test = [0.01; 0.02; 0.015];  % rad/s
    [w_out, is_outlier, info] = gyro_filter.apply(w_test);
    fprintf('   ✓ Gyro filter created and applied successfully\n');
    fprintf('     Input: [%.4f, %.4f, %.4f], Output: [%.4f, %.4f, %.4f], Outlier: %d\n', ...
        w_test(1), w_test(2), w_test(3), w_out(1), w_out(2), w_out(3), is_outlier);
catch e
    fprintf('   ✗ Error: %s\n', e.message);
end

% Test 3: Magnetometer Filter
fprintf('\n3. Magnetometer Filter Test\n');
try
    mag_filter = SensorFilter.createMagFilter();
    m_test = [20; 30; 10];  % nT
    [m_out, is_outlier, info] = mag_filter.apply(m_test);
    fprintf('   ✓ Mag filter created and applied successfully\n');
    fprintf('     Input: [%.1f, %.1f, %.1f], Output norm: %.1f, Outlier: %d\n', ...
        m_test(1), m_test(2), m_test(3), norm(m_out), is_outlier);
catch e
    fprintf('   ✗ Error: %s\n', e.message);
end

% Test 4: GPS Filter
fprintf('\n4. GPS Filter Test\n');
try
    gps_filter = SensorFilter.createGPSFilter();
    pos_test = [100.5; 50.2; -10.3];  % m
    [pos_out, is_outlier, info] = gps_filter.apply(pos_test);
    fprintf('   ✓ GPS filter created and applied successfully\n');
    fprintf('     Input: [%.1f, %.1f, %.1f], Output: [%.1f, %.1f, %.1f], Outlier: %d\n', ...
        pos_test(1), pos_test(2), pos_test(3), pos_out(1), pos_out(2), pos_out(3), is_outlier);
catch e
    fprintf('   ✗ Error: %s\n', e.message);
end

% Test 5: Barometer Filter
fprintf('\n5. Barometer Filter Test\n');
try
    baro_filter = SensorFilter.createBaroFilter();
    pressure = 101325;  % Pa
    [alt_out, is_outlier, info] = baro_filter.apply(pressure);
    fprintf('   ✓ Baro filter created and applied successfully\n');
    fprintf('     Input pressure: %.0f Pa, Output altitude: %.1f m, Outlier: %d\n', ...
        pressure, alt_out, is_outlier);
catch e
    fprintf('   ✗ Error: %s\n', e.message);
end

fprintf('\n=== All tests completed ===\n');

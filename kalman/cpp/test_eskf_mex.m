function test_eskf_mex()
% TEST_ESKF_MEX  ESKF MEX functions test
%
% Tests all ESKF MEX functions with sample data

fprintf('=== ESKF MEX Function Tests ===\n\n');

% Check if MEX is available
if exist('mex_eskf_core', 'file') ~= 3
    error('mex_eskf_core not found. Run build_mex first.');
end

fprintf('✓ mex_eskf_core found\n\n');

% Test parameters
dt = 0.01;
g = [0; 0; -9.81];

% Initial state
p = zeros(3,1);
v = zeros(3,1);
q = [1; 0; 0; 0];  % Identity quaternion
ba = zeros(3,1);
bg = zeros(3,1);
P = eye(15) * 0.01;

% Sensor measurements
a_meas = [0; 0; 9.81];  % Stationary, measuring gravity
w_meas = zeros(3,1);    % No rotation
m_meas = [0; 50; 0];    % Magnetic field (North)
m_world = [0; 50; 0];

% Thresholds
gyro_thr = ones(3,1) * 0.01;
accel_thr = ones(3,1) * 0.01;

% Noise covariances
R_mag = eye(3) * 0.1;
R_gps = eye(3) * 1.0;
R_baro = 1.0;

% GPS data
gps_pos = [35.0; 139.0; 100.0];  % [lat, lon, alt]
gps_origin = [35.0; 139.0; 100.0];

% Pressure
pressure = 101325;  % Sea level

%% Test 1: integrate_nominal
fprintf('Test 1: integrate_nominal... ');
try
    [p_new, v_new, q_new, ba_new, bg_new] = mex_eskf_core('integrate_nominal', ...
        p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_thr, accel_thr);
    
    % Check outputs
    assert(all(size(p_new) == [3,1]), 'Invalid p size');
    assert(all(size(v_new) == [3,1]), 'Invalid v size');
    assert(all(size(q_new) == [4,1]), 'Invalid q size');
    assert(all(size(ba_new) == [3,1]), 'Invalid ba size');
    assert(all(size(bg_new) == [3,1]), 'Invalid bg size');
    
    % Check quaternion normalization
    q_norm = norm(q_new);
    assert(abs(q_norm - 1.0) < 1e-6, 'Quaternion not normalized');
    
    fprintf('✓ PASS\n');
catch ME
    fprintf('✗ FAIL: %s\n', ME.message);
    rethrow(ME);
end

%% Test 2: update_accel
fprintf('Test 2: update_accel... ');
try
    q_new = mex_eskf_core('update_accel', q, a_meas, 1.0);
    
    % Check output
    assert(all(size(q_new) == [4,1]), 'Invalid q size');
    
    % Check quaternion normalization
    q_norm = norm(q_new);
    assert(abs(q_norm - 1.0) < 1e-6, 'Quaternion not normalized');
    
    fprintf('✓ PASS\n');
catch ME
    fprintf('✗ FAIL: %s\n', ME.message);
    rethrow(ME);
end

%% Test 3: update_mag
fprintf('Test 3: update_mag... ');
try
    [q_new, P_new, K, dx] = mex_eskf_core('update_mag', q, P, m_meas, m_world, R_mag);
    
    % Check outputs
    assert(all(size(q_new) == [4,1]), 'Invalid q size');
    assert(all(size(P_new) == [15,15]), 'Invalid P size');
    assert(size(K,2) == 3, 'Invalid K size');
    assert(size(dx,1) >= 9, 'Invalid dx size');
    
    % Check quaternion normalization
    q_norm = norm(q_new);
    assert(abs(q_norm - 1.0) < 1e-6, 'Quaternion not normalized');
    
    % Check P symmetry
    P_sym_err = norm(P_new - P_new', 'fro');
    assert(P_sym_err < 1e-10, 'P not symmetric');
    
    fprintf('✓ PASS\n');
catch ME
    fprintf('✗ FAIL: %s\n', ME.message);
    rethrow(ME);
end

%% Test 4: update_gps
fprintf('Test 4: update_gps... ');
try
    [p_new, v_new, P_new, K, dx] = mex_eskf_core('update_gps', ...
        p, v, P, gps_pos, gps_origin, R_gps);
    
    % Check outputs
    assert(all(size(p_new) == [3,1]), 'Invalid p size');
    assert(all(size(v_new) == [3,1]), 'Invalid v size');
    assert(all(size(P_new) == [15,15]), 'Invalid P size');
    assert(size(K,2) == 3, 'Invalid K size');
    assert(size(dx,1) >= 6, 'Invalid dx size');
    
    % Check P symmetry
    P_sym_err = norm(P_new - P_new', 'fro');
    assert(P_sym_err < 1e-10, 'P not symmetric');
    
    fprintf('✓ PASS\n');
catch ME
    fprintf('✗ FAIL: %s\n', ME.message);
    rethrow(ME);
end

%% Test 5: update_baro
fprintf('Test 5: update_baro... ');
try
    [p_new, P_new, K, dx] = mex_eskf_core('update_baro', ...
        p, P, pressure, gps_origin, R_baro);
    
    % Check outputs
    assert(all(size(p_new) == [3,1]), 'Invalid p size');
    assert(all(size(P_new) == [15,15]), 'Invalid P size');
    assert(size(K,2) == 1, 'Invalid K size');
    assert(size(dx,1) >= 3, 'Invalid dx size');
    
    % Check P symmetry
    P_sym_err = norm(P_new - P_new', 'fro');
    assert(P_sym_err < 1e-10, 'P not symmetric');
    
    fprintf('✓ PASS\n');
catch ME
    fprintf('✗ FAIL: %s\n', ME.message);
    rethrow(ME);
end

%% Performance test
fprintf('\nPerformance test (1000 iterations)...\n');

n_iter = 1000;

% Test integrate_nominal
tic;
for i = 1:n_iter
    [p, v, q, ba, bg] = mex_eskf_core('integrate_nominal', ...
        p, v, q, ba, bg, a_meas, w_meas, dt, g, gyro_thr, accel_thr);
end
t_integrate = toc;
fprintf('  integrate_nominal: %.3f ms/iter\n', t_integrate / n_iter * 1000);

% Test update_mag
tic;
for i = 1:n_iter
    [q, P, ~, ~] = mex_eskf_core('update_mag', q, P, m_meas, m_world, R_mag);
end
t_mag = toc;
fprintf('  update_mag: %.3f ms/iter\n', t_mag / n_iter * 1000);

fprintf('\n=== All Tests Passed! ===\n');
fprintf('\nMEX acceleration is working correctly.\n');
fprintf('You can now run run_simulation() to use MEX-accelerated ESKF.\n');

end

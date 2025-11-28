%% Test script for eskf_math library
% Tests the new stateless computation library

fprintf('=== Testing eskf_math library ===\n\n');

% Add paths
addpath('Common/Math');
addpath('cpp/bin');

% Test 1: Quaternion integration
fprintf('[Test 1] Quaternion integration\n');
q = [1; 0; 0; 0];  % Identity quaternion
w = [0.1; 0.2; 0.3];  % Angular velocity
dt = 0.01;
q_new = eskf_math('quaternion_integration', q, w, dt);
fprintf('  Input q: [%.4f, %.4f, %.4f, %.4f]\n', q);
fprintf('  Output q: [%.4f, %.4f, %.4f, %.4f]\n', q_new);
fprintf('  Norm: %.6f (should be 1.0)\n\n', norm(q_new));

% Test 2: Accel to quaternion
fprintf('[Test 2] Accel to quaternion\n');
a_meas = [0; 0; -9.81];  % Gravity pointing down
scale = 1.0;
q_rp = eskf_math('accel_to_quaternion', a_meas, scale);
fprintf('  Input a: [%.2f, %.2f, %.2f]\n', a_meas);
fprintf('  Output q: [%.4f, %.4f, %.4f, %.4f]\n', q_rp);
fprintf('  Norm: %.6f\n\n', norm(q_rp));

% Test 3: PV integration
fprintf('[Test 3] Position/Velocity integration\n');
p = [0; 0; 0];
v = [1; 0; 0];
a_world = [0; 0; 0];
g = [0; 0; 9.81];
dt = 0.1;
prev_a = [0; 0; 0];
prev_v = [1; 0; 0];
use_ab2 = 0;  % Euler
max_accel = 100;
max_vel = 100;

[p_new, v_new] = eskf_math('pv_integration', p, v, a_world, g, dt, prev_a, prev_v, use_ab2, max_accel, max_vel);
fprintf('  p: [%.4f, %.4f, %.4f] -> [%.4f, %.4f, %.4f]\n', p, p_new);
fprintf('  v: [%.4f, %.4f, %.4f] -> [%.4f, %.4f, %.4f]\n\n', v, v_new);

% Test 4: F matrix computation
fprintf('[Test 4] State transition matrix F\n');
q = [1; 0; 0; 0];
a_meas = [0; 0; -9.81];
ba = [0; 0; 0];
w_meas = [0; 0; 0];
bg = [0; 0; 0];
dt = 0.01;

F = eskf_math('compute_F_matrix', q, a_meas, ba, w_meas, bg, dt);
fprintf('  F size: %dx%d\n', size(F));
fprintf('  F(1,4) should be dt: %.4f (actual: %.4f)\n', dt, F(1,4));
fprintf('  F diagonal (first 5): [%.4f, %.4f, %.4f, %.4f, %.4f]\n\n', diag(F(1:5,1:5)));

% Test 5: Covariance prediction
fprintf('[Test 5] Covariance prediction\n');
P = eye(15) * 0.01;
Q = eye(15) * 0.0001;

P_new = eskf_math('covariance_prediction', P, F, Q);
fprintf('  P trace: %.6f -> %.6f\n', trace(P), trace(P_new));
fprintf('  P_new(1,1): %.6f\n\n', P_new(1,1));

% Test 6: Error state injection
fprintf('[Test 6] Error state injection\n');
p = [1; 2; 3];
v = [0.5; 0; 0];
q = [1; 0; 0; 0];
ba = [0.01; 0.02; 0.03];
bg = [0.001; 0.002; 0.003];
dx = [0.1; 0.1; 0.1; 0.05; 0; 0; deg2rad(1); deg2rad(2); deg2rad(3); 0; 0; 0; 0; 0; 0];

[p_new, v_new, q_new, ba_new, bg_new] = eskf_math('inject_error_state', p, v, q, ba, bg, dx);
fprintf('  p: [%.4f, %.4f, %.4f] -> [%.4f, %.4f, %.4f]\n', p, p_new);
fprintf('  q norm: %.6f\n\n', norm(q_new));

% Test 7: Kalman update
fprintf('[Test 7] Kalman update\n');
x = zeros(15, 1);
P = eye(15) * 0.1;
y = [0.5; 0.3; 0.1];  % Innovation
H = zeros(3, 15);
H(1:3, 1:3) = eye(3);  % Observe position
R = eye(3) * 0.01;

[x_new, P_new, K, S] = eskf_math('kalman_update', x, P, y, H, R);
fprintf('  x(1:3): [%.4f, %.4f, %.4f] -> [%.4f, %.4f, %.4f]\n', x(1:3), x_new(1:3));
fprintf('  P trace: %.6f -> %.6f\n', trace(P), trace(P_new));
fprintf('  K size: %dx%d\n\n', size(K));

% Test 8: Magnetic observation
fprintf('[Test 8] Magnetic field observation\n');
q = [1; 0; 0; 0];
m_world = [1; 0; 0];  % Magnetic north in world frame

m_body = eskf_math('mag_observation_prediction', q, m_world);
fprintf('  m_world: [%.4f, %.4f, %.4f]\n', m_world);
fprintf('  m_body:  [%.4f, %.4f, %.4f]\n\n', m_body);

% Test 9: GPS to local
fprintf('[Test 9] GPS to local coordinates\n');
origin = [35.0; 139.0; 0];  % Tokyo
gps_pos = [35.001; 139.001; 100];

pos_local = eskf_math('gps_to_local', gps_pos, origin);
fprintf('  Origin: [%.3f, %.3f, %.1f]\n', origin);
fprintf('  GPS:    [%.3f, %.3f, %.1f]\n', gps_pos);
fprintf('  Local:  [%.1f, %.1f, %.1f] (N, E, D)\n\n', pos_local);

% Test 10: Pressure to altitude
fprintf('[Test 10] Pressure to altitude\n');
pressure = 101325;  % Sea level
altitude = eskf_math('pressure_to_altitude', pressure);
fprintf('  Pressure: %.1f Pa\n', pressure);
fprintf('  Altitude: %.2f m (should be ~0)\n\n', altitude);

pressure = 89875;  % ~1000m
altitude = eskf_math('pressure_to_altitude', pressure);
fprintf('  Pressure: %.1f Pa\n', pressure);
fprintf('  Altitude: %.2f m (should be ~1000)\n\n', altitude);

fprintf('=== All tests completed ===\n');

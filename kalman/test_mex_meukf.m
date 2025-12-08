% MEX_MEUKF テストスクリプト
fprintf('Testing mex_meukf_step_v2...\n');

% 初期状態
state = struct();
state.p = [0;0;0];
state.v = [0;0;0];
state.q = [1;0;0;0];
state.ba = [0;0;0];
state.bg = [0;0;0];
state.P = eye(15) * 0.01;

% センサーデータ
sensor = struct();
sensor.dt = 0.0025;
sensor.accel = [0; 0; -9.81];
sensor.gyro = [0; 0; 0];
sensor.mag = [0; 50; 0];
sensor.gps_pos = [0; 0; 0];
sensor.update_accel = 0;
sensor.update_mag = 0;
sensor.update_gps = 0;
sensor.update_gyro = 0;

% パラメータ
params = struct();
params.g = [0; 0; -9.81];
params.mag_ref = [0; 1; 0];
params.noise_accel = [1; 1; 1];
params.noise_gyro = [1; 1; 1] * 1e-4;
params.noise_ba = [1; 1; 1] * 1e-4;
params.noise_bg = [1; 1; 1] * 1e-5;
params.noise_mag = [1; 1; 1] * 1e-3;
params.noise_gps = [1; 1; 1];
params.alpha = 0.1;
params.beta = 2.0;
params.kappa = 0.0;

try
    % Predict test
    fprintf('  Predict test...');
    [new_state, debug] = mex_meukf_step_v2(state, sensor, params);
    fprintf(' OK\n');
    fprintf('    Position: [%.6f, %.6f, %.6f]\n', new_state.p);
    fprintf('    Velocity: [%.6f, %.6f, %.6f]\n', new_state.v);
    
    % Accel update test
    fprintf('  Accel update test...');
    sensor.update_accel = 1;
    [new_state2, debug2] = mex_meukf_step_v2(new_state, sensor, params);
    fprintf(' OK\n');
    
    % Mag update test
    fprintf('  Mag update test...');
    sensor.update_accel = 0;
    sensor.update_mag = 1;
    [new_state3, debug3] = mex_meukf_step_v2(new_state2, sensor, params);
    fprintf(' OK\n');
    
    % GPS update test
    fprintf('  GPS update test...');
    sensor.update_mag = 0;
    sensor.update_gps = 1;
    [new_state4, debug4] = mex_meukf_step_v2(new_state3, sensor, params);
    fprintf(' OK\n');
    
    fprintf('\nAll tests passed!\n');
catch ME
    fprintf(' FAILED\n');
    fprintf('Error: %s\n', ME.message);
    disp(getReport(ME));
end

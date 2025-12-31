function test_meukf_integration()
% Simple numerical check between mex_meukf_step and mex_run_eskf('meukf_step')
% Prepare a dummy state/params matching expected structure used in mex_meukf_step
state.p = zeros(3,1);
state.v = zeros(3,1);
state.q = [1;0;0;0];
state.ba = zeros(3,1);
state.bg = zeros(3,1);
state.P = eye(15);

sensor.accel = zeros(3,1);
sensor.gyro = zeros(3,1);
sensor.mag = zeros(3,1);
sensor.gps_pos = zeros(3,1);
sensor.alt_baro = 0;
sensor.prev_mag = zeros(3,1);
sensor.prev_gps_pos = zeros(3,1);
sensor.prev_baro_alt = 0;
sensor.update_accel = 0;
sensor.update_gyro = 0;
sensor.update_mag = 0;
sensor.update_gps = 0;
sensor.update_baro = 0;
sensor.update_zupt = 0;
sensor.dt = 0.01;

params.g = [0;0;-9.81];
params.mag_ref = [1;0;0];
params.noise_accel = [0.01;0.01;0.01];
params.noise_gyro = [0.001;0.001;0.001];
params.noise_ba = [1e-6;1e-6;1e-6];
params.noise_bg = [1e-6;1e-6;1e-6];
params.noise_mag = [1e-4;1e-4;1e-4];
params.noise_gps = [1;1;1];
params.noise_baro = 1;
params.noise_zupt = [1;1;1];
params.alpha = 1; params.beta = 0.1; params.kappa = 0;

% Run reference
try
    [state_ref, P_ref] = mex_meukf_step(state, state.P, sensor, params);
catch ME
    disp('mex_meukf_step failed:'); disp(ME.message); return;
end

% Run integrated
try
    out = mex_run_eskf('meukf_step', state, sensor, params);
catch ME
    disp('mex_run_eskf meukf_step failed:'); disp(ME.message); return;
end

% out is a struct; try to extract P
if isstruct(out)
    state_int = out;
    P_int = out.P;
else
    state_int = out{1};
    P_int = out{2};
end

% Compare
pos_diff = norm(state_ref.p - state_int.p);
P_diff = norm(P_ref - P_int, 'fro');

fprintf('pos diff: %g, P fro diff: %g\n', pos_diff, P_diff);
if pos_diff < 1e-6 && P_diff < 1e-6
    disp('✓ MEUKF 統合成功')
else
    disp('✗ MEUKF 統合差あり')
end
end

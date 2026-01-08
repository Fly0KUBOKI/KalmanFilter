% Run accel update comparison between original and UKF-backed implementation
clear mex;
% Prepare prev_state (single)
prev_state.p = single([0,0,0]);
prev_state.v = single([0,0,0]);
prev_state.q = single([1,0,0,0]);
prev_state.ba = single([0,0,0]);
prev_state.bg = single([0,0,0]);
prev_state.P = single(0.01 * eye(15));

% Sensor
sensor.accel = single([0,0,-9.81]);
sensor.gyro = single([0,0,0]);
sensor.mag = single([0,0,0]);
sensor.update_accel = true;
sensor.update_gyro = false;
sensor.update_mag = false;
sensor.update_gps = false;
sensor.update_baro = false;
sensor.update_zupt = false;
sensor.dt = single(0.01);

% Params
params.g = single([0,0,-9.81]);
params.mag_ref = single([1,0,0]);
params.noise_accel = single([0.01,0.01,0.01]);
params.noise_gyro = single([0.01,0.01,0.01]);
params.noise_ba = single([1e-6,1e-6,1e-6]);
params.noise_bg = single([1e-6,1e-6,1e-6]);
params.noise_mag = single([0.01,0.01,0.01]);
params.noise_gps = double([1,1,1]); % GPS double in spec
params.noise_baro = single(0.5);
params.noise_zupt = single([0.01,0.01,0.01]);
params.alpha = single(1e-3);
params.beta = single(2.0);
params.kappa = single(0.0);

% Ensure mex folder is on MATLAB path
addpath('../bin');

% Run original (ensure env var not set)
unsetenv('MEUKF_USE_UKF');
[out1, dbg1, dbg_out1] = mex_run_eskf('meukf_step', prev_state, sensor, params);

% Run UKF-backed
setenv('MEUKF_USE_UKF','1');
[out2, dbg2, dbg_out2] = mex_run_eskf('meukf_step', prev_state, sensor, params);

% Compare
q1 = out1.q; q2 = out2.q;
P1 = out1.P; P2 = out2.P;

fprintf('Original q: %f %f %f %f\n', q1(1), q1(2), q1(3), q1(4));
fprintf('UKF q:      %f %f %f %f\n', q2(1), q2(2), q2(3), q2(4));

% Compare small-angle between quaternions
% Compute quaternion difference qd = q2 * conj(q1) (manual, no Aerospace Toolbox)
q1c = [q1(1), -q1(2), -q1(3), -q1(4)];
% q2 * q1c
w1=q2(1); x1=q2(2); y1=q2(3); z1=q2(4);
w2=q1c(1); x2=q1c(2); y2=q1c(3); z2=q1c(4);
qd = [w1*w2 - x1*x2 - y1*y2 - z1*z2, ...
	w1*x2 + x1*w2 + y1*z2 - z1*y2, ...
	w1*y2 - x1*z2 + y1*w2 + z1*x2, ...
	w1*z2 + x1*y2 - y1*x2 + z1*w2];
fprintf('quat diff (q2 * conj(q1)): %f %f %f %f\n', qd(1), qd(2), qd(3), qd(4));

% Print P attitude block
P1_mat = reshape(P1,15,15);
P2_mat = reshape(P2,15,15);
fprintf('Original P_att:\n'); disp(P1_mat(7:9,7:9));
fprintf('UKF P_att:\n'); disp(P2_mat(7:9,7:9));

% Save differences and debug outputs (use fallback if write denied)
try
	save('accel_update_compare.mat','out1','out2','dbg_out1','dbg_out2');
	fprintf('Saved accel_update_compare.mat in current folder\n');
catch ME
	alt_fn = fullfile(tempdir, 'accel_update_compare.mat');
	save(alt_fn,'out1','out2','dbg_out1','dbg_out2');
	fprintf('Warning: could not save in current folder, saved to %s\n', alt_fn);
end
fprintf('Saved accel_update_compare.mat\n');

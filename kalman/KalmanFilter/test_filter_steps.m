function test_filter_steps()
% Smoke test that runs KF, EKF, UKF one step each with the same inputs
root = fileparts(mfilename('fullpath'));
addpath(root);
commonDir = fullfile(root, 'Common Calculations');
if exist(commonDir, 'dir') && ~contains(path, commonDir)
    addpath(commonDir);
end

% initial state (10x1) and covariance
x0 = zeros(10,1);
P0 = eye(10) * 0.1;

% simple params
params.dt = 0.1;
params.kf.process_noise_accel = 0.1;
params.noise.pos = 0.5;
params.noise.vel = 0.2;
params.noise.heading = 0.05;
params.noise.accel3 = [0.1,0.1,0.1];
params.noise.mag3 = [0.01,0.01,0.01];
params.kf.adaptive_R_enabled = false; % start disabled for deterministic results
params.kf.ema_alpha = 0.05;
params.sensors.mag_field = [1;0;0];
params.kf.debug = true;

% measurement (gps + vel + heading)
meas.gps = [1.0; 0.5];
meas.vel = [0.1; -0.05];
meas.heading = [cos(0.1); sin(0.1)];

fprintf('--- Running KF step ---\n');
try
    [x_pred_kf, P_pred_kf, x_upd_kf, P_upd_kf, y_kf, S_kf, K_kf, params_kf] = kf_filter_step(x0, P0, meas, params);
    fprintf('KF x_upd norm = %g\n', norm(x_upd_kf));
catch ME
    fprintf('KF failed: %s\n', ME.message);
end

fprintf('\n--- Running EKF step ---\n');
try
    [x_pred_ekf, P_pred_ekf, x_upd_ekf, P_upd_ekf, y_ekf, S_ekf, K_ekf, params_ekf] = ekf_filter_step(x0, P0, meas, params);
    fprintf('EKF x_upd norm = %g\n', norm(x_upd_ekf));
catch ME
    fprintf('EKF failed: %s\n', ME.message);
end

fprintf('\n--- Running UKF step ---\n');
try
    [x_pred_ukf, P_pred_ukf, x_upd_ukf, P_upd_ukf, y_ukf, S_ukf, K_ukf, params_ukf] = ukf_filter_step(x0, P0, meas, params);
    fprintf('UKF x_upd norm = %g\n', norm(x_upd_ukf));
catch ME
    fprintf('UKF failed: %s\n', ME.message);
end

% quick comparison
fprintf('\nComparison of updated positions (first two states):\n');
fprintf('KF:  [%g, %g]\n', x_upd_kf(1), x_upd_kf(2));
fprintf('EKF: [%g, %g]\n', x_upd_ekf(1), x_upd_ekf(2));
fprintf('UKF: [%g, %g]\n', x_upd_ukf(1), x_upd_ukf(2));

end

% verify_meukf_cpp.m
% C++ MEUKF implementation verification script

clc; clear; close all;

% Add paths
addpath(genpath('../../Common'));
addpath(genpath('../../ESKF'));
addpath(genpath('../bin'));

% 1. Initialize Parameters and State
params = struct();
params.g = [0; 0; -9.81]; % Negative gravity (NED)
params.mag_ref = [20; 0; 40]; % Magnetic reference
params.noise_accel = [0.1; 0.1; 0.1].^2;
params.noise_gyro = [0.01; 0.01; 0.01].^2;
params.noise_mag = [0.5; 0.5; 0.5].^2;
params.noise_gps = [1.0; 1.0; 1.0].^2;
params.noise_ba = [1e-4; 1e-4; 1e-4].^2;
params.noise_bg = [1e-5; 1e-5; 1e-5].^2;
params.alpha = 0.1;
params.beta = 2.0;
params.kappa = 0.0;

% Initial State
state = struct();
state.p = [0; 0; 0];
state.v = [0; 0; 0];
state.q = [1; 0; 0; 0]; % w, x, y, z
state.ba = [0.01; -0.02; 0.03];
state.bg = [0.001; -0.001; 0.002];
state.P = eye(15) * 0.1;

% Sensor Data
sensor = struct();
sensor.dt = 0.01;
sensor.accel = [0.2; -0.1; 9.7]; % Slightly tilted gravity + noise
sensor.gyro = [0.01; 0.02; -0.01];
sensor.mag = [20; 0; 40];
sensor.gps_pos = [10; 20; 5];
sensor.update_accel = 1;
sensor.update_mag = 0;
sensor.update_gps = 0;

% 2. Run MATLAB Implementation (Manual MEUKF Step for Accel)
% Extract relevant parts
q_nom = state.q;
P_att = state.P(7:9, 7:9);
a_meas = sensor.accel;
R_acc = diag(params.noise_accel);
g_vec = params.g;

% Observation function for Accel: h(q) = R(q)' * g
h_func = @(q) h_accel(q, g_vec);

fprintf('Running MATLAB MEUKF Update...\n');
[dtheta, P_att_new, K, S, y] = meukf_update_attitude(P_att, q_nom, a_meas, h_func, R_acc, params.alpha, params.beta, params.kappa);

% Apply update to state (MATLAB side)
dq = QuaternionLib.small_angle_quat(dtheta);
q_matlab = QuaternionLib.multiply(q_nom, dq);
q_matlab = QuaternionLib.normalize(q_matlab);
P_matlab = state.P;
P_matlab(7:9, 7:9) = P_att_new;

% 3. Run C++ MEX Implementation
fprintf('Running C++ MEX MEUKF Update...\n');
input = struct();
input.prev_state = state;
input.sensor = sensor;
input.sensor.dt = 0; % Disable prediction to isolate Update step
input.params = params;

% Call MEX
% Note: MEX expects 3 arguments: prev_state, sensor, params
% Returns: updated_state, debug_info
[state_cpp, debug_info] = mex_meukf_step(state, input.sensor, params);

% 4. Compare Results
fprintf('\n--- Comparison Results ---\n');

% Quaternion
diff_q = norm(q_matlab - state_cpp.q);
fprintf('Quaternion Diff Norm: %e\n', diff_q);
disp('MATLAB q:'); disp(q_matlab');
disp('C++    q:'); disp(state_cpp.q');

% Covariance (Attitude block)
P_att_cpp = state_cpp.P(7:9, 7:9);
diff_P = norm(P_matlab(7:9, 7:9) - P_att_cpp);
fprintf('Covariance (Att) Diff Norm: %e\n', diff_P);
disp('MATLAB P_att(1:3,1:3):'); disp(P_matlab(7:9, 7:9));
disp('C++    P_att(1:3,1:3):'); disp(P_att_cpp);

% Check if other states are untouched (as expected for attitude-only update in this simplified test)
diff_p = norm(state.p - state_cpp.p);
fprintf('Position Diff Norm (should be 0): %e\n', diff_p);

if diff_q < 1e-5 && diff_P < 1e-5
    fprintf('\nSUCCESS: MATLAB and C++ implementations match (Update Only)!\n');
else
    fprintf('\nWARNING: Mismatch detected (Update Only)!\n');
end

% 5. Verify Prediction Step
fprintf('\n--- Verifying Prediction Step ---\n');
input_pred = struct();
input_pred.prev_state = state;
input_pred.sensor = sensor;
input_pred.sensor.update_accel = 0; % Disable update
input_pred.sensor.update_mag = 0;
input_pred.sensor.update_gps = 0;
input_pred.params = params;

% MATLAB Prediction
state_pred_matlab = predict_matlab(state, sensor, params);

% C++ Prediction
[state_pred_cpp, ~] = mex_meukf_step(state, input_pred.sensor, params);

% Compare
diff_p_pred = norm(state_pred_matlab.p - state_pred_cpp.p);
diff_v_pred = norm(state_pred_matlab.v - state_pred_cpp.v);
diff_q_pred = norm(state_pred_matlab.q - state_pred_cpp.q);
diff_P_pred = norm(state_pred_matlab.P - state_pred_cpp.P);

fprintf('Position Diff: %e\n', diff_p_pred);
fprintf('Velocity Diff: %e\n', diff_v_pred);
fprintf('Quaternion Diff: %e\n', diff_q_pred);
fprintf('Covariance Diff: %e\n', diff_P_pred);

if diff_P_pred > 1e-5
    disp('Difference in P (MATLAB - C++):');
    disp(state_pred_matlab.P - state_pred_cpp.P);
end

if diff_p_pred < 1e-5 && diff_v_pred < 1e-5 && diff_q_pred < 1e-5 && diff_P_pred < 1e-5
    fprintf('SUCCESS: Prediction matches!\n');
else
    fprintf('WARNING: Prediction mismatch!\n');
end

% 6. Verify Mag Update
fprintf('\n--- Verifying Mag Update ---\n');
input_mag = struct();
input_mag.prev_state = state;
input_mag.sensor = sensor;
input_mag.sensor.dt = 0; % Disable prediction
input_mag.sensor.update_accel = 0;
input_mag.sensor.update_mag = 1;
input_mag.sensor.update_gps = 0;
input_mag.params = params;

% MATLAB Mag Update
q_nom = state.q;
P_att = state.P(7:9, 7:9);
m_meas = sensor.mag;
R_mag = diag(params.noise_mag);
mag_ref = params.mag_ref;
h_func_mag = @(q) h_mag(q, mag_ref);

[dtheta_mag, P_att_mag, ~, ~, ~] = meukf_update_attitude(P_att, q_nom, m_meas, h_func_mag, R_mag, params.alpha, params.beta, params.kappa);

dq_mag = QuaternionLib.small_angle_quat(dtheta_mag);
q_mag_matlab = QuaternionLib.multiply(q_nom, dq_mag);
q_mag_matlab = QuaternionLib.normalize(q_mag_matlab);
P_mag_matlab = state.P;
P_mag_matlab(7:9, 7:9) = P_att_mag;

% C++ Mag Update
[state_mag_cpp, ~] = mex_meukf_step(state, input_mag.sensor, params);

% Compare
diff_q_mag = norm(q_mag_matlab - state_mag_cpp.q);
diff_P_mag = norm(P_mag_matlab(7:9, 7:9) - state_mag_cpp.P(7:9, 7:9));

fprintf('Quaternion Diff: %e\n', diff_q_mag);
fprintf('Covariance Diff: %e\n', diff_P_mag);

if diff_q_mag < 1e-5 && diff_P_mag < 1e-5
    fprintf('SUCCESS: Mag Update matches!\n');
else
    fprintf('WARNING: Mag Update mismatch!\n');
end

% --- Helper Functions ---
function z_pred = h_mag(q, mag_ref)
    R = QuaternionLib.quat2rotm(q);
    z_pred = R' * mag_ref;
end

function z_pred = h_accel(q, g)
    R = QuaternionLib.quat2rotm(q);
    % Accelerometer measures upward force (-g in body frame)
    % If g is [0;0;-9.8] (downward), then -g is [0;0;9.8] (upward)
    % z_pred = R' * (-g) = - (R' * g)
    z_pred = - (R' * g);
end

function [state_pred] = predict_matlab(state, sensor, params)
    state_pred = state;
    dt = sensor.dt;
    g = params.g;
    
    w_corr = sensor.gyro - state.bg;
    a_corr = sensor.accel - state.ba;
    
    % 1. Nominal State
    % Attitude
    w_norm = norm(w_corr);
    if w_norm * dt < 1e-6
        dq = [1; 0; 0; 0];
    else
        half_angle = w_norm * dt * 0.5;
        s = sin(half_angle);
        dq = [cos(half_angle); (w_corr/w_norm)*s];
    end
    q_pred = QuaternionLib.multiply(state.q, dq);
    state_pred.q = QuaternionLib.normalize(q_pred);
    
    % Position/Velocity
    R = QuaternionLib.quat2rotm(state.q);
    % a_corr is proper acceleration (includes reaction to gravity)
    % a_kinematic = R * a_corr + g (where g is [0;0;-9.8])
    a_world = R * a_corr + g;
    state_pred.p = state.p + state.v * dt + a_world * (0.5 * dt^2);
    state_pred.v = state.v + a_world * dt;
    
    % 2. Error Covariance
    F = eye(15);
    F(1:3, 4:6) = eye(3) * dt;
    
    % dv/dtheta = -R * [a_corr]x * dt
    a_skew = [0 -a_corr(3) a_corr(2); a_corr(3) 0 -a_corr(1); -a_corr(2) a_corr(1) 0];
    F(4:6, 7:9) = -R * a_skew * dt;
    
    % dv/dba = -R * dt
    F(4:6, 10:12) = -R * dt;
    
    % dtheta/dtheta = I - [w_corr]x * dt
    w_skew = [0 -w_corr(3) w_corr(2); w_corr(3) 0 -w_corr(1); -w_corr(2) w_corr(1) 0];
    F(7:9, 7:9) = eye(3) - w_skew * dt;
    
    % dtheta/dbg = -I * dt
    F(7:9, 13:15) = -eye(3) * dt;
    
    % Process Noise Q
    Q = zeros(15);
    dt2 = dt^2;
    Q(4:6, 4:6) = diag(params.noise_accel) * dt2;
    Q(7:9, 7:9) = diag(params.noise_gyro) * dt2;
    Q(10:12, 10:12) = diag(params.noise_ba) * dt;
    Q(13:15, 13:15) = diag(params.noise_bg) * dt;
    
    state_pred.P = F * state.P * F' + Q;
end

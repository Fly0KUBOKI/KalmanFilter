% diagnose_unified_filter.m
% Quick diagnostics for mex_unified_filter and gyro biquad filter
% - Calls mex_unified_filter on a stationary state and checks quaternion
% - Compares mex_sensor_filter('gyro',...) output with MATLAB Biquad fallback

fprintf('Running diagnose_unified_filter...\n');

% Try to make project root available on path (best-effort)
try
    scriptPath = fileparts(mfilename('fullpath'));
    projectRoot = fullfile(scriptPath, '..', '..', '..');
    addpath(projectRoot);
catch
    % fallback: assume user has project on path
end

% Ensure MEX filter library starts from clean state when present
try
    if exist('mex_sensor_filter','file') == 3
        mex_sensor_filter('reset');
    end
catch
    % ignore
end

%% 1) Unified filter identity test (stationary)
prev_state = struct();
prev_state.p = [0;0;0];
prev_state.v = [0;0;0];
prev_state.q = [1;0;0;0];
prev_state.ba = [0;0;0];
prev_state.bg = [0;0;0];
prev_state.P = eye(15) * 1e-6;

input = struct();
input.dt = 0.01;
input.accel = [0;0;9.80665];
input.gyro = [0;0;0];
input.mag = [0;0;0]; input.mag_valid = false;
input.gps_pos = [0;0;0]; input.gps_valid = false;
input.baro_alt = 0; input.baro_valid = false;
input.prev_mag = [0;0;0]; input.prev_gps_pos = [0;0;0]; input.prev_baro_alt = 0;
input.g = [0;0;9.80665];
input.mag_ref = [1;0;0];
input.noise_accel = 0.01; input.noise_gyro = 1.74e-4; input.noise_mag = 0.1;
input.noise_gps = 1.0; input.noise_baro = 1.0;
input.alpha = 0.001; input.beta = 2.0; input.kappa = 0.0;

fprintf('\n[1] Calling mex_unified_filter (identity test)\n');
try
    out = mex_unified_filter(prev_state, input);
    q_out = out.quaternion(:);
    q_diff = norm(q_out - prev_state.q);
    fprintf(' -> quaternion diff norm = %.3e\n', q_diff);
    if q_diff < 1e-6
        fprintf(' --> PASS: quaternion essentially unchanged.\n');
    else
        fprintf(' --> WARN: quaternion changed (diff > 1e-6).\n');
    end
    if isfield(out, 'roll') && isfield(out, 'pitch')
        fprintf(' -> reported roll/pitch/yaw = %.6g / %.6g / %.6g deg\n', out.roll, out.pitch, out.yaw);
    end
catch ME
    fprintf(' ERROR: mex_unified_filter call failed: %s\n', ME.message);
end

%% 2) Gyro Biquad comparison (mex vs MATLAB wrapper)
fprintf('\n[2] Gyro biquad comparison\n');
dt = 1/200; cutoff = 30; sample_rate = 200;
w_sample = [0.1; 0.2; -0.05]; % rad/s sample

% Call mex_sensor_filter('gyro', ...)
mex_available = (exist('mex_sensor_filter','file') == 3);
if mex_available
    try
        out_mex = mex_sensor_filter('gyro', w_sample, dt, cutoff);
        fprintf(' -> mex_sensor_filter -> [%.6g %.6g %.6g]\n', out_mex(1), out_mex(2), out_mex(3));
    catch ME
        fprintf(' -> mex_sensor_filter error: %s\n', ME.message);
        out_mex = [];
    end
else
    fprintf(' -> mex_sensor_filter not available on path.\n');
    out_mex = [];
end

% MATLAB Biquad reference using BiquadFilter_cpp (fallback to MATLAB internals)
try
    bf = BiquadFilter_cpp(sample_rate, cutoff);
    out_matlab = bf.apply(w_sample);
    fprintf(' -> MATLAB BiquadFilter_cpp -> [%.6g %.6g %.6g]\n', out_matlab(1), out_matlab(2), out_matlab(3));
catch ME
    fprintf(' -> MATLAB BiquadFilter_cpp error: %s\n', ME.message);
    out_matlab = [];
end

if ~isempty(out_mex) && ~isempty(out_matlab)
    diff = norm(out_mex - out_matlab);
    fprintf(' -> filter output diff norm = %.3e\n', diff);
    if diff < 1e-6
        fprintf(' --> PASS: mex and MATLAB outputs match closely.\n');
    else
        fprintf(' --> WARN: mex vs MATLAB difference detected.\n');
    end
end

% --- Detailed per-axis check (coefficients + one-step DF-II) ---
if ~isempty(out_matlab)
    try
        fprintf('\n[Detail] MATLAB coefficients and one-step DF-II calculation:\n');
        b0 = bf.b0; b1 = bf.b1; b2 = bf.b2; a1 = bf.a1; a2 = bf.a2;
        fprintf(' MATLAB coeffs: b0=%.8g b1=%.8g b2=%.8g a1=%.8g a2=%.8g\n', b0, b1, b2, a1, a2);

        % assume initial states zero
        x1 = zeros(3,1); x2 = zeros(3,1);
        w = w_sample - a1 * x1 - a2 * x2;
        y_df = b0 * w + b1 * x1 + b2 * x2;
        fprintf(' DF-II manual result -> [%.8g %.8g %.8g]\n', y_df(1), y_df(2), y_df(3));
    catch ME
        fprintf(' Detail calc error: %s\n', ME.message);
    end
end

fprintf('\nDiagnosis complete.\n');

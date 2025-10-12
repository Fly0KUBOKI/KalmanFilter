function params = config_params()
% config_params - central configuration for simulation and filters
params = struct();

% timing
params.dt = 0.0025;
params.T = 10;

% initial state for [x y vx vy theta ax ay omega z vz]
params.initial_state = [0;0;1;0;0;0;0;0;0;0];

% motion
params.motion.mode = 'circular'; % 'circular' or 'random'
params.motion.center = [0,0];
params.motion.radius = 50;
params.motion.omega = 0.2;
params.motion.phase_noise = deg2rad(0.5);
params.motion.speed_mean = 0.01;
params.motion.speed_std = 0.0;
params.motion.heading_change_std = deg2rad(90);
params.motion.change_interval = 0.1;


% --- Nominal (white) sensor noise levels (per-sensor defaults) ---
% These are the baseline Gaussian noise stddev values used by sim_generate.
params.noise.pos = 0.1;           % position noise (m)
params.noise.vel = 0.1;           % velocity noise (m/s)
params.noise.accel3 = [0.1, 0.1, 0.1];
params.noise.gyro3 = deg2rad([0.1, 0.1, 0.1]);
params.noise.mag3 = [0.1, 0.1, 0.1];
params.noise.gps = 1.0;
params.noise.baro = 0.01;
params.noise.heading = deg2rad(0.1);


% Global pink-noise settings (apply pink noise to multiple sensors)
% Set enable=true to add pink noise to the listed sensors. Per-sensor std
% defaults are taken from the nominal noise fields above.
params.noise.pink.enable = true;
params.noise.pink.std.pos = 0.1;
params.noise.pink.std.vel = 0.1;
params.noise.pink.std.accel3 = [0.1, 0.1, 0.0];
params.noise.pink.std.gyro3 = deg2rad([0.1, 0.1, 0.1]);
params.noise.pink.std.mag3 = [0.1, 0.1, 0.1];
params.noise.pink.std.gps = 1.0;
params.noise.pink.std.baro = 1.0;
params.noise.pink.std.heading = deg2rad(0.1);


params.noise.gyro_allan.enable = true;
params.noise.gyro_allan.bias_sigma = deg2rad(0.8);
params.noise.gyro_allan.rate_rw_sigma = deg2rad(0.01);


params.noise.baro_allan.enable = true;
params.noise.baro_allan.bias_sigma = 0.8;
params.noise.baro_allan.rate_rw_sigma = 0.01;


params.noise.outlier.prob_per = struct();
params.noise.outlier.prob_per.pos = 0.01;
params.noise.outlier.prob_per.vel = 0.01;
params.noise.outlier.prob_per.accel3 = 0.01;
params.noise.outlier.prob_per.gyro3 = 0.01;
params.noise.outlier.prob_per.mag3 = 0.01;
params.noise.outlier.prob_per.gps = 0.1;
params.noise.outlier.prob_per.baro = 0.1;
params.noise.outlier.prob_per.heading = 0.01;

% nominal magnetic field in world frame (2D + z=0)
params.sensors.mag_field = [1; 0; 0]; % arbitrary unit vector pointing along +x

% EKF params (10-state model)
% initial state for EKF: [x y vx vy theta ax ay omega z vz]
params.kf.x0 = [0;0;1;0;0;0;0;0;0;0]; % default: vx=1
% initial covariance (10x10)
params.kf.P0 = diag([10,10,5,5,1,1,1,1,1,1]);
params.kf.process_noise_accel = 0.5;
% filter type: 'ekf', 'ukf', or 'kf' (step-wise KF)
params.kf.type = 'eskf';

% EMA adaptive R defaults
% number of samples to collect before switching to EMA (shortened default)
params.kf.ema_warmup = 0; % disable warmup by default
% EMA smoothing factor (alpha)
params.kf.ema_alpha = 0.1;
% adaptive R enabled flag (default: disabled)
params.kf.adaptive_R_enabled = false;

% safety defaults for initialization and gating
% initialize from first GPS observation if available
params.kf.init_from_first_gps = true;
% disable Mahalanobis gating for the first N steps to avoid excluding
% legitimate large initial residuals
params.kf.maha_disable_steps = 3;
% Mahalanobis gating inflation defaults (max multiplier and scale)
params.kf.maha_inflation_max = 100;    % cap for R inflation multiplier when gating
params.kf.maha_inflation_scale = 50;   % scale factor mapping (d2 - gate) to inflation

% Adaptive R estimation safeguards
params.kf.R_est_max_mult = 10;         % per-step max multiplicative growth for R_est
params.kf.R_est_abs_max = 1e4;         % absolute upper bound on estimated R (variance)

% filter options
params.filter.method = 'none'; % 'none','avg10','ema'
params.filter.alpha = 0.2;

% data source default: 'sim' or 'csv'
params.data.source = 'csv';
% default CSV file: place sim_data.csv next to this config function (GenerateData folder)
params.data.file = fullfile(fileparts(mfilename('fullpath')),'sim_data.csv');

end

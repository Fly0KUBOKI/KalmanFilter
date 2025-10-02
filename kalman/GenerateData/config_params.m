function params = config_params()
% config_params - central configuration for simulation and filters
params = struct();

% timing
params.dt = 0.1;
params.T = 300;

% initial state for [x y vx vy theta ax ay omega z vz]
params.initial_state = [0;0;1;0;0;0;0;0;0;0];

% motion
params.motion.mode = 'circular';
params.motion.center = [0,0];
params.motion.radius = 20;
params.motion.omega = 0.2;
params.motion.phase_noise = deg2rad(0.5);
params.motion.speed_mean = 2.0;
params.motion.speed_std = 0.5;
params.motion.heading_change_std = deg2rad(15);
params.motion.change_interval = 2.0;

% sensor noise defaults (standard deviations)
params.noise.pos = 1.0;            % position (m)
params.noise.vel = 0.2;            % velocity (m/s)
params.noise.accel = 0.1;          % scalar accel magnitude (fallback)
params.noise.heading = deg2rad(5);

% Per-sensor 3-axis / per-sensor noise settings
% Each can be scalar (applied to all axes) or a 1x3 vector [sx sy sz]
params.noise.accel3 = [0.05, 0.05, 0.05];   % accelerometer noise (m/s^2)
params.noise.gyro3  = deg2rad([0.05, 0.05, 0.05]); % gyroscope noise (rad/s)
params.noise.mag3   = [0.5, 0.5, 0.5];   % magnetometer noise (arb. units)
params.noise.gps    = 2.0;                 % GPS position noise (m) (scalar or [sx sy])
params.noise.baro   = 1.0;                 % barometer noise (m)

% Global pink-noise settings (apply pink noise to multiple sensors)
% Set enable=true to add pink noise to the listed sensors. Per-sensor std
% defaults are taken from the nominal noise fields above.
params.noise.pink.enable = true;
params.noise.pink.std.pos = params.noise.pos;
params.noise.pink.std.vel = params.noise.vel;
params.noise.pink.std.accel3 = params.noise.accel3;
params.noise.pink.std.gyro3 = params.noise.gyro3;
params.noise.pink.std.mag3 = params.noise.mag3;
params.noise.pink.std.gps = params.noise.gps;
params.noise.pink.std.baro = params.noise.baro;
params.noise.pink.std.heading = params.noise.heading;


params.noise.gyro_allan.enable = true;
params.noise.gyro_allan.bias_sigma = deg2rad(0.9);
params.noise.gyro_allan.rate_rw_sigma = deg2rad(0.01);


params.noise.baro_allan.enable = true;
params.noise.baro_allan.bias_sigma = 0.9;
params.noise.baro_allan.rate_rw_sigma = 0.01;

% Outlier configuration: probability per-sample and magnitude per-sensor
% - prob: probability (0..1) that a given sample is an outlier
% - mag: per-sensor magnitude (same fields as used in sim_generate)
params.noise.outlier.prob = 0.1; % default 1% of samples
% leave mag empty here so sim_generate can derive sensible defaults from nominal noise
params.noise.outlier.mag = struct();

% nominal magnetic field in world frame (2D + z=0)
params.sensors.mag_field = [1; 0; 0]; % arbitrary unit vector pointing along +x

% EKF params (10-state model)
% initial state for EKF: [x y vx vy theta ax ay omega z vz]
params.kf.x0 = [0;0;1;0;0;0;0;0;0;0]; % default: vx=1
% initial covariance (10x10)
params.kf.P0 = diag([10,10,5,5,1,1,1,1,1,1]);
params.kf.process_noise_accel = 0.5;

% filter options
params.filter.method = 'none'; % 'none','avg10','ema'
params.filter.alpha = 0.2;

% data source default: 'sim' or 'csv'
params.data.source = 'csv';
params.data.file = fullfile(pwd,'sim_data.csv');

end

function params = config_params()
% Configuration parameters for sim_generate
% Coordinate systems:
%   Body: Forward(+X), Right(+Y), Down(+Z) - for velocity, acceleration, gyro, magnetometer
%   Geographic: GPS coordinates only (lat, lon, alt)

params = struct();

% Simulation timing
params.dt = 0.0025;     % Sample period (seconds) - 10Hz
params.T = 10;       % Total simulation time (seconds)

% Motion type selection
params.motion_type = 'circular';  % 'circular' or 'random_walk'

% Heading mode: 'fixed_north' => aircraft nose always points to geographic north (yaw=0)
%               'align_velocity' => aircraft nose always aligned with velocity direction
params.heading_mode = 'align_velocity'; % 'fixed_north' or 'align_velocity'

% Sensor noise parameters (1-sigma standard deviations)
params.noise = struct();
params.noise.accel_std = 0.0;   % Accelerometer noise (m/s^2)
params.noise.gyro_std = 0.0;   % Gyroscope noise (deg/s)
params.noise.mag_std = 0.0;      % Magnetometer noise (nT)
params.noise.baro_std = 0.0;     % Barometer noise (meters)
params.noise.gps_std = 0.0;      % GPS position noise (meters)

% Motion parameters
params.motion = struct();

% Circular motion parameters
params.motion.circular = struct();
params.motion.circular.radius = 50;       % Circular trajectory radius (meters)
params.motion.circular.omega = 5;       % Angular velocity (deg/s)
params.motion.circular.altitude = 0;    % Flight altitude (meters above sea level)

% Random walk parameters  
params.motion.random_walk = struct();
params.motion.random_walk.velocity_std = 0.0;     % Velocity change std (m/s)
params.motion.random_walk.angular_std = 0.0;     % Angular velocity std (deg/s)
params.motion.random_walk.altitude_std = 0.0;     % Altitude change std (m/s)

% GPS reference point
params.gps_origin = struct();
params.gps_origin.lat = 36.0;  % degrees (default origin as requested)
params.gps_origin.lon = 140.0; % degrees
params.gps_origin.alt = 0;        % Sea level reference (meters)

% Initial conditions (all in body frame except GPS position)
params.initial = struct();
params.initial.gps_position = [params.gps_origin.lat, params.gps_origin.lon, 100];  % Initial GPS [lat, lon, alt]
params.initial.velocity = [5, 0, 0];       % Initial velocity [Forward, Right, Down] (m/s) - body frame
params.initial.attitude = [0, 0, 0];       % Initial attitude [Roll, Pitch, Yaw] (degrees)

% Output settings: directory and filenames for CSV outputs
cfg_dir = fileparts(mfilename('fullpath'));
params.output = struct();
params.output.dir = cfg_dir;                    % default output directory (GenerateData folder)
params.output.truth_filename = 'truth_data.csv';
params.output.sensor_filename = 'sensor_data.csv';

params.thresholds = 1.0e-6;  % Thresholds

end

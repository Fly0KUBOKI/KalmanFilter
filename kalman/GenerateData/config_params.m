function params = config_params()
% Configuration parameters for sim_generate
% Coordinate systems:
%   Body: Forward(+X), Right(+Y), Down(+Z) - for velocity, acceleration, gyro, magnetometer
%   Geographic: GPS coordinates only (lat, lon, alt)

params = struct();

% Simulation timing
params.dt = 0.0025;     % Sample period (seconds) - 400Hz
params.T = 100;       % Total simulation time (seconds)
params.static_time = 5; % Initial static period for calibration (seconds)

% Motion type selection
params.motion_type = 'circular';  % 'circular' or 'random_walk'

% Heading mode: 'fixed_north' => aircraft nose always points to geographic north (yaw=0)
%               'align_velocity' => aircraft nose always aligned with velocity direction
params.heading_mode = 'align_velocity'; % 'fixed_north' or 'align_velocity'

% Sensor noise parameters (1-sigma standard deviations)
params.noise = struct();
params.noise.accel_std = 0.05;   % Accelerometer noise (m/s^2)
params.noise.gyro_std = 0.05;   % Gyroscope noise (deg/s)
params.noise.mag_std = 5.0;      % Magnetometer noise (nT)
params.noise.baro_std = 1.0;     % Barometer noise (meters)
params.noise.gps_std = 2;      % GPS position noise (meters)

% Pink noise parameters (1/f noise)
params.noise.accel_pink_std = 0.05;   % Accelerometer pink noise (m/s^2)
params.noise.gyro_pink_std = 0.05;    % Gyroscope pink noise (deg/s)
params.noise.gps_pink_std = 0.0;      % GPS pink noise (meters)

% Allan deviation parameters (bias instability)
params.noise.gyro_allan_std = 0.1;   % Gyroscope Allan deviation (deg/s)
params.noise.baro_allan_std = 0.1;   % Barometer Allan deviation (meters)

% Motion parameters
params.motion = struct();

% Circular motion parameters
params.motion.circular = struct();
params.motion.circular.radius = 10;       % Circular trajectory radius (meters)
params.motion.circular.omega = 4;       % Angular velocity (deg/s)
params.motion.circular.altitude = 0;    % Flight altitude (meters above sea level)
params.motion.circular.accel_time = 5;  % Soft start acceleration time (seconds) - after static period
params.motion.circular.angular_std = 5.0;  % Angular velocity fluctuation std (deg/s)
params.motion.circular.angular_tau = 5.0;  % Angular velocity fluctuation time constant (seconds) - larger = slower fluctuation

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
params.initial.gps_position = [params.gps_origin.lat, params.gps_origin.lon, 0];  % Initial GPS [lat, lon, alt]
params.initial.velocity = [0, 0, 0];       % Initial velocity [Forward, Right, Down] (m/s) - body frame
params.initial.attitude = [0, 0, 0];       % Initial attitude [Roll, Pitch, Yaw] (degrees)

% Output settings: directory and filenames for CSV outputs
cfg_dir = fileparts(mfilename('fullpath'));
params.output = struct();
params.output.dir = cfg_dir;                    % default output directory (GenerateData folder)
params.output.truth_filename = 'truth_data.csv';
params.output.sensor_filename = 'sensor_data.csv';

params.thresholds = 1.0e-6;  % Thresholds

end

% sim_generate.m - Generate ground truth and sensor observations
% Coordinate systems:
%   Body: Forward(+X), Right(+Y), Down(+Z) - for velocity, acceleration, gyro, magnetometer
%   Geographic: GPS coordinates only (lat, lon, alt)
function sim_generate(params)

% Allow running as a standalone program: if no params passed, load config
if nargin < 1 || isempty(params)
    params = config_params();
end

dt = params.dt;
T = params.T;
N = floor(T/dt)+1;
t = (0:N-1)' * dt;

% Initialize arrays - all in body frame except GPS
vel_body = zeros(N,3);     % Body frame velocity [forward, right, down] m/s
accel_body = zeros(N,3);   % Body frame acceleration [forward, right, down] m/s^2
attitude = zeros(N,3);     % Attitude [roll, pitch, yaw] radians
pos_local = zeros(N,3);    % Local position for trajectory calculation [x, y, z] meters

% Initial conditions
vel_body(1,:) = params.initial.velocity;
attitude(1,:) = params.initial.attitude;
pos_local(1,:) = [0, 0, 0];  % Start at origin of local frame

% Generate motion based on selected type
motion_type = params.motion_type;

% Generate motion trajectory
if strcmp(motion_type, 'circular')
    % Circular motion parameters
    radius = params.motion.circular.radius;
    omega = params.motion.circular.omega;
    speed = omega * radius;
    
    for i = 1:N
        angle = omega * t(i);
        
        % Local position (for GPS conversion)
        pos_local(i,1) = radius * cos(angle);     % X (East direction)
        pos_local(i,2) = radius * sin(angle);     % Y (North direction)
        pos_local(i,3) = -params.motion.circular.altitude;  % Z (Down, negative = up)
        
        % Body frame velocity: constant forward motion
        vel_body(i,1) = speed;                    % Forward velocity
        vel_body(i,2) = 0;                        % No lateral velocity
        vel_body(i,3) = 0;                        % No vertical velocity
        
        % Attitude: yaw follows trajectory direction
        attitude(i,1) = 0;                        % Roll
        attitude(i,2) = 0;                        % Pitch  
        attitude(i,3) = angle + pi/2;             % Yaw (90° offset for forward direction)
        
        % Body frame acceleration: centripetal acceleration
        accel_body(i,1) = 0;                      % No forward acceleration
        accel_body(i,2) = -speed^2/radius;        % Centripetal acceleration (rightward)
        accel_body(i,3) = 0;                      % No vertical acceleration
    end
    
elseif strcmp(motion_type, 'random_walk')
    % Random walk motion but constrained to forward motion only
    % We vary forward speed and yaw; lateral/vertical body velocities remain zero
    vel_std = params.motion.random_walk.velocity_std;  % m/s per sqrt(sec)
    ang_std = params.motion.random_walk.angular_std;    % rad/s per sqrt(sec)
    
    % Initialize random number generator for repeatability
    rng(42);
    
    % initialize forward speed from initial condition
    v_forward = vel_body(1,1);
    for i = 2:N
        % Update forward speed with small random walk (discrete-time)
        dv = randn() * vel_std * sqrt(dt);
        v_forward = max(0, v_forward + dv);  % do not allow negative forward speed
        vel_body(i,1) = v_forward;
        vel_body(i,2:3) = [0, 0];

        % Update yaw (heading) with random angular velocity around Z
        yaw_rate = randn() * ang_std;  % rad/s
        attitude(i,3) = attitude(i-1,3) + yaw_rate * dt;
        attitude(i,1:2) = attitude(i-1,1:2);  % keep roll/pitch unchanged for simplicity

        % Convert forward body velocity to local frame displacement using yaw
        yaw = attitude(i-1,3);
        R_body_to_local = [cos(yaw), -sin(yaw), 0; 
                          sin(yaw),  cos(yaw), 0;
                          0,         0,        1];
        local_vel = R_body_to_local * [vel_body(i,1); 0; 0];
        pos_local(i,:) = pos_local(i-1,:) + (local_vel * dt)';

        % Acceleration: from change in forward speed only (body frame)
        accel_body(i,:) = [(vel_body(i,1)-vel_body(i-1,1))/dt, 0, 0];
    end
    
else
    error('Unknown motion_type: %s. Use ''circular'' or ''random_walk''', motion_type);
end

% Generate sensor observations
gyro_body = zeros(N,3);
mag_body = zeros(N,3);  
baro = zeros(N,1);
gps_lat = zeros(N,1);
gps_lon = zeros(N,1);
gps_alt = zeros(N,1);

% GPS reference point from config
lat0 = params.gps_origin.lat;    % Reference latitude (degrees)
lon0 = params.gps_origin.lon;    % Reference longitude (degrees)  
alt0 = params.gps_origin.alt;    % Reference altitude (meters)
m_per_deg_lat = 111320; % meters per degree latitude
m_per_deg_lon = 91290;  % meters per degree longitude (at 35°N)

% Physical constants
g = 9.81;               % gravity acceleration (m/s^2)
% Magnetic field strength (nanotesla, nT). Use 50 nT as requested.
mag_strength = 50;      % nT

% Generate sensor measurements (all in body frame except GPS)
for i = 1:N
    roll = attitude(i,1);
    pitch = attitude(i,2);
    yaw = attitude(i,3);
    
    % Rotation matrix: Geographic to Body frame (for gravity and magnetic field)
    R = eul2rotm([yaw, pitch, roll], 'ZYX');
    
    % Accelerometer: gravity + motion acceleration in body frame
    gravity_geo = [0, 0, g];  % Gravity in geographic frame (down = +Z)
    accel_body(i,:) = (R * gravity_geo')' + accel_body(i,:);  % Gravity + motion acceleration
    
    % Gyroscope: angular velocity in body frame
    if i > 1
        % Calculate angular velocity from attitude changes
        att_diff = attitude(i,:) - attitude(i-1,:);
        gyro_body(i,:) = att_diff / dt;
    else
        gyro_body(i,:) = [0, 0, 0];  % Zero initial angular velocity
    end
    
    % Magnetometer: magnetic north vector in body frame  
    % Assume magnetic north aligns with geographic north (+Y direction)
    mag_north_geo = [0, mag_strength, 0];  % North = +Y in geographic frame
    mag_body(i,:) = (R * mag_north_geo')';
    
    % Barometer: altitude above sea level (from local Z position)
    baro(i) = alt0 - pos_local(i,3);  % Convert down coordinate to altitude
    
    % GPS: convert local position to lat/lon/alt (only GPS uses geographic coordinates)
    gps_lat(i) = lat0 + pos_local(i,2) / m_per_deg_lat;    % North to latitude
    gps_lon(i) = lon0 + pos_local(i,1) / m_per_deg_lon;    % East to longitude  
    gps_alt(i) = alt0 - pos_local(i,3);                    % Down to altitude
end

% Add sensor noise if specified
if isfield(params, 'noise')
    accel_body = accel_body + randn(N,3) * params.noise.accel_std;
    gyro_body = gyro_body + randn(N,3) * params.noise.gyro_std;
    mag_body = mag_body + randn(N,3) * params.noise.mag_std;
    baro = baro + randn(N,1) * params.noise.baro_std;
    gps_lat = gps_lat + randn(N,1) * params.noise.gps_std / m_per_deg_lat;
    gps_lon = gps_lon + randn(N,1) * params.noise.gps_std / m_per_deg_lon;
    gps_alt = gps_alt + randn(N,1) * params.noise.gps_std;
end

% Prepare truth data (body frame centered) - do NOT include GPS true values per user request
truth_data = [t, vel_body, accel_body, attitude];
truth_headers = {'time', 'vel_forward', 'vel_right', 'vel_left', ...
                 'accel_forward', 'accel_right', 'accel_left', ...
                 'roll', 'pitch', 'yaw'};

% Prepare sensor observation data (body frame for IMU/mag, geographic for GPS)
sensor_data = [t, accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt];
sensor_headers = {'time', 'accel_forward', 'accel_right', 'accel_left', ...
                  'gyro_forward', 'gyro_right', 'gyro_left', ...
                  'mag_forward', 'mag_right', 'mag_left', ...
                  'baro', 'gps_lat', 'gps_lon', 'gps_altitude'};

% Save to CSV files (write into params.output if provided)
out_dir = '.';
if isfield(params, 'output') && isfield(params.output, 'dir') && ~isempty(params.output.dir)
    out_dir = params.output.dir;
end
truth_path = fullfile(out_dir, 'truth_data.csv');
sensor_path = fullfile(out_dir, 'sensor_data.csv');
if isfield(params, 'output') && isfield(params.output, 'truth_filename')
    truth_path = fullfile(out_dir, params.output.truth_filename);
end
if isfield(params, 'output') && isfield(params.output, 'sensor_filename')
    sensor_path = fullfile(out_dir, params.output.sensor_filename);
end

truth_table = array2table(truth_data, 'VariableNames', truth_headers);
writetable(truth_table, truth_path);

sensor_table = array2table(sensor_data, 'VariableNames', sensor_headers);
writetable(sensor_table, sensor_path);

fprintf('sim_generate: wrote %s and %s (%d samples)\n', truth_path, sensor_path, N);

end

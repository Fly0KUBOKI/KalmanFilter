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

% --- Convert angle-configs provided in degrees (user-facing) into radians for internals ---
if isfield(params, 'initial') && isfield(params.initial, 'attitude')
    % initial.attitude is specified in degrees in config; convert to radians for internal use
    params.initial.attitude = deg2rad(params.initial.attitude);
end
if isfield(params, 'motion') && isfield(params.motion, 'circular') && isfield(params.motion.circular, 'omega')
    % omega specified in deg/s in config; convert to rad/s
    params.motion.circular.omega = deg2rad(params.motion.circular.omega);
end
if isfield(params, 'motion') && isfield(params.motion, 'random_walk') && isfield(params.motion.random_walk, 'angular_std')
    % angular_std specified in deg/s in config; convert to rad/s
    params.motion.random_walk.angular_std = deg2rad(params.motion.random_walk.angular_std);
end

% Initialize arrays in world and body frames
pos_world = zeros(N,3);    % [x East, y North, z Up]
vel_world = zeros(N,3);
attitude = zeros(N,3);     % [alpha(roll), beta(pitch), gamma(yaw)]
vel_body = zeros(N,3);     % body-frame velocity (alpha,beta,gamma naming)
accel_body = zeros(N,3);

% Initial conditions
attitude(1,:) = params.initial.attitude; % radians
pos_world(1,:) = [0, 0, 0];

motion_type = params.motion_type;
heading_mode = 'fixed_north';
if isfield(params, 'heading_mode')
    heading_mode = params.heading_mode;
end

if strcmp(motion_type, 'circular')
    radius = params.motion.circular.radius;
    omega = params.motion.circular.omega;
    altitude = params.motion.circular.altitude;
    for i = 1:N
        angle = omega * t(i);
        pos_world(i,1) = radius * cos(angle);   % East
        pos_world(i,2) = radius * sin(angle);   % North
        pos_world(i,3) = altitude;               % Up
        vel_world(i,1) = -radius * omega * sin(angle);
        vel_world(i,2) =  radius * omega * cos(angle);
        vel_world(i,3) = 0;
    end

elseif strcmp(motion_type, 'random_walk')
    vel_std = params.motion.random_walk.velocity_std;
    ang_std = params.motion.random_walk.angular_std;
    rng(42);
    v_forward = params.initial.velocity(1);
    yaw = 0;
    for i = 2:N
        dv = randn() * vel_std * sqrt(dt);
        v_forward = max(0, v_forward + dv);
        yaw = yaw + randn() * ang_std * dt;
        vel_world(i,1) = v_forward * cos(yaw);
        vel_world(i,2) = v_forward * sin(yaw);
        vel_world(i,3) = 0;
        pos_world(i,:) = pos_world(i-1,:) + vel_world(i,:) * dt;
        attitude(i,:) = [0,0,yaw];
    end

else
    error('Unknown motion_type: %s. Use ''circular'' or ''random_walk''', motion_type);
end

% Finite difference velocities for any missing entries
for i = 1:N
    if i == 1
        if all(vel_world(1,:) == 0) && N > 1
            vel_world(1,:) = (pos_world(2,:) - pos_world(1,:)) / dt;
        end
    else
        if all(vel_world(i,:) == 0)
            vel_world(i,:) = (pos_world(i,:) - pos_world(i-1,:)) / dt;
        end
    end
end

% Compute body velocities and attitudes according to heading_mode
for i = 1:N
    vx = vel_world(i,1); vy = vel_world(i,2);

    if strcmp(heading_mode, 'fixed_north')
        yaw = 0;
    else
    
    yaw = mod(-atan2(vx, vy), 2*pi);
    end
    attitude(i,1:2) = [0,0];
    attitude(i,3) = yaw;

    R = eul2rotm([attitude(i,3), attitude(i,2), attitude(i,1)], 'ZYX');
    
    if strcmp(heading_mode, 'align_velocity')
        % Per spec: in align_velocity mode, body-frame velocity should have x=0, y=|v|, z=0.
        % Compute speed in horizontal plane and set body velocity accordingly.
        speed = hypot(vx, vy);
        % Keep the inverse-rotation computation for future non-zero roll/pitch cases, but
        % store the desired body-frame vector directly so x component is 0 as requested.
        vel_body(i,:) = [0, speed, 0];
    else
        vel_body(i,:) = (R' * vel_world(i,:)')';
    end
end


% Generate sensor observations
gyro_body = zeros(N,3);
mag_body = zeros(N,3);
baro = zeros(N,1);
gps_lat = zeros(N,1);
gps_lon = zeros(N,1);
gps_alt = zeros(N,1);

% GPS reference point from config
lat0 = params.gps_origin.lat;
lon0 = params.gps_origin.lon;
alt0 = params.gps_origin.alt;
% longitude scaling uses current latitude; we'll compute per-sample using last known latitude

% Physical constants
mag_strength = 50;      % nT

for i = 1:N
    roll = attitude(i,1);
    pitch = attitude(i,2);
    yaw = attitude(i,3);

    % rotation world -> body
    R = eul2rotm([yaw, pitch, roll], 'ZYX');

    % Accelerometer (specific force): IMU measures acceleration minus gravity
    % Compute world-frame acceleration from finite differences
    if i == 1
        a_world = [0,0,0];
    else
        dv = vel_body(i,:) - vel_body(i-1,:);
        for j = 1:3
            if abs(dv(j)) < params.thresholds
                dv(j) = 0;
            end
        end
        a_world = dv / dt;
    end
    % gravity in world frame (Up positive): [0,0,-9.81]
    g_world = [0,0,-9.81];
    % specific force (what accelerometer measures) in world frame
    specific_force_world = a_world - g_world;
    % map to body frame
    accel_body(i,:) = (R' * specific_force_world')';

    % Gyroscope: compute angular velocity from change in velocity direction (axis-angle)
    if i > 1
        u = vel_world(i-1,:); v = vel_world(i,:);
        nu = norm(u); nv = norm(v);
        if nu < params.thresholds || nv < params.thresholds
            omega_world = [0,0,0];
        else
            u_n = u / nu; v_n = v / nv;
            d = dot(u_n, v_n);
            d = min(1, max(-1, d));
            angle = acos(d);
            axis = cross(u_n, v_n);
            an = norm(axis);
            if an < params.thresholds
                omega_world = [0,0,0];
            else
                axis_n = axis / an;
                omega_world = (axis_n * (angle / dt));
            end
        end
        % rotate angular velocity into body frame
        gyro_body(i,:) = rad2deg(R' * omega_world')';
    else
        gyro_body(i,:) = [0,0,0];
    end

    % Magnetometer: base vector is [0, 50, 0] in world when nose fixed north
    mag_world = [0, mag_strength, 0];
    % Magnetometer: rotate world magnetic vector into body frame
    mag_body(i,:) = (R' * mag_world')';

    % Barometer: altitude is pos_world z (Up). Use initial alt0 as reference.
    % Barometer: output pressure [Pa] corresponding to altitude (alt0 + pos_world.z)
    % Use standard barometric formula inverse: P = P0 * (1 - alt/44330)^(1/0.1903)
    alt = alt0 + pos_world(i,3);
    P0 = 101325; % sea-level standard pressure [Pa]
    % guard against negative alt > 44330 which would give complex
    alt_clip = min(alt, 44330 - eps);
    baro(i) = P0 * (1 - (alt_clip / 44330))^(1/0.1903);

    % GPS: convert pos_world (East,North) in meters to lat/lon
    % Use conversion: lat = lat0 + north_m * 9.0e-6; lon = lon_prev + east_m * 9.0e-6 / cos(lat_prev)
    north_m = pos_world(i,2);
    east_m = pos_world(i,1);
    if i == 1
        lat_prev = lat0;
    else
        lat_prev = gps_lat(i-1);
    end
    dlat = north_m * 9.0e-6; % degrees
    dlon = east_m * (9.0e-6 / max(cosd(lat_prev), 1e-6));
    gps_lat(i) = lat0 + dlat;
    gps_lon(i) = lon0 + dlon;
    % GPS altitude should be altitude in meters (not barometer pressure)
    gps_alt(i) = alt;
end


% Add sensor noise if provided
if isfield(params, 'noise')
    accel_body = accel_body + randn(N,3) * params.noise.accel_std;
    gyro_body = gyro_body + randn(N,3) * params.noise.gyro_std;
    mag_body = mag_body + randn(N,3) * params.noise.mag_std;
    baro = baro + randn(N,1) * params.noise.baro_std;
    gps_lat = gps_lat + randn(N,1) * params.noise.gps_std * 9.0e-6;
    % longitude noise scaled by cos(lat)
    gps_lon = gps_lon + randn(N,1) .* (params.noise.gps_std * 9.0e-6 ./ max(cosd(gps_lat), 1e-6));
    gps_alt = gps_alt + randn(N,1) * params.noise.gps_std;
end

% Prepare truth data: time, world position (x,y,z), world velocity (vx,vy,vz), attitude (alpha,beta,gamma)
% Convert attitude to degrees for readability
att_deg = rad2deg(attitude);
truth_data = [t, pos_world, vel_world, att_deg];
truth_headers = {'time', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'pitch', 'roll', 'yaw'};

% Prepare sensor observation data
sensor_data = [t, accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt];
sensor_headers = {'time', 'accel_x', 'accel_y', 'accel_z', ...
                  'gyro_x', 'gyro_y', 'gyro_z', ...
                  'mag_x', 'mag_y', 'mag_z', ...
                  'baro', 'gps_lat', 'gps_lon', 'gps_alt'};

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

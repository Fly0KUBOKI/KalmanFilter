% sim_generate.m - Generate ground truth and sensor observations
% Coordinate systems:
%   Body: Forward(+X), Right(+Y), Down(+Z) - for velocity, acceleration, gyro, magnetometer
%   Geographic: GPS coordinates only (lat, lon, alt)
function sim_generate(params)

% パラメータ設定
if nargin < 1 || isempty(params)
    params = config_params();
end

% 角度単位変換（度→ラジアン）
params = convert_angle_units(params);

% 基本パラメータ
dt = params.dt;
T = params.T;
N = floor(T/dt)+1;
t = (0:N-1)' * dt;

motion_type = params.motion_type;
heading_mode = 'follow_velocity';
if isfield(params, 'heading_mode')
    heading_mode = params.heading_mode;
end

%% 運動生成
if strcmp(motion_type, 'circular')
    [pos_world, vel_world, attitude] = generate_circular_motion(params, t, N);
elseif strcmp(motion_type, 'random_walk')
    [pos_world, vel_world, attitude] = generate_random_walk(params, t, N);
end

%% 速度補完とボディフレーム速度計算
[vel_world, vel_body] = compute_body_velocities(pos_world, vel_world, attitude, params, heading_mode);

%% センサー観測生成
[accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt] = ...
    generate_sensor_observations(pos_world, vel_world, attitude, params, t, N);

%% ノイズ追加
[accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt] = ...
    add_sensor_noise(accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt, params);

%% データ保存
save_simulation_data(t, pos_world, vel_world, attitude, accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt, params);

end

function params = convert_angle_units(params)
% 角度単位を度からラジアンに変換
if isfield(params, 'initial') && isfield(params.initial, 'attitude')
    params.initial.attitude = deg2rad(params.initial.attitude);
end
if isfield(params, 'motion') && isfield(params.motion, 'circular') && isfield(params.motion.circular, 'omega')
    params.motion.circular.omega = deg2rad(params.motion.circular.omega);
end
if isfield(params, 'motion') && isfield(params.motion, 'random_walk') && isfield(params.motion.random_walk, 'angular_std')
    params.motion.random_walk.angular_std = deg2rad(params.motion.random_walk.angular_std);
end
end
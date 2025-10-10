function [nominal, P, innovations] = eskf_filter_step(nominal, P, meas, params)
% Minimal ESKF filter step for UAV state estimation (nominal + error-state covariance P)
% Inputs:
%  - nominal: struct with fields pos(3), vel(3), quat(4), bg(3), ba(3)
%  - P: error-state covariance (15x15) covering [pos; vel; theta; bg; ba]
%  - meas: struct with optional fields imu (gyro, accel), mag3, baro, gps
%  - params: configuration with dt and noise params
%
% Outputs:
%  - nominal: updated nominal state
%  - P: updated error covariance
%  - innovations: struct of innovations for debugging

% helpers
root = fileparts(mfilename('fullpath'));
addpath(root);

dt = params.dt;
innovations = struct();

% Unpack nominal
p = nominal.pos(:); v = nominal.vel(:); q = nominal.quat(:); bg = nominal.bg(:); ba = nominal.ba(:);

% 1) IMU prediction (if provided): propagate nominal with bias-corrected IMU
if isfield(meas,'imu') && ~isempty(meas.imu)
    gyro = meas.imu.gyro(:) - bg; % rad/s
    accel = meas.imu.accel(:) - ba; % m/s^2
    % attitude integration (quaternion)
    omega = [0; gyro];
    dq = eskf_utils('small_angle_quat', omega*dt);
    q = eskf_utils('quatmultiply', q, dq);
    q = eskf_utils('quatnormalize', q);
    Rb2w = eskf_utils('quat_to_rotm', q); % body->world
    g = [0;0;9.81]; % NED gravity (down positive)
    % velocity/position propagation
    v = v + (Rb2w * accel + g) * dt;
    p = p + v * dt;
    nominal.pos = p; nominal.vel = v; nominal.quat = q;
    % error-state covariance prediction (linearized)
    % state vector: [dp(3); dv(3); dtheta(3); dbg(3); dba(3)]
    F = eye(15);
    % position <- vel
    F(1:3,4:6) = eye(3) * dt;
    % vel <- attitude via accel: dv/dtheta ~= -Rb * skew(accel)
    F(4:6,7:9) = -Rb2w * eskf_utils('skew', accel) * dt;
    % vel <- accel bias
    F(4:6,13:15) = -Rb2w * dt;
    % attitude error <- gyro bias
    F(7:9,10:12) = -eye(3) * dt;
    % discrete process noise covariance
    Q = zeros(15);
    sigma_wg = params.noise.gyro_bias_walk; % gyro bias random walk
    sigma_wa = params.noise.accel_bias_walk; % accel bias random walk
    sigma_imu_gyro = params.noise.imu_gyro; % gyro noise
    sigma_imu_accel = params.noise.imu_accel; % accel noise
    % noise mapping
    % accel noise affects vel
    Q(4:6,4:6) = (sigma_imu_accel^2) * eye(3) * dt^2; % integrated accel noise
    Q(7:9,7:9) = (sigma_imu_gyro^2) * eye(3) * dt^2; % attitude angle error from gyro noise
    % bias walk
    Q(10:12,10:12) = (sigma_wg^2) * eye(3) * dt;
    Q(13:15,13:15) = (sigma_wa^2) * eye(3) * dt;
    % propagate P
    P = F * P * F' + Q;
end

% 2) magnetometer update (attitude error only) - if mag3 available
if isfield(meas,'mag3') && ~isempty(meas.mag3)
    zm = meas.mag3(:);
    % expected magnetic field in body: Rwb' * mag_field (world->body)
    mag_field = [1;0;0];
    if isfield(params,'sensors') && isfield(params.sensors,'mag_field')
        mag_field = params.sensors.mag_field(:);
    end
    Rwb = eskf_utils('quat_to_rotm', q); % body->world
    hb = Rwb' * mag_field;
    % measurement model linearized around small attitude error: delta_h = -Rwb' * skew(mag_field) * dtheta
    H = zeros(3,15);
    H(:,7:9) = -Rwb' * eskf_utils('skew', mag_field);
    Rm = (params.noise.mag3^2) * eye(3);
    z = zm;
    y = z - hb;
    S = H * P * H' + Rm;
    K = (P * H') / S;
    dx = K * y;
    P = (eye(15) - K*H) * P * (eye(15) - K*H)' + K * Rm * K';
    % inject attitude error into nominal
    dtheta = dx(7:9);
    dq = eskf_utils('small_angle_quat', dtheta);
    q = eskf_utils('quatmultiply', q, dq); q = eskf_utils('quatnormalize', q);
    nominal.quat = q;
    innovations.mag = y;
end

% 3) barometer update (height only -> pz)
if isfield(meas,'baro') && ~isempty(meas.baro)
    zb = meas.baro;
    H = zeros(1,15); H(3) = 1; % pz
    Rb = (params.noise.baro^2);
    y = zb - p(3);
    S = H * P * H' + Rb;
    K = (P * H') / S;
    dx = K * y;
    P = (eye(15) - K*H) * P * (eye(15) - K*H)' + K * Rb * K';
    % inject
    dp = dx(1:3); p = p + dp; nominal.pos = p;
    innovations.baro = y;
end

% 4) GPS update (position + velocity) and bias re-estimation
if isfield(meas,'gps') && ~isempty(meas.gps)
    zg = meas.gps(:);
    H = zeros(6,15);
    H(1:3,1:3) = eye(3); % pos
    H(4:6,4:6) = eye(3); % vel
    Rg = blkdiag((params.noise.gps_pos^2)*eye(3), (params.noise.gps_vel^2)*eye(3));
    % assemble measurement
    z = [zg; meas.vel(:)];
    h = [p; v];
    y = z - h;
    S = H * P * H' + Rg;
    K = (P * H') / S;
    dx = K * y;
    P = (eye(15) - K*H) * P * (eye(15) - K*H)' + K * Rg * K';
    % inject errors: pos, vel, attitude, biases
    p = p + dx(1:3);
    v = v + dx(4:6);
    dtheta = dx(7:9);
    dq = eskf_utils('small_angle_quat', dtheta);
    q = eskf_utils('quatmultiply', q, dq); q = eskf_utils('quatnormalize', q);
    bg = bg + dx(10:12);
    ba = ba + dx(13:15);
    nominal.pos = p; nominal.vel = v; nominal.quat = q; nominal.bg = bg; nominal.ba = ba;
    innovations.gps = y;
end

end

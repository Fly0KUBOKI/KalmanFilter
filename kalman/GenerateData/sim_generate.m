% copied from root sim_generate.m - generate batch simulation data
function [t, state, meas, params] = sim_generate(params)
dt = params.dt;
T = params.T;
N = floor(T/dt)+1;
t = (0:N-1)'.*dt;
state = zeros(N,10);
if isfield(params,'initial_state') && numel(params.initial_state) >= 1
    init = params.initial_state(:)';
    if numel(init) < 10
        init = [init, zeros(1,10-numel(init))];
    elseif numel(init) > 10
        warning('params.initial_state has %d elements; using first 10 for simulation.', numel(init));
        init = init(1:10);
    end
    state(1,:) = init;
else
    state(1,:) = [0,0,1,0,0,0,0,0,0,0];
end
rng('shuffle');
mode = 'random_walk';
if isfield(params,'motion') && isfield(params.motion,'mode')
    mode = params.motion.mode;
end
switch lower(mode)
    case 'linear_random'
        if ~isfield(params.motion,'speed_mean'), params.motion.speed_mean = 2.0; end
        if ~isfield(params.motion,'speed_std'), params.motion.speed_std = 0.5; end
        if ~isfield(params.motion,'heading_change_std'), params.motion.heading_change_std = deg2rad(15); end
        if ~isfield(params.motion,'change_interval'), params.motion.change_interval = 1.0; end
        v = state(1,3:4);
        speed = norm(v);
        if speed==0
            speed = max(0.1, params.motion.speed_mean + params.motion.speed_std*randn);
            ang = 2*pi*rand; v = speed*[cos(ang), sin(ang)];
        end
        change_steps = max(1, round(params.motion.change_interval/dt));
        for k=2:N
            v_prev = v;
            if mod(k-2, change_steps) == 0
                ang = atan2(v(2), v(1));
                ang = ang + params.motion.heading_change_std*randn;
                speed = max(0, params.motion.speed_mean + params.motion.speed_std*randn);
                v = speed*[cos(ang), sin(ang)];
            end
            pos = state(k-1,1:2) + v*dt;
            ax = (v(1)-v_prev(1))/dt;
            ay = (v(2)-v_prev(2))/dt;
            theta = atan2(v(2), v(1));
            omega = (theta - state(k-1,5))/dt;
            state(k,:) = [pos, v, theta, ax, ay, omega, 0, 0];
        end
    case 'circular'
        if ~isfield(params.motion,'center'), params.motion.center = [0,0]; end
        if ~isfield(params.motion,'radius'), params.motion.radius = 10; end
        if ~isfield(params.motion,'omega'), params.motion.omega = 0.2; end
        if ~isfield(params.motion,'phase_noise'), params.motion.phase_noise = deg2rad(1); end
        cx = params.motion.center(1); cy = params.motion.center(2);
        r = params.motion.radius; omega = params.motion.omega;
        x0 = state(1,1); y0 = state(1,2);
        if x0==0 && y0==0
            theta0 = 2*pi*rand;
        else
            theta0 = atan2(y0-cy, x0-cx);
        end
        for k=1:N
            th = theta0 + omega*(k-1)*dt + params.motion.phase_noise*randn;
            x = cx + r*cos(th); y = cy + r*sin(th);
            vx = -r*omega*sin(th); vy = r*omega*cos(th);
            ax = -r*omega^2*cos(th); ay = -r*omega^2*sin(th);
            theta = th; z = 0; vz = 0;
            state(k,:) = [x,y,vx,vy,theta,ax,ay,omega,z,vz];
        end
    otherwise
        accel = zeros(N,2);
        for k=2:N
            accel(k,:) = accel(k-1,:) + params.noise.accel*randn(1,2);
            vx = state(k-1,3) + accel(k,1)*dt;
            vy = state(k-1,4) + accel(k,2)*dt;
            x = state(k-1,1) + vx*dt; y = state(k-1,2) + vy*dt;
            theta = atan2(vy, vx); ax = accel(k,1); ay = accel(k,2); omega = 0; z = 0; vz = 0;
            state(k,:) = [x,y,vx,vy,theta,ax,ay,omega,z,vz];
        end
end
meas = struct();
meas.pos = state(:,1:2) + params.noise.pos.*randn(N,2);
meas.vel = state(:,3:4) + params.noise.vel.*randn(N,2);
vel = state(:,3:4);
true_accel_world = [zeros(1,2); diff(vel)./dt];
meas.accel3 = zeros(N,3);
if isfield(params.noise,'accel3')
    accel_noise = params.noise.accel3(:)'; if numel(accel_noise)==1, accel_noise = repmat(accel_noise,1,3); end
else
    accel_noise = [params.noise.accel, params.noise.accel, params.noise.accel];
end
for k=1:N
    th = state(k,5);
    Rwb = [cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 1];
    aw = [true_accel_world(k,1); true_accel_world(k,2); 0];
    g = -9.81;
    ab = Rwb * aw + [0;0;g];
    meas.accel3(k,:) = ab' + randn(1,3).*accel_noise;
end
theta = state(:,5);
vel_heading = atan2(state(:,4), state(:,3));
theta_use = theta;
zero_idx = (theta_use==0);
theta_use(zero_idx) = vel_heading(zero_idx);
theta_unwrap = unwrap(theta_use);
omega_z = [0; diff(theta_unwrap)./dt];
if isfield(params.noise,'gyro3')
    gyro_noise = params.noise.gyro3(:)'; if numel(gyro_noise)==1, gyro_noise = repmat(gyro_noise,1,3); end
else
    gyro_noise = [0,0,params.noise.heading];
end
meas.gyro3 = [zeros(N,2), omega_z] + randn(N,3).*gyro_noise;
mag_field = [1;0;0];
if isfield(params,'sensors') && isfield(params.sensors,'mag_field')
    mag_field = params.sensors.mag_field(:);
end
heading = theta_use;
meas.mag3 = zeros(N,3);
for k=1:N
    th = heading(k);
    Rz = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
    vec = Rz*mag_field;
    if isfield(params.noise,'mag3')
        mag_noise = params.noise.mag3(:)';
        if numel(mag_noise)==1
            mag_noise = repmat(mag_noise,1,3);
        end
    else
        mag_noise = [0.01,0.01,0.01];
    end
    meas.mag3(k,:) = vec' + randn(1,3).*mag_noise;
end
if isfield(params.noise,'gps')
    gps_noise = params.noise.gps;
    if numel(gps_noise)==1, gps_noise = [gps_noise, gps_noise]; end
else
    gps_noise = [params.noise.pos, params.noise.pos];
end
meas.gps = state(:,1:2) + randn(N,2).*gps_noise;
if isfield(params.noise,'baro')
    baro_noise = params.noise.baro;
else
    baro_noise = 0.5;
end
meas.baro = zeros(N,1) + baro_noise.*randn(N,1);
theta_vals = state(:,5);
theta_vals = theta_vals + params.noise.heading.*randn(N,1);
meas.heading = [cos(theta_vals), sin(theta_vals)];
meas.t = t;
end

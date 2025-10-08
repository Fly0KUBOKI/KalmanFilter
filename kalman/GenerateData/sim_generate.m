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
        % Prefer per-axis accel3 noise for planar random-walk; fall back to
        % scalar params.noise.accel or a safe default if neither present.
        if isfield(params.noise,'accel3') && ~isempty(params.noise.accel3)
            tmp = params.noise.accel3(:)'; if numel(tmp)==1, tmp = repmat(tmp,1,3); end
            accel_noise_vec = tmp(1:2);
        end
        accel = zeros(N,2);
        for k=2:N
            accel(k,:) = accel(k-1,:) + accel_noise_vec .* randn(1,2);
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
% accelerometer per-axis noise (white component)
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

% --- Gyro Allan-like noise: bias RW and rate RW for z-axis (optional) ---
gyro_bias = zeros(N,1);
gyro_rate_rw = zeros(N,1);
if isfield(params.noise,'gyro_allan') && isfield(params.noise.gyro_allan,'enable') && params.noise.gyro_allan.enable
    % interpret bias_sigma / rate_rw_sigma as per-sec values; scale by sqrt(dt)
    bs = params.noise.gyro_allan.bias_sigma;
    rs = params.noise.gyro_allan.rate_rw_sigma;
    for k=2:N
        gyro_bias(k) = gyro_bias(k-1) + bs * sqrt(dt) * randn;
        gyro_rate_rw(k) = gyro_rate_rw(k-1) + rs * sqrt(dt) * randn;
    end
end
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

% ---- Global pink-noise: apply pink noise to multiple sensors if enabled ----
if isfield(params.noise,'pink') && isfield(params.noise.pink,'enable') && params.noise.pink.enable
    % Paul Kellet's pink filter coefficients (approx 1/f)
    b_p = [0.049922035 -0.095993537 0.050612699 -0.004408786];
    a_p = [1 -2.494956002 2.017265875 -0.522189400];
    % helper to generate pink noise matrix N x D from std vector
    gen_pink = @(std_vec) deal_gen_pink(N, std_vec, b_p, a_p);

    % pos (2)
    pos_std = params.noise.pink.std.pos; if numel(pos_std)==1, pos_std = repmat(pos_std,1,2); end
    ppos = gen_pink(pos_std);
    meas.pos = meas.pos + ppos;

    % vel (2)
    vel_std = params.noise.pink.std.vel; if numel(vel_std)==1, vel_std = repmat(vel_std,1,2); end
    pvel = gen_pink(vel_std);
    meas.vel = meas.vel + pvel;

    % accel3 (3)
    accel3_std = params.noise.pink.std.accel3; if numel(accel3_std)==1, accel3_std = repmat(accel3_std,1,3); end
    pacc = gen_pink(accel3_std);
    meas.accel3 = meas.accel3 + pacc;

    % gyro3 (3)
    gyro3_std = params.noise.pink.std.gyro3; if numel(gyro3_std)==1, gyro3_std = repmat(gyro3_std,1,3); end
    pgyro = gen_pink(gyro3_std);
    meas.gyro3 = meas.gyro3 + pgyro;

    % mag3 (3)
    mag3_std = params.noise.pink.std.mag3; if numel(mag3_std)==1, mag3_std = repmat(mag3_std,1,3); end
    pmag = gen_pink(mag3_std);
    meas.mag3 = meas.mag3 + pmag;

    % gps (2)
    gps_std = params.noise.pink.std.gps; if numel(gps_std)==1, gps_std = repmat(gps_std,1,2); end
    pgps = gen_pink(gps_std);
    meas.gps = meas.gps + pgps;

    % baro (1)
    baro_std = params.noise.pink.std.baro;
    pbaro = gen_pink(baro_std);
    meas.baro = meas.baro + pbaro(:,1);

    % heading (angle) - perturb angle then normalize
    heading_std = params.noise.pink.std.heading;
    ph = gen_pink(heading_std);
    ang = atan2(meas.heading(:,2), meas.heading(:,1));
    nang = ang + ph(:,1);
    meas.heading(:,1) = cos(nang);
    meas.heading(:,2) = sin(nang);
end

% helper function (local inline) to generate pink noise matrix
function P = deal_gen_pink(nn, std_vec, bcoef, acoef)
    d = numel(std_vec);
    P = zeros(nn, d);
    for jj=1:d
        w = randn(nn,1);
        ptmp = filter(bcoef, acoef, w);
        ptmp = ptmp - mean(ptmp);
        sp = std(ptmp);
        if sp>0
            ptmp = ptmp / sp * std_vec(jj);
        else
            ptmp = zeros(size(ptmp));
        end
        P(:,jj) = ptmp;
    end
end

% ---- Inject random outliers into measurements ----
% Configure via params.noise.outlier
% fields: prob (per-sample probability), mag (struct with per-sensor magnitudes)
if isfield(params.noise,'outlier')
    out = params.noise.outlier;
else
    out = struct();
end
if ~isfield(out,'prob') || isempty(out.prob)
    out.prob = 0.01; % default 1% samples
end
% default outlier magnitudes scaled from nominal noise values
def = struct();
pos_n = params.noise.pos(:)'; if numel(pos_n)==1, pos_n = repmat(pos_n,1,2); end
vel_n = params.noise.vel(:)'; if numel(vel_n)==1, vel_n = repmat(vel_n,1,2); end
def.pos = 10*pos_n;
def.vel = 10*vel_n;
def.accel3 = 10*accel_noise;
def.gyro3 = 10*gyro_noise;
def.mag3 = 10*mag_noise;
def.gps = 10*(gps_noise(:)'); if numel(def.gps)==1, def.gps = repmat(def.gps,1,2); end
def.baro = 5*(baro_noise + eps);
def.heading = 5*(params.noise.heading + eps);

% merge user-specified out.mag with defaults so missing fields are filled
if ~isfield(out,'mag') || isempty(out.mag)
    out.mag = def;
else
    fn = fieldnames(def);
    for i=1:numel(fn)
        if ~isfield(out.mag, fn{i}) || isempty(out.mag.(fn{i}))
            out.mag.(fn{i}) = def.(fn{i});
        end
    end
end

M = out.mag;

% helper: get probability for sensor name (fallback to out.prob)
get_prob = @(name) ( (isfield(out,'prob_per') && isfield(out.prob_per, name) && ~isempty(out.prob_per.(name))) .* out.prob_per.(name) + (~(isfield(out,'prob_per') && isfield(out.prob_per, name) && ~isempty(out.prob_per.(name))) .* out.prob) );
mask_for = @(n,pval) (rand(n,1) < max(0,min(1,pval)));

% pos
m = mask_for(N, get_prob('pos'));
if any(m)
    s = sum(m);
    meas.pos(m,:) = meas.pos(m,:) + randn(s,2) .* repmat(M.pos, s, 1);
end

% vel
m = mask_for(N, get_prob('vel'));
if any(m)
    s = sum(m);
    meas.vel(m,:) = meas.vel(m,:) + randn(s,2) .* repmat(M.vel, s, 1);
end

% accel3
m = mask_for(N, get_prob('accel3'));
if any(m)
    s = sum(m);
    meas.accel3(m,:) = meas.accel3(m,:) + randn(s,3) .* repmat(M.accel3, s, 1);
end

% gyro3
m = mask_for(N, get_prob('gyro3'));
if any(m)
    s = sum(m);
    meas.gyro3(m,:) = meas.gyro3(m,:) + randn(s,3) .* repmat(M.gyro3, s, 1);
end

% mag3
m = mask_for(N, get_prob('mag3'));
if any(m)
    s = sum(m);
    meas.mag3(m,:) = meas.mag3(m,:) + randn(s,3) .* repmat(M.mag3, s, 1);
end

% gps
m = mask_for(N, get_prob('gps'));
if any(m)
    s = sum(m);
    meas.gps(m,:) = meas.gps(m,:) + randn(s,2) .* repmat(M.gps, s, 1);
end

% baro
m = mask_for(N, get_prob('baro'));
if any(m)
    s = sum(m);
    meas.baro(m) = meas.baro(m) + randn(s,1) .* M.baro;
end

% heading (perturb angle)
m = mask_for(N, get_prob('heading'));
if any(m)
    s = sum(m);
    ang = atan2(meas.heading(m,2), meas.heading(m,1));
    dth = randn(s,1) .* M.heading;
    nang = ang + dth;
    meas.heading(m,1) = cos(nang);
    meas.heading(m,2) = sin(nang);
end

meas.t = t;
end

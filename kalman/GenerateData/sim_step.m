function [state, meas] = sim_step(prev_state, params, step)
% sim_step - 1ステップ分の真値と観測を生成する（コピーを GenerateData に追加）
dt = params.dt;
if numel(prev_state) < 10
    prev = zeros(10,1);
    prev(1:numel(prev_state)) = prev_state(:);
else
    prev = prev_state(:);
end
if ~isfield(params,'motion'), params.motion.mode = 'linear_random'; end
mode = params.motion.mode;
switch lower(mode)
    case 'linear_random'
        if ~isfield(params.motion,'speed_mean'), params.motion.speed_mean = 2.0; end
        if ~isfield(params.motion,'speed_std'), params.motion.speed_std = 0.5; end
        if ~isfield(params.motion,'heading_change_std'), params.motion.heading_change_std = deg2rad(15); end
        if ~isfield(params.motion,'change_interval'), params.motion.change_interval = 1.0; end
        v_prev = prev(3:4)';
        v = v_prev;
        change_steps = max(1, round(params.motion.change_interval/dt));
        if mod(step-1, change_steps) == 0
            ang = atan2(v_prev(2), v_prev(1));
            ang = ang + params.motion.heading_change_std*randn;
            speed = max(0, params.motion.speed_mean + params.motion.speed_std*randn);
            v = speed*[cos(ang), sin(ang)];
        end
        pos = prev(1:2)' + v*dt;
        ax = (v(1)-v_prev(1))/dt; ay = (v(2)-v_prev(2))/dt;
        theta = atan2(v(2), v(1));
        omega = (theta - prev(5))/dt;
        state = [pos(1); pos(2); v(1); v(2); theta; ax; ay; omega; 0; 0];
    case 'circular'
        if ~isfield(params.motion,'center'), params.motion.center = [0,0]; end
        if ~isfield(params.motion,'radius'), params.motion.radius = 10; end
        if ~isfield(params.motion,'omega'), params.motion.omega = 0.2; end
        if ~isfield(params.motion,'phase_noise'), params.motion.phase_noise = deg2rad(1); end
        cx = params.motion.center(1); cy = params.motion.center(2);
        r = params.motion.radius; omega = params.motion.omega;
        if step==1
            th = 2*pi*rand;
        else
            th_prev = atan2(prev(2)-cy, prev(1)-cx);
            th = th_prev + omega*dt + params.motion.phase_noise*randn;
        end
        x = cx + r*cos(th); y = cy + r*sin(th);
        vx = -r*omega*sin(th); vy = r*omega*cos(th);
        ax = -r*omega^2*cos(th); ay = -r*omega^2*sin(th);
        theta = th; z = 0; vz = 0;
        state = [x; y; vx; vy; theta; ax; ay; omega; z; vz];
    otherwise
        if ~isfield(params.noise,'accel'), params.noise.accel = 0.1; end
        accel = params.noise.accel*randn(1,2);
        vx = prev(3) + accel(1)*dt; vy = prev(4) + accel(2)*dt;
        x = prev(1) + vx*dt; y = prev(2) + vy*dt;
        theta = atan2(vy, vx); ax = accel(1); ay = accel(2); omega = 0; z = 0; vz = 0;
        state = [x; y; vx; vy; theta; ax; ay; omega; z; vz];
end
meas = struct();
meas.pos = state(1:2)' + params.noise.pos.*randn(1,2);
meas.vel = state(3:4)' + params.noise.vel.*randn(1,2);
if isfield(params.noise,'accel3')
    accel_noise = params.noise.accel3(:)'; if numel(accel_noise)==1, accel_noise = repmat(accel_noise,1,3); end
else
    accel_noise = [params.noise.accel, params.noise.accel, params.noise.accel];
end
th = state(5);
Rwb = [cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 1];
aw = [state(6); state(7); 0];
g = -9.81;
ab = Rwb * aw + [0;0;g];
meas.accel3 = (ab'+ randn(1,3).*accel_noise);
if isfield(params.noise,'gyro3')
    gyro_noise = params.noise.gyro3(:)';
    if numel(gyro_noise)==1
        gyro_noise = repmat(gyro_noise,1,3);
    end
else
    gyro_noise = [0,0,params.noise.heading];
end
meas.gyro3 = [0,0,state(8)] + randn(1,3).*gyro_noise;
mag_field = [1;0;0];
if isfield(params,'sensors') && isfield(params.sensors,'mag_field')
    mag_field = params.sensors.mag_field(:);
end
Rz = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
vec = Rz * mag_field;
if isfield(params.noise,'mag3')
    mag_noise = params.noise.mag3(:)';
    if numel(mag_noise)==1
        mag_noise = repmat(mag_noise,1,3);
    end
else
    mag_noise = [0.01,0.01,0.01];
end
meas.mag3 = vec' + randn(1,3).*mag_noise;
if isfield(params.noise,'gps')
    gps_noise = params.noise.gps;
    if numel(gps_noise)==1
        gps_noise = [gps_noise,gps_noise];
    end
else
    gps_noise = [params.noise.pos, params.noise.pos];
end
meas.gps = state(1:2)' + randn(1,2).*gps_noise;
if isfield(params.noise,'baro')
    baro_noise = params.noise.baro;
else
    baro_noise = 0.5;
end
meas.baro = state(9) + baro_noise*randn;
theta_noisy = state(5) + params.noise.heading.*randn;
meas.heading = [cos(theta_noisy), sin(theta_noisy)];
end

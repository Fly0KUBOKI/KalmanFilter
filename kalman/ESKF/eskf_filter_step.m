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
    % safe retrieval of noise parameters with fallbacks
    if isfield(params,'noise') && isfield(params.noise,'gyro_bias_walk')
        sigma_wg = params.noise.gyro_bias_walk;
    elseif isfield(params,'noise') && isfield(params.noise,'gyro_allan') && isfield(params.noise.gyro_allan,'rate_rw_sigma')
        sigma_wg = params.noise.gyro_allan.rate_rw_sigma;
    else
        sigma_wg = 1e-5;
    end
    if isfield(params,'noise') && isfield(params.noise,'accel_bias_walk')
        sigma_wa = params.noise.accel_bias_walk;
    else
        sigma_wa = 1e-4;
    end
    if isfield(params,'noise') && isfield(params.noise,'imu_gyro')
        sigma_imu_gyro = params.noise.imu_gyro;
    elseif isfield(params,'noise') && isfield(params.noise,'gyro3')
        sigma_imu_gyro = mean(params.noise.gyro3(:));
    else
        sigma_imu_gyro = deg2rad(0.1);
    end
    if isfield(params,'noise') && isfield(params.noise,'imu_accel')
        sigma_imu_accel = params.noise.imu_accel;
    elseif isfield(params,'noise') && isfield(params.noise,'accel3')
        sigma_imu_accel = mean(params.noise.accel3(:));
    else
        sigma_imu_accel = 0.1;
    end
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
    % measurement noise for mag3: accept scalar or 3-vector
    if isfield(params,'noise') && isfield(params.noise,'mag3')
        mn = params.noise.mag3(:)';
        if numel(mn) == 1
            Rm = (mn^2) * eye(3);
        else
            Rm = diag((mn(:)).^2);
        end
    else
        Rm = 0.01 * eye(3);
    end
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
    if isfield(params,'noise') && isfield(params.noise,'baro')
        Rb = (params.noise.baro)^2;
    else
        Rb = 0.5^2;
    end
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
    % GPS noise (allow scalar or vector)
    if isfield(params,'noise') && isfield(params.noise,'gps_pos')
        gp = params.noise.gps_pos; if numel(gp)==1, gp = repmat(gp,1,3); end
    else
        gp = repmat(3.0,1,3);
    end
    if isfield(params,'noise') && isfield(params.noise,'gps_vel')
        gv = params.noise.gps_vel; if numel(gv)==1, gv = repmat(gv,1,3); end
    else
        gv = repmat(0.5,1,3);
    end
    Rg = blkdiag(diag((gp(:)).^2), diag((gv(:)).^2));
    % assemble measurement
    z = [zg; meas.vel(:)];
    h = [p; v];
    y = z - h;
    % save prior P for debugging
    P_prior = P;
    S = H * P * H' + Rg;
    K = (P * H') / S;
    dx = K * y;
    P = (eye(15) - K*H) * P * (eye(15) - K*H)' + K * Rg * K';
    % --- Debug logging: append GPS update diagnostics to CSV ---
    try
        dbg_dir = fullfile(root,'..','KalmanFilter','debug');
        if ~exist(dbg_dir,'dir'), mkdir(dbg_dir); end
        dbg_file = fullfile(dbg_dir,'eskf_gps_log.csv');
        if ~exist(dbg_file,'file')
            fid = fopen(dbg_file,'w');
            if fid~=-1
                fprintf(fid,'t,y_px,y_py,y_pz,y_vx,y_vy,y_vz,');
                fprintf(fid,'S1,S2,S3,S4,S5,S6,Rg1,Rg2,Rg3,Rg4,Rg5,Rg6,');
                fprintf(fid,'norm_dx_p,norm_dx_v,tracePprior_pos,tracePpost_pos\n');
                fclose(fid);
            end
        end
        % prepare values
        if isfield(meas,'t')
            tval = meas.t;
        else
            tval = NaN;
        end
        % force column vectors where applicable
        y = y(:);
        dS = diag(S); dS = dS(:);
        dRg = diag(Rg); dRg = dRg(:);
        norm_dx_p = norm(dx(1:3));
        norm_dx_v = norm(dx(4:6));
        tracePprior_pos = trace(P_prior(1:3,1:3));
        tracePpost_pos = trace(P(1:3,1:3));
        % safe scalar extraction with bounds checks
        getv = @(v,i) (numel(v)>=i) * v(i) + (numel(v)<i) * NaN;
        yp1 = getv(y,1); yp2 = getv(y,2); yp3 = getv(y,3);
        yv1 = getv(y,4); yv2 = getv(y,5); yv3 = getv(y,6);
        S1 = getv(dS,1); S2 = getv(dS,2); S3 = getv(dS,3); S4 = getv(dS,4); S5 = getv(dS,5); S6 = getv(dS,6);
        R1 = getv(dRg,1); R2 = getv(dRg,2); R3 = getv(dRg,3); R4 = getv(dRg,4); R5 = getv(dRg,5); R6 = getv(dRg,6);
        % append as fixed-format row (23 values)
        fid = fopen(dbg_file,'a');
        if fid~=-1
            fmt = ['%g,%g,%g,%g,%g,%g,%g,', ...        % t, y_px..y_vz
                   '%g,%g,%g,%g,%g,%g,', ...        % S1..S6
                   '%g,%g,%g,%g,%g,%g,', ...        % R1..R6
                   '%g,%g,%g,%g\n'];                 % norm_dx_p,norm_dx_v,tracePprior_pos,tracePpost_pos
            fprintf(fid,fmt, tval, yp1,yp2,yp3, yv1,yv2,yv3, S1,S2,S3,S4,S5,S6, R1,R2,R3,R4,R5,R6, norm_dx_p, norm_dx_v, tracePprior_pos, tracePpost_pos);
            fclose(fid);
        end
    catch ex
        % avoid throwing from debug logging
        warning('eskf_filter_step:dbg','Failed to write eskf_gps_log.csv: %s', ex.message);
    end
    % --- end debug logging ---
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

function [x_pred, P_pred, x_upd, P_upd, y, S, K, params] = ukf_filter_step(x_prev, P_prev, meas, params)
% Minimal UKF 1-step implementation (for demonstration).
% This is a simplified UKF: it uses standard unscented transform for process
% propagation and then a measurement update using linearized H similar to EKF
% for sensors already implemented in ekf_filter_step.

n = numel(x_prev);
alpha = 1e-3; beta = 2; kappa = 0;
lambda = alpha^2*(n+kappa)-n;
wm = [lambda/(n+lambda); repmat(1/(2*(n+lambda)), 2*n,1)];
wc = wm;
wc(1) = wc(1) + (1-alpha^2+beta);

% sigma points
sqrtP = chol((n+lambda)*P_prev, 'lower');
sig0 = x_prev;
sig = [sig0, x_prev + sqrtP, x_prev - sqrtP];

% process model: use same F and Q as ekf
dt = params.dt;
F = eye(n);
if n>=10
    F(1,3) = dt; F(1,6) = 0.5*dt^2;
    F(2,4) = dt; F(2,7) = 0.5*dt^2;
    F(3,6) = dt; F(4,7) = dt; F(5,8) = dt; F(9,10) = dt;
end
q_a = params.kf.process_noise_accel;
Q = zeros(n); if n>=10, Q(6,6) = (q_a)^2; Q(7,7) = (q_a)^2; Q(8,8) = (q_a*0.1)^2; Q(10,10) = (q_a)^2; end

function [x_pred, P_pred, x_upd, P_upd, y, S, K, params] = ukf_filter_step(x_prev, P_prev, meas, params)
% Full UKF 1-step implementation with measurement function evaluation via sigma points
% Supports same measurement set as ekf_filter_step and performs EMA noise update

n = numel(x_prev);
alpha = 1e-3; beta = 2; kappa = 0;
lambda = alpha^2*(n+kappa)-n;
Wm = [lambda/(n+lambda); repmat(1/(2*(n+lambda)), 2*n,1)];
Wc = Wm; Wc(1) = Wc(1) + (1-alpha^2+beta);

% state transition linear operator (same as EKF)
dt = params.dt;
F = eye(n);
if n>=10
    F(1,3) = dt; F(1,6) = 0.5*dt^2;
    F(2,4) = dt; F(2,7) = 0.5*dt^2;
    F(3,6) = dt; F(4,7) = dt; F(5,8) = dt; F(9,10) = dt;
end
q_a = params.kf.process_noise_accel;
Q = zeros(n);
if n>=10
    Q(6,6) = (q_a)^2; Q(7,7) = (q_a)^2; Q(8,8) = (q_a*0.1)^2; Q(10,10) = (q_a)^2;
end

% form sigma points
sqrtP = chol((n+lambda)*P_prev, 'lower');
sig = zeros(n, 2*n+1);
sig(:,1) = x_prev;
sig(:,2:n+1) = x_prev + sqrtP;
sig(:,n+2:end) = x_prev - sqrtP;

% propagate through process (linear F)
for i=1:size(sig,2)
    sig(:,i) = F * sig(:,i);
end

% predicted state and covariance
x_pred = sig * Wm;
P_pred = Q;
for i=1:size(sig,2)
    d = sig(:,i) - x_pred;
    P_pred = P_pred + Wc(i) * (d * d');
end

% Build measurement mapping function that maps state x to measurement vector
    function [z_vec, h_vec, H_lin, R_mat] = measure_from_state(x_state)
        % replicates ekf_filter_step measurement mapping for a single state
        z_vec = [];
        h_vec = [];
        H_lin = zeros(0,n);
        R_mat = [];
        % gps
        if isfield(meas,'gps') && ~isempty(meas.gps)
            hg = x_state(1:2);
            Hg = zeros(2,n); Hg(1,1)=1; Hg(2,2)=1;
            if isfield(params.noise,'gps')
                gn = params.noise.gps; if numel(gn)==1, gn=[gn,gn]; end
                Rg = diag(gn(:).^2);
            else
                Rg = (params.noise.pos^2)*eye(2);
            end
            z_vec = [z_vec; meas.gps(:)]; h_vec = [h_vec; hg]; H_lin = [H_lin; Hg]; R_mat = blkdiag(R_mat, Rg);
        end
        % vel
        if isfield(meas,'vel') && ~isempty(meas.vel)
            hv = x_state(3:4);
            Hv = zeros(2,n); Hv(1,3)=1; Hv(2,4)=1;
            if isfield(params.noise,'vel')
                vn = params.noise.vel; if numel(vn)==1, vn=[vn,vn]; end
                Rv = diag(vn(:).^2);
            else
                Rv = 0.1*eye(2);
            end
            z_vec = [z_vec; meas.vel(:)]; h_vec = [h_vec; hv]; H_lin = [H_lin; Hv]; R_mat = blkdiag(R_mat, Rv);
        end
        % accel3
        if isfield(meas,'accel3') && ~isempty(meas.accel3)
            th = x_state(5);
            aw = [x_state(6); x_state(7); 0];
            Rwb = [cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 1];
            g = -9.81;
            ha = Rwb * aw + [0;0;g];
            Ha = zeros(3,n);
            Ha(:,6) = Rwb(:,1);
            Ha(:,7) = Rwb(:,2);
            dR_dth = [-sin(th) cos(th) 0; -cos(th) -sin(th) 0; 0 0 0];
            Ha(:,5) = dR_dth * aw;
            if isfield(params.noise,'accel3')
                an = params.noise.accel3(:)'; if numel(an)==1, an = repmat(an,1,3); end
                Ra = diag(an(:).^2);
            else
                Ra = 0.1*eye(3);
            end
            z_vec = [z_vec; meas.accel3(:)]; h_vec = [h_vec; ha]; H_lin = [H_lin; Ha]; R_mat = blkdiag(R_mat, Ra);
        end
        % gyro3
        if isfield(meas,'gyro3') && ~isempty(meas.gyro3)
            zg = meas.gyro3(:);
            if numel(zg) >= 3, zg_use = zg(3); else zg_use = zg(1); end
            hg = x_state(8);
            Hg = zeros(1,n); Hg(8) = 1;
            if isfield(params.noise,'gyro3')
                gn = params.noise.gyro3(:)'; if numel(gn)==1, gn = repmat(gn,1,3); end
                Rg = gn(3)^2;
            else
                Rg = (params.noise.heading^2);
            end
            z_vec = [z_vec; zg_use]; h_vec = [h_vec; hg]; H_lin = [H_lin; Hg]; R_mat = blkdiag(R_mat, Rg);
        end
        % mag3
        if isfield(meas,'mag3') && ~isempty(meas.mag3)
            th = x_state(5);
            mag_field = [1;0;0]; if isfield(params,'sensors') && isfield(params.sensors,'mag_field'), mag_field = params.sensors.mag_field(:); end
            Rz = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
            hm = Rz * mag_field; dRz = [-sin(th) -cos(th) 0; cos(th) -sin(th) 0; 0 0 0];
            Hm = zeros(3,n); Hm(:,5) = dRz * mag_field;
            if isfield(params.noise,'mag3')
                mn = params.noise.mag3(:)'; if numel(mn)==1, mn = repmat(mn,1,3); end
                Rm = diag(mn(:).^2);
            else
                Rm = 0.01*eye(3);
            end
            z_vec = [z_vec; meas.mag3(:)]; h_vec = [h_vec; hm]; H_lin = [H_lin; Hm]; R_mat = blkdiag(R_mat, Rm);
        end
        % baro
        if isfield(meas,'baro') && ~isempty(meas.baro)
            hb = x_state(9); Hb = zeros(1,n); Hb(9) = 1;
            if isfield(params.noise,'baro'), rb = params.noise.baro; else rb = 0.5; end
            z_vec = [z_vec; meas.baro(:)]; h_vec = [h_vec; hb]; H_lin = [H_lin; Hb]; R_mat = blkdiag(R_mat, rb^2);
        end
        % heading
        if isfield(meas,'heading') && ~isempty(meas.heading)
            th = x_state(5);
            hh = [cos(th); sin(th)]; Hh = zeros(2,n); Hh(:,5) = [-sin(th); cos(th)];
            if isfield(params.noise,'heading'), hn = params.noise.heading; R_h = (hn^2)*eye(2); else R_h = 0.05*eye(2); end
            z_vec = [z_vec; meas.heading(:)]; h_vec = [h_vec; hh]; H_lin = [H_lin; Hh]; R_mat = blkdiag(R_mat, R_h);
        end
    end

% Build predicted measurement for each sigma point
nsig = size(sig,2);
Z = [];
for i=1:nsig
    [zi, ~, ~, ~] = measure_from_state(sig(:,i));
    if isempty(zi)
        Z = [];
        break;
    end
    Z(:,i) = zi;
end

if isempty(Z)
    x_upd = x_pred; P_upd = P_pred; y = []; S = []; K = []; return;
end

% predicted measurement mean
z_pred = Z * Wm;

% predicted measurement covariance
zlen = size(Z,1);
S = zeros(zlen);
for i=1:nsig
    dz = Z(:,i) - z_pred;
    S = S + Wc(i) * (dz * dz');
end

% add measurement noise evaluated at x_pred
[~, ~, H_full, R_full] = measure_from_state(x_pred);
if isempty(R_full), R_full = eps*eye(zlen); end
S = S + R_full;

% cross covariance
Pxz = zeros(n, zlen);
for i=1:nsig
    dx = sig(:,i) - x_pred;
    dz = Z(:,i) - z_pred;
    Pxz = Pxz + Wc(i) * (dx * dz');
end

% Kalman gain
K = Pxz / S;

% assemble actual measurement vector z in the same order
z = [];
if isfield(meas,'gps') && ~isempty(meas.gps), z=[z; meas.gps(:)]; end
if isfield(meas,'vel') && ~isempty(meas.vel), z=[z; meas.vel(:)]; end
if isfield(meas,'accel3') && ~isempty(meas.accel3), z=[z; meas.accel3(:)]; end
if isfield(meas,'gyro3') && ~isempty(meas.gyro3), zg = meas.gyro3(:); if numel(zg)>=3, z=[z; zg(3)]; else z=[z; zg(1)]; end; end
if isfield(meas,'mag3') && ~isempty(meas.mag3), z=[z; meas.mag3(:)]; end
if isfield(meas,'baro') && ~isempty(meas.baro), z=[z; meas.baro(:)]; end
if isfield(meas,'heading') && ~isempty(meas.heading), z=[z; meas.heading(:)]; end

y = z - z_pred;
x_upd = x_pred + K * y;
P_upd = P_pred - K * S * K';
P_upd = (P_upd + P_upd')/2;

% EMA-based noise update per measurement block
% build meas_tags same order as above
meas_tags = {};
idx = 1;
if isfield(meas,'gps') && ~isempty(meas.gps), meas_tags{end+1} = struct('name','gps','range',idx:idx+1); idx=idx+2; end
if isfield(meas,'vel') && ~isempty(meas.vel), meas_tags{end+1} = struct('name','vel','range',idx:idx+1); idx=idx+2; end
if isfield(meas,'accel3') && ~isempty(meas.accel3), meas_tags{end+1} = struct('name','accel3','range',idx:idx+2); idx=idx+3; end
if isfield(meas,'gyro3') && ~isempty(meas.gyro3), meas_tags{end+1} = struct('name','gyro3','range',idx:idx); idx=idx+1; end
if isfield(meas,'mag3') && ~isempty(meas.mag3), meas_tags{end+1} = struct('name','mag3','range',idx:idx+2); idx=idx+3; end
if isfield(meas,'baro') && ~isempty(meas.baro), meas_tags{end+1} = struct('name','baro','range',idx:idx); idx=idx+1; end
if isfield(meas,'heading') && ~isempty(meas.heading), meas_tags{end+1} = struct('name','heading','range',idx:idx+1); idx=idx+2; end

HPHT = H_full * P_pred * H_full'; HPHT_diag = diag(HPHT);
R_diag = diag(R_full);
if ~isfield(params,'kf') || ~isfield(params.kf,'ema_alpha'), alpha = 0.01; else alpha = params.kf.ema_alpha; end
if ~isfield(params,'kf') || ~isfield(params.kf,'R_est'), if ~isfield(params,'kf'), params.kf = struct(); end; params.kf.R_est = struct(); end

for i=1:numel(meas_tags)
    tag = meas_tags{i}; rng = tag.range; if isempty(rng), continue; end
    res_sq = (y(rng)).^2;
    hpht_comp = HPHT_diag(rng);
    innov_comp = res_sq - hpht_comp;
    if ~isfield(params.kf.R_est, tag.name), params.kf.R_est.(tag.name) = R_diag(rng); end
    R_prev = params.kf.R_est.(tag.name);
    if numel(R_prev) ~= numel(innov_comp), R_prev = repmat(mean(R_prev), numel(innov_comp), 1); end
    R_new = (1-alpha)*R_prev + alpha * innov_comp;
    R_new(R_new <= 0) = eps;
    params.kf.R_est.(tag.name) = R_new;
    switch tag.name
        case 'gps', params.noise.gps = sqrt(R_new(:));
        case 'vel', params.noise.vel = sqrt(R_new(:));
        case 'accel3', params.noise.accel3 = sqrt(R_new(:));
        case 'gyro3', params.noise.gyro3 = repmat(sqrt(mean(R_new)),1,3);
        case 'mag3', params.noise.mag3 = sqrt(R_new(:));
        case 'baro', params.noise.baro = sqrt(mean(R_new));
        case 'heading', params.noise.heading = sqrt(mean(R_new));
        otherwise, params.noise.(tag.name) = sqrt(R_new(:));
    end
end
end

function [x_pred, P_pred, x_upd, P_upd, y, S, K, params] = ekf_filter_step(x_prev, P_prev, meas, params)
dt = params.dt;
F = eye(10);
F(1,3) = dt; F(1,6) = 0.5*dt^2;
F(2,4) = dt; F(2,7) = 0.5*dt^2;
F(3,6) = dt; F(4,7) = dt; F(5,8) = dt; F(9,10) = dt;
q_a = params.kf.process_noise_accel;
Q = zeros(10);
Q(6,6) = (q_a)^2; Q(7,7) = (q_a)^2; Q(8,8) = (q_a*0.1)^2; Q(10,10) = (q_a)^2;
Q(1,1) = 0.01; Q(2,2) = 0.01; Q(3,3) = 0.01; Q(4,4) = 0.01;
x_pred = F*x_prev;
P_pred = F*P_prev*F' + Q;
z = [];
h = [];
H = zeros(0,10);
R = [];
% record ranges of appended measurement blocks for later adaptive noise updates
meas_tags = {};
if isfield(meas,'gps') && ~isempty(meas.gps)
    zg = meas.gps(:);
    hg = x_pred(1:2);
    Hg = zeros(2,10); Hg(1,1)=1; Hg(2,2)=1;
    if isfield(params.noise,'gps')
        gn = params.noise.gps; if numel(gn)==1, gn = [gn,gn]; end
        Rg = diag(gn(:).^2);
    else
        Rg = (params.noise.pos^2)*eye(2);
    end
    start = numel(z)+1;
    z = [z; zg]; h = [h; hg]; H = [H; Hg]; R = blkdiag(R, Rg);
    idx = start:(start+numel(zg)-1);
    meas_tags{end+1} = struct('name','gps','range',idx);
end
if isfield(meas,'vel') && ~isempty(meas.vel)
    zv = meas.vel(:);
    hv = x_pred(3:4);
    Hv = zeros(2,10); Hv(1,3)=1; Hv(2,4)=1;
    if isfield(params.noise,'vel')
        vn = params.noise.vel; if numel(vn)==1, vn = [vn,vn]; end
        Rv = diag(vn(:).^2);
    else
        Rv = 0.1*eye(2);
    end
    start = numel(z)+1;
    z = [z; zv]; h = [h; hv]; H = [H; Hv]; R = blkdiag(R, Rv);
    idx = start:(start+numel(zv)-1);
    meas_tags{end+1} = struct('name','vel','range',idx);
end
if isfield(meas,'accel3') && ~isempty(meas.accel3)
    za = meas.accel3(:);
    th = x_pred(5);
    aw = [x_pred(6); x_pred(7); 0];
    Rwb = [cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 1];
    g = -9.81;
    ha = Rwb * aw + [0;0;g];
    Ha = zeros(3,10);
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
    start = numel(z)+1;
    z = [z; za]; h = [h; ha]; H = [H; Ha]; R = blkdiag(R, Ra);
    idx = start:(start+numel(za)-1);
    meas_tags{end+1} = struct('name','accel3','range',idx);
end
if isfield(meas,'gyro3') && ~isempty(meas.gyro3)
    zg = meas.gyro3(:);
    if numel(zg) >= 3
        zg_use = zg(3);
    else
        zg_use = zg(1);
    end
    hg = x_pred(8);
    Hg = zeros(1,10); Hg(8) = 1;
    if isfield(params.noise,'gyro3')
        gn = params.noise.gyro3(:)'; if numel(gn)==1, gn = repmat(gn,1,3); end
        Rg = gn(3)^2;
    else
        Rg = (params.noise.heading^2);
    end
    start = numel(z)+1;
    z = [z; zg_use]; h = [h; hg]; H = [H; Hg]; R = blkdiag(R, Rg);
    idx = start:(start+1-1);
    meas_tags{end+1} = struct('name','gyro3','range',idx);
end
if isfield(meas,'mag3') && ~isempty(meas.mag3)
    zm = meas.mag3(:);
    th = x_pred(5);
    mag_field = [1;0;0];
    if isfield(params,'sensors') && isfield(params.sensors,'mag_field')
        mag_field = params.sensors.mag_field(:);
    end
    Rz = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
    hm = Rz * mag_field;
    dRz = [-sin(th) -cos(th) 0; cos(th) -sin(th) 0; 0 0 0];
    Hm = zeros(3,10);
    Hm(:,5) = dRz * mag_field;
    if isfield(params.noise,'mag3')
        mn = params.noise.mag3(:)'; if numel(mn)==1, mn = repmat(mn,1,3); end
        Rm = diag(mn(:).^2);
    else
        Rm = 0.01*eye(3);
    end
    start = numel(z)+1;
    z = [z; zm]; h = [h; hm]; H = [H; Hm]; R = blkdiag(R, Rm);
    idx = start:(start+numel(zm)-1);
    meas_tags{end+1} = struct('name','mag3','range',idx);
end
if isfield(meas,'baro') && ~isempty(meas.baro)
    zb = meas.baro(:);
    hb = x_pred(9);
    Hb = zeros(1,10); Hb(9) = 1;
    if isfield(params.noise,'baro')
        rb = params.noise.baro;
    else
        rb = 0.5;
    end
    start = numel(z)+1;
    z = [z; zb]; h = [h; hb]; H = [H; Hb]; R = blkdiag(R, rb^2);
    idx = start:(start+numel(zb)-1);
    meas_tags{end+1} = struct('name','baro','range',idx);
end
if isfield(meas,'heading') && ~isempty(meas.heading)
    zh = meas.heading(:);
    th = x_pred(5);
    hh = [cos(th); sin(th)];
    Hh = zeros(2,10);
    Hh(:,5) = [-sin(th); cos(th)];
    if isfield(params.noise,'heading')
        hn = params.noise.heading; R_h = (hn^2)*eye(2);
    else
        R_h = 0.05*eye(2);
    end
    start = numel(z)+1;
    z = [z; zh]; h = [h; hh]; H = [H; Hh]; R = blkdiag(R, R_h);
    idx = start:(start+numel(zh)-1);
    meas_tags{end+1} = struct('name','heading','range',idx);
end
if isempty(z)
    x_upd = x_pred; P_upd = P_pred; y = []; S = []; K = [];
    return;
end
y = z - h;
S = H*P_pred*H' + R;
% protect against NaN/Inf in z/h by inflating corresponding R entries so they're ignored
nan_idx = isnan(y) | isinf(y);
if any(nan_idx)
    % set innovation to zero for those entries and inflate R diagonal
    y(nan_idx) = 0;
    rd = diag(S); % use S diag as proxy for size
    large = max(1e6, 1e3*median(rd(rd>0)));
    % find indices of diagonal in R corresponding to nan_idx
    R_diag = diag(R);
    R_diag(nan_idx) = large;
    R = diag(R_diag); % replace R with inflated diag to avoid singularities
    S = H*P_pred*H' + R;
end

% ensure R diagonal positive and finite
R_diag = diag(R);
R_diag(~isfinite(R_diag) | R_diag<=0) = eps;
R = diag(R_diag);
S = H*P_pred*H' + R;

% regularize S if ill-conditioned or singular
n = size(S,1);
if n>0
    r = rcond(S);
    reg_scale = 1e-8; iter = 0;
    % baseline scale from S trace
    base = max(eps, trace(S)/n);
    while (isnan(r) || r < 1e-12) && iter < 8
        reg = (10^iter) * reg_scale * base;
        S = S + reg * eye(n);
        r = rcond(S);
        iter = iter + 1;
    end
    if isnan(r) || r < 1e-12
        warning('ekf_filter_step:IllConditionedS','S is ill-conditioned even after regularization. rcond=%g', r);
    end
    % optional debug print
    if isfield(params,'kf') && isfield(params.kf,'debug') && params.kf.debug
        fprintf('ekf_filter_step: rcond after reg = %g (iter=%d)\n', r, iter);
    end
end

K = P_pred*H'/S;
x_upd = x_pred + K*y;
P_upd = (eye(10)-K*H)*P_pred;

% ------------------------
% EMA-based adaptive noise update
% ------------------------
% alpha: EMA smoothing factor
if ~isfield(params,'kf') || ~isfield(params.kf,'ema_alpha')
    alpha = 0.1;
else
    alpha = params.kf.ema_alpha;
end
% storage for R estimates
if ~isfield(params,'kf') || ~isfield(params.kf,'R_est')
    if ~isfield(params,'kf'), params.kf = struct(); end
    params.kf.R_est = struct();
end

% innovation covariance contribution from state uncertainty
HPHT = H*P_pred*H';
HPHT_diag = diag(HPHT);
R_diag = diag(R);

for i=1:numel(meas_tags)
    tag = meas_tags{i};
    rng = tag.range;
    if isempty(rng), continue; end
    res_sq = (y(rng)).^2;
    hpht_comp = HPHT_diag(rng);
    innov_comp = res_sq - hpht_comp; % approximation of measurement noise
    % initialize previous R if missing
    if ~isfield(params.kf.R_est, tag.name)
        params.kf.R_est.(tag.name) = R_diag(rng);
    end
    R_prev = params.kf.R_est.(tag.name);
    % ensure vectors match
    if numel(R_prev) ~= numel(innov_comp)
        R_prev = repmat(mean(R_prev), numel(innov_comp), 1);
    end
    % EMA update
    R_new = (1-alpha)*R_prev + alpha * innov_comp;
    % clamp to small positive
    R_new(R_new <= 0) = eps;
    params.kf.R_est.(tag.name) = R_new;

    % map back to params.noise as standard deviations (expected by rest of code)
    switch tag.name
        case 'gps'
            params.noise.gps = sqrt(R_new(:));
        case 'vel'
            params.noise.vel = sqrt(R_new(:));
        case 'accel3'
            params.noise.accel3 = sqrt(R_new(:));
        case 'gyro3'
            % code often expects scalar or 3-vector; provide 3-vector with same value
            val = sqrt(mean(R_new));
            params.noise.gyro3 = repmat(val,1,3);
        case 'mag3'
            params.noise.mag3 = sqrt(R_new(:));
        case 'baro'
            params.noise.baro = sqrt(mean(R_new));
        case 'heading'
            params.noise.heading = sqrt(mean(R_new));
        otherwise
            % unknown tag: store generic entry
            params.noise.(tag.name) = sqrt(R_new(:));
    end
end

% return params unchanged (caller expects params as last output)
% (adaptive updates may modify params before returning in future)
% params = params; % explicit no-op to emphasize returned value
end

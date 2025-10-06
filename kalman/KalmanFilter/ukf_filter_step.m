function [x_pred, P_pred, x_upd, P_upd, y, S, K, params] = ukf_filter_step(x_prev, P_prev, meas, params)
% UKF 1-step: sigma-point predict for nonlinearity, then use linear update for measurements
    n = numel(x_prev);
    alpha = 1e-3; beta = 2; kappa = 0;
    lambda = alpha^2*(n+kappa)-n;
    wm = [lambda/(n+lambda); repmat(1/(2*(n+lambda)), 2*n,1)];
    wc = wm; wc(1) = wc(1) + (1-alpha^2+beta);

    % sigma points
    sqrtP = chol((n+lambda)*P_prev, 'lower');
    sig0 = x_prev;
    sig = [sig0, x_prev + sqrtP, x_prev - sqrtP];

    % process propagation: use linear F as default (could be replaced by f(x))
    % If user provided a nonlinear process function in params.kf.process_fn, use it
    for i=1:size(sig,2)
        if isfield(params.kf,'process_fn') && ~isempty(params.kf.process_fn)
            sig(:,i) = params.kf.process_fn(sig(:,i), params);
        else
            % fallback to linear F
            [F,Q] = kf_core.build_F_Q(params, n);
            sig(:,i) = F * sig(:,i);
        end
    end

    x_pred = sig * wm;
    P_pred = zeros(n);
    if isfield(params.kf,'process_noise_Q') && ~isempty(params.kf.process_noise_Q)
        P_pred = params.kf.process_noise_Q;
    end
    for i=1:size(sig,2)
        d = sig(:,i) - x_pred;
        P_pred = P_pred + wc(i) * (d*d');
    end

    % Build simple measurement mapping (reuse ekf handling for sensors like gps/vel)
    z = [];
    h = [];
    H = zeros(0,n);
    R = [];
    if isfield(meas,'gps') && ~isempty(meas.gps)
        zg = meas.gps(:); hg = x_pred(1:2); Hg = zeros(2,n); Hg(1,1)=1; Hg(2,2)=1;
        if isfield(params.noise,'gps')
            gn = params.noise.gps; if numel(gn)==1, gn=[gn,gn]; end; Rg = diag(gn(:).^2);
        else
            Rg = (params.noise.pos^2)*eye(2);
        end
        z=[z;zg]; h=[h;hg]; H=[H;Hg]; R=blkdiag(R,Rg);
    end
    if isfield(meas,'vel') && ~isempty(meas.vel)
        zv = meas.vel(:); hv = x_pred(3:4); Hv=zeros(2,n); Hv(1,3)=1; Hv(2,4)=1;
        if isfield(params.noise,'vel')
            vn = params.noise.vel; if numel(vn)==1, vn=[vn,vn]; end; Rv = diag(vn(:).^2);
        else
            Rv = 0.1*eye(2);
        end
        z=[z;zv]; h=[h;hv]; H=[H;Hv]; R=blkdiag(R,Rv);
    end

    meas_tags = {};
    [x_upd, P_upd, y, S, K, params] = kf_core.linear_update(x_pred, P_pred, z, h, H, R, params, meas_tags);
end

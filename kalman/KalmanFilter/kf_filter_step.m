function [x_pred, P_pred, x_upd, P_upd, y, S, K, params] = kf_filter_step(x_prev, P_prev, meas, params)
% Simple 1-step linear Kalman filter using kf_core helpers
    [x_pred, P_pred, ~, ~] = kf_core.predict_linear(x_prev, P_prev, params);

    % Build measurement vectors (simple: gps and vel only)
    z = [];
    h = [];
    H = zeros(0,numel(x_prev));
    R = [];
    if isfield(meas,'gps') && ~isempty(meas.gps)
        z = [z; meas.gps(:)];
        h = [h; x_pred(1:2)];
        Hg = zeros(2,numel(x_prev)); Hg(1,1)=1; Hg(2,2)=1; H=[H;Hg];
        if isfield(params.noise,'gps')
            gn = params.noise.gps; if numel(gn)==1, gn=[gn,gn]; end
            Rg = diag(gn(:).^2);
        else
            Rg = (params.noise.pos^2)*eye(2);
        end
        R = blkdiag(R,Rg);
    end
    if isfield(meas,'vel') && ~isempty(meas.vel)
        z = [z; meas.vel(:)];
        h = [h; x_pred(3:4)];
        Hv = zeros(2,numel(x_prev)); Hv(1,3)=1; Hv(2,4)=1; H=[H;Hv];
        if isfield(params.noise,'vel')
            vn = params.noise.vel; if numel(vn)==1, vn=[vn,vn]; end
            Rv = diag(vn(:).^2);
        else
            Rv = 0.1*eye(2);
        end
        R = blkdiag(R,Rv);
    end

    meas_tags = {};
    [x_upd, P_upd, y, S, K, params] = kf_core.linear_update(x_pred, P_pred, z, h, H, R, params, meas_tags);
end

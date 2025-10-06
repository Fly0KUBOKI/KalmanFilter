function [x_pred, P_pred, x_upd, P_upd, y, S, K, params] = ekf_filter_step(x_prev, P_prev, meas, params)
% EKF 1-step: build nonlinear h/H per sensor, then call kf_core for predict/update
    [x_pred, P_pred, ~, ~] = kf_core.predict_linear(x_prev, P_prev, params);

    z = [];
    h = [];
    H = zeros(0,numel(x_prev));
    R = [];
    meas_tags = {};

    % gps
    if isfield(meas,'gps') && ~isempty(meas.gps)
        zg = meas.gps(:);
        hg = x_pred(1:2);
        Hg = zeros(2,numel(x_prev)); Hg(1,1)=1; Hg(2,2)=1;
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

    % vel
    if isfield(meas,'vel') && ~isempty(meas.vel)
        zv = meas.vel(:);
        hv = x_pred(3:4);
        Hv = zeros(2,numel(x_prev)); Hv(1,3)=1; Hv(2,4)=1;
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

    % accel3 (nonlinear mapping)
    if isfield(meas,'accel3') && ~isempty(meas.accel3)
        za = meas.accel3(:);
        th = x_pred(5);
        aw = [x_pred(6); x_pred(7); 0];
        Rwb = [cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 1];
        g = -9.81;
        ha = Rwb * aw + [0;0;g];
        Ha = zeros(3,numel(x_prev));
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

    % gyro3
    if isfield(meas,'gyro3') && ~isempty(meas.gyro3)
        zg = meas.gyro3(:);
        if numel(zg) >= 3
            zg_use = zg(3);
        else
            zg_use = zg(1);
        end
        hg = x_pred(8);
        Hg = zeros(1,numel(x_prev)); Hg(8) = 1;
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

    % mag3
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
        Hm = zeros(3,numel(x_prev));
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

    % baro
    if isfield(meas,'baro') && ~isempty(meas.baro)
        zb = meas.baro(:);
        hb = x_pred(9);
        Hb = zeros(1,numel(x_prev)); Hb(9) = 1;
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

    % heading
    if isfield(meas,'heading') && ~isempty(meas.heading)
        zh = meas.heading(:);
        th = x_pred(5);
        hh = [cos(th); sin(th)];
        Hh = zeros(2,numel(x_prev));
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

    [x_upd, P_upd, y, S, K, params] = kf_core.linear_update(x_pred, P_pred, z, h, H, R, params, meas_tags);
end

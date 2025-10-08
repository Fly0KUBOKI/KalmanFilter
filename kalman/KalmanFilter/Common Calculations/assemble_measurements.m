function [z, h, H, R, meas_tags] = assemble_measurements(meas, x_pred, params)
% Assemble measurement vector/matrices from sensor fields in meas
z = [];
h = [];
H = zeros(0, numel(x_pred));
R = [];
meas_tags = {};

% helper to add block
    function add_block(zb, hb, Hb, Rb, name)
        start = numel(z) + 1;
        z = [z; zb];
        h = [h; hb];
        H = [H; Hb];
        R = blkdiag(R, Rb);
        idx = start:(start+numel(zb)-1);
        meas_tags{end+1} = struct('name', name, 'range', idx);
    end

% gps
if isfield(meas,'gps') && ~isempty(meas.gps)
    zg = meas.gps(:);
    hg = x_pred(1:2);
    Hg = zeros(2,numel(x_pred)); Hg(1,1)=1; Hg(2,2)=1;
    if isfield(params.noise,'gps')
        gn = params.noise.gps; if numel(gn)==1, gn = [gn,gn]; end
        Rg = diag(gn(:).^2);
    else
        Rg = (params.noise.pos^2)*eye(2);
    end
    add_block(zg, hg, Hg, Rg, 'gps');
end

% vel
if isfield(meas,'vel') && ~isempty(meas.vel)
    zv = meas.vel(:);
    hv = x_pred(3:4);
    Hv = zeros(2,numel(x_pred)); Hv(1,3)=1; Hv(2,4)=1;
    if isfield(params.noise,'vel')
        vn = params.noise.vel; if numel(vn)==1, vn = [vn,vn]; end
        Rv = diag(vn(:).^2);
    else
        Rv = 0.1*eye(2);
    end
    add_block(zv, hv, Hv, Rv, 'vel');
end

% accel3
if isfield(meas,'accel3') && ~isempty(meas.accel3)
    za = meas.accel3(:);
    th = x_pred(5);
    aw = [x_pred(6); x_pred(7); 0];
    Rwb = [cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 1];
    g = -9.81;
    ha = Rwb * aw + [0;0;g];
    Ha = zeros(3,numel(x_pred));
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
    add_block(za, ha, Ha, Ra, 'accel3');
end

% gyro3 (map to 1D heading sensor in this model)
if isfield(meas,'gyro3') && ~isempty(meas.gyro3)
    zg = meas.gyro3(:);
    if numel(zg) >= 3
        zg_use = zg(3);
    else
        zg_use = zg(1);
    end
    hg = x_pred(8);
    Hg = zeros(1,numel(x_pred)); Hg(1,8) = 1;
    if isfield(params.noise,'gyro3')
        gn = params.noise.gyro3(:)'; if numel(gn)==1, gn = repmat(gn,1,3); end
        Rg = gn(3)^2;
    else
        Rg = (params.noise.heading^2);
    end
    add_block(zg_use, hg, Hg, Rg, 'gyro3');
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
    Hm = zeros(3,numel(x_pred));
    Hm(:,5) = dRz * mag_field;
    if isfield(params.noise,'mag3')
        mn = params.noise.mag3(:)'; if numel(mn)==1, mn = repmat(mn,1,3); end
        Rm = diag(mn(:).^2);
    else
        Rm = 0.01*eye(3);
    end
    add_block(zm, hm, Hm, Rm, 'mag3');
end

% barometer removed: no baro measurement

% heading (cos/sin)
if isfield(meas,'heading') && ~isempty(meas.heading)
    zh = meas.heading(:);
    th = x_pred(5);
    hh = [cos(th); sin(th)];
    Hh = zeros(2,numel(x_pred));
    Hh(:,5) = [-sin(th); cos(th)];
    if isfield(params.noise,'heading')
        hn = params.noise.heading; R_h = (hn^2)*eye(2);
    else
        R_h = 0.05*eye(2);
    end
    add_block(zh, hh, Hh, R_h, 'heading');
end

end

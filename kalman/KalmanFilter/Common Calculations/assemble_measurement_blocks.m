function blocks = assemble_measurement_blocks(meas, x_pred, params)
% Return measurement "blocks" for each available sensor in meas.
% Each block is a struct with fields: name, z, h, H, R, state_idx (indices in full state)
% This mirrors the logic in assemble_measurements.m but does not concatenate blocks.

blocks = {};

    function add(b)
        blocks{end+1} = b;
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
    add(struct('name','gps','z',zg,'h',hg,'H',Hg,'R',Rg,'state_idx',[1,2]));
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
    add(struct('name','vel','z',zv,'h',hv,'H',Hv,'R',Rv,'state_idx',[3,4]));
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
    add(struct('name','accel3','z',za,'h',ha,'H',Ha,'R',Ra,'state_idx',[5,6,7]));
end

% gyro3 (map to single heading-rate measurement as in original)
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
    add(struct('name','gyro3','z',zg_use,'h',hg,'H',Hg,'R',Rg,'state_idx',8));
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
    add(struct('name','mag3','z',zm,'h',hm,'H',Hm,'R',Rm,'state_idx',[5]));
end

% baro
if isfield(meas,'baro') && ~isempty(meas.baro)
    zb = meas.baro(:);
    hb = x_pred(9);
    Hb = zeros(1,numel(x_pred)); Hb(1,9) = 1;
    if isfield(params.noise,'baro')
        rb = params.noise.baro;
    else
        rb = 0.5;
    end
    add(struct('name','baro','z',zb,'h',hb,'H',Hb,'R',rb^2,'state_idx',9));
end

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
    add(struct('name','heading','z',zh,'h',hh,'H',Hh,'R',R_h,'state_idx',5));
end

end

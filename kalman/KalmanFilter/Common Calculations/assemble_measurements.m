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

% gps (position) -> x_pred indices 6:7 (x,y)
if isfield(meas,'gps') && ~isempty(meas.gps)
    zg = meas.gps(:);
    hg = x_pred(6:7);
    Hg = zeros(2,numel(x_pred)); Hg(1,6)=1; Hg(2,7)=1;
    if isfield(params.noise,'gps')
        gn = params.noise.gps; if numel(gn)==1, gn = [gn,gn]; end
        Rg = diag(gn(:).^2);
    else
        Rg = (params.noise.pos^2)*eye(2);
    end
    add_block(zg, hg, Hg, Rg, 'gps');
end

% vel: measurement zv=[vx;vy;vz] -> compute from v and quaternion
if isfield(meas,'vel') && ~isempty(meas.vel)
    zv = meas.vel(:);
    v = x_pred(1);
    q = x_pred(2:5);
    qw=q(1); qx=q(2); qy=q(3); qz=q(4);
    R = [1-2*(qy^2+qz^2), 2*(qx*qy- qz*qw), 2*(qx*qz+ qy*qw);
         2*(qx*qy+ qz*qw), 1-2*(qx^2+qz^2), 2*(qy*qz- qx*qw);
         2*(qx*qz- qy*qw), 2*(qy*qz+ qx*qw), 1-2*(qx^2+qy^2)];
    hv = R * ([1;0;0] * v);
    Hv = zeros(numel(hv), numel(x_pred));
    % partials wrt v
    Hv(:,1) = R * [1;0;0];
    % partials wrt quaternion are non-trivial; omit for minimal implementation
    if isfield(params.noise,'vel')
        vn = params.noise.vel; if numel(vn)==1, vn = [vn,vn,vn]; end
        Rv = diag(vn(:).^2);
    else
        Rv = 0.1*eye(numel(hv));
    end
    add_block(zv, hv, Hv, Rv, 'vel');
end

% accel3: skip detailed mapping; ESKF/IMU handles IMU fusion. If accel present and user
% expects direct mapping to v, that should be handled in assemble_measurements explicitly.

% gyro3: IMU handled by ESKF; do not add as direct measurement in this minimal model.

% mag3
if isfield(meas,'mag3') && ~isempty(meas.mag3)
    zm = meas.mag3(:);
    th = x_pred(4);  % theta now at index 4
    mag_field = [1;0;0];
    if isfield(params,'sensors') && isfield(params.sensors,'mag_field')
        mag_field = params.sensors.mag_field(:);
    end
    Rz = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
    hm = Rz * mag_field;
    dRz = [-sin(th) -cos(th) 0; cos(th) -sin(th) 0; 0 0 0];
    Hm = zeros(3,numel(x_pred));
    Hm(:,4) = dRz * mag_field;  % dh/dtheta now at index 4
    if isfield(params.noise,'mag3')
        mn = params.noise.mag3(:)'; if numel(mn)==1, mn = repmat(mn,1,3); end
        Rm = diag(mn(:).^2);
    else
        Rm = 0.01*eye(3);
    end
    add_block(zm, hm, Hm, Rm, 'mag3');
end

% baro (altitude) -> z is index 8
if isfield(meas,'baro') && ~isempty(meas.baro)
    zb = meas.baro(:);
    hb = x_pred(8);
    Hb = zeros(1,numel(x_pred)); Hb(1,8) = 1;
    if isfield(params.noise,'baro')
        rb = params.noise.baro;
    else
        rb = 0.5;
    end
    add_block(zb, hb, Hb, rb^2, 'baro');
end

% heading: omit - quaternion holds orientation; if a heading sensor is provided,
% higher-level code can convert quaternion to yaw and compare.


end

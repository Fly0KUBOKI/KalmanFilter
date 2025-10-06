function [x_pred, P_pred, x_upd, P_upd, y, S, K, params] = kf_filter_step(x_prev, P_prev, meas, params)
% Simple 1-step linear Kalman filter matching EKF state layout
% Assumes same F and Q construction as ekf_filter_step for process model
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

% Build linear measurement matrix from meas similar to ekf_filter_step
z = [];
h = [];
H = zeros(0,10);
R = [];
if isfield(meas,'gps') && ~isempty(meas.gps)
    z = [z; meas.gps(:)];
    h = [h; x_pred(1:2)];
    Hg = zeros(2,10); Hg(1,1)=1; Hg(2,2)=1; H=[H;Hg];
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
    Hv = zeros(2,10); Hv(1,3)=1; Hv(2,4)=1; H=[H;Hv];
    if isfield(params.noise,'vel')
        vn = params.noise.vel; if numel(vn)==1, vn=[vn,vn]; end
        Rv = diag(vn(:).^2);
    else
        Rv = 0.1*eye(2);
    end
    R = blkdiag(R,Rv);
end
% other sensors omitted for brevity; fallback to ekf_filter_step for full sensors
if isempty(z)
    x_upd = x_pred; P_upd = P_pred; y=[]; S=[]; K=[]; return;
end

y = z - h;
S = H*P_pred*H' + R;
K = P_pred*H'/S;
x_upd = x_pred + K*y;
P_upd = (eye(10)-K*H)*P_pred;
end

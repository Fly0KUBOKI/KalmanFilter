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

% propagate sigma points
for i=1:size(sig,2)
    sig(:,i) = F*sig(:,i); % no process nonlinearity here
end
x_pred = sig*wm;
P_pred = Q;
for i=1:size(sig,2)
    d = sig(:,i) - x_pred;
    P_pred = P_pred + wc(i)*(d*d');
end

% build measurement vectors using ekf-style observation mapping
z = [];
h = [];
H = zeros(0,n);
R = [];
% reuse logic from ekf: gps and vel handled here for simplicity
if isfield(meas,'gps') && ~isempty(meas.gps)
    zg = meas.gps(:);
    hg = x_pred(1:2);
    Hg = zeros(2,n); Hg(1,1)=1; Hg(2,2)=1;
    if isfield(params.noise,'gps')
        gn = params.noise.gps; if numel(gn)==1, gn=[gn,gn]; end
        Rg = diag(gn(:).^2);
    else
        Rg = (params.noise.pos^2)*eye(2);
    end
    z=[z;zg]; h=[h;hg]; H=[H;Hg]; R=blkdiag(R,Rg);
end
if isfield(meas,'vel') && ~isempty(meas.vel)
    zv = meas.vel(:);
    hv = x_pred(3:4);
    Hv=zeros(2,n); Hv(1,3)=1; Hv(2,4)=1;
    if isfield(params.noise,'vel')
        vn = params.noise.vel; if numel(vn)==1, vn=[vn,vn]; end
        Rv = diag(vn(:).^2);
    else
        Rv = 0.1*eye(2);
    end
    z=[z;zv]; h=[h;hv]; H=[H;Hv]; R=blkdiag(R,Rv);
end

if isempty(z)
    x_upd = x_pred; P_upd = P_pred; y=[]; S=[]; K=[]; return;
end

y = z - h;
S = H*P_pred*H' + R;
K = P_pred*H'/S;
x_upd = x_pred + K*y;
P_upd = (eye(n)-K*H)*P_pred;
end

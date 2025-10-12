function [x_pred, P_pred] = predict_state_covariance(x_prev, P_prev, params)
% Predict state mean and covariance (minimal Jacobian) for state [v q1 q2 q3 q4 x y z]

% mean
x_pred = predict_state(x_prev, params);

% covariance: approximate Jacobian assuming v and quaternion are constant in predict_state
n = numel(x_prev);
dt = params.dt;
F = eye(n);

% position depends on v and quaternion; here we approximate partials w.r.t v only (minimal)
if n >= 8
    % dpos/dv ~ R * body_forward * dt (use current quaternion)
    q = x_prev(2:5); qw=q(1); qx=q(2); qy=q(3); qz=q(4);
    R = [1-2*(qy^2+qz^2), 2*(qx*qy- qz*qw), 2*(qx*qz+ qy*qw);
         2*(qx*qy+ qz*qw), 1-2*(qx^2+qz^2), 2*(qy*qz- qx*qw);
         2*(qx*qz- qy*qw), 2*(qy*qz+ qx*qw), 1-2*(qx^2+qy^2)];
    body_forward = [1;0;0];
    dp_dv = R * body_forward * dt; % 3x1
    % map into F for x,y,z rows and v column
    F(6,1) = dp_dv(1);
    F(7,1) = dp_dv(2);
    F(8,1) = dp_dv(3);
end

% process noise: small uncertainty for v and position; quaternion process noise small
q_v = params.kf.process_noise_accel;
Q = zeros(n);
if n >= 8
    Q(1,1) = (q_v)^2;       % v process noise
    Q(2,2) = 1e-4; Q(3,3)=1e-4; Q(4,4)=1e-4; Q(5,5)=1e-4; % quaternion small noise
    Q(6,6) = 0.01; Q(7,7)=0.01; Q(8,8)=0.01; % position noise
end

P_pred = F * P_prev * F' + Q;
end

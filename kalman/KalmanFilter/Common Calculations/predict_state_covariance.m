function [x_pred, P_pred] = predict_state_covariance(x_prev, P_prev, params)
% Predict state mean and covariance using a linear model
% Delegates mean propagation to predict_state for consistency with UKF

% mean
x_pred = predict_state(x_prev, params);

% covariance
n = numel(x_prev);
dt = params.dt;
F = eye(n);
% support 8-state model: [x y vx vy theta ax ay omega]
if n >= 8
    F(1,3) = dt; F(1,6) = 0.5*dt^2;
    F(2,4) = dt; F(2,7) = 0.5*dt^2;
    F(3,6) = dt; F(4,7) = dt; F(5,8) = dt;
end

q_a = params.kf.process_noise_accel;
Q = zeros(n);
if n >= 8
    Q(6,6) = (q_a)^2; Q(7,7) = (q_a)^2; Q(8,8) = (q_a*0.1)^2;
    Q(1,1) = 0.01; Q(2,2) = 0.01; Q(3,3) = 0.01; Q(4,4) = 0.01;
else
    Q = eye(n) * (q_a^2);
end

P_pred = F * P_prev * F' + Q;
end

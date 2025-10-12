function x_pred = predict_state(x, params)
% Predict the next state mean (minimal model) for state:
% [v q1 q2 q3 q4 x y z]
% Minimal dynamics: keep v and quaternion constant (IMU integration handled by ESKF);
% update position by moving along the body-forward axis rotated by quaternion.

dt = params.dt;
n = numel(x);
x_pred = x;

if n >= 8
    v = x(1);
    q = x(2:5); % quaternion [w x y z]
    pos = x(6:8);

    % rotation matrix from quaternion (world = R_body_to_world * body)
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    R = [1-2*(qy^2+qz^2), 2*(qx*qy- qz*qw), 2*(qx*qz+ qy*qw);
         2*(qx*qy+ qz*qw), 1-2*(qx^2+qz^2), 2*(qy*qz- qx*qw);
         2*(qx*qz- qy*qw), 2*(qy*qz+ qx*qw), 1-2*(qx^2+qy^2)];

    % body-forward vector (assume forward along x axis in body frame)
    body_forward = [1; 0; 0];
    world_vel = R * (body_forward * v); % [vx; vy; vz]

    % update position
    pos_new = pos + world_vel * dt;

    % keep v and quaternion unchanged here (ESKF will integrate IMU)
    x_pred(1) = v;
    x_pred(2:5) = q;
    x_pred(6:8) = pos_new;
end
end

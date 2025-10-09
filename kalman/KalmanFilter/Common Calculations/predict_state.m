function x_pred = predict_state(x, params)
% Predict the next state mean using a simple linear process model.
% This single function replaces previous duplicate definitions and
% avoids function name conflicts inside the file.

dt = params.dt;
n = numel(x);
F = eye(n);
if n >= 10
    % Standard kinematic model with acceleration
    F(1,3) = dt; F(1,6) = 0.5*dt^2;
    F(2,4) = dt; F(2,7) = 0.5*dt^2;
    F(3,6) = dt; F(4,7) = dt; F(5,8) = dt; F(9,10) = dt;
    
    % Additional coupling: use current heading (theta) to refine velocity direction
    % This helps when velocity sensors have drift but heading is accurate
    % Apply small correction based on heading: if |v| and theta are known,
    % the velocity direction should align with theta
    theta = x(5);
    vx = x(3); vy = x(4);
    v_mag = sqrt(vx^2 + vy^2);
    if v_mag > 0.1  % only apply when moving
        % Expected velocity direction from heading
        vx_expected = v_mag * cos(theta);
        vy_expected = v_mag * sin(theta);
        % Blend current velocity with heading-based direction (small correction weight)
        alpha = 0.1;  % weight for heading-based correction
        x(3) = (1-alpha)*vx + alpha*vx_expected;
        x(4) = (1-alpha)*vy + alpha*vy_expected;
    end
end

x_pred = F * x;
end

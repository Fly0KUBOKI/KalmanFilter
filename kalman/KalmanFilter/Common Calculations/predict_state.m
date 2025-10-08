function x_pred = predict_state(x, params)
% Predict the next state mean using a simple linear process model.
% This single function replaces previous duplicate definitions and
% avoids function name conflicts inside the file.

dt = params.dt;
n = numel(x);
F = eye(n);
if n >= 10
    F(1,3) = dt; F(1,6) = 0.5*dt^2;
    F(2,4) = dt; F(2,7) = 0.5*dt^2;
    F(3,6) = dt; F(4,7) = dt; F(5,8) = dt; F(9,10) = dt;
end

x_pred = F * x;
end

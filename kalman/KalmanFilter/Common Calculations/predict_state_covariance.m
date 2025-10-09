function [x_pred, P_pred] = predict_state_covariance(x_prev, P_prev, params)
% Predict state mean and covariance using a linear model
% Uses build_process_model for F and Q construction

n = numel(x_prev);

% Get F and Q from params if provided, otherwise build them
if isfield(params, 'kf') && isfield(params.kf, 'F') && isfield(params.kf, 'Q')
    F = params.kf.F;
    Q = params.kf.Q;
else
    % Build F and Q using helper
    dt = params.dt;
    [F, Q] = build_process_model(n, dt, params);
end

% Mean prediction
x_pred = F * x_prev;

% Covariance prediction
P_pred = F * P_prev * F' + Q;

end

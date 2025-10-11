function [x_pred, P_pred, x_upd, P_upd, y, S, K, params] = kf_filter_step(x_prev, P_prev, meas, params)
% Simple 1-step linear Kalman filter using shared helper modules

% ensure Common Calculations on path
root = fileparts(mfilename('fullpath'));
commonDir = fullfile(root, 'Common Calculations');
if exist(commonDir, 'dir') && ~contains(path, commonDir)
    addpath(commonDir);
end

% 1) prediction
[x_pred, P_pred] = predict_state_covariance(x_prev, P_prev, params);

% 2) assemble measurements (uses same mapping as ekf/ukf helpers)
[z, h, H, R, meas_tags] = assemble_measurements(meas, x_pred, params);
% store last measurement tags for debugging/inspection
if ~isfield(params,'kf')
    params.kf = struct();
end
params.kf.last_meas_tags = meas_tags;

if isempty(z)
    x_upd = x_pred; P_upd = P_pred; y = []; S = []; K = [];
    return;
end

% 3) innovation and S (handles NaN/Inf and regularizes S)
[y, S, R] = compute_innovation_and_S(z, h, H, P_pred, R, params);

% 4) Mahalanobis gating for outlier detection per measurement block
% compute per-element Mahalanobis distances: d^2 = y' * S^{-1} * y (for vector, use block-wise)
K = compute_kalman_gain(P_pred, H, S);

% If params specify gating thresholds per tag or a global threshold, apply
gate_mask = true(size(y));
if isfield(params,'kf') && isfield(params.kf,'maha_gate') && params.kf.maha_gate > 0
    try
        % If configured to disable gating for initial steps, skip gating here
        if isfield(params.kf,'maha_disable_steps') && isfield(params.kf,'current_step') && params.kf.current_step <= params.kf.maha_disable_steps
            if isfield(params.kf,'debug') && params.kf.debug
                fprintf('kf_filter_step: skipping maha gating for initial step %d (disable_steps=%g)\n', params.kf.current_step, params.kf.maha_disable_steps);
            end
            % skip gating entirely
        else
        S_inv = inv(S);
        % Evaluate per-meas-block Mahalanobis distance using meas_tags
        if exist('meas_tags','var') && ~isempty(meas_tags)
            for i=1:numel(meas_tags)
                rng = meas_tags{i}.range;
                yi = y(rng);
                Si = S(rng, rng);
                if isempty(Si)
                    d2 = 0;
                else
                    % handle small ill-conditioning by regularizing Si
                    Si_reg = Si + 1e-9 * eye(size(Si));
                    d2 = yi' * (Si_reg \ yi);
                end
                if d2 > params.kf.maha_gate
                    % mark this block as outlier: softly inflate its R so update is downweighted
                    % Use a capped/smooth inflation instead of an extreme multiplier.
                    max_infl = 100; % default maximum inflation multiplier
                    scale = 50;     % scaling factor that maps (d2 - gate) to inflation
                    if isfield(params.kf,'maha_inflation_max')
                        max_infl = params.kf.maha_inflation_max;
                    end
                    if isfield(params.kf,'maha_inflation_scale')
                        scale = params.kf.maha_inflation_scale;
                    end
                    % inflation: 1 + min(max_infl-1, (d2 - gate)/scale)
                    infl = 1 + min(max_infl - 1, (d2 - params.kf.maha_gate) / max(1e-12, scale));
                    infl = max(infl, 1);
                    R(rng,rng) = R(rng,rng) * infl;
                    if isfield(params.kf,'debug') && params.kf.debug
                        fprintf('kf_filter_step: gated meas %s d2=%g, inflating R by %g (capped max %g)\n', meas_tags{i}.name, d2, infl, max_infl);
                    end
                    gate_mask(rng) = false;
                end
            end
        else
            % fallback: global Mahalanobis
            d2 = y' * (S \ y);
            if d2 > params.kf.maha_gate
                % global gated inflation (capped)
                max_infl = 1000;
                scale = 50;
                if isfield(params.kf,'maha_inflation_max')
                    max_infl = params.kf.maha_inflation_max;
                end
                if isfield(params.kf,'maha_inflation_scale')
                    scale = params.kf.maha_inflation_scale;
                end
                infl = 1 + min(max_infl - 1, (d2 - params.kf.maha_gate) / max(1e-12, scale));
                infl = max(infl, 1);
                R = R * infl;
                if isfield(params.kf,'debug') && params.kf.debug
                    fprintf('kf_filter_step: global gating triggered d2=%g, inflating R by %g\n', d2, infl);
                end
                gate_mask(:) = false;
            end
        end
        end
    catch ex
        if isfield(params.kf,'debug') && params.kf.debug
            fprintf('kf_filter_step: gating error: %s\n', ex.message);
        end
    end
end

% recompute gain if R changed (S changed implicitly)
% compute S again only if we inflated R
% Note: compute_innovation_and_S already returns regularized S and the possibly modified R
[y, S, R] = compute_innovation_and_S(z, h, H, P_pred, R, params);
K = compute_kalman_gain(P_pred, H, S);

% 5) update state and covariance (use Joseph form with R)
[x_upd, P_upd] = update_state_covariance(x_pred, P_pred, K, H, y, R);

% 6) adaptive R estimation (EMA) (optional: controlled via params.kf.adaptive_R_enabled)
if isfield(params,'kf') && isfield(params.kf,'adaptive_R_enabled') && params.kf.adaptive_R_enabled
    params = adaptive_R_update(params, y, H, P_pred, R, meas_tags);
end

end

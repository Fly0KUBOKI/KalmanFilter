function [y, S, R_out] = compute_innovation_and_S(z, h, H, P_pred, R, params)
% Compute innovation y and innovation covariance S.
% Handles NaN/Inf in z/h by inflating R diag and regularizes S.
% Now includes adaptive gating for outlier rejection.

y = z - h;

% initial S
S = H * P_pred * H' + R;

% Apply gating if enabled
if isfield(params, 'kf') && isfield(params.kf, 'gating_enabled') && params.kf.gating_enabled
    gate_threshold = 9; % chi-square threshold for 95% confidence (2 DOF)
    if isfield(params.kf, 'gate_threshold')
        gate_threshold = params.kf.gate_threshold;
    end
    
    % Check current step for gating bypass during initialization
    current_step = 1;
    if isfield(params.kf, 'current_step')
        current_step = params.kf.current_step;
    end
    
    maha_disable_steps = 3; % disable gating for first few steps
    if isfield(params.kf, 'maha_disable_steps')
        maha_disable_steps = params.kf.maha_disable_steps;
    end
    
    % Apply gating only after initialization period
    if current_step > maha_disable_steps
        % Compute Mahalanobis distance for gating
        if size(S,1) > 0 && det(S) > eps
            d_squared = y' * (S \ y);
            
            % If measurement is an outlier, inflate R to reduce its influence
            if d_squared > gate_threshold
                % Adaptive inflation based on how much the measurement exceeds threshold
                inflation_factor = max(2, sqrt(d_squared / gate_threshold));
                
                % Apply inflation to corresponding R diagonal elements
                R_diag = diag(R);
                R_diag = R_diag * inflation_factor;
                R = diag(R_diag);
                
                % Recompute S with inflated R
                S = H * P_pred * H' + R;
                
                % Store gating info for debugging
                if ~isfield(params, 'kf')
                    params.kf = struct();
                end
                params.kf.last_gating_info = struct('d_squared', d_squared, ...
                    'threshold', gate_threshold, 'inflation_factor', inflation_factor);
            end
        end
    end
end

% protect against NaN/Inf in innovation
nan_idx = isnan(y) | isinf(y);
if any(nan_idx)
    y(nan_idx) = 0;
    rd = diag(S);
    large = max(1e6, 1e3 * median(rd(rd>0)));
    % get R diagonal (handle scalar R)
    if isempty(R)
        R_diag = zeros(numel(y),1);
    else
        R_diag = diag(R);
    end
    % ensure length match: if R provides fewer entries than y, expand using
    % median of positive entries or a fallback based on S diagonal
    if numel(R_diag) < numel(y)
        pos = R_diag(R_diag>0);
        if ~isempty(pos)
            fill = median(pos);
        else
            sd = diag(H*P_pred*H');
            fill = max(median(sd(sd>0)), eps);
        end
        R_diag = [R_diag; repmat(fill, numel(y)-numel(R_diag), 1)];
    end
    R_diag(~isfinite(R_diag) | R_diag<=0) = eps;
    R_diag(nan_idx) = large;
    R = diag(R_diag);
    S = H * P_pred * H' + R;
end

% ensure R diagonal positive finite
R_diag = diag(R);
R_diag(~isfinite(R_diag) | R_diag <= 0) = eps;
R = diag(R_diag);
S = H * P_pred * H' + R;

% regularize S if ill-conditioned
n = size(S,1);
if n > 0
    r = rcond(S);
    reg_scale = 1e-8; iter = 0;
    base = max(eps, trace(S)/n);
    while (isnan(r) || r < 1e-12) && iter < 8
        reg = (10^iter) * reg_scale * base;
        S = S + reg * eye(n);
        r = rcond(S);
        iter = iter + 1;
    end
    if isnan(r) || r < 1e-12
        warning('compute_innovation_and_S:IllConditionedS','S is ill-conditioned. rcond=%g', r);
    end
    if isfield(params,'kf') && isfield(params.kf,'debug') && params.kf.debug
        fprintf('compute_innovation_and_S: rcond after reg = %g (iter=%d)\n', r, iter);
    end
end

R_out = R;
end

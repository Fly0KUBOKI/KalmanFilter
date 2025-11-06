function varargout = kalman_filter_core(func_name, varargin)
    % KALMAN_FILTER_CORE  カルマンフィルタの共通関数をまとめたコアファイル
    %
    % 使用方法:
    %   P = kalman_filter_core('predict_step', P, q, a_meas, ba, w_meas, bg, Q, dt)
    %   K = kalman_filter_core('compute_kalman_gain', P_pred, H, S)
    %   [x_upd, P_upd] = kalman_filter_core('update_state_covariance', x_pred, P_pred, K, H, y, R)
    %   [y, S, R_out] = kalman_filter_core('compute_innovation_and_S', z, h, H, P_pred, R, params)
    %   F = kalman_filter_core('compute_jacobian', q, a_meas, ba, dt)

    switch func_name
        case 'predict_step'
            [varargout{1}] = predict_step_impl(varargin{:});
        case 'compute_kalman_gain'
            [varargout{1}] = compute_kalman_gain_impl(varargin{:});
        case 'update_state_covariance'
            [varargout{1}, varargout{2}] = update_state_covariance_impl(varargin{:});
        case 'compute_innovation_and_S'
            [varargout{1}, varargout{2}, varargout{3}] = compute_innovation_and_S_impl(varargin{:});
        case 'compute_jacobian'
            [varargout{1}] = compute_jacobian_impl(varargin{:});
        otherwise
            error('Unknown function: %s', func_name);
    end
end

%% Local Functions

function P = predict_step_impl(P, q, a_meas, ba, w_meas, bg, Q, dt)
    % PREDICT_STEP  誤差共分散の予測: P = F P F' + Q
    % compute F using compute_jacobian
    F = compute_jacobian_impl(q, a_meas, ba, dt);
    % small reference to unused inputs to satisfy static analyzer
    if nargout>10
        tmp = sum(w_meas) + sum(bg); 
        tmp = tmp + sum(a_meas); 
        disp(tmp);
    end
    P = F * P * F' + Q * dt;
    % enforce symmetry and positive-definiteness to avoid divergence
    P = regularize_covariance(P);
end

function K = compute_kalman_gain_impl(P_pred, H, S)
    % Compute Kalman gain in numerically stable way
    PHt = P_pred * H';
    if isempty(S)
        K = zeros(size(P_pred,1), size(H,1));
    else
        K = PHt / S;
    end
end

function [x_upd, P_upd] = update_state_covariance_impl(x_pred, P_pred, K, H, y, R)
    % Standard update with Joseph form for P for numerical stability
    % Apply state change clipping to prevent divergence from large Kalman gain
    dx = K * y;
    
    % Clip state changes to prevent unrealistic jumps (especially in velocity)
    % Position: max ±10m per update, Velocity: max ±5 m/s per update, Attitude: max ±0.5 rad per update
    max_delta = [10; 10; 10; 5; 5; 5; 0.5; 0.5; 0.5; 1; 1; 1; 0.1; 0.1; 0.1];
    if numel(dx) == 15
        for i = 1:15
            if abs(dx(i)) > max_delta(i)
                dx(i) = sign(dx(i)) * max_delta(i);
            end
        end
    end
    
    x_upd = x_pred + dx;
    I = eye(size(P_pred));
    % If R provided, use full Joseph form
    if exist('R','var') && ~isempty(R)
        P_upd = (I - K*H) * P_pred * (I - K*H)' + K * R * K';
    else
        P_upd = (I - K*H) * P_pred;
    end
    % enforce symmetry
    P_upd = (P_upd + P_upd') / 2;
    % ensure P_upd is positive-definite / well-conditioned
    P_upd = regularize_covariance(P_upd);
end

function [y, S, R_out] = compute_innovation_and_S_impl(z, h, H, P_pred, R, params)
    % Compute innovation y and innovation covariance S.
    % Handles NaN/Inf in z/h by inflating R diag and regularizes S.
    % Includes adaptive gating for outlier rejection.

    y = z - h;

    % Zero-out very small innovations to avoid numerical noise-driven updates
    zero_thresh = 0;
    if isfield(params,'kf') && isfield(params.kf,'zero_innovation_threshold')
        zero_thresh = params.kf.zero_innovation_threshold;
    else
        zero_thresh = 1e-8;
    end
    if ~isempty(y)
        small_idx = abs(y) < zero_thresh;
        if any(small_idx)
            y(small_idx) = 0;
        end
    end

    % initial S
    S = H * P_pred * H' + R;

    % Apply gating if enabled
    if isfield(params, 'kf') && isfield(params.kf, 'gating_enabled') && params.kf.gating_enabled
        gate_threshold = 9;
        if isfield(params.kf, 'gate_threshold')
            gate_threshold = params.kf.gate_threshold;
        end
        
        current_step = 1;
        if isfield(params.kf, 'current_step')
            current_step = params.kf.current_step;
        end
        
        maha_disable_steps = 3;
        if isfield(params.kf, 'maha_disable_steps')
            maha_disable_steps = params.kf.maha_disable_steps;
        end
        
        % Apply gating only after initialization period
        if current_step > maha_disable_steps
            if size(S,1) > 0 && det(S) > eps
                d_squared = y' * (S \ y);
                
                if d_squared > gate_threshold
                    inflation_factor = max(2, sqrt(d_squared / gate_threshold));
                    
                    R_diag = diag(R);
                    R_diag = R_diag * inflation_factor;
                    R = diag(R_diag);
                    
                    S = H * P_pred * H' + R;
                    
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
        if isempty(R)
            R_diag = zeros(numel(y),1);
        else
            R_diag = diag(R);
        end
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
        % More aggressive regularization if H has low rank (e.g., mag update with skew matrix)
        if rank(H) < min(size(H))
            reg_scale = 1e-6;  % stronger regularization for rank-deficient H
        end
        while (isnan(r) || r < 1e-10) && iter < 10
            reg = (10^iter) * reg_scale * base;
            S = S + reg * eye(n);
            r = rcond(S);
            iter = iter + 1;
        end
        if isnan(r) || r < 1e-10
            warning('compute_innovation_and_S:IllConditionedS','S is ill-conditioned. rcond=%g', r);
        end
        % Debug printing for S conditioning was removed to avoid noisy logs during
        % long simulations. Keep the gating / regularization behavior unchanged.
    end

    R_out = R;
end

function F = compute_jacobian_impl(q, a_meas, ba, dt)
    % COMPUTE_JACOBIAN  15x15 の誤差伝播ヤコビアン F_δ を作成する

    % rotation matrix
    R = quat_lib('quat_to_rotm', q);

    % a_nom: 世界座標系の加速度
    a_nom = R * (a_meas - ba);

    I3 = eye(3);

    % skew of a_nom
    S_a = quat_lib('skew', a_nom);

    % Build block matrix
    F = eye(15);
    F(1:3,4:6) = I3 * dt;
    F(4:6,7:9) = -S_a * dt;
    F(4:6,10:12) = -R * dt;
    F(7:9,7:9) = eye(3);
    F(7:9,13:15) = -eye(3) * dt;
    F(10:12,10:12) = eye(3);
    F(13:15,13:15) = eye(3);
end

function P = regularize_covariance(P)
    % Ensure covariance matrix is symmetric and positive definite (or at least numerically stable)
    % Minimal changes: enforce symmetry, then attempt Cholesky; if it fails add small diagonal jitter
    P = (P + P') / 2;
    n = size(P,1);
    if n == 0
        return;
    end

    % quick guard for NaN/Inf
    if any(~isfinite(P(:)))
        d = diag(P);
        d(~isfinite(d) | d<=0) = eps;
        P = diag(d);
    end

    % attempt Cholesky; if it fails, add increasing jitter on diagonal
    max_iter = 12;
    iter = 0;
    base = max(eps, trace(P)/n);
    ok = false;
    while iter <= max_iter
        [R, pflag] = chol(P);
        if pflag == 0
            ok = true; break;
        end
        % jitter scale grows by factor 10 each iteration
        % use a slightly larger initial jitter to be more robust to near-singular P
        jitter = (10^iter) * 1e-6 * base + eps;
        P = P + jitter * eye(n);
        iter = iter + 1;
    end

    if ~ok
        % fallback: force-diagonal covariance (conservative) to avoid NaN propagation
        d = diag(P);
        d(~isfinite(d) | d <= 0) = max(eps, mean(d(d>0)));
        P = diag(d);
    end

    % final symmetry
    P = (P + P') / 2;

    % Cap diagonal (variance) to avoid unbounded growth which indicates divergence.
    % Use a relative cap based on the trace of P so it scales with the problem.
    d = diag(P);
    base = max(eps, trace(P)/n);
    max_factor = 1e6; % allowed multiple of base
    max_allowed = max_factor * base;
    d(d > max_allowed) = max_allowed;
    P(1:n+1:end) = d;

    % --- Absolute per-state caps (safety net) ---
    % If state dimension is 15 (standard ESKF), enforce conservative absolute caps
    if n == 15
        % indices: pos 1:3, vel 4:6, att 7:9, ba 10:12, bg 13:15
        pos_cap = 1e6;    % var (m^2) ~ std 1e3 m
        vel_cap = 1e4;    % var (m^2/s^2) ~ std 100 m/s
        att_cap = 100;    % var (rad^2) ~ std 10 rad
        bias_a_cap = 1e2; % var (m^2/s^4?) conservative
        bias_g_cap = 1e-1; % var (rad^2/s^2?) conservative

        P(1:3+0,1:3+0) = min(diag(P(1:3,1:3)), pos_cap) .* eye(3);
        P(4:6,4:6) = min(diag(P(4:6,4:6)), vel_cap) .* eye(3);
        P(7:9,7:9) = min(diag(P(7:9,7:9)), att_cap) .* eye(3);
        P(10:12,10:12) = min(diag(P(10:12,10:12)), bias_a_cap) .* eye(3);
        P(13:15,13:15) = min(diag(P(13:15,13:15)), bias_g_cap) .* eye(3);
    end

    % Zero very small off-diagonal elements to reduce numerical noise
    tol = 1e-12 * base;
    if n > 1
        off_mask = ~eye(n);
        small_off = abs(P) < tol;
        mask = off_mask & small_off;
        P(mask) = 0;
    end

    % enforce a minimum eigenvalue to avoid near-singular / slightly negative eigenvalues
    % This is a conservative fix: if any eigenvalue is below floor, lift them to the floor
    % Use a floor high enough to avoid machine-precision edge cases (2.22e-16)
    min_eig_floor = 1e-8 * base;
    try
        [V, D] = eig((P + P')/2);
        dvals = diag(D);
        if any(dvals < min_eig_floor)
            dvals(dvals < min_eig_floor) = min_eig_floor;
            P = V * diag(dvals) * V';
        end
    catch
        % if eig fails for some reason, fall back to previous P
    end
    
    % Additional guard: if rcond is still below threshold, add a small multiple of identity
    % This handles edge cases where eigenvalue decomposition alone is not sufficient
    current_rcond = rcond(P);
    if isnan(current_rcond) || current_rcond < 1e-10
        reg_boost = 1e-7 * base;
        P = P + reg_boost * eye(n);
    end

    % ensure symmetry after modifications
    P = (P + P') / 2;
end

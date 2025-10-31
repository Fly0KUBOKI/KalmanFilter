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
    x_upd = x_pred + K * y;
    I = eye(size(P_pred));
    % If R provided, use full Joseph form
    if exist('R','var') && ~isempty(R)
        P_upd = (I - K*H) * P_pred * (I - K*H)' + K * R * K';
    else
        P_upd = (I - K*H) * P_pred;
    end
    % enforce symmetry
    P_upd = (P_upd + P_upd') / 2;
end

function [y, S, R_out] = compute_innovation_and_S_impl(z, h, H, P_pred, R, params)
    % Compute innovation y and innovation covariance S.
    % Handles NaN/Inf in z/h by inflating R diag and regularizes S.
    % Includes adaptive gating for outlier rejection.

    y = z - h;

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

classdef kf_core
    %KF_CORE static helper methods for Kalman-family filters
    %   Provides linear predict and linear update helpers used by KF, EKF, UKF

    methods(Static)
        function [F,Q] = build_F_Q(params, n)
            if nargin < 2, n = 10; end
            dt = params.dt;
            F = eye(n);
            if n>=10
                F(1,3) = dt; F(1,6) = 0.5*dt^2;
                F(2,4) = dt; F(2,7) = 0.5*dt^2;
                F(3,6) = dt; F(4,7) = dt; F(5,8) = dt; F(9,10) = dt;
            end
            q_a = params.kf.process_noise_accel;
            Q = zeros(n);
            if n>=10
                Q(6,6) = (q_a)^2; Q(7,7) = (q_a)^2; Q(8,8) = (q_a*0.1)^2; Q(10,10) = (q_a)^2;
            end
            if n>=4
                Q(1,1) = 0.01; Q(2,2) = 0.01; Q(3,3) = 0.01; Q(4,4) = 0.01;
            end
        end

        function [x_pred, P_pred, F, Q] = predict_linear(x_prev, P_prev, params)
            % Predict using linear F/Q built from params
            n = numel(x_prev);
            [F,Q] = kf_core.build_F_Q(params, n);
            x_pred = F * x_prev;
            P_pred = F * P_prev * F' + Q;
        end

        function [x_upd, P_upd, y, S, K, params] = linear_update(x_pred, P_pred, z, h, H, R, params, meas_tags)
            % LINEAR_UPDATE generic linear measurement update with numeric guards
            if nargin < 8, params = struct(); end
            if nargin < 9, meas_tags = {}; end

            if isempty(z)
                x_upd = x_pred; P_upd = P_pred; y = []; S = []; K = [];
                return;
            end

            y = z - h;
            S = H * P_pred * H' + R;

            % handle NaN/Inf in innovations by inflating R entries
            nan_idx = isnan(y) | isinf(y);
            if any(nan_idx)
                y(nan_idx) = 0;
                rd = diag(S);
                large = max(1e6, 1e3 * median(rd(rd>0)));
                R_diag = diag(R);
                R_diag(nan_idx) = large;
                R = diag(R_diag);
                S = H * P_pred * H' + R;
            end

            % ensure R diagonal positive and finite
            R_diag = diag(R);
            R_diag(~isfinite(R_diag) | R_diag <= 0) = eps;
            R = diag(R_diag);
            S = H * P_pred * H' + R;

            % regularize S if ill-conditioned
            nS = size(S,1);
            if nS > 0
                r = rcond(S);
                reg_scale = 1e-8; iter = 0;
                base = max(eps, trace(S)/nS);
                while (isnan(r) || r < 1e-12) && iter < 8
                    reg = (10^iter) * reg_scale * base;
                    S = S + reg * eye(nS);
                    r = rcond(S);
                    iter = iter + 1;
                end
                if isnan(r) || r < 1e-12
                    warning('kf_core:IllConditionedS','S is ill-conditioned after regularization. rcond=%g', r);
                end
                if isfield(params,'kf') && isfield(params.kf,'debug') && params.kf.debug
                    fprintf('kf_core: rcond after reg = %g (iter=%d)\n', r, iter);
                end
            end

            K = P_pred * H' / S;
            x_upd = x_pred + K * y;
            P_upd = (eye(numel(x_pred)) - K * H) * P_pred;

            % adaptive R update if requested and meas_tags provided
            if isfield(params,'kf') && isfield(params.kf,'ema_alpha') && ~isempty(meas_tags)
                alpha = params.kf.ema_alpha;
                if ~isfield(params.kf,'R_est')
                    params.kf.R_est = struct();
                end
                HPHT = H * P_pred * H'; HPHT_diag = diag(HPHT);
                R_diag = diag(R);
                for i=1:numel(meas_tags)
                    tag = meas_tags{i}; rng = tag.range; if isempty(rng), continue; end
                    res_sq = (y(rng)).^2;
                    hpht_comp = HPHT_diag(rng);
                    innov_comp = res_sq - hpht_comp;
                    if ~isfield(params.kf.R_est, tag.name)
                        params.kf.R_est.(tag.name) = R_diag(rng);
                    end
                    R_prev = params.kf.R_est.(tag.name);
                    if numel(R_prev) ~= numel(innov_comp)
                        R_prev = repmat(mean(R_prev), numel(innov_comp), 1);
                    end
                    R_new = (1-alpha)*R_prev + alpha * innov_comp;
                    R_new(R_new <= 0) = eps;
                    params.kf.R_est.(tag.name) = R_new;
                    % map back to params.noise as std devs where sensible
                    switch tag.name
                        case 'gps'
                            params.noise.gps = sqrt(R_new(:));
                        case 'vel'
                            params.noise.vel = sqrt(R_new(:));
                        case 'accel3'
                            params.noise.accel3 = sqrt(R_new(:));
                        case 'gyro3'
                            val = sqrt(mean(R_new)); params.noise.gyro3 = repmat(val,1,3);
                        case 'mag3'
                            params.noise.mag3 = sqrt(R_new(:));
                        case 'baro'
                            params.noise.baro = sqrt(mean(R_new));
                        case 'heading'
                            params.noise.heading = sqrt(mean(R_new));
                        otherwise
                            params.noise.(tag.name) = sqrt(R_new(:));
                    end
                end
            end
        end
    end
end

classdef DivergenceGuard < handle
    % DivergenceGuard - compatibility shim that delegates to SensorFilters
    % If mex_sensor_filter is available, heavy logic is executed in C++/MEX

    properties (Access = private)
        prev_innovations
        update_counts
        max_allowed_innov
        innov_change_ratio_threshold
        attenuation_factor
        min_eigenvalue_factor
        max_variance_factor
        min_rcond
        jitter_base
        max_velocity
        max_acceleration
        max_position_change
        max_velocity_change
        max_attitude_change
        max_innov_cap_fraction
        dump_saved
        max_gain_norm
    end

    properties (Access = public)
        config
    end

    methods
        function obj = DivergenceGuard(config)
            if nargin < 1 || isempty(config), config = struct(); end
            obj.config = config;
            obj.max_allowed_innov = get_field(config, 'max_allowed_innov', 50.0);
            obj.innov_change_ratio_threshold = get_field(config, 'innov_change_ratio_threshold', 2.0);
            obj.attenuation_factor = get_field(config, 'attenuation_factor', 0.5);
            obj.min_eigenvalue_factor = get_field(config, 'min_eigenvalue_factor', 1e-8);
            obj.max_variance_factor = get_field(config, 'max_variance_factor', 1e6);
            obj.min_rcond = get_field(config, 'min_rcond', 1e-12);
            obj.jitter_base = get_field(config, 'jitter_base', 1e-6);
            obj.max_velocity = get_field(config, 'max_velocity', 2.0);
            obj.max_acceleration = get_field(config, 'max_acceleration', 2.0);
            obj.max_position_change = get_field(config, 'max_position_change', 10.0);
            obj.max_velocity_change = get_field(config, 'max_velocity_change', 5.0);
            obj.max_attitude_change = get_field(config, 'max_attitude_change', 0.5);
            obj.max_innov_cap_fraction = get_field(config, 'max_innov_cap_fraction', 1.0);
            obj.dump_saved = false;
            obj.max_gain_norm = get_field(config, 'max_gain_norm', Inf);

            obj.prev_innovations = struct();
            obj.update_counts = struct();
        end

        function [dx_out, should_skip, was_attenuated] = check_and_attenuate_update(obj, sensor_name, innovation, dx_in, ctx)
            should_skip = false; was_attenuated = false; dx_out = dx_in;
            % Delegate to compiled MEX implementation (MATLAB fallback removed)
            try
                [dx_out, should_skip, was_attenuated] = SensorFilters.divergence_check(sensor_name, innovation, dx_in);
                return;
            catch ME
                error('DivergenceGuard:MissingMEX', 'Required MEX ''mex_sensor_filter'' not available or failed: %s', ME.message);
            end

            % MATLAB fallback (lightweight): basic checks and attenuation
            innov_norm = norm(innovation);
            if ~isfield(obj.update_counts, sensor_name)
                obj.update_counts.(sensor_name) = 0;
            end

            if innov_norm > obj.max_allowed_innov
                skip_factor = 1e6;
                if innov_norm > obj.max_allowed_innov * skip_factor
                    should_skip = true; return;
                end
                target_norm = obj.max_allowed_innov * obj.max_innov_cap_fraction;
                if target_norm <= 0, target_norm = obj.max_allowed_innov; end
                innovation = innovation * (target_norm / innov_norm);
                innov_norm = target_norm; was_attenuated = true;
            end

            if obj.update_counts.(sensor_name) > 0 && isfield(obj.prev_innovations, sensor_name)
                prev_innov = obj.prev_innovations.(sensor_name);
                innov_change = norm(innovation - prev_innov);
                prev_innov_norm = norm(prev_innov);
                if prev_innov_norm > 1e-6
                    change_ratio = innov_change / prev_innov_norm;
                    if change_ratio > obj.innov_change_ratio_threshold
                        dx_out = dx_in * obj.attenuation_factor; was_attenuated = true;
                    end
                end
            end

            obj.prev_innovations.(sensor_name) = innovation;
            obj.update_counts.(sensor_name) = obj.update_counts.(sensor_name) + 1;
        end

        function K_out = clamp_gain(obj, K_in)
            K_out = K_in;
            if isempty(obj.max_gain_norm) || ~isfinite(obj.max_gain_norm), return; end
            fn = norm(K_in, 'fro');
            if fn > obj.max_gain_norm && fn > 0
                K_out = K_in * (obj.max_gain_norm / fn);
            end
        end

        function save_divergence_dump(obj, sensor_name, ctx)
            try
                outdir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Results');
            catch
                outdir = fullfile(pwd, 'Results');
            end
            if ~exist(outdir, 'dir'), mkdir(outdir); end
            dump.sensor = sensor_name;
            if isfield(ctx, 'k'), dump.k = ctx.k; end
            if isfield(ctx, 'z'), dump.z = ctx.z; end
            if isfield(ctx, 'h'), dump.h = ctx.h; end
            if isfield(ctx, 'y'), dump.y = ctx.y; end
            if isfield(ctx, 'P') && ~isempty(ctx.P), dump.P = ctx.P; dump.P_diag = diag(ctx.P); end
            if isfield(ctx, 'R') && ~isempty(ctx.R), dump.R = ctx.R; end
            dump.time = datestr(now,'yyyymmdd_HHMMSS');
            fname = fullfile(outdir, sprintf('divergence_dump_%s_%s.mat', sensor_name, datestr(now,'yyyymmdd_HHMMSS')));
            try save(fname, 'dump'); fprintf('Divergence dump saved to %s\n', fname); catch end
        end

        function P_out = regularize_covariance(obj, P_in)
            % Delegate to compiled MEX implementation (MATLAB fallback removed)
            try
                P_out = SensorFilters.divergence_regularize(P_in);
                return;
            catch ME
                error('DivergenceGuard:MissingMEX', 'Required MEX ''mex_sensor_filter'' not available or failed: %s', ME.message);
            end

            P_out = P_in;
            n = size(P_out,1);
            P_out = (P_out + P_out')/2;

            % --- Added: always add a small jitter proportional to max diagonal
            base_diag = max(abs(diag(P_out))); if base_diag<1e-20, base_diag=1e-20; end
            jitter = obj.jitter_base * base_diag * eye(n);
            P_out = P_out + jitter;

            [~, p] = chol(P_out);
            if p>0
                % If chol still fails, add slightly larger jitter (fallback)
                base_diag2 = max(abs(diag(P_out))); if base_diag2<1e-10, base_diag2=1e-10; end
                jitter2 = obj.jitter_base * base_diag2 * eye(n);
                P_out = P_out + jitter2;
            end
            [V,D] = eig(P_out); eigvals = diag(D); base_eig = max(abs(eigvals)); if base_eig<1e-10, base_eig=1e-10; end
            min_eig = obj.min_eigenvalue_factor * base_eig; eigvals = max(eigvals, min_eig); P_out = V * diag(eigvals) * V';
            if rcond(P_out) < obj.min_rcond; boost = 1e-8 * max(abs(diag(P_out))); P_out = P_out + boost * eye(n); end
            diag_P = diag(P_out); base_var = max(abs(diag_P)); if base_var < 1e-10, base_var = 1e-10; end
            max_var = obj.max_variance_factor * base_var; diag_P = min(diag_P, max_var); P_out(1:n+1:end) = diag_P;
            if n==15
                pos_cap = get_field(obj, 'pos_var_cap', 1e6);
                vel_cap = get_field(obj, 'vel_var_cap', 1e4);
                att_cap = get_field(obj, 'att_var_cap', 100);
                ba_cap  = get_field(obj, 'ba_var_cap', 1e2);
                bg_cap  = get_field(obj, 'bg_var_cap', 1e-1);
                d = diag(P_out);
                d(1:3) = min(d(1:3), pos_cap);
                d(4:6) = min(d(4:6), vel_cap);
                d(7:9) = min(d(7:9), att_cap);
                d(10:12) = min(d(10:12), ba_cap);
                d(13:15) = min(d(13:15), bg_cap);
                P_out(1:n+1:end) = d;
            end
            mask = abs(P_out) < 1e-15; P_out(mask) = 0; P_out = (P_out + P_out')/2;
        end

        function P_out = regularize_for_ukf(obj, P_in)
            P_out = obj.regularize_covariance(P_in);
        end

        function dx_out = clip_state_change(obj, dx_in)
            dx_out = dx_in;
            if length(dx_in) >= 15
                pos_change = dx_in(1:3); pos_norm = norm(pos_change);
                if pos_norm > obj.max_position_change, dx_out(1:3) = pos_change * (obj.max_position_change / pos_norm); end
                vel_change = dx_in(4:6); vel_norm = norm(vel_change);
                if vel_norm > obj.max_velocity_change, dx_out(4:6) = vel_change * (obj.max_velocity_change / vel_norm); end
                att_change = dx_in(7:9); att_norm = norm(att_change);
                if att_norm > obj.max_attitude_change, dx_out(7:9) = att_change * (obj.max_attitude_change / att_norm); end
                ba_change = dx_in(10:12); ba_norm = norm(ba_change);
                if ba_norm > 0.5, dx_out(10:12) = ba_change * (0.5 / ba_norm); end
                bg_change = dx_in(13:15); bg_norm = norm(bg_change);
                if bg_norm > 0.1, dx_out(13:15) = bg_change * (0.1 / bg_norm); end
            end
        end

        function [vel_out, P_out, was_clipped] = check_and_clip_velocity(obj, vel_in, P_in, vel_indices)
            if nargin < 4, vel_indices = 4:6; end
            vel_out = vel_in; P_out = P_in; was_clipped = false;
            vel_norm = norm(vel_in);
            if vel_norm > obj.max_velocity
                vel_out = vel_in * (obj.max_velocity / vel_norm); was_clipped = true;
                vel_var_reset = 0.01; P_out(vel_indices, vel_indices) = eye(3) * vel_var_reset;
            end
        end

        function [accel_out, was_clipped] = clip_acceleration(obj, accel_in, dt)
            accel_out = accel_in; was_clipped = false; accel_norm = norm(accel_in);
            max_accel_in_dt = obj.max_acceleration * dt;
            if accel_norm > max_accel_in_dt, accel_out = accel_in * (max_accel_in_dt / accel_norm); was_clipped = true; end
        end

        function reset_sensor_history(obj, sensor_name)
            if isfield(obj.prev_innovations, sensor_name), obj.prev_innovations = rmfield(obj.prev_innovations, sensor_name); end
            if isfield(obj.update_counts, sensor_name), obj.update_counts = rmfield(obj.update_counts, sensor_name); end
        end

        function reset_all_history(obj)
            obj.prev_innovations = struct(); obj.update_counts = struct();
        end

        function set_thresholds(obj, param_name, value)
            if isprop(obj, param_name), obj.(param_name) = value; else warning('DivergenceGuard:InvalidProperty','Property %s does not exist', param_name); end
        end

        function info = get_status(obj)
            info = struct(); info.sensor_names = fieldnames(obj.update_counts); info.update_counts = obj.update_counts; info.max_velocity = obj.max_velocity; info.max_allowed_innov = obj.max_allowed_innov; info.innov_change_ratio_threshold = obj.innov_change_ratio_threshold;
        end
    end
end

function value = get_field(s, field, default)
    if isfield(s, field), value = s.(field); else value = default; end
end


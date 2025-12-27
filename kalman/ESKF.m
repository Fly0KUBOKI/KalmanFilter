classdef ESKF < handle
    % ESKF - Error State Kalman Filter
    
    properties
        p; v; q; ba; bg; P; Q; dt; g
        state_handle
        noiseEstimator; sensor_filters; accel_filter; divergence_guard
        options
        prev_accel; prev_gyro; prev_mag
        prev_gps_lat; prev_gps_lon; prev_gps_alt
        prev_baro; buffer_tolerance
        freq_mag; freq_baro; freq_gps; freq_accel; gps_origin
        zupt_threshold_accel; zupt_threshold_gyro; zupt_min_duration
        zupt_counter; is_stationary
        Q_nominal; adaptive_q_enabled
        last_reset_step; velocity_damping; gyro_noise_threshold
        w_body; quaternion_norm; accel_innovation_norm
        enable_accel_z_integration; accel_z_threshold
        accel_z_damping; baro_weight
    end

    methods
        function obj = ESKF(obs, static_time, dt)
            if nargin < 3 || isempty(dt), dt = 1/100; end
            obj.dt = dt;
            
            static_idx = [];
            if nargin >= 2 && ~isempty(static_time) && static_time > 0
                N_static = floor(static_time / dt);
                if isfield(obs, 'ax') && length(obs.ax) >= N_static
                    static_idx = 1:N_static;
                end
            end
            
            obj.p = zeros(3,1);
            obj.v = zeros(3,1);
            obj.g = [0;0;9.80665];

            if ~isempty(static_idx) && length(static_idx) > 10
                accel_static = obj.get_field_impl(obs, {'ax', 'accel_x'}, static_idx, 3);
                accel_mean = mean(accel_static, 1);
                sigma_a = mean(std(accel_static - accel_mean, [], 1));
                
                phi = atan2(-accel_mean(2), -accel_mean(3));
                theta = atan2(accel_mean(1), sqrt(accel_mean(2)^2 + accel_mean(3)^2));
                q_temp = mex_quaternion_lib('from_euler', rad2deg([phi; theta; 0]));
                
                gyro_static = obj.get_field_impl(obs, {'wx', 'gyro_x'}, static_idx, 3);
                sigma_g = deg2rad(mean(std(gyro_static, [], 1)));
                
                has_mag = obj.has_field_impl(obs, {'mx', 'mag_x'});
                if has_mag
                    mag_static = obj.get_field_impl(obs, {'mx', 'mag_x'}, static_idx, 3);
                    mag_mean = mean(mag_static, 1);
                    sigma_mag = mean(std(mag_static - mag_mean, [], 1));
                    R_rp = mex_quaternion_lib('to_rotation_matrix', q_temp);
                    m_level = R_rp * mag_mean';
                    psi = -atan2(m_level(2), m_level(1));
                    obj.q = mex_quaternion_lib('from_euler', rad2deg([phi; theta; psi]));
                else
                    sigma_mag = 10.0;
                    obj.q = q_temp;
                end
                
                if obj.has_field_impl(obs, {'pressure', 'baro'})
                    pressure_static = obj.get_field_impl(obs, {'pressure', 'baro'}, static_idx, 1);
                    alt_baro_static = 44330 * (1 - (pressure_static / 101325).^0.1903);
                    sigma_press = std(alt_baro_static - mean(alt_baro_static));
                else
                    sigma_press = 1.0;
                end
                
                has_gps = obj.has_field_impl(obs, {'lat', 'gps_lat'}) && ...
                          obj.has_field_impl(obs, {'lon', 'gps_lon'}) && ...
                          obj.has_field_impl(obs, {'alt', 'gps_alt'});
                if has_gps
                    lat_static = obj.get_field_impl(obs, {'lat', 'gps_lat'}, static_idx, 1);
                    lon_static = obj.get_field_impl(obs, {'lon', 'gps_lon'}, static_idx, 1);
                    alt_static = obj.get_field_impl(obs, {'alt', 'gps_alt'}, static_idx, 1);
                    lat0 = mean(lat_static);
                    lon0 = mean(lon_static);
                    alt0 = mean(alt_static);
                    y_m = (lat_static - lat0) / 9.0e-6;
                    x_m = (lon_static - lon0) / (9.0e-6 / cosd(lat0));
                    z_m = alt_static - alt0;
                    sigma_gps = mean([std(x_m); std(y_m); std(z_m)]);
                    obj.gps_origin = [lat0; lon0; alt0];
                else
                    sigma_gps = 1.0;
                    obj.gps_origin = [0; 0; 0];
                end
                
                if obj.has_field_impl(obs, {'wx', 'gyro_x'})
                    gyro_all = obj.get_field_impl(obs, {'wx', 'gyro_x'}, static_idx, 3);
                    gyro_all = deg2rad(gyro_all);
                else
                    gyro_all = deg2rad([obs.wx(:), obs.wy(:), obs.wz(:)]);
                end
                obj.gyro_noise_threshold = 2 * max(std(gyro_all, [], 1));
            else
                [sigma_a, sigma_g, sigma_mag, sigma_press, sigma_gps] = deal(0.1, deg2rad(0.1), 10.0, 1.0, 1.0);
                obj.gyro_noise_threshold = deg2rad(0.1);
                obj.q = [1;0;0;0];
            end
            
            obj.ba = zeros(3,1);
            obj.bg = zeros(3,1);

            obj.Q = zeros(15);
            obj.Q(4:6, 4:6) = eye(3) * 0.003^2;
            obj.Q(7:9, 7:9) = eye(3) * 0.003^2;
            obj.Q(10:12, 10:12) = eye(3) * (sigma_a^2 * 1e-3);
            obj.Q(13:15, 13:15) = eye(3) * (sigma_g^2 * 1e-3);

            obj.P = eye(15) * 0.01;
            obj.P(1:3, 1:3) = eye(3) * 5.0;
            obj.P(4:6, 4:6) = eye(3) * 0.5;
            obj.P(10:12, 10:12) = eye(3) * 0.5;
            obj.P(13:15, 13:15) = eye(3) * 0.1;

            if exist('mex_eskf_init', 'file') == 3
                state_in = struct('p', obj.p(:), 'v', obj.v(:), 'q', obj.q(:), ...
                    'ba', obj.ba(:), 'bg', obj.bg(:), 'P', obj.P);
                obj.state_handle = mex_eskf_init(state_in, struct());
            else
                obj.state_handle = uint64(0);
            end

            obj.noiseEstimator = struct();
            obj.noiseEstimator.getRnoise = @(s) mex_sensor_filter('get_R', s);
            obj.noiseEstimator.estimate = @(s, innov, H, P) mex_sensor_filter('noise_estimate', s, innov, H, P);
            obj.noiseEstimator.R_accel = ones(3,1) * sigma_a^2;
            obj.noiseEstimator.R_gyro = ones(3,1) * sigma_g^2;
            obj.noiseEstimator.R_mag = ones(3,1) * sigma_mag^2;
            obj.noiseEstimator.R_baro = sigma_press^2;
            obj.noiseEstimator.R_gps = ones(3,1) * sigma_gps^2;

            obj.sensor_filters = struct('accel', [], 'gyro', [], 'mag', [], 'gps', [], 'baro', []);
            obj.accel_filter = [];

            config = struct('max_velocity', 3, 'max_acceleration', 3, 'max_allowed_innov', 100, ...
                'max_innov_cap_fraction', 0.6, 'max_gain_norm', 150, 'innov_change_ratio_threshold', 2.5, ...
                'attenuation_factor', 0.6, 'max_attitude_variance', (deg2rad(15))^2, 'max_mag_gain_element', 0.2);
            obj.divergence_guard = struct();
            obj.divergence_guard.check_and_attenuate_update = @(s, i, d, c) mex_sensor_filter('divergence_check', s, i, d);
            obj.divergence_guard.regularize_covariance = @(P) mex_sensor_filter('divergence_regularize', P);
            obj.divergence_guard.check_and_clip_velocity = @(v, P, idx) mex_sensor_filter('divergence_clip_velocity', v, P, idx);
            
            [obj.prev_accel, obj.prev_gyro, obj.prev_mag] = deal(zeros(3,1));
            [obj.prev_gps_lat, obj.prev_gps_lon, obj.prev_gps_alt, obj.prev_baro] = deal(0);
            obj.buffer_tolerance = 1e-9;
            [obj.freq_accel, obj.freq_mag, obj.freq_baro, obj.freq_gps] = deal(1);
            
            [obj.zupt_threshold_accel, obj.zupt_threshold_gyro, obj.zupt_min_duration] = deal(1.0, deg2rad(3.0), 10);
            [obj.zupt_counter, obj.is_stationary] = deal(0, false);
            
            obj.Q_nominal = obj.Q;
            obj.adaptive_q_enabled = true;
            
            [obj.w_body, obj.last_reset_step, obj.velocity_damping] = deal(zeros(3,1), [], 0.0);
            [obj.accel_innovation_norm, obj.quaternion_norm] = deal(0, 1.0);
            
            [obj.enable_accel_z_integration, obj.accel_z_threshold, obj.accel_z_damping, obj.baro_weight] = ...
                deal(true, 0.5, 0.1, 0.2);
            
            % 段階的MEX置き換え用フラグ（環境変数から読み込み）
            obj.options = struct('preproc_in_matlab', true);
            obj.options.use_mex_predict_postprocess = strcmp(getenv('USE_MEX_PREDICT_POSTPROCESS'), '1');
        end

        function output = call_unified_filter(obj, input_struct)
            prev_state = struct('p', obj.p(:), 'v', obj.v(:), 'q', obj.q(:), ...
                'ba', obj.ba(:), 'bg', obj.bg(:), 'P', obj.P);
            output = mex_unified_filter(prev_state, input_struct);
            obj.p = output.position(:);
            obj.v = output.velocity(:);
            obj.q = output.quaternion(:);
            obj.ba = output.accel_bias(:);
            obj.bg = output.gyro_bias(:);
            obj.P = output.covariance;
        end

        function reset_sensor_filters(obj, method)
            if nargin < 2 || strcmp(method, 'reset')
                mex_sensor_filter('reset');
            else
                mex_sensor_filter('reset_zero');
            end
        end

        function varargout = utils(obj, method, varargin)
            switch method
                case 'get_euler'
                    varargout{1} = mex_quaternion_lib('to_euler', obj.q);
                case 'get_field'
                    varargout{1} = obj.get_field_impl(varargin{1}, varargin{2}, varargin{3}, varargin{4});
                otherwise
                    error('Unknown utils method: %s', method);
            end
        end

        function data = get_field_impl(obj, obs, field_names, idx, num_cols)
            if isempty(idx), error('idx is empty'); end
            data = mex_matlab_helpers('get_field', obs, field_names, idx, num_cols);
        end

        function tf = has_field_impl(obj, obs, field_names)
            tf = mex_matlab_helpers('has_field', obs, field_names);
        end

        function predict(obj, a_meas, w_meas)
            if any(isnan([obj.p; obj.v; obj.q; obj.P(:)]))
                warning('ESKF:predict:NaN', 'NaN detected before predict');
                return;
            end
            
            if ~isempty(obj.accel_filter)
                a_expected = obj.accel_filter.a_filtered;
                if norm(a_expected) < 1e-3, a_expected = a_meas; end
                [a_filtered, ~] = obj.accel_filter.filter(a_meas, a_expected);
                a_for_vel = a_filtered;
            else
                a_for_vel = a_meas;
            end
            
            [p_new, v_new, q_new, ba_new, bg_new, P_new] = mex_adaptive_predict('predict', ...
                obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P, ...
                a_for_vel, w_meas, obj.dt, obj.Q_nominal, obj.adaptive_q_enabled, ...
                zeros(3,1), zeros(3,1), obj.g);
            
            [obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P] = deal(p_new, v_new, q_new, ba_new, bg_new, P_new);
            obj.w_body = w_meas;
            obj.quaternion_norm = norm(obj.q);
            
            % 段階的MEX置き換え: predict()の後処理部分
            % MEX実装のみを使用（Phase 1完了）
            use_mex_postprocess = isfield(obj.options, 'use_mex_predict_postprocess') && obj.options.use_mex_predict_postprocess;
            if use_mex_postprocess && exist('mex_eskf_predict_postprocess', 'file') == 3
                [obj.v, obj.P] = mex_eskf_predict_postprocess('postprocess', ...
                    obj.v, obj.q, obj.P, a_for_vel, obj.dt, obj.g, ...
                    obj.enable_accel_z_integration, obj.accel_z_threshold, obj.accel_z_damping, ...
                    obj.velocity_damping);
            else
                error('ESKF:predict:postprocess', ...
                    'mex_eskf_predict_postprocess is required. Set USE_MEX_PREDICT_POSTPROCESS=1 and ensure MEX file is built.');
            end
        end

        function update_filter(obj, obs, k)
            a = [obs.accel_x(k); obs.accel_y(k); obs.accel_z(k)];
            w = deg2rad([obs.gyro_x(k); obs.gyro_y(k); obs.gyro_z(k)]);
            obj.predict(a, w);
            obj.sensor_updates('accel', a);
            obj.sensor_updates('mag', [obs.mag_x(k); obs.mag_y(k); obs.mag_z(k)]);
            obj.sensor_updates('baro', obs.baro(k));
            if ~isnan(obs.gps_lat(k)) && ~isnan(obs.gps_lon(k))
                obj.sensor_updates('gps', obs.gps_lat(k), obs.gps_lon(k), obs.gps_alt(k), k);
            end
            obj.reset('check', obs, k);
        end

        function sensor_updates(obj, sensor_type, varargin)
            if isfield(obj, 'options') && isfield(obj.options, 'preproc_in_matlab') && ~obj.options.preproc_in_matlab
                thin_sensor_update(obj, sensor_type, varargin{:});
                return;
            end
            switch sensor_type
                case 'accel'
                    a_meas = varargin{1};
                    [a_corrected, is_outlier, no_change] = mex_sensor_preprocessor('preprocess_accel', a_meas, obj.prev_accel);
                    if no_change || any(isnan(a_corrected)) || is_outlier || (norm(obj.w_body) > 1.5), return; end
                    obj.prev_accel = a_meas;
                    sample = []; if length(varargin) >= 2, sample = varargin{2}; end
                    if ~isempty(sample)
                        do_cpp_update(obj, 'accel', a_corrected, sample);
                    else
                        do_cpp_update(obj, 'accel', a_corrected);
                    end

                case 'mag'
                    m_meas = varargin{1};
                    [m_filtered, is_outlier, no_change] = mex_sensor_preprocessor('preprocess_mag', m_meas, obj.prev_mag);
                    if no_change || any(isnan(m_filtered)) || is_outlier, return; end
                    obj.prev_mag = m_meas;
                    sample = []; if length(varargin) >= 2, sample = varargin{2}; end
                    if ~isempty(sample)
                        do_cpp_update(obj, 'mag', m_filtered, sample);
                    else
                        do_cpp_update(obj, 'mag', m_filtered);
                    end

                case 'gps'
                    [lat, lon, alt] = deal(varargin{1}, varargin{2}, varargin{3});
                    [z_gps, is_outlier, no_change] = mex_sensor_preprocessor('preprocess_gps', lat, lon, alt, obj.gps_origin);
                    if no_change || is_outlier, return; end
                    [obj.prev_gps_lat, obj.prev_gps_lon, obj.prev_gps_alt] = deal(lat, lon, alt);
                    sample = []; if length(varargin) >= 4, sample = varargin{4}; end
                    if ~isempty(sample)
                        do_cpp_update(obj, 'gps', z_gps, sample);
                    else
                        do_cpp_update(obj, 'gps', z_gps);
                    end

                case 'baro'
                    pressure = varargin{1};
                    if abs(pressure - obj.prev_baro) <= obj.buffer_tolerance, return; end
                    obj.prev_baro = pressure;
                    [alt_baro, is_outlier] = mex_sensor_preprocessor('preprocess_baro', pressure);
                    if any(isnan(alt_baro)) || is_outlier, return; end
                    weight_factor = 1.0 / obj.baro_weight;
                    obj.P(3,3) = obj.P(3,3) * weight_factor;
                    sample = []; if length(varargin) >= 2, sample = varargin{2}; end
                    if ~isempty(sample)
                        do_cpp_update(obj, 'baro', alt_baro, sample);
                    else
                        do_cpp_update(obj, 'baro', alt_baro);
                    end
                    obj.P(3,3) = obj.P(3,3) / weight_factor;
                    
                otherwise
                    error('Unknown sensor update method: %s', sensor_type);
            end
        end

        function do_cpp_update(obj, sensor_type, meas, sample)
            sensor_data = struct('accel', zeros(3,1), 'gyro', zeros(3,1), 'mag', zeros(3,1), ...
                'gps_pos', zeros(3,1), 'alt_baro', 0, 'dt', obj.dt);
            params = struct('g', obj.g, 'mag_ref', [50;0;0], 'noise_accel', zeros(3,1), ...
                'noise_gyro', zeros(3,1), 'noise_ba', zeros(3,1), 'noise_bg', zeros(3,1), ...
                'noise_mag', zeros(3,1), 'noise_gps', zeros(3,1), 'noise_baro', 0, ...
                'alpha', 1e-3, 'beta', 2, 'kappa', 0);

            switch sensor_type
                case 'accel'
                    R = diag(obj.noiseEstimator.getRnoise('accel')) * 1.5;
                    sensor_data.accel = meas;
                    params.noise_accel = R(1:3);
                case 'mag'
                    R = diag(obj.noiseEstimator.getRnoise('mag')) * 1.5;
                    sensor_data.mag = meas;
                    params.noise_mag = R(1:3);
                case 'gps'
                    tmpR = obj.noiseEstimator.getRnoise('gps');
                    if ismatrix(tmpR) && all(size(tmpR) == [3,3])
                        Rdiag = diag(tmpR);
                    else
                        Rdiag = tmpR(:);
                    end
                    sensor_data.gps_pos = meas;
                    params.noise_gps = Rdiag(1:3);
                case 'baro'
                    sensor_data.alt_baro = meas;
                    params.noise_baro = obj.noiseEstimator.getRnoise('baro');
                case 'zupt'
                    params.noise_zupt = [0.01^2; 0.01^2; 0.01^2];
            end

            sensor_data.update_accel = strcmp(sensor_type, 'accel');
            sensor_data.update_gyro = false;
            sensor_data.update_mag = strcmp(sensor_type, 'mag');
            sensor_data.update_gps = strcmp(sensor_type, 'gps');
            sensor_data.update_baro = strcmp(sensor_type, 'baro');
            sensor_data.update_zupt = strcmp(sensor_type, 'zupt');

            mex_params = struct('g', params.g(:), 'mag_ref', params.mag_ref(:), ...
                'noise_accel', params.noise_accel(:), 'noise_gyro', params.noise_gyro(:), ...
                'noise_ba', params.noise_ba(:), 'noise_bg', params.noise_bg(:), ...
                'noise_mag', params.noise_mag(:), 'noise_gps', params.noise_gps(:), ...
                'noise_baro', params.noise_baro, 'noise_zupt', zeros(3,1), ...
                'alpha', params.alpha, 'beta', params.beta, 'kappa', params.kappa, ...
                'trace_sample', NaN);
            
            if strcmp(sensor_type,'gps')
                tmpR = obj.noiseEstimator.getRnoise('gps');
                if ismatrix(tmpR) && all(size(tmpR)==[3,3])
                    mex_params.noise_gps = diag(tmpR);
                else
                    mex_params.noise_gps = tmpR(:);
                end
            end
            if exist('sample','var') && ~isempty(sample)
                mex_params.trace_sample = sample;
            end
            if isfield(params, 'noise_zupt')
                mex_params.noise_zupt = params.noise_zupt(:);
            end

            state = struct('p', obj.p(:), 'v', obj.v(:), 'q', obj.q(:), ...
                'ba', obj.ba(:), 'bg', obj.bg(:), 'P', obj.P);
            [new_state, dbg_out, mex_debug] = obj.call_meukf_step(state, sensor_data, mex_params, sensor_type, sample);

            if isstruct(dbg_out) && isfield(dbg_out, 'innov') && isfield(dbg_out, 'H')
                obj.noiseEstimator.estimate(sensor_type, dbg_out.innov, dbg_out.H, obj.P);
            end

            should_apply_mex_state = true;
            if isstruct(dbg_out) && isfield(dbg_out, 'dx')
                dx_in = dbg_out.dx(:);
                ctx = struct('sensor', sensor_type, 'sample', sample);
                [dx_out, should_skip, was_attenuated] = obj.divergence_guard.check_and_attenuate_update(sensor_type, dbg_out.innov, dx_in, ctx);
                if should_skip, return; end

                if was_attenuated && ~isempty(dx_out)
                    dx_apply = dx_out(:);
                    new_state.p = state.p + dx_apply(1:3);
                    new_state.v = state.v + dx_apply(4:6);
                    phi = dx_apply(7:9);
                    dq = [1; 0.5 * phi(:)];
                    q1 = dq; q2 = state.q(:);
                    [w1, x1, y1, z1] = deal(q1(1), q1(2), q1(3), q1(4));
                    [w2, x2, y2, z2] = deal(q2(1), q2(2), q2(3), q2(4));
                    q_new = [w1*w2 - x1*x2 - y1*y2 - z1*z2; ...
                             w1*x2 + x1*w2 + y1*z2 - z1*y2; ...
                             w1*y2 - x1*z2 + y1*w2 + z1*x2; ...
                             w1*z2 + x1*y2 - y1*x2 + z1*w2];
                    q_new = q_new / norm(q_new);
                    new_state.q = q_new;
                    new_state.ba = state.ba + dx_apply(10:12);
                    new_state.bg = state.bg + dx_apply(13:15);
                    if isfield(new_state, 'P')
                        new_state.P = (new_state.P + new_state.P')/2;
                    else
                        new_state.P = obj.P;
                    end
                end
            end

            if should_apply_mex_state
                [obj.p, obj.v, obj.q, obj.ba, obj.bg] = deal(new_state.p, new_state.v, new_state.q, new_state.ba, new_state.bg);
                if isfield(new_state, 'P')
                    obj.P = (new_state.P + new_state.P')/2;
                end
            end
        end

        function [new_state, dbg_out, mex_debug] = call_meukf_step(obj, state, sensor_data, mex_params, sensor_type, sample)
            if nargin < 5, sensor_type = 'unknown'; end
            if nargin < 6, sample = []; end
            [new_state, dbg_out, mex_debug] = mex_meukf_step_v2(state, sensor_data, mex_params);
            fields_ok = isstruct(new_state) && isfield(new_state,'p') && isfield(new_state,'v') && ...
                isfield(new_state,'q') && isfield(new_state,'ba') && isfield(new_state,'bg') && isfield(new_state,'P');
            if ~fields_ok || any(isnan([new_state.p(:); new_state.v(:); new_state.q(:); new_state.ba(:); new_state.bg(:)])) || ...
                any(isnan(new_state.P(:)))
                error('ESKF:call_meukf_step:NaN','C++ update produced NaN for sensor %s', sensor_type);
            end
        end

        function reset(obj, method, varargin)
            if strcmp(method, 'check')
                if nargin < 3 || isempty(varargin{2}), return; end
                obs = varargin{1}; k = varargin{2};
                reset_needed = mex_filter_management('check_divergence', obj.P);
                if any(isnan([obj.p; obj.v; obj.q])) || any(isinf([obj.p; obj.v])) || ...
                    norm(obj.v) > 10.0 || norm(obj.p) > 1000.0
                    reset_needed = true;
                end
                if reset_needed
                    if ~isempty(obs)
                        obj.reset('filter', obs, k);
                    else
                        obj.last_reset_step = k;
                        obj.P(1:3, 1:3) = eye(3) * 20.0;
                        obj.P(4:6, 4:6) = eye(3) * 2.0;
                        obj.P(7:9, 7:9) = eye(3) * (deg2rad(30))^2;
                        obj.v = zeros(3,1);
                    end
                end
            elseif strcmp(method, 'filter')
                obs = varargin{1}; k = varargin{2};
                obj.last_reset_step = k;
                [~, ~, ~, ~, ~, P_reset] = mex_filter_management('reset_state', obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P, 0.01);
                obj.P = P_reset;
                obj.P(1:3, 1:3) = eye(3) * 20.0;
                obj.P(4:6, 4:6) = eye(3) * 2.0;
                obj.P(7:9, 7:9) = eye(3) * (deg2rad(30))^2;
                obj.v = zeros(3,1);
                if isfield(obs, 'lat') && k <= length(obs.lat) && ~isnan(obs.lat(k))
                    lat0 = obj.gps_origin(1);
                    lon0 = obj.gps_origin(2);
                    alt0 = obj.gps_origin(3);
                    obj.p = [(obs.lon(k) - lon0) / (9.0e-6 / cosd(lat0)); ...
                             (obs.lat(k) - lat0) / 9.0e-6; ...
                             obs.alt(k) - alt0];
                end
                if any(isnan(obj.q)), obj.q = [1;0;0;0]; end
                [obj.ba, obj.bg] = deal(zeros(3,1));
            end
        end

        function varargout = zupt(obj, method, varargin)
            if strcmp(method, 'check')
                a_meas = varargin{1}; w_meas = varargin{2};
                a_norm = norm(a_meas);
                gravity_deviation = abs(a_norm - 9.81);
                w_norm = norm(w_meas);
                if gravity_deviation < obj.zupt_threshold_accel && w_norm < obj.zupt_threshold_gyro
                    obj.zupt_counter = obj.zupt_counter + 1;
                else
                    obj.zupt_counter = 0;
                end
                obj.is_stationary = (obj.zupt_counter >= obj.zupt_min_duration);
                varargout{1} = obj.is_stationary;
            elseif strcmp(method, 'update')
                if ~obj.is_stationary, return; end
                [v_new, P_new] = mex_filter_management('apply_zupt', obj.v, obj.P);
                [obj.v, obj.P] = deal(v_new, P_new);
            end
        end

        function delete(obj)
            if isprop(obj, 'state_handle') && ~isempty(obj.state_handle) && obj.state_handle ~= uint64(0)
                if exist('mex_eskf_free', 'file') == 3
                    mex_eskf_free(obj.state_handle);
                end
            end
        end
    end
end

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
            
            % MEXコンストラクタを使用（Phase 1 - フォールバック削除）
            if exist('mex_eskf_constructor', 'file') ~= 3
                error('ESKF:MEX:Missing', 'mex_eskf_constructor not found. Please build MEX files first.');
            end
            
            init_data = mex_eskf_constructor('init', obs, static_time, dt);
            obj = obj.set_from_init_data(init_data);
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
            
            % MEX実装: predict()の後処理部分（Phase 1完了）
            [obj.v, obj.P] = mex_eskf_predict_postprocess('postprocess', ...
                obj.v, obj.q, obj.P, a_for_vel, obj.dt, obj.g, ...
                obj.enable_accel_z_integration, obj.accel_z_threshold, obj.accel_z_damping, ...
                obj.velocity_damping);
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
            % MEX統合版: sensor_updates() 完全MEX化
            switch sensor_type
                case 'accel'
                    a_meas = varargin{1};
                    sample = []; if length(varargin) >= 2, sample = varargin{2}; end
                    state_in = struct('p', obj.p(:), 'v', obj.v(:), 'q', obj.q(:), ...
                        'ba', obj.ba(:), 'bg', obj.bg(:), 'P', obj.P, 'g', obj.g(:), ...
                        'dt', obj.dt, 'w_body', obj.w_body(:), 'prev_accel', obj.prev_accel(:));
                    [obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P, obj.prev_accel] = ...
                        mex_eskf_sensor_updates_full('accel', a_meas(:), state_in, sample);
                    
                case 'mag'
                    m_meas = varargin{1};
                    sample = []; if length(varargin) >= 2, sample = varargin{2}; end
                    state_in = struct('p', obj.p(:), 'v', obj.v(:), 'q', obj.q(:), ...
                        'ba', obj.ba(:), 'bg', obj.bg(:), 'P', obj.P, 'g', obj.g(:), ...
                        'dt', obj.dt, 'prev_mag', obj.prev_mag(:));
                    [obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P, obj.prev_mag] = ...
                        mex_eskf_sensor_updates_full('mag', m_meas(:), state_in, sample);
                    
                case 'gps'
                    [lat, lon, alt] = deal(varargin{1}, varargin{2}, varargin{3});
                    sample = []; if length(varargin) >= 4, sample = varargin{4}; end
                    state_in = struct('p', obj.p(:), 'v', obj.v(:), 'q', obj.q(:), ...
                        'ba', obj.ba(:), 'bg', obj.bg(:), 'P', obj.P, 'g', obj.g(:), ...
                        'dt', obj.dt, 'gps_origin', obj.gps_origin(:));
                    [obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P, obj.prev_gps_lat, obj.prev_gps_lon, obj.prev_gps_alt] = ...
                        mex_eskf_sensor_updates_full('gps', lat, lon, alt, state_in, sample);
                    
                case 'baro'
                    pressure = varargin{1};
                    sample = []; if length(varargin) >= 2, sample = varargin{2}; end
                    state_in = struct('p', obj.p(:), 'v', obj.v(:), 'q', obj.q(:), ...
                        'ba', obj.ba(:), 'bg', obj.bg(:), 'P', obj.P, 'g', obj.g(:), ...
                        'dt', obj.dt, 'prev_baro', obj.prev_baro, 'buffer_tolerance', obj.buffer_tolerance, ...
                        'baro_weight', obj.baro_weight);
                    [obj.p, obj.v, obj.q, obj.ba, obj.bg, obj.P, obj.prev_baro] = ...
                        mex_eskf_sensor_updates_full('baro', pressure, state_in, sample);
                    
                otherwise
                    error('Unknown sensor update method: %s', sensor_type);
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
            % MEX実装: zupt() (Phase 5)
            if strcmp(method, 'check')
                a_meas = varargin{1}; w_meas = varargin{2};
                [is_stationary, new_counter] = mex_eskf_zupt('check', a_meas, w_meas, ...
                    obj.zupt_counter, obj.zupt_threshold_accel, obj.zupt_threshold_gyro, obj.zupt_min_duration);
                obj.zupt_counter = new_counter;
                obj.is_stationary = is_stationary;
                varargout{1} = obj.is_stationary;
            elseif strcmp(method, 'update')
                if ~obj.is_stationary, return; end
                [obj.v, obj.P] = mex_eskf_zupt('update', obj.v, obj.P);
            end
        end

        function delete(obj)
            if isprop(obj, 'state_handle') && ~isempty(obj.state_handle) && obj.state_handle ~= uint64(0)
                if exist('mex_eskf_free', 'file') == 3
                    mex_eskf_free(obj.state_handle);
                end
            end
        end
        
        function obj = set_from_init_data(obj, init_data)
            % MEXコンストラクタからの初期化データを適用（Phase 1）
            obj.p = init_data.p(:);
            obj.v = init_data.v(:);
            obj.q = init_data.q(:);
            obj.ba = init_data.ba(:);
            obj.bg = init_data.bg(:);
            obj.P = init_data.P;
            obj.Q = init_data.Q;
            obj.g = init_data.g(:);
            obj.dt = init_data.dt;
            obj.gps_origin = init_data.gps_origin(:);
            obj.gyro_noise_threshold = init_data.gyro_noise_threshold;
            
            % noiseEstimatorの設定
            obj.noiseEstimator = struct();
            obj.noiseEstimator.getRnoise = @(s) mex_sensor_filter('get_R', s);
            obj.noiseEstimator.estimate = @(s, innov, H, P) mex_sensor_filter('noise_estimate', s, innov, H, P);
            obj.noiseEstimator.R_accel = init_data.noiseEstimator.R_accel(:);
            obj.noiseEstimator.R_gyro = init_data.noiseEstimator.R_gyro(:);
            obj.noiseEstimator.R_mag = init_data.noiseEstimator.R_mag(:);
            obj.noiseEstimator.R_baro = init_data.noiseEstimator.R_baro;
            obj.noiseEstimator.R_gps = init_data.noiseEstimator.R_gps(:);
            
            % sensor_filters と accel_filter
            obj.sensor_filters = struct('accel', [], 'gyro', [], 'mag', [], 'gps', [], 'baro', []);
            obj.accel_filter = [];
            
            % divergence_guard
            obj.divergence_guard = struct();
            obj.divergence_guard.check_and_attenuate_update = @(s, i, d, c) mex_sensor_filter('divergence_check', s, i, d);
            obj.divergence_guard.regularize_covariance = @(P) mex_sensor_filter('divergence_regularize', P);
            obj.divergence_guard.check_and_clip_velocity = @(v, P, idx) mex_sensor_filter('divergence_clip_velocity', v, P, idx);
            
            % prev_* 初期化
            obj.prev_accel = init_data.prev_accel(:);
            obj.prev_gyro = init_data.prev_gyro(:);
            obj.prev_mag = init_data.prev_mag(:);
            obj.prev_gps_lat = init_data.prev_gps_lat;
            obj.prev_gps_lon = init_data.prev_gps_lon;
            obj.prev_gps_alt = init_data.prev_gps_alt;
            obj.prev_baro = init_data.prev_baro;
            obj.buffer_tolerance = init_data.buffer_tolerance;
            
            % freq_* 初期化
            obj.freq_accel = init_data.freq_accel;
            obj.freq_mag = init_data.freq_mag;
            obj.freq_baro = init_data.freq_baro;
            obj.freq_gps = init_data.freq_gps;
            
            % zupt_* 初期化
            obj.zupt_threshold_accel = init_data.zupt_threshold_accel;
            obj.zupt_threshold_gyro = init_data.zupt_threshold_gyro;
            obj.zupt_min_duration = init_data.zupt_min_duration;
            obj.zupt_counter = init_data.zupt_counter;
            obj.is_stationary = init_data.is_stationary;
            
            % Q_nominal, adaptive_q
            obj.Q_nominal = init_data.Q_nominal;
            obj.adaptive_q_enabled = init_data.adaptive_q_enabled;
            
            % その他
            obj.w_body = init_data.w_body(:);
            obj.last_reset_step = init_data.last_reset_step;
            obj.velocity_damping = init_data.velocity_damping;
            obj.accel_innovation_norm = init_data.accel_innovation_norm;
            obj.quaternion_norm = init_data.quaternion_norm;
            
            % accel_z 関連
            obj.enable_accel_z_integration = init_data.enable_accel_z_integration;
            obj.accel_z_threshold = init_data.accel_z_threshold;
            obj.accel_z_damping = init_data.accel_z_damping;
            obj.baro_weight = init_data.baro_weight;
            
            % オプション設定
            obj.options = struct('preproc_in_matlab', true);
            
            % state_handle
            if exist('mex_eskf_init', 'file') == 3
                state_in = struct('p', obj.p(:), 'v', obj.v(:), 'q', obj.q(:), ...
                    'ba', obj.ba(:), 'bg', obj.bg(:), 'P', obj.P);
                obj.state_handle = mex_eskf_init(state_in, struct());
            else
                obj.state_handle = uint64(0);
            end
        end
    end
end

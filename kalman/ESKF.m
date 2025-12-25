classdef ESKF < handle
    % ESKF - Error State Kalman Filter (C++実装使用)
    
    properties
        % 状態
        p; v; q; ba; bg; P; Q; dt; g
        % C++ MEX state handle (uint64), 0 if not created
        state_handle
        % ヘルパー
        noiseEstimator; sensor_filters; accel_filter; divergence_guard
        % オプション/フラグ
        options
        % SensorDataBuffer統合 (Common削除のため)
        prev_accel; prev_gyro; prev_mag
        prev_gps_lat; prev_gps_lon; prev_gps_alt
        prev_baro; buffer_tolerance
        % 設定
        freq_mag; freq_baro; freq_gps; freq_accel; gps_origin
        % ZUPT
        zupt_threshold_accel; zupt_threshold_gyro; zupt_min_duration
        zupt_counter; is_stationary
        % Adaptive Q
        Q_nominal; adaptive_q_enabled
        % その他
        last_reset_step; velocity_damping; gyro_noise_threshold
        w_body; quaternion_norm; accel_innovation_norm
        % Z軸積分
        enable_accel_z_integration; accel_z_threshold
        accel_z_damping; baro_weight
    end

    methods
        function obj = ESKF(obs, static_time, dt)
            if nargin < 3 || isempty(dt), dt = 1/100; end
            obj.dt = dt;
            
            if nargin >= 2 && ~isempty(static_time) && static_time > 0
                N_static = floor(static_time / dt);
                if isfield(obs, 'ax') && length(obs.ax) >= N_static
                    static_idx = 1:N_static;
                else
                    static_idx = [];
                end
            else
                static_idx = [];
            end
            
            % 初期状態
            obj.p = zeros(3,1);
            obj.v = zeros(3,1);
            
            % gravity
            obj.g = [0;0;9.80665];

            % Initialize noise parameters
            if ~isempty(static_idx) && length(static_idx) > 10
                accel_static = obj.utils('get_field', obs, {'ax', 'accel_x'}, static_idx, 3);
                accel_mean = mean(accel_static, 1);
                sigma_a = mean(std(accel_static - accel_mean, [], 1));
                
                % Roll/Pitch計算
                phi = atan2(-accel_mean(2), -accel_mean(3));
                theta = atan2(accel_mean(1), sqrt(accel_mean(2)^2 + accel_mean(3)^2));
                
                q_temp = mex_quaternion_lib('from_euler', rad2deg([phi; theta; 0]));
                gyro_static = obj.utils('get_field', obs, {'wx', 'gyro_x'}, static_idx, 3);
                sigma_g = deg2rad(mean(std(gyro_static, [], 1)));
                
                if obj.has_field_impl(obs, {'mx', 'mag_x'})
                    mag_static = obj.utils('get_field', obs, {'mx', 'mag_x'}, static_idx, 3);
                    has_mag = true;
                else
                    has_mag = false;
                end

                if has_mag
                    mag_mean = mean(mag_static, 1);
                    sigma_mag = mean(std(mag_static - mag_mean, [], 1));
                    
                    % Yaw計算
                    R_rp = mex_quaternion_lib('to_rotation_matrix', q_temp);
                    m_level = R_rp * mag_mean';
                    
                    % Calculate Yaw angle
                    % NED frame: North is X, East is Y.
                    % m_level points to North in the leveled body frame.
                    % psi = -atan2(my, mx)
                    psi = -atan2(m_level(2), m_level(1));
                    
                    fprintf('Initialized Yaw from Mag: %.2f deg\n', rad2deg(psi));
                    
                    % Update quaternion with calculated Yaw
                    obj.q = mex_quaternion_lib('from_euler', rad2deg([phi; theta; psi]));
                else
                    sigma_mag = 10.0;
                    obj.q = q_temp;
                end
                
                if obj.has_field_impl(obs, {'pressure', 'baro'})
                    pressure_static = obj.utils('get_field', obs, {'pressure', 'baro'}, static_idx, 1);
                    alt_baro_static = 44330 * (1 - (pressure_static / 101325).^0.1903);
                    sigma_press = std(alt_baro_static - mean(alt_baro_static));
                else
                    sigma_press = 1.0;
                end
                
                if obj.has_field_impl(obs, {'lat', 'gps_lat'}) && obj.has_field_impl(obs, {'lon', 'gps_lon'}) && obj.has_field_impl(obs, {'alt', 'gps_alt'})
                    lat_static = obj.utils('get_field', obs, {'lat', 'gps_lat'}, static_idx, 1);
                    lon_static = obj.utils('get_field', obs, {'lon', 'gps_lon'}, static_idx, 1);
                    alt_static = obj.utils('get_field', obs, {'alt', 'gps_alt'}, static_idx, 1);
                else
                    lat_static = [];
                end

                if ~isempty(lat_static)
                    lat0 = mean(lat_static);
                    lon0 = mean(lon_static);
                    y_m = (lat_static - lat0) / (9.0e-6);
                    x_m = (lon_static - lon0) / (9.0e-6 / cosd(lat0));
                    z_m = alt_static - mean(alt_static);
                    sigma_gps = mean([std(x_m); std(y_m); std(z_m)]);
                    
                    % GPS origin設定
                    alt0 = mean(alt_static);
                    obj.gps_origin = [lat0; lon0; alt0];
                else
                    sigma_gps = 1.0;
                    obj.gps_origin = [0; 0; 0]; % デフォルト値
                end
                
                % Gyro noise threshold
                if obj.has_field_impl(obs, {'wx', 'gyro_x'})
                    gyro_all = obj.utils('get_field', obs, {'wx', 'gyro_x'}, static_idx, 3);
                    gyro_all = deg2rad(gyro_all);
                else
                    wx_all = deg2rad(obs.wx(:));
                    wy_all = deg2rad(obs.wy(:));
                    wz_all = deg2rad(obs.wz(:));
                    gyro_all = [wx_all, wy_all, wz_all];
                end
                obj.gyro_noise_threshold = 2 * max(std(gyro_all, [], 1));
            else
                % Default values
                sigma_a = 0.1;
                sigma_g = deg2rad(0.1);
                sigma_mag = 10.0;
                sigma_press = 1.0;
                sigma_gps = 1.0;
                obj.gyro_noise_threshold = deg2rad(0.1);
                obj.q = [1;0;0;0]; % 静止データがない場合は水平と仮定
            end
            
            obj.ba = zeros(3,1);
            obj.bg = zeros(3,1);

            % Process noise Q
            obj.Q = zeros(15);
            obj.Q(4:6, 4:6) = eye(3) * (0.003^2); % 速度のランダムウォークをさらに小さく (0.005 -> 0.003)
            obj.Q(7:9, 7:9) = eye(3) * (0.003^2); % 姿勢のランダムウォークをさらに小さく (0.005 -> 0.003)
            obj.Q(10:12, 10:12) = eye(3) * (sigma_a^2 * 1e-3); % 加速度バイアス変動を大幅に増加 (1e-4 -> 1e-3)
            obj.Q(13:15, 13:15) = eye(3) * (sigma_g^2 * 1e-3); % ジャイロバイアス変動を大幅に増加 (1e-4 -> 1e-3)

            % Initial covariance
            % 位置・速度の初期不確かさを大きく設定（GPS更新を効果的にするため）
            obj.P = eye(15) * 0.01;
            obj.P(1:3, 1:3) = eye(3) * 5.0;  % 位置の初期分散を小さく（10 -> 5 m^2）より保守的に
            obj.P(4:6, 4:6) = eye(3) * 0.5;   % 速度の初期分散を少し大きく (0.05 -> 0.5)
            obj.P(10:12, 10:12) = eye(3) * 0.5; % 加速度バイアスの初期分散を大きく（0.05 -> 0.5）
            obj.P(13:15, 13:15) = eye(3) * 0.1; % ジャイロバイアスの初期分散を大きく（0.01 -> 0.1）

            % Initialize C++ ESKF state if available (let errors propagate if initialization fails)
            if exist('mex_eskf_init', 'file') == 3
                state_in = struct();
                state_in.p = obj.p(:);
                state_in.v = obj.v(:);
                state_in.q = obj.q(:);
                state_in.ba = obj.ba(:);
                state_in.bg = obj.bg(:);
                state_in.P = obj.P;
                % params struct can be empty for now
                obj.state_handle = mex_eskf_init(state_in, struct());
            else
                obj.state_handle = uint64(0);
            end

            % ヘルパークラス初期化
            if exist('mex_sensor_filter','file') == 3
                obj.noiseEstimator = struct();
                obj.noiseEstimator.getRnoise = @(s) mex_sensor_filter('get_R', s);
                obj.noiseEstimator.estimate = @(s, innov, H, P) mex_sensor_filter('noise_estimate', s, innov, H, P);
            else
                obj.noiseEstimator = NoiseEstimator(10);
            end
            % Default R fields (works for struct or class)
            try
                obj.noiseEstimator.R_accel = ones(3,1) * (sigma_a^2);
                obj.noiseEstimator.R_gyro  = ones(3,1) * (sigma_g^2);
                obj.noiseEstimator.R_mag   = ones(3,1) * (sigma_mag^2);
                obj.noiseEstimator.R_baro  = (sigma_press^2);
                obj.noiseEstimator.R_gps   = ones(3,1) * (sigma_gps^2);
            catch
            end

            obj.sensor_filters = struct();
            % NOTE: Prefer MEX implementations. Create MATLAB instances only
            % when MEX handler is not available to avoid deprecation warnings.
            if exist('mex_sensor_filter','file') == 3
                obj.sensor_filters.accel = [];
                obj.sensor_filters.gyro  = [];
                obj.sensor_filters.mag   = [];
                obj.sensor_filters.gps   = [];
                obj.sensor_filters.baro  = [];
            else
                obj.sensor_filters.accel = SensorAccelFilter(struct('ema_alpha', 0.3, 'history_size', 20, 'gravity_range', [8.5, 10.5]));
                % Gyro MATLAB implementation removed — do not create gyro filter
                obj.sensor_filters.gyro  = [];
                obj.sensor_filters.mag   = SensorMagFilter(struct('ema_alpha', 0.2, 'history_size', 20, 'mag_norm_expected', 50));
                obj.sensor_filters.gps   = SensorGPSFilter(struct('ema_alpha', 0.15, 'history_size', 10, 'horizontal_accuracy', 2.5, 'vertical_accuracy', 5.0));
                obj.sensor_filters.baro  = SensorBaroFilter(struct('ema_alpha', 0.1, 'history_size', 50, 'altitude_per_pressure', 44330));
            end

            % MATLAB AccelFilter instance removed for Phase2 MEX — keep placeholder
            obj.accel_filter = [];

            % DivergenceGuard (use MEX if available)
            config = struct('max_velocity', 3, 'max_acceleration', 3, ...
                'max_allowed_innov', 100, 'max_innov_cap_fraction', 0.6, ...
                'max_gain_norm', 150, 'innov_change_ratio_threshold', 2.5, ...
                'attenuation_factor', 0.6, 'max_attitude_variance', (deg2rad(15))^2, ...
                'max_mag_gain_element', 0.2);
            if exist('mex_sensor_filter','file') == 3
                obj.divergence_guard = struct();
                obj.divergence_guard.check_and_attenuate_update = @(sensor_name, innov, dx_in, ctx) mex_sensor_filter('divergence_check', sensor_name, innov, dx_in);
                obj.divergence_guard.regularize_covariance = @(P) mex_sensor_filter('divergence_regularize', P);
                % mex_sensor_filter does not provide check_and_clip_velocity; provide MATLAB fallback
                obj.divergence_guard.check_and_clip_velocity = @(vel_in, P_in, vel_indices) obj.divergence_check_velocity_impl(vel_in, P_in, vel_indices);
            else
                obj.divergence_guard = DivergenceGuard(config);
            end
            
            % SensorDataBuffer統合 (Common削除)
            obj.prev_accel = zeros(3,1);
            obj.prev_gyro = zeros(3,1);
            obj.prev_mag = zeros(3,1);
            obj.prev_gps_lat = 0;
            obj.prev_gps_lon = 0;
            obj.prev_gps_alt = 0;
            obj.prev_baro = 0;
            obj.buffer_tolerance = 1e-9;

            % フィルタ周波数（注: 実際の更新判定はGenerateDataの周期制御とC++の変更検知で実施）
            % 互換性のため変数は残すが、update_filterでは使用しない
            [obj.freq_accel, obj.freq_mag, obj.freq_baro, obj.freq_gps] = deal(1, 1, 1, 1);
            
            % ZUPT
            obj.zupt_threshold_accel = 1.0;
            obj.zupt_threshold_gyro = deg2rad(3.0);
            obj.zupt_min_duration = 10;
            obj.zupt_counter = 0;
            obj.is_stationary = false;
            
            % Adaptive Q
            obj.Q_nominal = obj.Q;
            obj.adaptive_q_enabled = true;
            
            % その他
            obj.w_body = zeros(3,1);
            obj.last_reset_step = [];
            obj.velocity_damping = 0.0;
            obj.accel_innovation_norm = 0;
            obj.quaternion_norm = 1.0;
            
            % Z軸積分
            obj.enable_accel_z_integration = true;
            obj.accel_z_threshold = 0.5;
            obj.accel_z_damping = 0.1;
            obj.baro_weight = 0.2;

            % defaults for options
            obj.options = struct();
            obj.options.preproc_in_matlab = true; % set to false to enable thin-wrapper (direct MEX)
        end

        function output = call_unified_filter(obj, input_struct)
            % CALL_UNIFIED_FILTER  C++統合フィルタを呼び出す（クラス内実装）
            prev_state = struct();
            prev_state.p = obj.p(:);
            prev_state.v = obj.v(:);
            prev_state.q = obj.q(:);
            prev_state.ba = obj.ba(:);
            prev_state.bg = obj.bg(:);
            prev_state.P = obj.P;

            % MEX呼び出し
            output = mex_unified_filter(prev_state, input_struct);

            % ESKFの状態を更新
            obj.p = output.position(:);
            obj.v = output.velocity(:);
            obj.q = output.quaternion(:);
            obj.ba = output.accel_bias(:);
            obj.bg = output.gyro_bias(:);
            obj.P = output.covariance;
        end

        % --------- KF integration helpers (proxy methods) ---------
        function reset_sensor_filters(obj)
            % Reset sensor filters (prefer MEX, fallback to MATLAB instances)
            if exist('mex_sensor_filter','file') == 3
                mex_sensor_filter('reset');
            else
                if isfield(obj.sensor_filters, 'accel') && ~isempty(obj.sensor_filters.accel)
                    if ismethod(obj.sensor_filters.accel, 'reset')
                        obj.sensor_filters.accel.reset();
                    end
                end
            end
        end

        function reset_sensor_filters_zero(obj)
            if exist('mex_sensor_filter','file') == 3
                mex_sensor_filter('reset_zero');
            elseif exist('SensorFilters','file') == 3
                SensorFilters.reset_zero();
            end
        end

        function R = get_sensor_R(obj, sensor_type)
            % Return sensor noise covariance via mex_sensor_filter (preferred) or SensorFilters/NoiseEstimator
            if exist('mex_sensor_filter','file') == 3
                R = mex_sensor_filter('get_R', sensor_type);
                return;
            elseif exist('SensorFilters','file') == 2 || exist('SensorFilters','file') == 3
                R = SensorFilters.get_R(sensor_type);
                return;
            end
            R = obj.noiseEstimator.getRnoise(sensor_type);
        end

        function estimate_noise(obj, sensor_type, innov, H, P)
            % Update noise estimator (MATLAB side) and notify MEX if present
            try
                if isfield(obj.noiseEstimator, 'estimate')
                    obj.noiseEstimator.estimate(sensor_type, innov, H, P);
                elseif exist('mex_sensor_filter','file') == 3
                    mex_sensor_filter('noise_estimate', sensor_type, innov, H, P);
                end
            catch
            end
            if exist('mex_sensor_filter','file') == 3
                try mex_sensor_filter('noise_estimate', sensor_type, innov, H, P); catch, end
            elseif exist('SensorFilters','file') == 2 || exist('SensorFilters','file') == 3
                SensorFilters.noise_estimate(sensor_type, innov, H, P);
            end
        end

        function [dx_out, should_skip, was_attenuated] = divergence_check(obj, sensor_name, innov, dx_in, ctx)
            % Run divergence guard checks: prefer MATLAB DivergenceGuard, fall back to SensorFilters/MEX
            [dx_out, should_skip, was_attenuated] = obj.divergence_guard.check_and_attenuate_update(sensor_name, innov, dx_in, ctx);
        end

        function varargout = utils(obj, method, varargin)
            % Utility unified method (migrated from utils.m)
            switch method
                case 'get_euler'
                    varargout{1} = obj.get_euler_impl();
                case 'get_field'
                    varargout{1} = obj.get_field_impl(varargin{1}, varargin{2}, varargin{3}, varargin{4});
                otherwise
                    error('Unknown utils method: %s', method);
            end
        end

        function euler = get_euler_impl(obj)
            % Return Euler angles in degrees
            euler = mex_quaternion_lib('to_euler', obj.q);
        end

        function data = get_field_impl(obj, obs, field_names, idx, num_cols)
            % Get first matching field from obs according to candidates
            % Phase 1: MEX実装を使用（フォールバック付き）
            % 注意: idxが配列の場合はMEXを使わずMATLAB実装を使用（MEXは単一インデックスのみ対応）
            if exist('mex_matlab_helpers', 'file') == 3 && ~isempty(idx) && isscalar(idx)
                try
                    data = mex_matlab_helpers('get_field', obs, field_names, idx, num_cols);
                    return;
                catch ME
                    warning('ESKF:get_field_impl:MEX', 'MEX call failed, using MATLAB fallback: %s', ME.message);
                end
            end
            
            % MATLAB フォールバック（既存実装）
            for i = 1:length(field_names)
                if isfield(obs, field_names{i})
                    if num_cols == 1
                        data = obs.(field_names{i})(idx);
                    elseif num_cols == 3
                        field_base = field_names{i};
                        if endsWith(field_base, '_x')
                            field_base = field_base(1:end-2);
                        end
                        if startsWith(field_base, 'gyro') || startsWith(field_base, 'mag') || ...
                           startsWith(field_base, 'accel') || startsWith(field_base, 'gps')
                            data = [obs.([field_base '_x'])(idx), ...
                                    obs.([field_base '_y'])(idx), ...
                                    obs.([field_base '_z'])(idx)];
                        else
                            suffix = field_base(1);
                            data = [obs.([suffix 'x'])(idx), ...
                                    obs.([suffix 'y'])(idx), ...
                                    obs.([suffix 'z'])(idx)];
                        end
                    end
                    return;
                end
            end
            error('ESKF:utils:get_field', 'None of the fields found: %s', strjoin(field_names, ', '));
        end

        function tf = has_field_impl(obj, obs, field_names)
            % Phase 1: MEX実装を使用（フォールバック付き）
            if exist('mex_matlab_helpers', 'file') == 3
                try
                    tf = mex_matlab_helpers('has_field', obs, field_names);
                    return;
                catch ME
                    warning('ESKF:has_field_impl:MEX', 'MEX call failed, using MATLAB fallback: %s', ME.message);
                end
            end
            
            % MATLAB フォールバック（既存実装）
            tf = false;
            for ii = 1:length(field_names)
                if isfield(obs, field_names{ii})
                    tf = true; return;
                end
            end
        end

        function predict(obj, a_meas, w_meas)
            % 予測ステップ (C++実装: mex_meukf_step_v2)
            
            % NaN check
            if any(isnan(obj.p)) || any(isnan(obj.v)) || any(isnan(obj.q)) || any(isnan(obj.P(:)))
                warning('ESKF:predict:NaN', 'NaN detected before predict');
                return;
            end
            
            % ジャイロフィルタ適用 (MATLAB側で実施)
            if isprop(obj, 'enable_gyro_filter') && ~isempty(obj.enable_gyro_filter) && obj.enable_gyro_filter && ...
               isfield(obj.sensor_filters, 'gyro') && ~isempty(obj.sensor_filters.gyro)
                w_expected = obj.sensor_filters.gyro.w_filtered;
                [w_filtered, w_is_outlier, ~] = obj.sensor_filters.gyro.apply(w_meas, w_expected);
                if w_is_outlier
                    w_meas = w_expected;
                else
                    if obj.enable_yaw_raw_gyro
                        w_filtered(3) = w_meas(3);
                    end
                    w_meas = w_filtered;
                end
            end
            
            % 加速度フィルタ適用 (MATLAB側で実施)
            if ~isempty(obj.accel_filter)
                a_expected = obj.accel_filter.a_filtered;
                if norm(a_expected) < 1e-3
                    a_expected = a_meas;
                end
                [a_filtered, is_outlier] = obj.accel_filter.filter(a_meas, a_expected);
                a_for_vel = a_filtered;
            else
                a_for_vel = a_meas;
            end
            
            % Adaptive Q (MATLAB側で計算)
            Q_adapted = obj.Q;
            if obj.adaptive_q_enabled
                a_norm = norm(a_meas);
                gravity_error = abs(a_norm - 9.81);
                accel_scale = 1.0 + (gravity_error / 3.0);
                w_norm = norm(w_meas);
                gyro_scale = 1.0 + (w_norm / deg2rad(15.0));
                q_scale = max(accel_scale, gyro_scale);
                q_scale = min(q_scale, 5.0);
                Q_adapted = obj.Q_nominal * q_scale;
            end
            
            % C++ MEX用パラメータ準備
            dt2 = obj.dt^2;
            noise_accel = diag(Q_adapted(4:6, 4:6)) / dt2;
            noise_gyro = diag(Q_adapted(7:9, 7:9)) / dt2;
            noise_ba = diag(Q_adapted(10:12, 10:12)) / obj.dt;
            noise_bg = diag(Q_adapted(13:15, 13:15)) / obj.dt;
            
            % センサー構造体
            sensor_data.accel = a_for_vel;
            sensor_data.gyro = w_meas;
            sensor_data.mag = zeros(3,1);
            sensor_data.gps_pos = zeros(3,1);
            sensor_data.alt_baro = 0;
            sensor_data.dt = obj.dt;
            
            % 更新フラグ (予測のみ)
            sensor_data.update_accel = false;
            sensor_data.update_gyro = false;
            sensor_data.update_mag = false;
            sensor_data.update_gps = false;
            sensor_data.update_baro = false;
            
            % パラメータ構造体
            mex_params.g = obj.g;
            mex_params.mag_ref = [50; 0; 0]; % Dummy
            mex_params.noise_accel = noise_accel;
            mex_params.noise_gyro = noise_gyro;
            mex_params.noise_ba = noise_ba;
            mex_params.noise_bg = noise_bg;
            mex_params.noise_mag = zeros(3,1);
            mex_params.noise_gps = zeros(3,1);
            mex_params.noise_baro = 0;
            mex_params.alpha = 1e-3;
            mex_params.beta = 2;
            mex_params.kappa = 0;
            
            % 状態構造体
            state_in.p = obj.p;
            state_in.v = obj.v;
            state_in.q = obj.q;
            state_in.ba = obj.ba;
            state_in.bg = obj.bg;
            state_in.P = obj.P;

            % C++ MEX呼び出し（共通ラッパーを経由）
            state_out = obj.call_meukf_step(state_in, sensor_data, mex_params);

            % 状態更新
            obj.p = state_out.p;
            obj.v = state_out.v;
            obj.q = state_out.q;
            obj.ba = state_out.ba;
            obj.bg = state_out.bg;
            obj.P = state_out.P;

            % 角速度を保存
            obj.w_body = w_meas;
            obj.quaternion_norm = norm(obj.q);
            
            % Z軸加速度積分
            if obj.enable_accel_z_integration
                R = mex_quaternion_lib('to_rotation_matrix', obj.q);
                a_ned = R * a_for_vel - [0; 0; obj.g(3)];
                az_excess = a_ned(3);
                if abs(az_excess) > obj.accel_z_threshold
                    obj.v(3) = obj.v(3) * (1.0 - obj.accel_z_damping) + az_excess * obj.dt;
                end
            end
            
            % XY速度減衰
            if ~isempty(obj.velocity_damping) && obj.velocity_damping > 0
                obj.v(1:2) = obj.v(1:2) * (1.0 - obj.velocity_damping * obj.dt);
            end
            
            % 共分散正則化と制限
            obj.P = obj.divergence_guard.regularize_covariance(obj.P);
            max_var = [100^2*ones(3,1); 20^2*ones(3,1); (deg2rad(45))^2*ones(3,1); 0.1*ones(3,1); 0.01*ones(3,1)];
            for i = 1:15
                if obj.P(i,i) > max_var(i)
                    factor = sqrt(max_var(i) / obj.P(i,i));
                    obj.P(i,:) = obj.P(i,:) * factor;
                    obj.P(:,i) = obj.P(:,i) * factor;
                    obj.P(i,i) = max_var(i);
                end
            end
            
            [obj.v, obj.P, ~] = obj.divergence_guard.check_and_clip_velocity(obj.v, obj.P, 4:6);
        end

        function update_filter(obj, obs, k)
            % 1ステップ更新実行
            % 注: センサー更新周期はGenerateData段階で制御済み
            %     C++側で前回値との差分により自動的に更新判定される
            
            % センサーデータ取得
            a = [obs.accel_x(k); obs.accel_y(k); obs.accel_z(k)];
            w = deg2rad([obs.gyro_x(k); obs.gyro_y(k); obs.gyro_z(k)]);
            
            % 予測ステップ
            obj.predict(a, w);
            
            % センサー更新（すべて毎回実行、C++側で変更検知により自動更新）
            obj.sensor_updates('accel', a);
            obj.sensor_updates('mag', [obs.mag_x(k); obs.mag_y(k); obs.mag_z(k)]);
            obj.sensor_updates('baro', obs.baro(k));
            
            if ~isnan(obs.gps_lat(k)) && ~isnan(obs.gps_lon(k))
                obj.sensor_updates('gps', obs.gps_lat(k), obs.gps_lon(k), obs.gps_alt(k), k);
            end
            
            % 発散チェックとリセット
            obj.reset('check', obs, k);
        end

        function varargout = sensor_updates(obj, method, varargin)
            % センサー更新統合メソッド
            % 使用例:
            %   obj.sensor_updates('accel', a_meas)
            %   obj.sensor_updates('mag', m_meas)
            %   obj.sensor_updates('gps', lat, lon, alt, k)
            %   obj.sensor_updates('baro', pressure)
            
            % Thin-wrapper: if MATLAB preproc is disabled, skip heavy MATLAB-side preprocessing
            if isfield(obj, 'options') && isfield(obj.options, 'preproc_in_matlab') && ~obj.options.preproc_in_matlab
                thin_sensor_update(obj, method, varargin{:});
                return;
            end
            
            % 統合センサー更新入口に移譲
            update_sensor_impl(obj, method, varargin{:});
        end



        function update_sensor_impl(obj, sensor_type, varargin)
            % 統合センサー更新: 変更検知、MEX優先フィルタ、異常判定、C++更新呼出し
            switch sensor_type
                case 'accel'
                    a_meas = varargin{1};
                    if norm(a_meas - obj.prev_accel) <= obj.buffer_tolerance
                        return;
                    end
                    obj.prev_accel = a_meas;
                    if ~isempty(obj.w_body) && norm(obj.w_body) > 1.5; return; end
                    if exist('mex_sensor_filter','file') == 3
                        try
                            [a_corrected, is_outlier] = mex_sensor_filter('accel', a_meas, zeros(3,1));
                        catch
                            a_corrected = a_meas; is_outlier = false;
                        end
                    elseif exist('SensorFilters','file') == 3
                        [a_corrected, is_outlier] = SensorFilters.accel(a_meas, zeros(3,1));
                    else
                        a_corrected = a_meas; is_outlier = false;
                    end
                    if any(isnan(a_corrected)) || is_outlier; return; end
                    a_norm = norm(a_corrected);
                    if a_norm < 0.1 || abs(a_norm - 9.81) > 3.0; return; end
                    sample = [];
                    if ~isempty(varargin) && length(varargin) >= 2
                        sample = varargin{2};
                    end
                    if ~isempty(sample)
                        do_cpp_update(obj, 'accel', a_corrected, sample);
                    else
                        do_cpp_update(obj, 'accel', a_corrected);
                    end

                case 'mag'
                    m_meas = varargin{1};
                    if norm(m_meas - obj.prev_mag) <= obj.buffer_tolerance
                        return;
                    end
                    obj.prev_mag = m_meas;
                    if exist('mex_sensor_filter','file') == 3
                        try
                            [m_filtered, is_outlier] = mex_sensor_filter('mag', m_meas, obj.prev_mag);
                        catch
                            m_filtered = m_meas; is_outlier = false;
                        end
                    elseif exist('SensorFilters','file') == 3
                        [m_filtered, is_outlier] = SensorFilters.mag(m_meas, obj.prev_mag);
                    else
                        m_filtered = m_meas; is_outlier = false;
                    end
                    if any(isnan(m_filtered)) || is_outlier; return; end
                    sample = [];
                    if ~isempty(varargin) && length(varargin) >= 2
                        sample = varargin{2};
                    end
                    if ~isempty(sample)
                        do_cpp_update(obj, 'mag', m_filtered, sample);
                    else
                        do_cpp_update(obj, 'mag', m_filtered);
                    end

                case 'gps'
                    lat = varargin{1}; lon = varargin{2}; alt = varargin{3};
                    if (abs(lat - obj.prev_gps_lat) <= obj.buffer_tolerance) && (abs(lon - obj.prev_gps_lon) <= obj.buffer_tolerance) && (abs(alt - obj.prev_gps_alt) <= obj.buffer_tolerance)
                        return;
                    end
                    obj.prev_gps_lat = lat; obj.prev_gps_lon = lon; obj.prev_gps_alt = alt;
                    lat0 = obj.gps_origin(1); lon0 = obj.gps_origin(2); alt0 = obj.gps_origin(3);
                    y_m = (lat - lat0) / (9.0e-6);
                    x_m = (lon - lon0) / (9.0e-6 / cosd(lat0));
                    z_m = alt - alt0;
                    z_gps = [y_m; x_m; -z_m];
                    sample = [];
                    if ~isempty(varargin) && length(varargin) >= 4
                        sample = varargin{4};
                    end
                    if ~isempty(sample)
                        do_cpp_update(obj, 'gps', z_gps, sample);
                    else
                        do_cpp_update(obj, 'gps', z_gps);
                    end

                case 'baro'
                    pressure = varargin{1};
                    if abs(pressure - obj.prev_baro) <= obj.buffer_tolerance
                        return;
                    end
                    obj.prev_baro = pressure;
                    if exist('mex_sensor_filter','file') == 3
                        try
                            [alt_baro, is_outlier] = mex_sensor_filter('baro', pressure);
                        catch
                            P0 = 101325; ALT_COEFF = 44330;
                            alt_baro = ALT_COEFF * (1 - (pressure / P0)^0.1903);
                            is_outlier = false;
                        end
                    elseif exist('SensorFilters','file') == 3
                        [alt_baro, is_outlier] = SensorFilters.baro(pressure);
                    else
                        P0 = 101325; ALT_COEFF = 44330;
                        alt_baro = ALT_COEFF * (1 - (pressure / P0)^0.1903);
                        is_outlier = false;
                    end
                    if any(isnan(alt_baro)) || is_outlier; return; end
                    weight_factor = 1.0 / obj.baro_weight;
                    obj.P(3,3) = obj.P(3,3) * weight_factor;
                    sample = [];
                    if ~isempty(varargin) && length(varargin) >= 2
                        sample = varargin{2};
                    end
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
            % 内部ヘルパー: C++ MEX に直接渡して更新を行う
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
                    % Handle NoiseEstimator.getRnoise returning either a 3x3 matrix or 3x1 vector
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
                    R_diag = [0.01^2; 0.01^2; 0.01^2];
                    params.noise_zupt = R_diag;
            end

            % 更新フラグ
            sensor_data.update_accel = strcmp(sensor_type, 'accel');
            sensor_data.update_gyro = false;
            sensor_data.update_mag = strcmp(sensor_type, 'mag');
            sensor_data.update_gps = strcmp(sensor_type, 'gps');
            sensor_data.update_baro = strcmp(sensor_type, 'baro');
            sensor_data.update_zupt = strcmp(sensor_type, 'zupt');

            % mex パラメータ整形
            mex_params.g = params.g(:);
            mex_params.mag_ref = params.mag_ref(:);
            mex_params.noise_accel = params.noise_accel(:);
            mex_params.noise_gyro = params.noise_gyro(:);
            mex_params.noise_ba = params.noise_ba(:);
            mex_params.noise_bg = params.noise_bg(:);
            mex_params.noise_mag = params.noise_mag(:);
            mex_params.noise_gps = params.noise_gps(:);
            % Override MEX GPS noise with MATLAB NoiseEstimator current value to
            % ensure parity (use estimator's variance vector). Do this only for GPS
            % updates to avoid changing other sensor behavior.
            if strcmp(sensor_type,'gps')
                tmpR = obj.noiseEstimator.getRnoise('gps');
                if ismatrix(tmpR) && all(size(tmpR)==[3,3])
                    mex_params.noise_gps = diag(tmpR);
                else
                    mex_params.noise_gps = tmpR(:);
                end
            end
            mex_params.noise_baro = params.noise_baro;
            if isfield(params, 'noise_zupt')
                mex_params.noise_zupt = params.noise_zupt(:);
            else
                mex_params.noise_zupt = zeros(3,1);
            end
            mex_params.alpha = params.alpha;
            mex_params.beta = params.beta;
            mex_params.kappa = params.kappa;
            % Pass sample index into mex params for optional C++-side tracing
            if exist('sample','var') && ~isempty(sample)
                mex_params.trace_sample = sample;
            else
                mex_params.trace_sample = NaN;
            end

            % (debug/tracing removed)

            state.p = obj.p(:);
            state.v = obj.v(:);
            state.q = obj.q(:);
            state.ba = obj.ba(:);
            state.bg = obj.bg(:);
            state.P = obj.P;

            % 呼び出しは共通ラッパーに移譲して検証/trace を一元化
            [new_state, dbg_out, mex_debug] = obj.call_meukf_step(state, sensor_data, mex_params, sensor_type, sample);

            % If MEX provides innovation/H/dx, use them for noise estimation
            try
                if isstruct(dbg_out) && isfield(dbg_out, 'innov') && isfield(dbg_out, 'H')
                    try
                        obj.estimate_noise(sensor_type, dbg_out.innov, dbg_out.H, obj.P);
                    catch
                        % Non-fatal: ensure filter still proceeds if estimator fails
                    end
                end
            catch
            end

            % If MEX provided a proposed error-state update 'dx', run divergence guard
            should_apply_mex_state = true;
            try
                if isstruct(dbg_out) && isfield(dbg_out, 'dx')
                    dx_in = dbg_out.dx(:);
                    ctx = struct('sensor', sensor_type, 'sample', sample);
                    try
                        [dx_out, should_skip, was_attenuated] = obj.divergence_check(sensor_type, dbg_out.innov, dx_in, ctx);
                    catch
                        should_skip = false; was_attenuated = false; dx_out = dx_in;
                    end

                    if should_skip
                        % Skip applying this sensor update entirely
                        return;
                    end

                    if was_attenuated && ~isempty(dx_out)
                        % Apply attenuated error-state to previous state instead of mex full state
                        dx_apply = dx_out(:);
                        % position and velocity
                        new_state.p = state.p + dx_apply(1:3);
                        new_state.v = state.v + dx_apply(4:6);
                        % small-angle quaternion correction: dq ~= [1; 0.5*phi]
                        phi = dx_apply(7:9);
                        dq = [1; 0.5 * phi(:)];
                        % quaternion multiply dq * q (both as [w;x;y;z])
                        q1 = dq; q2 = state.q(:);
                        w1=q1(1); x1=q1(2); y1=q1(3); z1=q1(4);
                        w2=q2(1); x2=q2(2); y2=q2(3); z2=q2(4);
                        qw = w1*w2 - x1*x2 - y1*y2 - z1*z2;
                        qx = w1*x2 + x1*w2 + y1*z2 - z1*y2;
                        qy = w1*y2 - x1*z2 + y1*w2 + z1*x2;
                        qz = w1*z2 + x1*y2 - y1*x2 + z1*w2;
                        q_new = [qw; qx; qy; qz];
                        q_new = q_new / norm(q_new);
                        new_state.q = q_new;
                        % biases
                        new_state.ba = state.ba + dx_apply(10:12);
                        new_state.bg = state.bg + dx_apply(13:15);
                        % keep covariance returned by mex (may be conservative)
                        % but ensure symmetry
                        if isfield(new_state, 'P')
                            new_state.P = (new_state.P + new_state.P')/2;
                        else
                            new_state.P = obj.P;
                        end
                        should_apply_mex_state = true;
                    end
                end
            catch
            end

            % 状態割当（デフォルト: MEXの new_state を使用）
            if should_apply_mex_state
                obj.p = new_state.p;
                obj.v = new_state.v;
                obj.q = new_state.q;
                obj.ba = new_state.ba;
                obj.bg = new_state.bg;
                % Ensure covariance symmetry before assigning
                if isfield(new_state, 'P')
                    obj.P = (new_state.P + new_state.P')/2;
                else
                    obj.P = obj.P;
                end
            end
        end

        function [new_state, dbg_out, mex_debug] = call_meukf_step(obj, state, sensor_data, mex_params, sensor_type, sample)
            % Minimal wrapper around mex_meukf_step_v2: call and validate outputs.
            if nargin < 5, sensor_type = 'unknown'; end
            if nargin < 6, sample = []; end

            % Call MEX
            [new_state, dbg_out, mex_debug] = mex_meukf_step_v2(state, sensor_data, mex_params);

            % Validate outputs (no file IO or tracing here)
            fields_ok = isstruct(new_state) && isfield(new_state,'p') && isfield(new_state,'v') && isfield(new_state,'q') && isfield(new_state,'ba') && isfield(new_state,'bg') && isfield(new_state,'P');
            if ~fields_ok || any(isnan([new_state.p(:); new_state.v(:); new_state.q(:); new_state.ba(:); new_state.bg(:)])) || any(isnan(new_state.P(:)))
                error('ESKF:call_meukf_step:NaN','C++ update produced NaN or invalid outputs for sensor %s', sensor_type);
            end
        end

        function varargout = reset(obj, method, varargin)
            % リセット関連メソッド統合
            % 使用例:
            %   obj.reset('check', obs, k)
            %   obj.reset('filter', obs, k)
            
            switch method
                case 'check'
                    check_and_reset_impl(obj, varargin{1}, varargin{2});
                case 'filter'
                    reset_filter_impl(obj, varargin{1}, varargin{2});
                otherwise
                    error('Unknown reset method: %s', method);
            end
        end

        function check_and_reset_impl(obj, obs, k)
            % 発散チェックとリセット
            if isempty(k); return; end
            
            reset_needed = false;
            reason = '';
            
            if any(isnan(obj.p)) || any(isnan(obj.v)) || any(isnan(obj.q))
                reset_needed = true;
                reason = 'NaN in state';
            elseif any(isinf(obj.p)) || any(isinf(obj.v))
                reset_needed = true;
                reason = 'Inf in state';
            end
            
            if reset_needed
                if ~isempty(obs)
                    reset_filter_impl(obj, obs, k);
                else
                    obj.last_reset_step = k;
                    obj.P(1:3, 1:3) = eye(3) * 20.0;
                    obj.P(4:6, 4:6) = eye(3) * 2.0;
                    obj.P(7:9, 7:9) = eye(3) * (deg2rad(30))^2;
                    obj.v = zeros(3,1);
                end
            end
        end

        function reset_filter_impl(obj, obs, k)
            % フィルタをリセット
            obj.last_reset_step = k;
            obj.P = eye(15) * 0.01;
            obj.P(1:3, 1:3) = eye(3) * 20.0;
            obj.P(4:6, 4:6) = eye(3) * 2.0;
            obj.P(7:9, 7:9) = eye(3) * (deg2rad(30))^2;
            obj.v = zeros(3,1);
            
            if isfield(obs, 'lat') && k <= length(obs.lat) && ~isnan(obs.lat(k))
                lat0 = obj.gps_origin(1);
                lon0 = obj.gps_origin(2);
                alt0 = obj.gps_origin(3);
                y_m = (obs.lat(k) - lat0) / (9.0e-6);
                x_m = (obs.lon(k) - lon0) / (9.0e-6 / cosd(lat0));
                z_m = obs.alt(k) - alt0;
                obj.p = [x_m; y_m; z_m];
            end
            
            if any(isnan(obj.q)); obj.q = [1;0;0;0]; end
            obj.ba = zeros(3,1);
            obj.bg = zeros(3,1);
        end

        function varargout = zupt(obj, method, varargin)
            % ZUPT関連メソッド統合
            % 使用例:
            %   is_stat = obj.zupt('check', a_meas, w_meas)
            %   obj.zupt('update')
            
            switch method
                case 'check'
                    varargout{1} = check_stationary_impl(obj, varargin{1}, varargin{2});
                case 'update'
                    update_zupt_impl(obj);
                otherwise
                    error('Unknown ZUPT method: %s', method);
            end
        end

        function is_stat = check_stationary_impl(obj, a_meas, w_meas)
            % 静止判定: 加速度と角速度の大きさをチェック
            a_norm = norm(a_meas);
            gravity_deviation = abs(a_norm - 9.81);
            w_norm = norm(w_meas);
            
            if gravity_deviation < obj.zupt_threshold_accel && w_norm < obj.zupt_threshold_gyro
                obj.zupt_counter = obj.zupt_counter + 1;
            else
                obj.zupt_counter = 0;
            end
            
            obj.is_stationary = (obj.zupt_counter >= obj.zupt_min_duration);
            is_stat = obj.is_stationary;
        end

        function update_zupt_impl(obj)
            % ZUPT (Zero Velocity Update): 速度をゼロに補正
            if ~obj.is_stationary; return; end
            % Prepare sensor/state/params for ZUPT
            sensor_data = struct('accel', zeros(3,1), 'gyro', zeros(3,1), 'mag', zeros(3,1), ...
                'gps_pos', zeros(3,1), 'alt_baro', 0, 'dt', obj.dt);
            params = struct('g', obj.g, 'mag_ref', [50;0;0], 'noise_accel', zeros(3,1), ...
                'noise_gyro', zeros(3,1), 'noise_ba', zeros(3,1), 'noise_bg', zeros(3,1), ...
                'noise_mag', zeros(3,1), 'noise_gps', zeros(3,1), 'noise_baro', 0, ...
                'alpha', 1e-3, 'beta', 2, 'kappa', 0, 'noise_zupt', [0.01^2;0.01^2;0.01^2]);

            sensor_data.update_accel = false;
            sensor_data.update_gyro = false;
            sensor_data.update_mag = false;
            sensor_data.update_gps = false;
            sensor_data.update_baro = false;
            sensor_data.update_zupt = true;

            mex_params.g = params.g(:);
            mex_params.mag_ref = params.mag_ref(:);
            mex_params.noise_accel = params.noise_accel(:);
            mex_params.noise_gyro = params.noise_gyro(:);
            mex_params.noise_ba = params.noise_ba(:);
            mex_params.noise_bg = params.noise_bg(:);
            mex_params.noise_mag = params.noise_mag(:);
            mex_params.noise_gps = params.noise_gps(:);
            mex_params.noise_baro = params.noise_baro;
            mex_params.noise_zupt = params.noise_zupt(:);
            mex_params.alpha = params.alpha;
            mex_params.beta = params.beta;
            mex_params.kappa = params.kappa;

            state.p = obj.p(:);
            state.v = obj.v(:);
            state.q = obj.q(:);
            state.ba = obj.ba(:);
            state.bg = obj.bg(:);
            state.P = obj.P;

            new_state = obj.call_meukf_step(state, sensor_data, mex_params, 'zupt', []);
            obj.p = new_state.p;
            obj.v = new_state.v;
            obj.q = new_state.q;
            obj.ba = new_state.ba;
            obj.bg = new_state.bg;
            obj.P = new_state.P;
        end

        function delete(obj)
            % Free C++ persistent state if allocated
            if isprop(obj, 'state_handle') && ~isempty(obj.state_handle) && obj.state_handle ~= uint64(0)
                if exist('mex_eskf_free', 'file') == 3
                    mex_eskf_free(obj.state_handle);
                end
            end
        end

        function [vel_out, P_out, was_clipped] = divergence_check_velocity_impl(obj, vel_in, P_in, vel_indices)
            % Fallback implementation for check_and_clip_velocity when MEX lacks it
            vel_out = vel_in;
            P_out = P_in;
            was_clipped = false;

            % Define max variances consistent with predict() limits
            max_var = [100^2*ones(3,1); 20^2*ones(3,1); (deg2rad(45))^2*ones(3,1); 0.1*ones(3,1); 0.01*ones(3,1)];

            % vel_indices are indices into the full state (e.g., 4:6), vel_in is 3x1
            for ii = 1:length(vel_indices)
                i = vel_indices(ii);
                if P_out(i,i) > max_var(i)
                    factor = sqrt(max_var(i) / P_out(i,i));
                    P_out(i,:) = P_out(i,:) * factor;
                    P_out(:,i) = P_out(:,i) * factor;
                    P_out(i,i) = max_var(i);
                    was_clipped = true;
                end
            end

            % Clip velocity magnitude to configured maximum (default 3 m/s)
            max_vel = 3.0;
            if norm(vel_out) > max_vel
                vel_out = vel_out * (max_vel / norm(vel_out));
                was_clipped = true;
            end

            % Ensure symmetry
            P_out = (P_out + P_out')/2;
        end
    end
end

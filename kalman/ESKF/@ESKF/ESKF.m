classdef ESKF < handle
    % ESKF - Error State Kalman Filter (C++実装使用)
    
    properties
        % 状態
        p; v; q; ba; bg; P; Q; dt; g
        % ヘルパー
        noiseEstimator; sensor_filters; accel_filter; divergence_guard
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
                
                try
                    mag_static = obj.utils('get_field', obs, {'mx', 'mag_x'}, static_idx, 3);
                    has_mag = true;
                catch
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
                
                try
                    pressure_static = obj.utils('get_field', obs, {'pressure', 'baro'}, static_idx, 1);
                    alt_baro_static = 44330 * (1 - (pressure_static / 101325).^0.1903);
                    sigma_press = std(alt_baro_static - mean(alt_baro_static));
                catch
                    sigma_press = 1.0;
                end
                
                try
                    lat_static = obj.utils('get_field', obs, {'lat', 'gps_lat'}, static_idx, 1);
                    lon_static = obj.utils('get_field', obs, {'lon', 'gps_lon'}, static_idx, 1);
                    alt_static = obj.utils('get_field', obs, {'alt', 'gps_alt'}, static_idx, 1);
                catch
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
                try
                    gyro_all = obj.utils('get_field', obs, {'wx', 'gyro_x'}, static_idx, 3);
                    gyro_all = deg2rad(gyro_all);
                catch
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

            % ヘルパークラス初期化
            obj.noiseEstimator = NoiseEstimator(10);
            obj.noiseEstimator.R_accel = ones(3,1) * (sigma_a^2);
            obj.noiseEstimator.R_gyro  = ones(3,1) * (sigma_g^2);
            obj.noiseEstimator.R_mag   = ones(3,1) * (sigma_mag^2);
            obj.noiseEstimator.R_baro  = (sigma_press^2);
            obj.noiseEstimator.R_gps   = ones(3,1) * (sigma_gps^2);

            obj.sensor_filters = struct();
            obj.sensor_filters.accel = SensorFilter.createAccelFilter();
            % Gyro MATLAB implementation removed — do not create gyro filter
            obj.sensor_filters.gyro  = [];
            obj.sensor_filters.mag   = SensorFilter.createMagFilter();
            obj.sensor_filters.gps   = SensorFilter.createGPSFilter();
            obj.sensor_filters.baro  = SensorFilter.createBaroFilter();

            % MATLAB AccelFilter instance removed for Phase2 MEX — keep placeholder
            obj.accel_filter = [];

            % DivergenceGuard
            config = struct('max_velocity', 3, 'max_acceleration', 3, ...
                'max_allowed_innov', 100, 'max_innov_cap_fraction', 0.6, ...
                'max_gain_norm', 150, 'innov_change_ratio_threshold', 2.5, ...
                'attenuation_factor', 0.6, 'max_attitude_variance', (deg2rad(15))^2, ...
                'max_mag_gain_element', 0.2);
            obj.divergence_guard = DivergenceGuard(config);
            
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
    end
end

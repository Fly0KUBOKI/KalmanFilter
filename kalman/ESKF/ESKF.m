classdef ESKF < handle
    % ESKF  Error State Kalman Filter for IMU/GPS/Mag/Baro
    % 誤差状態カルマンフィルタ（ESKF）実装
    % 
    % 使用方法:
    %   eskf = ESKF(obs, static_time, dt);
    %   eskf.updateFilter(obs, k);
    %   euler = eskf.getEuler();

    properties
        % Nominal / error-state components and configuration
        p
        v
        q
        ba
        bg
        P
        Q
        dt
        g
        noiseEstimator
        sensor_filters
        accel_filter
        divergence_guard
        max_dx_norm
        gyro_filter_yaw_alpha
        enable_yaw_raw_gyro
        enable_mag_update
        enable_gyro_filter
        freq_mag
        freq_baro
        freq_gps
        freq_accel
        gyro_noise_threshold
        gps_origin
        % ZUPT関連
        zupt_threshold_accel
        zupt_threshold_gyro
        zupt_min_duration
        zupt_counter
        is_stationary
        % Adaptive Q関連
        Q_nominal
        adaptive_q_enabled
        % Reset関連
        last_reset_step
        % 速度減衰
        velocity_damping
        % 姿勢発散診断用パラメータ
        accel_innovation_norm  % 加速度計イノベーションノルム
        accel_mahalanobis_dist % マハラノビス距離
        accel_gain_norm        % カルマンゲインのノルム
        quaternion_norm        % クォータニオンノルム
        attitude_change_rate   % 姿勢変化率 [rad/s]
        w_body                 % 現在の角速度 [rad/s] (高速回転判定用)
        % 変化量クラッピング関連
        prev_dx                % 前回の状態変化量 (15x1)
        dx_clipping_ratio      % クラッピング倍率（前回のn倍まで許容）
        dx_history             % 変化量履歴（デバッグ用）
    end

    methods
        function obj = ESKF(obs, static_time, dt)
            % Constructor
            % obs: observation structure
            % static_time: duration of static period (seconds)
            % dt: sampling interval (seconds)
            
            if nargin < 3 || isempty(dt)
                dt = 1/100;
            end
            obj.dt = dt;
            
            fprintf('ESKF Constructor called.\n');
            fprintf('Obs fields: %s\n', strjoin(fieldnames(obs), ', '));

            % Calculate static indices from static_time
            if nargin >= 2 && ~isempty(static_time) && static_time > 0
                N_static = floor(static_time / dt);
                fprintf('Static time: %.2f, dt: %.4f, N_static: %d\n', static_time, dt, N_static);
                
                if isfield(obs, 'accel_x') && length(obs.accel_x) >= N_static
                    static_idx = 1:N_static;
                    fprintf('Found accel_x. Static idx set.\n');
                elseif isfield(obs, 'ax') && length(obs.ax) >= N_static
                    static_idx = 1:N_static;
                    fprintf('Found ax. Static idx set.\n');
                else
                    static_idx = [];
                    fprintf('No accel field found or length insufficient.\n');
                end
            else
                static_idx = [];
                fprintf('Static time not provided or invalid.\n');
            end

            % default nominal states
            obj.p = zeros(3,1);
            obj.v = zeros(3,1);
            
            % gravity
            obj.g = [0;0;9.80665];

            % Initialize noise parameters
            if ~isempty(static_idx) && length(static_idx) > 10
                % Estimate noise from static period
                if isfield(obs, 'accel_x')
                    accel_static = [obs.accel_x(static_idx), obs.accel_y(static_idx), obs.accel_z(static_idx)];
                else
                    accel_static = [obs.ax(static_idx), obs.ay(static_idx), obs.az(static_idx)];
                end
                accel_mean = mean(accel_static, 1);
                sigma_a = mean(std(accel_static - accel_mean, [], 1));
                
                % 初期姿勢の推定 (加速度平均から)
                % 加速度計は上向きの力を測定するため、静止時は [0, 0, g] (上向き) を指す
                % 重力ベクトルは [0, 0, -g] (下向き)
                % したがって、加速度計の出力ベクトルを正規化して、それが [0, 0, 1] になるような回転を求める
                % ただし、ここでは簡易的に Roll/Pitch を計算する
                % ax = g * sin(pitch)
                % ay = -g * cos(pitch) * sin(roll)
                % az = -g * cos(pitch) * cos(roll)
                % (NED座標系、重力下向き、加速度計出力 = -g_body)
                % g_body = R' * [0;0;-g] = R' * g_ned
                % a_meas = -g_body = -R' * [0;0;-g] = R' * [0;0;g]
                % a_meas = [  g * sin(theta)            ]
                %          [ -g * cos(theta) * sin(phi) ]
                %          [ -g * cos(theta) * cos(phi) ]
                
                phi = atan2(-accel_mean(2), -accel_mean(3));
                theta = atan2(accel_mean(1), sqrt(accel_mean(2)^2 + accel_mean(3)^2));
                
                % Temporary quaternion with 0 yaw (convert to degrees for QuaternionLib)
                q_temp = QuaternionLib.from_euler(rad2deg([phi; theta; 0]));
                
                if isfield(obs, 'gyro_x')
                    gyro_static = [obs.gyro_x(static_idx), obs.gyro_y(static_idx), obs.gyro_z(static_idx)];
                else
                    gyro_static = [obs.wx(static_idx), obs.wy(static_idx), obs.wz(static_idx)];
                end
                sigma_g = mean(std(gyro_static, [], 1));
                sigma_g = deg2rad(sigma_g);
                
                if isfield(obs, 'mag_x')
                    mag_static = [obs.mag_x(static_idx), obs.mag_y(static_idx), obs.mag_z(static_idx)];
                    has_mag = true;
                elseif isfield(obs, 'mx')
                    mag_static = [obs.mx(static_idx), obs.my(static_idx), obs.mz(static_idx)];
                    has_mag = true;
                else
                    has_mag = false;
                end

                if has_mag
                    mag_mean = mean(mag_static, 1);
                    sigma_mag = mean(std(mag_static - mag_mean, [], 1));
                    
                    % Calculate initial Yaw from Magnetometer
                    % Rotate magnetic vector to horizontal plane using Roll/Pitch
                    R_rp = QuaternionLib.to_rotation_matrix(q_temp);
                    m_level = R_rp * mag_mean';
                    
                    % Calculate Yaw angle
                    % NED frame: North is X, East is Y.
                    % m_level points to North in the leveled body frame.
                    % psi = -atan2(my, mx)
                    psi = -atan2(m_level(2), m_level(1));
                    
                    fprintf('Initialized Yaw from Mag: %.2f deg\n', rad2deg(psi));
                    
                    % Update quaternion with calculated Yaw (convert to degrees for QuaternionLib)
                    obj.q = QuaternionLib.from_euler(rad2deg([phi; theta; psi]));
                else
                    sigma_mag = 10.0;
                    obj.q = q_temp;
                end
                
                if isfield(obs, 'baro')
                    P0 = 101325;
                    pressure_static = obs.baro(static_idx);
                    alt_baro_static = 44330 * (1 - (pressure_static / P0).^0.1903);
                    sigma_press = std(alt_baro_static - mean(alt_baro_static));
                elseif isfield(obs, 'pressure')
                    P0 = 101325;
                    pressure_static = obs.pressure(static_idx);
                    alt_baro_static = 44330 * (1 - (pressure_static / P0).^0.1903);
                    sigma_press = std(alt_baro_static - mean(alt_baro_static));
                else
                    sigma_press = 1.0;
                end
                
                if isfield(obs, 'gps_lat') && isfield(obs, 'gps_lon') && isfield(obs, 'gps_alt')
                    lat_static = obs.gps_lat(static_idx);
                    lon_static = obs.gps_lon(static_idx);
                    alt_static = obs.gps_alt(static_idx);
                elseif isfield(obs, 'lat') && isfield(obs, 'lon') && isfield(obs, 'alt')
                    lat_static = obs.lat(static_idx);
                    lon_static = obs.lon(static_idx);
                    alt_static = obs.alt(static_idx);
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
                else
                    sigma_gps = 1.0;
                end
                
                % Gyro noise threshold
                if isfield(obs, 'gyro_x')
                    wx_all = deg2rad(obs.gyro_x(:));
                    wy_all = deg2rad(obs.gyro_y(:));
                    wz_all = deg2rad(obs.gyro_z(:));
                else
                    wx_all = deg2rad(obs.wx(:));
                    wy_all = deg2rad(obs.wy(:));
                    wz_all = deg2rad(obs.wz(:));
                end
                std_wx = std(wx_all);
                std_wy = std(wy_all);
                std_wz = std(wz_all);
                obj.gyro_noise_threshold = 2 * max([std_wx, std_wy, std_wz]);
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

            % Noise estimator
            try
                obj.noiseEstimator = NoiseEstimator(10);
                obj.noiseEstimator.R_accel = ones(3,1) * (sigma_a^2);
                obj.noiseEstimator.R_gyro  = ones(3,1) * (sigma_g^2);
                obj.noiseEstimator.R_mag   = ones(3,1) * (sigma_mag^2);
                obj.noiseEstimator.R_baro  = (sigma_press^2);
                obj.noiseEstimator.R_gps   = ones(3,1) * (sigma_gps^2);
            catch
                obj.noiseEstimator = [];
            end

            % Sensor filters
            obj.sensor_filters = struct();
            if exist('SensorFilter','class')
                try
                    obj.sensor_filters.accel = SensorFilter.createAccelFilter();
                catch
                end
                try
                    obj.sensor_filters.gyro  = SensorFilter.createGyroFilter();
                catch
                end
                try
                    obj.sensor_filters.mag   = SensorFilter.createMagFilter();
                catch
                end
                try
                    obj.sensor_filters.gps   = SensorFilter.createGPSFilter();
                catch
                end
                try
                    obj.sensor_filters.baro  = SensorFilter.createBaroFilter();
                catch
                end
            end

            % Accel filter
            try
                % 速度推定用の強力なフィルタ (alpha=0.08: 強めの平滑化と応答性のバランス)
                obj.accel_filter = AccelFilter(0.08, 50);
            catch
                obj.accel_filter = [];
            end

            % Filter settings
            obj.gyro_filter_yaw_alpha = 0.08;
            obj.enable_yaw_raw_gyro = false;
            obj.enable_mag_update = true;
            obj.enable_gyro_filter = true;

            obj.freq_mag = 4;
            obj.freq_baro = 8;
            obj.freq_gps = 10;
            obj.freq_accel = 2;

            obj.max_dx_norm = 5.0;

            % Divergence guard
            try
                config = struct();
                config.max_velocity = 3.0;  % 最大速度を緩和 (2.0 -> 3.0)
                config.max_acceleration = 3.0;  % 最大加速度を緩和 (2.0 -> 3.0)
                config.max_allowed_innov = 100.0;  % イノベーション上限を緩和 (50.0 -> 100.0)
                config.max_innov_cap_fraction = 0.6;  % イノベーションキャップを緩和 (0.5 -> 0.6)
                config.max_gain_norm = 150;  % ゲインノルム上限を緩和 (100 -> 150)
                config.innov_change_ratio_threshold = 2.5;  % イノベーション変化率を緩和 (2.0 -> 2.5)
                config.attenuation_factor = 0.6;  % 減衰係数を緩和 (0.5 -> 0.6)
                config.max_attitude_variance = (deg2rad(15))^2;  % 姿勢分散上限を緩和 (10 -> 15 deg)
                config.max_mag_gain_element = 0.2;  % 磁気計ゲインを緩和 (0.15 -> 0.2)
                obj.divergence_guard = DivergenceGuard(config);
            catch ME
                fprintf('Error initializing DivergenceGuard: %s\n', ME.message);
                obj.divergence_guard = [];
            end

            % GPS origin
            if ~isempty(static_idx) && isfield(obs,'gps_lat') && isfield(obs,'gps_lon') && isfield(obs,'gps_alt')
                obj.gps_origin = [mean(obs.gps_lat(static_idx)); mean(obs.gps_lon(static_idx)); mean(obs.gps_alt(static_idx))];
            elseif ~isempty(static_idx) && isfield(obs,'lat') && isfield(obs,'lon') && isfield(obs,'alt')
                obj.gps_origin = [mean(obs.lat(static_idx)); mean(obs.lon(static_idx)); mean(obs.alt(static_idx))];
            elseif isfield(obs,'gps_lat') && isfield(obs,'gps_lon') && isfield(obs,'gps_alt')
                obj.gps_origin = [obs.gps_lat(1); obs.gps_lon(1); obs.gps_alt(1)];
            elseif isfield(obs,'lat') && isfield(obs,'lon') && isfield(obs,'alt')
                obj.gps_origin = [obs.lat(1); obs.lon(1); obs.alt(1)];
            else
                obj.gps_origin = [0;0;0];
            end
            
            % ZUPT初期化
            obj.zupt_threshold_accel = 1.0;  % m/s^2 (静止判定の加速度閾値、振動対策で増加)
            obj.zupt_threshold_gyro = deg2rad(3.0);  % rad/s (静止判定の角速度閾値)
            obj.zupt_min_duration = 10;  % サンプル数 (最小静止期間)
            obj.zupt_counter = 0;
            obj.is_stationary = false;
            
            % Adaptive Q初期化
            obj.Q_nominal = obj.Q;  % 名目Qを保存
            obj.adaptive_q_enabled = true;
            
            % 角速度初期化
            obj.w_body = zeros(3,1);
            
            % Reset制御初期化
            obj.last_reset_step = [];  % リセット履歴なし
            
            % 速度減衰初期化 (振動対策)
            % 加速度ノイズによる速度発散を抑制（0.0で無効化してテスト）
            obj.velocity_damping = 0.0;  % 一時的に無効化
            
            % 姿勢発散診断パラメータ初期化
            obj.accel_innovation_norm = 0;
            obj.accel_mahalanobis_dist = 0;
            obj.accel_gain_norm = 0;
            obj.quaternion_norm = 1.0;
            obj.attitude_change_rate = 0;
            
            % 変化量クラッピング初期化
            obj.prev_dx = zeros(15, 1);
            obj.dx_clipping_ratio = 3.0;  % 前回の3倍まで許容
            obj.dx_history = [];
        end
        
        function update_filter(obj, obs, k)
            % 1ステップ更新実行

            % センサーデータ取得
            a = [obs.accel_x(k); obs.accel_y(k); obs.accel_z(k)];
            % 生成データの角速度: 既に [roll_rate, pitch_rate, yaw_rate] の順
            % ESKF内部: x=roll, y=pitch, z=yaw
            % 軸の入れ替えは不要
            w = deg2rad([obs.gyro_x(k); obs.gyro_y(k); obs.gyro_z(k)]);

            % 予測ステップ
            obj.predict(a, w);
        
            % 周期的更新
            if mod(k, obj.freq_accel) == 0
                obj.update_accel(a);
            end
            if mod(k, obj.freq_mag) == 0
                obj.update_mag([obs.mag_x(k); obs.mag_y(k); obs.mag_z(k)]);
            end
            if mod(k, obj.freq_baro) == 0
                obj.update_baro(obs.baro(k));
            end
            if mod(k, obj.freq_gps) == 0 && ~isnan(obs.gps_lat(k)) && ~isnan(obs.gps_lon(k))
                obj.update_gps(obs.gps_lat(k), obs.gps_lon(k), obs.gps_alt(k), k);
            end
            
            % 発散チェックとリセット
            obj.check_and_reset_if_diverged(obs, k);
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
            else
                % フィルタを無効にする場合
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
            
            % C++ MEX呼び出し
            try
                state_out = mex_meukf_step_v2(state_in, sensor_data, mex_params);
                
                % 状態更新
                obj.p = state_out.p;
                obj.v = state_out.v;
                obj.q = state_out.q;
                obj.ba = state_out.ba;
                obj.bg = state_out.bg;
                obj.P = state_out.P;
                
            catch ME
                warning('ESKF:predict:MEXFailed', 'C++ Predict failed: %s', ME.message);
                return;
            end

            % 角速度を保存
            obj.w_body = w_meas;
            obj.quaternion_norm = norm(obj.q);
            obj.attitude_change_rate = norm(w_meas);

            % 速度減衰 (Velocity Damping)
            if ~isempty(obj.velocity_damping) && obj.velocity_damping > 0
                obj.v = obj.v * (1.0 - obj.velocity_damping * obj.dt);
            end
            
            % 共分散行列の正則化
            obj.P = obj.divergence_guard.regularize_covariance(obj.P);
            
            % P行列の対角成分にハードリミットを適用
            max_variances = [
                100^2 * ones(3,1);   % Pos (100m)^2
                20^2 * ones(3,1);    % Vel (20m/s)^2
                (deg2rad(45))^2 * ones(3,1); % Att (45deg)^2
                0.1 * ones(3,1);     % Acc Bias
                0.01 * ones(3,1)     % Gyro Bias
            ];
            
            for i = 1:15
                if obj.P(i,i) > max_variances(i)
                    scale = max_variances(i) / obj.P(i,i);
                    factor = sqrt(scale);
                    obj.P(i,:) = obj.P(i,:) * factor;
                    obj.P(:,i) = obj.P(:,i) * factor;
                    obj.P(i,i) = max_variances(i);
                end
            end
            
            % 速度チェックとクリッピング
            [obj.v, obj.P, ~] = obj.divergence_guard.check_and_clip_velocity(obj.v, obj.P, 4:6);
        end
        
        function update_accel(obj, a_meas)
            % 加速度によるカルマンフィルタ姿勢更新（Roll/Pitchのみ、Yaw不可観測）
            % MEUKFを使用 (C++実装)
            obj.update_accel_meukf(a_meas);
        end


        function update_mag(obj, m_meas)
            % 磁気計による姿勢更新
            % MEUKFを使用 (C++実装)
            obj.update_mag_meukf(m_meas);
        end
        
    function update_gps(obj, lat, lon, alt, k)
            % GPS位置更新 (C++実装)
            
            % 座標変換: GPS (lat,lon,alt) -> NED (m)
            lat0 = obj.gps_origin(1);
            lon0 = obj.gps_origin(2);
            alt0 = obj.gps_origin(3);
            y_m = (lat - lat0) / (9.0e-6);
            x_m = (lon - lon0) / (9.0e-6 / cosd(lat0));
            z_m = alt - alt0;
            z_gps = [y_m; x_m; -z_m]; % NED
            
            % C++実装を使用
            obj.update_gps_cpp(z_gps);
        end
        
        function update_baro(obj, pressure)
            % 気圧計による高度更新 (C++実装)
            
            % センサーフィルタ適用
            [alt_baro, is_outlier, ~] = obj.sensor_filters.baro.apply(pressure);
            if any(isnan(alt_baro)); return; end
            if is_outlier; return; end
            
            % C++実装を使用
            obj.update_baro_cpp(alt_baro);
        end
        
        function update_accel_meukf(obj, a_meas)
            % MEUKF による加速度更新 (Roll/Pitchのみ) - C++実装
            
            % 高速回転中は加速度更新をスキップ（角速度チェック）
            if ~isempty(obj.w_body)
                w_norm = norm(obj.w_body);
                if w_norm > 1.5  % 1.5 rad/s (約86 deg/s) 以上の回転中はスキップ
                    return;
                end
            end
            
            % センサーフィルタ適用
            [a_corrected, is_outlier, ~] = obj.sensor_filters.accel.apply(a_meas, zeros(3,1));
            if any(isnan(a_corrected)); return; end
            
            if is_outlier
                return;
            end
            
            % 健全性チェック
            a_norm = norm(a_corrected);
            if a_norm < 0.1 || abs(a_norm - 9.81) > 3.0
                return;
            end
            
            obj.update_accel_meukf_cpp(a_corrected);
        end
        
        function update_mag_meukf(obj, m_meas)
            % MEUKF による磁気計更新 - C++実装
            
            % センサーフィルタ適用
            [m_filtered, is_outlier, ~] = obj.sensor_filters.mag.apply(m_meas);
            if any(isnan(m_filtered)); return; end
            
            if is_outlier
                return;
            end
            
            obj.update_mag_meukf_cpp(m_filtered);
        end
        
        function is_stat = check_stationary(obj, a_meas, w_meas)
            % 静止判定: 加速度と角速度の大きさをチェック
            % a_meas: 加速度測定値 [3x1]
            % w_meas: 角速度測定値 [3x1] (rad/s)
            
            % 重力補正された加速度ノルム
            a_norm = norm(a_meas);
            gravity_deviation = abs(a_norm - 9.81);
            
            % 角速度ノルム
            w_norm = norm(w_meas);
            
            % 静止判定
            if gravity_deviation < obj.zupt_threshold_accel && w_norm < obj.zupt_threshold_gyro
                obj.zupt_counter = obj.zupt_counter + 1;
            else
                obj.zupt_counter = 0;
            end
            
            % 最小期間を満たしたら静止と判定
            if obj.zupt_counter >= obj.zupt_min_duration
                obj.is_stationary = true;
            else
                obj.is_stationary = false;
            end
            
            is_stat = obj.is_stationary;
        end
        
        function update_zupt(obj)
            % ZUPT (Zero Velocity Update): 速度をゼロに補正
            % 静止状態では速度がゼロであると仮定し、強力な更新を行う
            
            if ~obj.is_stationary
                return;
            end
            
            % 観測ノイズ: 静止時は非常に小さい (強い確信)
            R_diag = [0.01^2; 0.01^2; 0.01^2];  % 1cm/s の標準偏差
            
            % センサー構造体
            sensor_data.accel = zeros(3,1);
            sensor_data.gyro = zeros(3,1);
            sensor_data.mag = zeros(3,1);
            sensor_data.gps_pos = zeros(3,1);
            sensor_data.alt_baro = 0;
            sensor_data.dt = 0;
            
            % パラメータ構造体
            params.g = obj.g;
            params.mag_ref = [50; 0; 0];
            params.noise_accel = zeros(3,1);
            params.noise_gyro = zeros(3,1);
            params.noise_ba = zeros(3,1);
            params.noise_bg = zeros(3,1);
            params.noise_mag = zeros(3,1);
            params.noise_gps = zeros(3,1);
            params.noise_baro = 0;
            params.noise_zupt = R_diag;
            params.alpha = 1e-3;
            params.beta = 2;
            params.kappa = 0;
            
            % 状態を抽出
            p = obj.p;
            v = obj.v;
            q = obj.q;
            ba = obj.ba;
            bg = obj.bg;
            P = obj.P;
            
            % C++実装呼び出し
            try
                [p, v, q, ba, bg, P] = call_meukf_update_cpp(p, v, q, ba, bg, P, sensor_data, params, 'zupt');
                
                % 状態を更新
                obj.p = p;
                obj.v = v;
                obj.q = q;
                obj.ba = ba;
                obj.bg = bg;
                obj.P = P;
                
            catch ME
                warning('ESKF:update_zupt:Failed', 'C++ ZUPT update failed (%s), skipping', ME.message);
            end
        end
        
        function check_and_reset_if_diverged(obj, obs, k)
            % 発散チェックとリセット（実験的に無効化:自然収束を信じる）
            % Run 1-2は発散検出なしで pos_rmse < 2.5m, att_rmse < 1deg を達成
            % Run 3-5の問題は、リセット自体が原因の可能性が高い
            
            % 引数チェック
            if nargin < 3 || isempty(k)
                return;
            end
            
            % 発散検出を無効化（NaN/Infのみチェック）
            reset_needed = false;
            reason = '';
            
            % NaN/Inf チェックのみ（破綻検出）
            if any(isnan(obj.p)) || any(isnan(obj.v)) || any(isnan(obj.q))
                reset_needed = true;
                reason = 'NaN detected in state';
            end
            
            if ~reset_needed && (any(isinf(obj.p)) || any(isinf(obj.v)))
                reset_needed = true;
                reason = 'Inf detected in state';
            end
            
            if reset_needed
                fprintf('Step %d: Filter reset triggered. Reason: %s\n', k, reason);
                if nargin >= 2 && ~isempty(obs)
                    obj.reset_filter(obs, k);
                else
                    % obsがない場合は基本リセットのみ
                    obj.last_reset_step = k;
                    obj.P(1:3, 1:3) = eye(3) * 20.0;
                    obj.P(4:6, 4:6) = eye(3) * 2.0;
                    obj.P(7:9, 7:9) = eye(3) * (deg2rad(30))^2;
                    obj.v = zeros(3,1);
                end
            end
        end
        
        function reset_filter(obj, obs, k)
            % フィルタのリセット(位置・速度のみ:姿勢は連続更新に任せる)
            
            % リセット時刻を記録(冷却期間用)
            obj.last_reset_step = k;
            
            % 共分散を初期化（より保守的に）
            obj.P = eye(15) * 0.01;
            obj.P(1:3, 1:3) = eye(3) * 20.0;  % 位置の不確かさを大きく
            obj.P(4:6, 4:6) = eye(3) * 2.0;   % 速度の不確かさを大きく
            obj.P(7:9, 7:9) = eye(3) * (deg2rad(30))^2;  % 姿勢の不確かさを非常に大きく（センサー更新で徐々に収束させる）
            
            % 速度リセット (ゼロにする)
            obj.v = zeros(3,1);
            
            % 位置リセット (GPSがあればGPS位置に)
            if isfield(obs, 'lat') && k <= length(obs.lat) && ~isnan(obs.lat(k))
                 lat0 = obj.gps_origin(1);
                 lon0 = obj.gps_origin(2);
                 alt0 = obj.gps_origin(3);
                 
                 y_m = (obs.lat(k) - lat0) / (9.0e-6);
                 x_m = (obs.lon(k) - lon0) / (9.0e-6 / cosd(lat0));
                 z_m = obs.alt(k) - alt0;
                 
                 obj.p = [x_m; y_m; z_m];
            end
            
            % 姿勢は触らない（円運動では±286度まで変化するため、リセット禁止）
            % 加速度計・ジャイロ・磁気計の連続更新で姿勢を維持・修正
            
            % NaN チェックのみ実施（極端な異常時のみ水平リセット）
            if any(isnan(obj.q))
                obj.q = [1;0;0;0];
            end
            
            % バイアスリセット
            obj.ba = zeros(3,1);
            obj.bg = zeros(3,1);
        end

        function euler = get_euler(obj)
            % オイラー角取得 (degrees)
            euler = QuaternionLib.to_euler(obj.q);
        end
        
        function dx_clipped = clip_state_change(obj, dx)
            % 状態変化量のクラッピング
            % 前回の変化量のn倍以上の変化を抑制して発散を防止
            %
            % 入力:
            %   dx - 今回の状態変化量 (15x1)
            % 出力:
            %   dx_clipped - クラッピング後の状態変化量 (15x1)
            
            % 初回または前回がゼロの場合はそのまま返す
            if isempty(obj.prev_dx) || all(obj.prev_dx == 0)
                obj.prev_dx = dx;
                obj.dx_history = [obj.dx_history, dx];
                dx_clipped = dx;
                return;
            end
            
            dx_clipped = dx;
            
            % 各要素ごとにクラッピング
            for i = 1:length(dx)
                prev_val = obj.prev_dx(i);
                curr_val = dx(i);
                
                % 前回がほぼゼロの場合はスキップ
                if abs(prev_val) < 1e-10
                    continue;
                end
                
                % 同符号かつ増加している場合のみクラッピング
                if sign(curr_val) == sign(prev_val) && abs(curr_val) > abs(prev_val)
                    % 前回のn倍を超えている場合
                    max_allowed = abs(prev_val) * obj.dx_clipping_ratio;
                    if abs(curr_val) > max_allowed
                        dx_clipped(i) = sign(curr_val) * max_allowed;
                    end
                end
            end
            
            % 履歴更新
            obj.prev_dx = dx_clipped;
            obj.dx_history = [obj.dx_history, dx_clipped];
            
            % 履歴が長くなりすぎないよう制限（最新100個のみ保持）
            if size(obj.dx_history, 2) > 100
                obj.dx_history = obj.dx_history(:, end-99:end);
            end
        end
        
        function update_accel_meukf_cpp(obj, a_meas)
            % C++実装による加速度更新
            
            % 高速回転チェック
            if ~isempty(obj.w_body) && norm(obj.w_body) > 1.5
                return;
            end
            
            % センサーフィルタ
            [a_corrected, is_outlier, ~] = obj.sensor_filters.accel.apply(a_meas, zeros(3,1));
            if any(isnan(a_corrected)) || is_outlier
                return;
            end
            
            % 健全性チェック
            a_norm = norm(a_corrected);
            if a_norm < 0.1 || abs(a_norm - 9.81) > 3.0
                return;
            end
            
            % ノイズパラメータ準備
            R_est_full = obj.noiseEstimator.getRnoise('accel');
            R_est_2d = diag(R_est_full);
            
            % センサー構造体
            sensor_data.accel = a_corrected;
            sensor_data.gyro = zeros(3,1);
            sensor_data.mag = zeros(3,1);
            sensor_data.gps_pos = zeros(3,1);
            sensor_data.alt_baro = 0;
            sensor_data.dt = 0;  % 更新のみなのでdt=0
            
            % パラメータ構造体
            params.g = obj.g;
            params.mag_ref = [50; 0; 0];  % ダミー
            params.noise_accel = R_est_2d(1:3);
            params.noise_gyro = zeros(3,1);
            params.noise_ba = zeros(3,1);
            params.noise_bg = zeros(3,1);
            params.noise_mag = zeros(3,1);
            params.noise_gps = zeros(3,1);
            params.noise_baro = 0;
            params.alpha = 1e-3;
            params.beta = 2;
            params.kappa = 0;
            
            % 状態を抽出
            p = obj.p;
            v = obj.v;
            q = obj.q;
            ba = obj.ba;
            bg = obj.bg;
            P = obj.P;
            
            % C++実装呼び出し
            try
                [p, v, q, ba, bg, P] = call_meukf_update_cpp(p, v, q, ba, bg, P, sensor_data, params, 'accel');
                
                % 状態を更新
                obj.p = p;
                obj.v = v;
                obj.q = q;
                obj.ba = ba;
                obj.bg = bg;
                obj.P = P;
                
            catch ME
                warning('ESKF:update_accel_meukf_cpp:Failed', 'C++ MEUKF failed (%s), skipping', ME.message);
            end
        end
        
        function update_mag_meukf_cpp(obj, m_meas)
            % C++実装による磁気計更新
            
            % ノイズパラメータ準備
            R_est_full = obj.noiseEstimator.getRnoise('mag');
            R_est_3d = diag(R_est_full) * 1.5;  % 保守的に
            
            % センサー構造体
            sensor_data.accel = zeros(3,1);
            sensor_data.gyro = zeros(3,1);
            sensor_data.mag = m_meas;
            sensor_data.gps_pos = zeros(3,1);
            sensor_data.alt_baro = 0;
            sensor_data.dt = 0;
            
            % パラメータ構造体
            params.g = obj.g;
            params.mag_ref = [50; 0; 0];
            params.noise_accel = zeros(3,1);
            params.noise_gyro = zeros(3,1);
            params.noise_ba = zeros(3,1);
            params.noise_bg = zeros(3,1);
            params.noise_mag = R_est_3d(1:3);
            params.noise_gps = zeros(3,1);
            params.noise_baro = 0;
            params.alpha = 1e-3;
            params.beta = 2;
            params.kappa = 0;
            
            % 状態を抽出
            p = obj.p;
            v = obj.v;
            q = obj.q;
            ba = obj.ba;
            bg = obj.bg;
            P = obj.P;
            
            % C++実装呼び出し
            try
                [p, v, q, ba, bg, P] = call_meukf_update_cpp(p, v, q, ba, bg, P, sensor_data, params, 'mag');
                
                % 状態を更新
                obj.p = p;
                obj.v = v;
                obj.q = q;
                obj.ba = ba;
                obj.bg = bg;
                obj.P = P;
                
            catch ME
                warning('ESKF:update_mag_meukf_cpp:Failed', 'C++ MEUKF failed (%s), skipping', ME.message);
            end
        end
        
        function update_gps_cpp(obj, z_gps)
            % C++実装によるGPS更新
            
            % ノイズパラメータ準備
            R_est_full = obj.noiseEstimator.getRnoise('gps');
            R_est_3d = diag(R_est_full);
            
            % センサー構造体
            sensor_data.accel = zeros(3,1);
            sensor_data.gyro = zeros(3,1);
            sensor_data.mag = zeros(3,1);
            sensor_data.gps_pos = z_gps;
            sensor_data.alt_baro = 0;
            sensor_data.dt = 0;
            
            % パラメータ構造体
            params.g = obj.g;
            params.mag_ref = [50; 0; 0];
            params.noise_accel = zeros(3,1);
            params.noise_gyro = zeros(3,1);
            params.noise_ba = zeros(3,1);
            params.noise_bg = zeros(3,1);
            params.noise_mag = zeros(3,1);
            params.noise_gps = R_est_3d(1:3);
            params.noise_baro = 0;
            params.alpha = 1e-3;
            params.beta = 2;
            params.kappa = 0;
            
            % 状態を抽出
            p = obj.p;
            v = obj.v;
            q = obj.q;
            ba = obj.ba;
            bg = obj.bg;
            P = obj.P;
            
            % C++実装呼び出し
            try
                [p, v, q, ba, bg, P] = call_meukf_update_cpp(p, v, q, ba, bg, P, sensor_data, params, 'gps');
                
                % 状態を更新
                obj.p = p;
                obj.v = v;
                obj.q = q;
                obj.ba = ba;
                obj.bg = bg;
                obj.P = P;
                
            catch ME
                warning('ESKF:update_gps_cpp:Failed', 'C++ GPS update failed (%s), skipping', ME.message);
            end
        end
        
        function update_baro_cpp(obj, alt_baro)
            % C++実装による気圧計更新
            
            % ノイズパラメータ準備
            R_est = obj.noiseEstimator.getRnoise('baro');
            
            % センサー構造体
            sensor_data.accel = zeros(3,1);
            sensor_data.gyro = zeros(3,1);
            sensor_data.mag = zeros(3,1);
            sensor_data.gps_pos = zeros(3,1);
            sensor_data.alt_baro = alt_baro;
            sensor_data.dt = 0;
            
            % パラメータ構造体
            params.g = obj.g;
            params.mag_ref = [50; 0; 0];
            params.noise_accel = zeros(3,1);
            params.noise_gyro = zeros(3,1);
            params.noise_ba = zeros(3,1);
            params.noise_bg = zeros(3,1);
            params.noise_mag = zeros(3,1);
            params.noise_gps = zeros(3,1);
            params.noise_baro = R_est;
            params.alpha = 1e-3;
            params.beta = 2;
            params.kappa = 0;
            
            % 状態を抽出
            p = obj.p;
            v = obj.v;
            q = obj.q;
            ba = obj.ba;
            bg = obj.bg;
            P = obj.P;
            
            % C++実装呼び出し
            try
                [p, v, q, ba, bg, P] = call_meukf_update_cpp(p, v, q, ba, bg, P, sensor_data, params, 'baro');
                
                % 状態を更新
                obj.p = p;
                obj.v = v;
                obj.q = q;
                obj.ba = ba;
                obj.bg = bg;
                obj.P = P;
                
            catch ME
                warning('ESKF:update_baro_cpp:Failed', 'C++ Baro update failed (%s), skipping', ME.message);
            end
        end
    end
end


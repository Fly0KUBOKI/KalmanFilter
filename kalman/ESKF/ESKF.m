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

            % Calculate static indices from static_time
            if nargin >= 2 && ~isempty(static_time) && static_time > 0
                N_static = floor(static_time / dt);
                if isfield(obs, 'accel_x') && length(obs.accel_x) >= N_static
                    static_idx = 1:N_static;
                else
                    static_idx = [];
                end
            else
                static_idx = [];
            end

            % default nominal states
            obj.p = zeros(3,1);
            obj.v = zeros(3,1);
            obj.q = [1;0;0;0];
            obj.ba = zeros(3,1);
            obj.bg = zeros(3,1);

            % gravity
            obj.g = [0;0;-9.80665];

            % Initialize noise parameters
            if ~isempty(static_idx) && length(static_idx) > 10
                % Estimate noise from static period
                accel_static = [obs.accel_x(static_idx), obs.accel_y(static_idx), obs.accel_z(static_idx)];
                accel_mean = mean(accel_static, 1);
                sigma_a = mean(std(accel_static - accel_mean, [], 1));
                
                gyro_static = [obs.gyro_x(static_idx), obs.gyro_y(static_idx), obs.gyro_z(static_idx)];
                sigma_g = mean(std(gyro_static, [], 1));
                sigma_g = deg2rad(sigma_g);
                
                if isfield(obs, 'mag_x')
                    mag_static = [obs.mag_x(static_idx), obs.mag_y(static_idx), obs.mag_z(static_idx)];
                    mag_mean = mean(mag_static, 1);
                    sigma_mag = mean(std(mag_static - mag_mean, [], 1));
                else
                    sigma_mag = 10.0;
                end
                
                if isfield(obs, 'baro')
                    P0 = 101325;
                    pressure_static = obs.baro(static_idx);
                    alt_baro_static = 44330 * (1 - (pressure_static / P0).^0.1903);
                    sigma_press = std(alt_baro_static - mean(alt_baro_static));
                else
                    sigma_press = 1.0;
                end
                
                if isfield(obs, 'gps_lat') && isfield(obs, 'gps_lon') && isfield(obs, 'gps_alt')
                    lat_static = obs.gps_lat(static_idx);
                    lon_static = obs.gps_lon(static_idx);
                    alt_static = obs.gps_alt(static_idx);
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
                wx_all = deg2rad(obs.gyro_x(:));
                wy_all = deg2rad(obs.gyro_y(:));
                wz_all = deg2rad(obs.gyro_z(:));
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
            end

            % Process noise Q
            obj.Q = zeros(15);
            obj.Q(4:6, 4:6) = eye(3) * (0.01^2);
            obj.Q(7:9, 7:9) = eye(3) * (0.01^2);
            obj.Q(10:12, 10:12) = eye(3) * (sigma_a^2 * 1e-4);
            obj.Q(13:15, 13:15) = eye(3) * (sigma_g^2 * 1e-5);

            % Initial covariance
            % 位置・速度の初期不確かさを大きく設定（GPS更新を効果的にするため）
            obj.P = eye(15) * 0.01;
            obj.P(1:3, 1:3) = eye(3) * 10.0;  % 位置の初期分散（10 m^2）
            obj.P(4:6, 4:6) = eye(3) * 1.0;   % 速度の初期分散（1 m^2/s^2）

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
                obj.accel_filter = AccelFilter(0.3, 20);
            catch
                obj.accel_filter = [];
            end

            % Filter settings
            obj.gyro_filter_yaw_alpha = 0.08;
            obj.enable_yaw_raw_gyro = false;
            obj.enable_mag_update = false;
            obj.enable_gyro_filter = true;

            obj.freq_mag = 4;
            obj.freq_baro = 8;
            obj.freq_gps = 10;
            obj.freq_accel = 4;

            obj.max_dx_norm = 5.0;

            % Divergence guard
            try
                config = struct();
                config.max_velocity = 2.0;
                config.max_acceleration = 2.0;
                config.max_allowed_innov = 50.0;
                config.max_innov_cap_fraction = 0.5;
                config.max_gain_norm = 100;
                config.innov_change_ratio_threshold = 2.0;
                config.attenuation_factor = 0.5;
                config.max_attitude_variance = (deg2rad(10))^2;
                config.max_mag_gain_element = 0.15;
                obj.divergence_guard = DivergenceGuard(config);
            catch
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
        end
        
        function predict(obj, a_meas, w_meas)
            % 予測ステップ

            % ジャイロフィルタ適用
            if isprop(obj, 'enable_gyro_filter') && ~isempty(obj.enable_gyro_filter) && obj.enable_gyro_filter
                % フィルタを使用する場合は既存のセンサーフィルタを適用
                % w_expectedには前回のフィルタ済み値を使用（bgは静止時専用）
                w_expected = obj.sensor_filters.gyro.w_filtered;
                [w_filtered, w_is_outlier, ~] = obj.sensor_filters.gyro.apply(w_meas, w_expected);
                if w_is_outlier
                    % 外れ値の場合は前回のフィルタ済み値を使用
                    w_meas = w_expected;
                else
                    % Yaw 軸の個別処理(必要に応じて生値に戻す)
                    if obj.enable_yaw_raw_gyro
                        w_filtered(3) = w_meas(3);
                    end
                    w_meas = w_filtered;
                end
            else
                % フィルタを無効にする場合: 生の角速度を使用(バイアス補正は integrate_nominal 内で行う)
                % ここでは w_meas をそのまま渡す
            end

            % ノミナル状態の積分
            % --- NoiseEstimatorから閾値を取得 ---
            if ~isempty(obj.noiseEstimator)
                % 軸ごとの閾値を取得（2σを使用）
                [accel_thr_vec, ~] = obj.noiseEstimator.getThreshold('accel', 2.0);
                [gyro_thr_vec, ~] = obj.noiseEstimator.getThreshold('gyro', 2.0);
              
                accel_thr_vec = max(accel_thr_vec, 0.001);  % 最低 0.001 m/s^2
                gyro_thr_vec = max(gyro_thr_vec, obj.gyro_noise_threshold);  % 初期推定値を下限に
            else
                accel_thr_vec = ones(3,1) * 0.1;
                gyro_thr_vec = ones(3,1) * obj.gyro_noise_threshold;
            end

            % ノミナル状態の積分（角速度はそのまま使用）
            [obj.p, obj.v, obj.q, obj.ba, obj.bg] = eskf_core_mex('integrate_nominal', ...
                obj.p, obj.v, obj.q, obj.ba, obj.bg, a_meas, w_meas, obj.dt, obj.g, gyro_thr_vec, accel_thr_vec);

            % 共分散の予測（MEX化）
            obj.P = eskf_core_mex('predict_covariance', obj.P, obj.q, a_meas, obj.ba, w_meas, obj.bg, obj.Q, obj.dt);
            
            % 共分散行列の正則化
            obj.P = obj.divergence_guard.regularize_covariance(obj.P);
            
            % P行列の姿勢部分（7-9行）に上限を適用
            if isfield(obj.divergence_guard.config, 'max_attitude_variance')
                max_var = obj.divergence_guard.config.max_attitude_variance;
                for i = 7:9
                    if obj.P(i,i) > max_var
                        obj.P(i,i) = max_var;
                    end
                end
            end
            
            % 速度チェックとクリッピング
            [obj.v, obj.P, ~] = obj.divergence_guard.check_and_clip_velocity(obj.v, obj.P, 4:6);
        end
        
        function update_accel(obj, a_meas)
            % 加速度によるカルマンフィルタ姿勢更新（Roll/Pitchのみ、Yaw不可観測）
            % ギザギザ抑制: 強力なゲイン制限 + 時間的整合性チェック
            
            % センサーフィルタ適用
            [a_corrected, is_outlier, ~] = obj.sensor_filters.accel.apply(a_meas, zeros(3,1));
            
            if is_outlier
                return;
            end
            
            % 健全性チェック
            a_norm = norm(a_corrected);
            if a_norm < 0.1 || abs(a_norm - 9.81) > 3.0
                return;
            end
            
            % 予測値計算：現在の姿勢から重力のボディ座標系表現を計算
            Rb = QuaternionLib.to_rotation_matrix(obj.q);
            g_body = Rb' * obj.g;  % 重力ベクトルをボディ座標へ変換
            
            % 観測モデル：加速度計は -g（上向き）を測定
            h = -g_body;
            
            % 観測行列：H = ∂h/∂θ = -[g_body]×
            H_full = [zeros(3,6), -RotationLib.skew_symmetric(g_body), zeros(3,6)];
            
            % ジャイロバイアス補正を追加（∂h/∂b_g ≈ (∂h/∂θ)*(∂θ/∂b_g) = H_theta * (-dt)）
            H_full(:,13:15) = -H_full(:,7:9) * obj.dt;
            
            % x,y成分のみを使用（Yaw干渉回避 + 線形化誤差低減）
            H = H_full(1:2, :);
            z = a_corrected(1:2);
            h_pred = h(1:2);
            
            % ノイズ共分散（ArduPilot式: 動的R調整）
            R_est_full = obj.noiseEstimator.getRnoise('accel');
            R_est_2d = diag(R_est_full);
            R_est_2d = R_est_2d(1:2);
            
            % 動的R調整: 重力偏差に応じてRを増減
            gravity_deviation = abs(a_norm - 9.81);
            R_scale = 1.0 + (gravity_deviation / 2.0);  % 0m/s² → 1.0倍, 2m/s² → 2.0倍
            
            % ノイズ下限（平滑化強化: 0.01 → 0.02）
            R_floor = 0.04;  % 測定ノイズを保守的に見積もる
            R = diag(max(R_est_2d, R_floor) * R_scale);
            
            % --- ブロック化最適化: 非ゼロ列のみで計算 ---
            idx_nz = [7:9, 13:15];  % 姿勢とジャイロバイアス（計6列）
            H_sub = H(:, idx_nz);    % 2x6
            P_sub = obj.P(idx_nz, idx_nz);  % 6x6 対称部分
            P_cross = obj.P(:, idx_nz);     % 15x6 (K計算用)
            
            % イノベーション計算
            y = z - h_pred;
            
            % S = H_sub * P_sub * H_sub' + R (2x2)
            S = H_sub * (P_sub * H_sub') + R;
            
            % S正則化（Cholesky前に対称化とジッタ追加）
            S = (S + S') / 2;  % 対称化
            try
                s_rcond = rcond(S);
            catch
                s_rcond = 0;
            end
            if isempty(s_rcond) || s_rcond < 1e-12
                jitter = max(1e-8, abs(trace(S)) * 1e-6);
                S = S + eye(size(S)) * jitter;
            end
            
            R_used = R;
            
            % ArduPilot式 平滑化技術
            
            % 1. イノベーション制限 (Innovation Clamping)
            %    大きなイノベーションを±0.3rad (±17度) に制限 (強化)
            max_innovation = 0.1;  % rad (0.5 → 0.3 でより滑らかに)
            innov_norm = norm(y);
            if innov_norm > max_innovation
                y = y * (max_innovation / innov_norm);  % 正規化して制限
            end
            
            % 2. マハラノビス距離計算
            mahalanobis_dist = sqrt(y' / S * y);
            
            % 3. 5-Sigma圧縮スケール (Compression Scale Factor)
            %    5-sigma以上のイノベーションをスケールダウン
            if mahalanobis_dist > 5.0
                innov_comp_scale = 5.0 / mahalanobis_dist;
                y = y * innov_comp_scale;
                % 注: Sも調整する必要があるが、簡易版では省略
            end
            
            % 4. 外れ値判定（緩和: 3.0 → 5.0）
            if mahalanobis_dist > 5.0
                return;  % 5-sigma以上は棄却
            end
            
            % --- カルマンゲイン計算（ブロック化 + Cholesky安定化） ---
            % K = P * H' / S = P_cross * H_sub' * inv(S)
            % Cholesky分解で安定に解く: S = U'*U => K = P_cross * H_sub' / (U'*U)
            try
                U = chol(S);  % 上三角
                tmp = P_cross * H_sub';  % 15x2
                % solve tmp / S via: tmp = K * S => K = tmp / S
                % tmp' = S' * K' = S * K' (Sは対称)
                % U' * U * K' = tmp' => K' = U \ (U' \ tmp')
                K = (U \ (U' \ tmp'))';  % 15x2
            catch
                % フォールバック: 直接逆行列
                try
                    K = P_cross * (H_sub' / S);  % 15x2
                catch
                    return;
                end
            end
            
            K = obj.divergence_guard.clamp_gain(K);
            
            % 姿勢ゲイン制限（平滑化強化: 0.1 → 0.05）
            % より保守的なゲインで滑らかな応答を実現
            max_attitude_gain = 0.05;  % 5%以下（イノベーション制限強化と併用）
            if size(K,1) >= 9
                K(7:9,:) = max(min(K(7:9,:), max_attitude_gain), -max_attitude_gain);
            end
            
            % 状態修正量計算
            dx = K * y;
            if numel(dx) < 15
                dx_full = zeros(15,1);
                dx_full(1:numel(dx)) = dx(:);
                dx = dx_full;
            end
            
            % 時間的整合性チェック: dx が異常に大きい場合はスケールダウン
            % 平滑化強化のため閾値を下げる
            dx_attitude_norm = norm(dx(7:9));
            if dx_attitude_norm > deg2rad(1.5)  % 1.5度以上の変化は抑制
                scale_down = deg2rad(1.5) / dx_attitude_norm;
                dx(7:9) = dx(7:9) * scale_down;
            end
            
            % 姿勢更新（微小角近似、Yaw不可観測）
            dtheta = dx(7:9);
            dtheta(3) = 0;  % Yaw強制ゼロ
            
            dq = QuaternionLib.small_angle_quat(dtheta);
            obj.q = QuaternionLib.multiply(obj.q, dq);
            obj.q = QuaternionLib.normalize(obj.q);
            
            % 共分散更新（ブロック最適化版: 観測に関連する部分のみ更新）
            % accel は姿勢(7:9)とジャイロバイアス(13:15)のみに影響
            idx_obs = [7:9, 13:15];
            
            % 小ブロックでJoseph更新
            I_KH_block = eye(length(idx_obs)) - K(idx_obs,:) * H(:,idx_obs);
            P_block = obj.P(idx_obs, idx_obs);
            P_block_new = I_KH_block * P_block * I_KH_block' + K(idx_obs,:) * R_used * K(idx_obs,:)';
            
            % 更新されたブロックを戻す
            obj.P(idx_obs, idx_obs) = P_block_new;
            
            % クロス項更新: P(:, idx_obs) の全行
            for i = 1:15
                if ~ismember(i, idx_obs)
                    obj.P(i, idx_obs) = obj.P(i, idx_obs) - K(i,:) * (H(:,idx_obs) * obj.P(idx_obs, idx_obs));
                    obj.P(idx_obs, i) = obj.P(i, idx_obs)';
                end
            end
            
            % P対称化
            obj.P = (obj.P + obj.P') / 2;
            
            % ノイズ推定更新（3要素にパディング）
            y_full = zeros(3,1);
            y_full(1:2) = y;
            H_full_for_estimator = [H; zeros(1,15)];
            obj.noiseEstimator.estimate('accel', y_full, H_full_for_estimator, obj.P);
        end


        function update_mag(obj, m_meas)
            % 磁気計による姿勢更新
            
            % センサーフィルタ適用
            [m_filtered, is_outlier, ~] = obj.sensor_filters.mag.apply(m_meas);
            
            if is_outlier
                return;  % 外れ値の場合は更新をスキップ
            end
            
            m_world = [0; 50; 0];
            Rb = QuaternionLib.to_rotation_matrix(obj.q);
            h_mag = Rb' * m_world;
            
            % 磁気計は正規化されたベクトルとして扱う（SensorFilterと整合）
            h_mag_norm = norm(h_mag);
            if h_mag_norm > 1e-6
                h_mag = h_mag / h_mag_norm;
            end
            
            z = m_filtered;
            h = h_mag;
            H = [zeros(3,6), RotationLib.skew_symmetric(h), zeros(3,6)];
            
            % ジャイロバイアス補正を追加（磁気でYawバイアスを推定）
            H(:,13:15) = -H(:,7:9) * obj.dt;
            
            % 現在のノイズ推定値を使用
            R_est = obj.noiseEstimator.getRnoise('mag');
            
            % --- ブロック化最適化: 非ゼロ列のみで計算 ---
            idx_nz = [7:9, 13:15];  % 姿勢とジャイロバイアス（計6列）
            H_sub = H(:, idx_nz);    % 3x6
            P_sub = obj.P(idx_nz, idx_nz);  % 6x6 対称部分
            P_cross = obj.P(:, idx_nz);     % 15x6 (K計算用)
            
            % イノベーション計算
            y = z - h;
            
            % S = H_sub * P_sub * H_sub' + R_est (3x3)
            S = H_sub * (P_sub * H_sub') + R_est;
            
            % S対称化とCholesky安定化
            S = (S + S') / 2;
            try
                s_rcond = rcond(S);
            catch
                s_rcond = 0;
            end
            if isempty(s_rcond) || s_rcond < 1e-12
                jitter = max(1e-8, abs(trace(S)) * 1e-6);
                S = S + eye(size(S)) * jitter;
            end
            
            R_used = R_est;
            
            % カルマンゲイン計算（Cholesky安定化）
            try
                U = chol(S);  % 上三角
                tmp = P_cross * H_sub';  % 15x3
                K = (U \ (U' \ tmp'))';  % 15x3
            catch
                % フォールバック
                try
                    K = P_cross * (H_sub' / S);
                catch
                    return;
                end
            end
            
            % ノイズ推定を更新
            obj.noiseEstimator.estimate('mag', y, H, obj.P);
            
            % カルマンゲインをクランプ
            K = obj.divergence_guard.clamp_gain(K);
            
            % MAG更新専用のゲイン制限（姿勢部分のみ）
            if isfield(obj.divergence_guard.config, 'max_mag_gain_element')
                max_gain = obj.divergence_guard.config.max_mag_gain_element;
                % K(7:9,:)の各要素を制限（姿勢部分）
                if size(K,1) >= 9
                    K(7:9,:) = max(min(K(7:9,:), max_gain), -max_gain);
                end
            end
            
            dx = K * y;
            
            % dxのサイズ確認とベクトル化
            if numel(dx) < 9
                % dx が十分な要素を持っていない場合、ゼロパディング
                dx_full = zeros(15, 1);
                dx_full(1:numel(dx)) = dx(:);
                dx = dx_full;
            end

            % 磁気計は3軸の姿勢を観測するので、全姿勢誤差dx(7:9)を使用
            dtheta = dx(7:9);
            
            dq = QuaternionLib.small_angle_quat(dtheta);
            obj.q = QuaternionLib.multiply(obj.q, dq);
            obj.q = QuaternionLib.normalize(obj.q);

            % 共分散更新（ブロック最適化版: 観測に関連する部分のみ更新）
            % mag は姿勢(7:9)とジャイロバイアス(13:15)のみに影響
            idx_obs = [7:9, 13:15];
            
            I_KH_block = eye(length(idx_obs)) - K(idx_obs,:) * H(:,idx_obs);
            P_block = obj.P(idx_obs, idx_obs);
            P_block_new = I_KH_block * P_block * I_KH_block' + K(idx_obs,:) * R_used * K(idx_obs,:)';
            obj.P(idx_obs, idx_obs) = P_block_new;
            
            % クロス項更新
            for i = 1:15
                if ~ismember(i, idx_obs)
                    obj.P(i, idx_obs) = obj.P(i, idx_obs) - K(i,:) * (H(:,idx_obs) * obj.P(idx_obs, idx_obs));
                    obj.P(idx_obs, i) = obj.P(i, idx_obs)';
                end
            end
            
            obj.P = (obj.P + obj.P') / 2;
        end
        
    function update_gps(obj, lat, lon, alt, k)
            % GPS位置更新（ブロック化 EKF ベース）
            
            lat0 = obj.gps_origin(1);
            lon0 = obj.gps_origin(2);
            alt0 = obj.gps_origin(3);
            
            % 座標変換
            y_m = (lat - lat0) / (9.0e-6);
            x_m = (lon - lon0) / (9.0e-6 / cosd(lat0));
            z_m = alt - alt0;
            
            z_gps = [x_m; y_m; z_m];
            
            % センサーフィルタ使用（初期化問題のため一時的にスキップ）
            % [z_gps_filtered, is_outlier, ~] = obj.sensor_filters.gps.apply(z_gps);
            % if is_outlier
            %     return;
            % end
            z_gps_filtered = z_gps;  % フィルタをバイパス

            R = obj.noiseEstimator.getRnoise('gps');
            
            % GPS更新前にPを正則化
            obj.P = obj.divergence_guard.regularize_for_ukf(obj.P);
            
            % --- UKF 更新: 位置・速度の6次元サブシステムに適用 ---
            % 状態サブセット: x_sub = [p; v] (6x1)
            idx_pv = 1:6;
            x_sub = [obj.p; obj.v];
            P_sub = obj.P(idx_pv, idx_pv);  % 6x6
            
            % 観測関数: 位置のみを返す (3x1)
            h_func = @(x_pv) x_pv(1:3);
            
            % UKF 更新を実行
            ukf_success = false;
            try
                [x_sub_upd, P_sub_upd, K_ukf, S, y_innov] = ukf_update(x_sub, P_sub, z_gps_filtered, h_func, R);
                ukf_success = true;
            catch ME
                % UKF失敗時はEKFにフォールバック
                warning('ESKF:update_gps:UKF_Failed', 'UKF update failed (%s), using EKF fallback', ME.message);
                ukf_success = false;
            end
            
            % 観測行列（位置のみ）: H = [I3, 0_{3x12}]
            H_gps = [eye(3), zeros(3, 12)];
            
            if ukf_success
                % UKF成功時: Sとy_innovは既に計算済み
                % 全状態へのカルマンゲインを計算
                S = (S + S') / 2;
                if rcond(S) < 1e-12
                    S = S + eye(3) * 1e-8;
                end
                
                try
                    U = chol(S);
                    tmp = obj.P(:, 1:3);
                    K = (U \ (U' \ tmp'))';  % 15x3
                catch
                    K = obj.P(:, 1:3) / S;
                end
                
                K = obj.divergence_guard.clamp_gain(K);
                dx = K * y_innov;
            else
                % EKFフォールバック
                idx_pos = 1:3;
                y_innov = z_gps_filtered - obj.p;
                P_pos = obj.P(idx_pos, idx_pos);
                S = P_pos + R;
                S = (S + S') / 2;
                
                try
                    U = chol(S);
                    tmp = obj.P(:, idx_pos);
                    K = (U \ (U' \ tmp'))';
                catch
                    K = obj.P(:, idx_pos) / S;
                end
                
                K = obj.divergence_guard.clamp_gain(K);
                dx = K * y_innov;
            end
            
            % OutlierGuard チェック（UKF成功時は信頼してスキップ）
            if ukf_success
                % UKFが成功した場合は既にsigma pointsでロバストに処理されているため、
                % OutlierGuardをバイパスして直接更新を適用
                should_update = true;
                y_used = y_innov;
                dx_used = [];
            else
                % EKFフォールバック時のみOutlierGuardを使用
                H_gps = [eye(3), zeros(3, 12)];
                R_updated = obj.noiseEstimator.getRnoise('gps');
                ctx = struct();
                ctx.k = k;
                ctx.z = z_gps;
                ctx.h = obj.p;
                ctx.y = y_innov;
                ctx.P_diag = diag(obj.P);
                ctx.R_diag = diag(R);
                ctx.gps = ctx;
                [should_update, y_used, ~, dx_used, ~] = OutlierGuard.checkAndApply('gps', z_gps, obj.p, H_gps, obj.P, R_updated, [], dx, obj.divergence_guard, obj.noiseEstimator, ctx);
            end
            
            if ~should_update
                return;
            end

            % ノイズ推定を更新
            obj.noiseEstimator.estimate('gps', y_used, H_gps, obj.P);

            % クリップ済み dx があれば使う
            if ~isempty(dx_used)
                dx = dx_used;
            end

            % 状態更新
            if ukf_success
                % UKF成功時: 更新された状態との差分を適用
                % x_sub_updは絶対値なので、事前推定x_subとの差分をとる
                dx_ukf = x_sub_upd - x_sub;
                obj.p = obj.p + dx_ukf(1:3);
                obj.v = obj.v + dx_ukf(4:6);
            else
                % EKFフォールバック時: dx補正を適用
                obj.p = obj.p + dx(1:3);
                obj.v = obj.v + dx(4:6);
            end
            
            % 共分散更新
            if ukf_success
                % UKF成功時: サブシステムの更新済み共分散を使用
                idx_obs = 1:6;
                obj.P(idx_obs, idx_obs) = P_sub_upd;
                
                % クロス項更新 (姿勢・バイアスとの相関を更新)
                for i = 7:15
                    obj.P(i, idx_obs) = obj.P(i, idx_obs) - K(i,:) * (H_gps(:,idx_obs) * obj.P(idx_obs, idx_obs));
                    obj.P(idx_obs, i) = obj.P(i, idx_obs)';
                end
            else
                % EKFフォールバック時: Joseph形式で更新
                idx_obs = 1:6;
                I_KH_block = eye(length(idx_obs)) - K(idx_obs,:) * H_gps(:,idx_obs);
                P_block = obj.P(idx_obs, idx_obs);
                P_block_new = I_KH_block * P_block * I_KH_block' + K(idx_obs,:) * R_updated * K(idx_obs,:)';
                obj.P(idx_obs, idx_obs) = P_block_new;
                
                % クロス項更新
                for i = 7:15
                    obj.P(i, idx_obs) = obj.P(i, idx_obs) - K(i,:) * (H_gps(:,idx_obs) * obj.P(idx_obs, idx_obs));
                    obj.P(idx_obs, i) = obj.P(i, idx_obs)';
                end
            end
            
            obj.P = (obj.P + obj.P') / 2;
            
            % 速度チェックとクリッピング
            [obj.v, obj.P, ~] = obj.divergence_guard.check_and_clip_velocity(obj.v, obj.P, 4:6);
        end
        
        function update_baro(obj, pressure)
            % 気圧計による高度更新
            
            % センサーフィルタ適用
            [alt_baro, is_outlier, ~] = obj.sensor_filters.baro.apply(pressure);
            
            if is_outlier
                return;  % 外れ値の場合は更新をスキップ
            end
            
            H = [0,0,1, zeros(1,12)];
            z = alt_baro;
            h = obj.p(3);
            
            % 現在のノイズ推定値を使用
            R_est = obj.noiseEstimator.getRnoise('baro');
            
            [y, S, R_used] = kalman_filter_core('compute_innovation_and_S', z, h, H, obj.P, R_est, struct());
            
            % フィルタリング（外れ値判定）
            [y_filtered, should_update] = SensorFilter.filterInnovation(y, R_used);
            if ~should_update
                return;
            end
            
            % --- 外れ値でない場合のみノイズ推定を更新 ---
            obj.noiseEstimator.estimate('baro', y_filtered, H, obj.P);
            
            K = kalman_filter_core('compute_kalman_gain', obj.P, H, S);
            K = obj.divergence_guard.clamp_gain(K);
            
            % dx計算: Kは15x1、y_filteredはスカラー
            dx = K * y_filtered;
            
            % dxのサイズ確認とベクトル化
            if numel(dx) == 1
                % スカラーの場合、位置の高度のみ更新
                dz = dx;
                dx = zeros(15, 1);
                dx(3) = dz;
            end
            
            % 高度更新（閾値チェック付き）
            if abs(dx(3)) >= 0.1
                obj.p(3) = obj.p(3) + dx(3);
            end
            
            % 共分散更新
            x_pred = zeros(15,1);
            x_pred(1:3) = obj.p;
            [~, obj.P] = kalman_filter_core('update_state_covariance', x_pred, obj.P, K, H, y_filtered, R_used);
        end
        
        function euler = get_euler(obj)
            % オイラー角取得
            euler = QuaternionLib.to_euler(obj.q);
        end
    end
end

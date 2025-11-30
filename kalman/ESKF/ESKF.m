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
        % MEUKF関連
        use_meukf
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
            
            % gravity
            obj.g = [0;0;-9.80665];

            % Initialize noise parameters
            if ~isempty(static_idx) && length(static_idx) > 10
                % Estimate noise from static period
                accel_static = [obs.accel_x(static_idx), obs.accel_y(static_idx), obs.accel_z(static_idx)];
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
                
                obj.q = QuaternionLib.from_euler([phi; theta; 0]);
                
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
                obj.q = [1;0;0;0]; % 静止データがない場合は水平と仮定
            end
            
            obj.ba = zeros(3,1);
            obj.bg = zeros(3,1);

            % Process noise Q
            obj.Q = zeros(15);
            obj.Q(4:6, 4:6) = eye(3) * (0.005^2); % 速度のランダムウォークを小さく (0.01 -> 0.005)
            obj.Q(7:9, 7:9) = eye(3) * (0.005^2); % 姿勢のランダムウォークを小さく (0.01 -> 0.005)
            obj.Q(10:12, 10:12) = eye(3) * (sigma_a^2 * 1e-5); % バイアス変動を小さく
            obj.Q(13:15, 13:15) = eye(3) * (sigma_g^2 * 1e-6);

            % Initial covariance
            % 位置・速度の初期不確かさを大きく設定（GPS更新を効果的にするため）
            obj.P = eye(15) * 0.01;
            obj.P(1:3, 1:3) = eye(3) * 10.0;  % 位置の初期分散（10 m^2）
            obj.P(4:6, 4:6) = eye(3) * 0.1;   % 速度の初期分散を小さく (1.0 -> 0.1) 初期は静止しているため
            obj.P(10:12, 10:12) = eye(3) * 0.1; % 加速度バイアスの初期分散を少し大きくして収束を早める

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
                % 速度推定用の強力なフィルタ (alpha=0.05: 非常に強い平滑化)
                obj.accel_filter = AccelFilter(0.05, 50);
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
            
            % ZUPT初期化
            obj.zupt_threshold_accel = 0.5;  % m/s^2 (静止判定の加速度閾値)
            obj.zupt_threshold_gyro = deg2rad(2.0);  % rad/s (静止判定の角速度閾値)
            obj.zupt_min_duration = 10;  % サンプル数 (最小静止期間)
            obj.zupt_counter = 0;
            obj.is_stationary = false;
            
            % Adaptive Q初期化
            obj.Q_nominal = obj.Q;  % 名目Qを保存
            obj.adaptive_q_enabled = true;
            
            % MEUKF初期化
            obj.use_meukf = true;  % MEUKFを有効化 (ZUPT + Adaptive Q + MEUKF)
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
            if isprop(obj, 'enable_gyro_filter') && ~isempty(obj.enable_gyro_filter) && obj.enable_gyro_filter && ...
               isfield(obj.sensor_filters, 'gyro') && ~isempty(obj.sensor_filters.gyro)
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
            
            % 加速度フィルタ適用（速度推定用）
            % ユーザー要望: 強めにフィルタリングをしてそれを積分する
            if ~isempty(obj.accel_filter)
                % 期待値として現在の推定加速度（重力除去前）を使用したいが、
                % 簡易的に前回のフィルタ値を使用
                a_expected = obj.accel_filter.a_filtered;
                if norm(a_expected) < 1e-3
                    a_expected = a_meas; % 初期化
                end
                
                [a_filtered, is_outlier] = obj.accel_filter.filter(a_meas, a_expected);
                
                if is_outlier
                    % 外れ値の場合は更新しない（前回の値を使うか、あるいは信頼度を下げる）
                    % ここではフィルタ値をそのまま使う（filterメソッド内で前回値を返すようになっている）
                end
                
                % 速度更新にはフィルタリングされた加速度を使用
                a_for_vel = a_filtered;
            else
                a_for_vel = a_meas;
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

            % ノミナル状態の積分
            % 速度更新には a_for_vel を使用し、姿勢更新には w_meas を使用
            % eskf_core_mex は単一の加速度入力を受け取るため、
            % 姿勢更新にも a_for_vel が使われることになるが、
            % 姿勢の予測ステップでは加速度は使われない（角速度のみ）ので問題ない。
            % (加速度は update_accel で姿勢補正に使われる)
            
            [obj.p, obj.v, obj.q, obj.ba, obj.bg] = eskf_core_mex('integrate_nominal', ...
                obj.p, obj.v, obj.q, obj.ba, obj.bg, a_for_vel, w_meas, obj.dt, obj.g, gyro_thr_vec, accel_thr_vec);

            % Adaptive Q: IMU信号の安定性に基づいてQをスケーリング
            Q_adapted = obj.Q;
            if obj.adaptive_q_enabled
                % 加速度ノルムが重力に近いほど信頼度が高い (スケール減少)
                a_norm = norm(a_meas);
                gravity_error = abs(a_norm - 9.81);
                accel_scale = 1.0 + (gravity_error / 2.0);  % 重力から2m/s^2離れると2倍
                
                % 角速度が小さいほど信頼度が高い (スケール減少)
                w_norm = norm(w_meas);
                gyro_scale = 1.0 + (w_norm / deg2rad(10.0));  % 10deg/sで2倍
                
                % 総合スケール (保守的: 大きい方を採用)
                q_scale = max(accel_scale, gyro_scale);
                q_scale = min(q_scale, 3.0);  % 上限3倍に削減して安定性向上
                
                Q_adapted = obj.Q_nominal * q_scale;
            end
            
            % 共分散の予測（MEX化）
            % ここでも a_for_vel を使用
            obj.P = eskf_core_mex('predict_covariance', obj.P, obj.q, a_for_vel, obj.ba, w_meas, obj.bg, Q_adapted, obj.dt);
            
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
            % use_meukf=true の場合はMEUKFを使用、falseの場合はEKFを使用
            
            % MEUKFモード
            if obj.use_meukf
                obj.update_accel_meukf(a_meas);
                return;
            end
            
            % 以下、EKFモード
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
            R_scale = 1.0 + (gravity_deviation / 1.0);  % 感度を下げる (2.0 -> 1.0)
            
            % ノイズ下限（平滑化強化: 0.05 → 0.1）
            R_floor = 0.1;  % 測定ノイズをより保守的に見積もる
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
            
            % 姿勢ゲイン制限（平滑化強化: 0.04 → 0.02）
            % より保守的なゲインで滑らかな応答を実現
            max_attitude_gain = 0.02;  % 2%以下（イノベーション制限強化と併用）
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
            if dx_attitude_norm > deg2rad(0.5)  % 0.5度以上の変化は抑制 (1.0 -> 0.5)
                scale_down = deg2rad(0.5) / dx_attitude_norm;
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
            % use_meukf=true の場合はMEUKFを使用
            
            % MEUKFモード
            if obj.use_meukf
                obj.update_mag_meukf(m_meas);
                return;
            end
            
            % 以下、EKFモード
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
                % alpha=1e-3, beta=2, kappa=0 (標準設定)
                [x_sub_upd, P_sub_upd, K_ukf, S, y_innov] = ukf_update(x_sub, P_sub, z_gps_filtered, h_func, R, 1e-3, 2, 0);
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
                
                % UKFの場合でも、イノベーションが大きすぎる場合は棄却する (外れ値対策強化)
                % 位置のイノベーションが 5m 以上なら棄却
                if norm(y_innov(1:3)) > 5.0
                    should_update = false;
                end
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
        
        function update_accel_meukf(obj, a_meas)
            % MEUKF による加速度更新 (Roll/Pitchのみ)
            % 外れ値パルス抑制強化版
            
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
            
            % 観測関数: クォータニオンから加速度予測値を計算
            h_func = @(q) compute_accel_observation(q, obj.g);
            
            % ノイズ共分散 (動的R調整)
            R_est_full = obj.noiseEstimator.getRnoise('accel');
            R_est_2d = diag(R_est_full);
            R_est_2d = R_est_2d(1:2);
            
            % 動的R調整 (バランス調整)
            gravity_deviation = abs(a_norm - 9.81);
            R_scale = 1.0 + (gravity_deviation / 0.7);  % 感度を少し緩和 (0.5 -> 0.7)
            R_floor = 0.25;  % ノイズフロアを上げて安定性向上 (0.15 -> 0.25)
            R = diag(max(R_est_2d, R_floor) * R_scale);
            
            % 観測値 (x, y成分のみ)
            z = a_corrected(1:2);
            
            % 姿勢誤差の共分散 (3x3)
            P_attitude = obj.P(7:9, 7:9);
            
            % MEUKF更新
            try
                [dtheta, P_attitude_upd, K, S, y] = meukf_update_attitude(P_attitude, obj.q, z, h_func, R);
            catch ME
                warning('ESKF:update_accel_meukf:Failed', 'MEUKF failed (%s), skipping', ME.message);
                return;
            end
            
            % イノベーション制限 (安定化: 0.05rad ≈ 2.9度)
            max_innovation = 0.05;
            innov_norm = norm(y);
            if innov_norm > max_innovation
                y = y * (max_innovation / innov_norm);
            end
            
            % マハラノビス距離チェック (3.5-sigma棄却で安定性向上)
            try
                mahal_dist = sqrt(y' / S * y);
                if mahal_dist > 3.5
                    return;  % 外れ値として棄却
                end
                % 2.5-sigma以上は減衰
                if mahal_dist > 2.5
                    attenuation = 2.5 / mahal_dist;
                    y = y * attenuation;
                end
            catch
                % S が特異の場合はスキップ
                return;
            end
            
            % dtheta再計算 (フィルタリング後のイノベーションで)
            dtheta = K * y;
            
            % dthetaの大きさ制限 (0.6度で安定化)
            dtheta_norm = norm(dtheta(1:2));  % Roll/Pitchのみチェック
            if dtheta_norm > deg2rad(0.6)
                scale = deg2rad(0.6) / dtheta_norm;
                dtheta(1:2) = dtheta(1:2) * scale;
            end
            
            % Yaw成分を強制ゼロ
            dtheta(3) = 0;
            
            % 姿勢更新
            dq = QuaternionLib.small_angle_quat(dtheta);
            obj.q = QuaternionLib.multiply(obj.q, dq);
            obj.q = QuaternionLib.normalize(obj.q);
            
            % 共分散更新: MEUKFで得られた姿勢ブロック + フルクロス項の更新
            P_attitude_upd = (P_attitude_upd + P_attitude_upd') / 2;
            max_var = (deg2rad(5))^2;
            for i = 1:3
                if P_attitude_upd(i,i) > max_var
                    P_attitude_upd(i,i) = max_var;
                end
            end
            
            % フル状態のクロス共分散を更新（EKFブランチと同様のロジック）
            % 観測に関連するインデックス: 姿勢(7:9)のみ（加速度は姿勢のみ観測）
            idx_obs = 7:9;
            
            % 観測行列の近似（線形化: 加速度のx,y成分に対する姿勢の感度）
            % H_sub ≈ MEUKFのPxzから逆算するか、EKFと同じ線形化を使用
            Rb = QuaternionLib.to_rotation_matrix(obj.q);
            g_body = Rb' * obj.g;
            H_attitude = -RotationLib.skew_symmetric(g_body);
            H_sub = H_attitude(1:2, :);  % x,y成分のみ
            
            % フル状態に対するカルマンゲイン（EKFと整合的に計算）
            P_cross = obj.P(:, idx_obs);  % 15x3
            try
                % K_full = P_cross * H_sub' / S
                U = chol(S);
                tmp = P_cross * H_sub';  % 15x2
                K_full = (U \ (U' \ tmp'))';  % 15x2
            catch
                try
                    K_full = P_cross * (H_sub' / S);
                catch
                    % フォールバック: 姿勢ブロックのみ更新
                    obj.P(idx_obs, idx_obs) = P_attitude_upd;
                    obj.P = (obj.P + obj.P') / 2;
                    % ノイズ推定更新
                    y_full = zeros(3,1);
                    y_full(1:2) = y;
                    H_full = [zeros(3,6), eye(3), zeros(3,6)];
                    obj.noiseEstimator.estimate('accel', y_full, H_full, obj.P);
                    return;
                end
            end
            
            % ゲインクリッピング
            K_full = obj.divergence_guard.clamp_gain(K_full);
            
            % Joseph形式でフル共分散を更新
            I_KH_block = eye(length(idx_obs)) - K_full(idx_obs,:) * H_sub;
            obj.P(idx_obs, idx_obs) = I_KH_block * P_attitude_upd * I_KH_block' + K_full(idx_obs,:) * R * K_full(idx_obs,:)';
            
            % クロス項更新
            for i = 1:15
                if ~ismember(i, idx_obs)
                    obj.P(i, idx_obs) = obj.P(i, idx_obs) - K_full(i,:) * (H_sub * obj.P(idx_obs, idx_obs));
                    obj.P(idx_obs, i) = obj.P(i, idx_obs)';
                end
            end
            
            obj.P = (obj.P + obj.P') / 2;
            
            % ノイズ推定更新
            y_full = zeros(3,1);
            y_full(1:2) = y;
            H_full = [zeros(3,6), eye(3), zeros(3,6)];
            obj.noiseEstimator.estimate('accel', y_full, H_full, obj.P);
        end
        
        function update_mag_meukf(obj, m_meas)
            % MEUKF による磁気計更新
            % 外れ値パルス抑制強化版
            
            % センサーフィルタ適用
            [m_filtered, is_outlier, ~] = obj.sensor_filters.mag.apply(m_meas);
            
            if is_outlier
                return;
            end
            
            % 地磁気ベクトル (北向き)
            m_world = [0; 50; 0];
            
            % 観測関数
            h_func = @(q) compute_mag_observation(q, m_world);
            
            % ノイズ共分散 (保守的に)
            R_est = obj.noiseEstimator.getRnoise('mag');
            R_est = R_est * 1.5;  % 磁気計ノイズを増やして保守的に
            
            % 姿勢誤差の共分散 (3x3)
            P_attitude = obj.P(7:9, 7:9);
            
            % MEUKF更新
            try
                [dtheta, P_attitude_upd, K, S, y] = meukf_update_attitude(P_attitude, obj.q, m_filtered, h_func, R_est);
            catch ME
                warning('ESKF:update_mag_meukf:Failed', 'MEUKF failed (%s), skipping', ME.message);
                return;
            end
            
            % イノベーション制限 (0.1rad ≈ 6度)
            max_innovation = 0.1;
            innov_norm = norm(y);
            if innov_norm > max_innovation
                y = y * (max_innovation / innov_norm);
            end
            
            % マハラノビス距離チェック (4-sigma棄却、磁気計は少し緩め)
            try
                mahal_dist = sqrt(y' / S * y);
                if mahal_dist > 4.0
                    return;
                end
                if mahal_dist > 2.5
                    attenuation = 2.5 / mahal_dist;
                    y = y * attenuation;
                end
            catch
                return;
            end
            
            % dtheta再計算
            dtheta = K * y;
            
            % dthetaの大きさ制限 (Yaw含めて1.0度以下に)
            dtheta_norm = norm(dtheta);
            if dtheta_norm > deg2rad(1.0)
                scale = deg2rad(1.0) / dtheta_norm;
                dtheta = dtheta * scale;
            end
            
            % 姿勢更新 (磁気計は全3軸更新可能)
            dq = QuaternionLib.small_angle_quat(dtheta);
            obj.q = QuaternionLib.multiply(obj.q, dq);
            obj.q = QuaternionLib.normalize(obj.q);
            
            % 共分散更新: MEUKFで得られた姿勢ブロック + フルクロス項の更新
            P_attitude_upd = (P_attitude_upd + P_attitude_upd') / 2;
            max_var = (deg2rad(8))^2;
            for i = 1:3
                if P_attitude_upd(i,i) > max_var
                    P_attitude_upd(i,i) = max_var;
                end
            end
            
            % フル状態のクロス共分散を更新（EKFブランチと同様）
            idx_obs = 7:9;
            
            % 観測行列（磁気計は全3軸）
            Rb = QuaternionLib.to_rotation_matrix(obj.q);
            h_mag = Rb' * m_world;
            h_mag_norm = norm(h_mag);
            if h_mag_norm > 1e-6
                h_mag = h_mag / h_mag_norm;
            end
            H_sub = RotationLib.skew_symmetric(h_mag);  % 3x3
            
            % フル状態に対するカルマンゲイン
            P_cross = obj.P(:, idx_obs);  % 15x3
            try
                U = chol(S);
                tmp = P_cross * H_sub';  % 15x3
                K_full = (U \ (U' \ tmp'))';  % 15x3
            catch
                try
                    K_full = P_cross * (H_sub' / S);
                catch
                    % フォールバック
                    obj.P(idx_obs, idx_obs) = P_attitude_upd;
                    obj.P = (obj.P + obj.P') / 2;
                    H = [zeros(3,6), eye(3), zeros(3,6)];
                    obj.noiseEstimator.estimate('mag', y, H, obj.P);
                    return;
                end
            end
            
            % ゲインクリッピング
            K_full = obj.divergence_guard.clamp_gain(K_full);
            
            % MAG専用ゲイン制限
            if isfield(obj.divergence_guard.config, 'max_mag_gain_element')
                max_gain = obj.divergence_guard.config.max_mag_gain_element;
                if size(K_full,1) >= 9
                    K_full(7:9,:) = max(min(K_full(7:9,:), max_gain), -max_gain);
                end
            end
            
            % Joseph形式でフル共分散を更新
            I_KH_block = eye(length(idx_obs)) - K_full(idx_obs,:) * H_sub;
            obj.P(idx_obs, idx_obs) = I_KH_block * P_attitude_upd * I_KH_block' + K_full(idx_obs,:) * R_est * K_full(idx_obs,:)';
            
            % クロス項更新
            for i = 1:15
                if ~ismember(i, idx_obs)
                    obj.P(i, idx_obs) = obj.P(i, idx_obs) - K_full(i,:) * (H_sub * obj.P(idx_obs, idx_obs));
                    obj.P(idx_obs, i) = obj.P(i, idx_obs)';
                end
            end
            
            obj.P = (obj.P + obj.P') / 2;
            
            % ノイズ推定更新
            H = [zeros(3,6), eye(3), zeros(3,6)];
            obj.noiseEstimator.estimate('mag', y, H, obj.P);
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
            
            % 観測: 速度 = [0; 0; 0]
            z_vel = [0; 0; 0];
            h_vel = obj.v;  % 予測速度
            
            % 観測行列: H = [0_{3x3}, I_{3x3}, 0_{3x9}]
            H = [zeros(3,3), eye(3), zeros(3,9)];
            
            % 観測ノイズ: 静止時は非常に小さい (強い確信)
            R = eye(3) * (0.01^2);  % 1cm/s の標準偏差
            
            % イノベーション
            y = z_vel - h_vel;
            
            % S = H * P * H' + R
            S = H * obj.P * H' + R;
            S = (S + S') / 2;  % 対称化
            
            % Cholesky安定化
            try
                s_rcond = rcond(S);
            catch
                s_rcond = 0;
            end
            if isempty(s_rcond) || s_rcond < 1e-12
                jitter = max(1e-8, abs(trace(S)) * 1e-6);
                S = S + eye(size(S)) * jitter;
            end
            
            % カルマンゲイン
            try
                U = chol(S);
                tmp = obj.P * H';
                K = (U \ (U' \ tmp'))';
            catch
                K = obj.P * H' / S;
            end
            
            % ゲインクリッピング
            if ~isempty(obj.divergence_guard)
                K = obj.divergence_guard.clamp_gain(K);
            end
            
            % 状態更新
            dx = K * y;
            
            % 速度更新
            obj.v = obj.v + dx(4:6);
            
            % 共分散更新 (Joseph form)
            I_KH = eye(15) - K * H;
            obj.P = I_KH * obj.P * I_KH' + K * R * K';
            obj.P = (obj.P + obj.P') / 2;  % 対称化
        end
        
        function euler = get_euler(obj)
            % オイラー角取得
            euler = QuaternionLib.to_euler(obj.q);
        end
    end
end

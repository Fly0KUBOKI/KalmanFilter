classdef ESKF < handle
    % ESKF  Error State Kalman Filter for IMU/GPS/Mag/Baro
    % 誤差状態カルマンフィルタ（ESKF）実装
    % 
    % 使用方法:
    %   eskf = ESKF(obs, static_time, dt);
    %   eskf.updateFilter(obs, k);
    %   euler = eskf.getEuler();

    properties
        % ノミナル状態
        p           % 位置 [x; y; z] (m)
        v           % 速度 [vx; vy; vz] (m/s)
        q           % 姿勢クォータニオン [w; x; y; z]
        ba          % 加速度バイアス [bax; bay; baz] (m/s^2)
        bg          % ジャイロバイアス [bgx; bgy; bgz] (rad/s)
        
        % 共分散行列
        P           % 状態共分散行列 (15x15)
        Q           % プロセスノイズ共分散
        
        % ノイズ推定器
        noiseEstimator
        
        % センサー更新頻度
        dt                      % サンプリング周期 (s)
        freq_mag                % 磁気計更新頻度
        freq_gps                % GPS更新頻度
        freq_baro               % 気圧計更新頻度
        
        % 参照値
        gps_origin              % GPS原点 [lat0; lon0; alt0]
        g                       % 重力加速度 (m/s^2)
        
        % ノイズ判定閾値
        gyro_noise_threshold    % ジャイロノイズ閾値
        
        % 発散対策
        divergence_guard    % DivergenceGuardインスタンス
        % 状態修正量(dx)の最大ノルム（これを超えるdxはスケールダウンされる）
        max_dx_norm
        
        % 加速度フィルタ
        accel_filter        % AccelFilterインスタンス（レガシー）
        
        % 統一フィルタシステム
        sensor_filters      % struct: 各センサーのフィルタインスタンス
        
        % Yaw 角速度統合制御
        gyro_filter_yaw_alpha    % Yaw 軸のEMA α値（デフォルト: 0.08 = 弱平滑化）
        enable_yaw_raw_gyro      % true の場合、Yaw はフィルタリングなしで積分
        
        % 磁気計更新を一時無効化するフラグ
        enable_mag_update        % true = 磁気計更新を行う, false = スキップ
        
        % 角速度フィルタの有効化フラグ
        enable_gyro_filter       % true = ジャイロフィルタを使用, false = 生の角速度を使用
    end

    methods
        % コンストラクタ
        function obj = ESKF(obs, static_time, dt)
            % ESKF コンストラクタ
            
            if nargin < 2
                static_time = 5.0;
            end
            if nargin < 3
                dt = mean(diff(obs.time));
            end
            
            obj.dt = dt;
            obj.g = [0; 0; -9.81];  % 重力加速度（下向き）
            
            % 静止期間のインデックス
            static_samples = round(static_time / dt);
            static_idx = 1:min(static_samples, length(obs.time));
            
            % 状態初期化
            obj.p = zeros(3, 1);
            obj.v = zeros(3, 1);
            obj.q = QuaternionLib.normalize([1; 0; 0; 0]);
            
            % バイアスの初期推定
            % 静止時、センサーは比力 f = a_true - g を測定
            % 真の加速度 a_true = 0（静止）
            % 重力 g = [0; 0; -9.81]（下向き）
            % 理論的な比力 f_ideal = 0 - [0; 0; -9.81] = [0; 0; 9.81]（上向き）
            % バイアス ba = 測定値 - 理論値
            if length(static_idx) > 10
                accel_static_mean = [mean(obs.ax(static_idx)); mean(obs.ay(static_idx)); mean(obs.az(static_idx))];
                % 静止時の理論的な比力（上向き）から偏差がバイアス
                f_ideal_static = [0; 0; 9.81];
                obj.ba = accel_static_mean - f_ideal_static;
                
                obj.bg = [mean(obs.wx(static_idx)); mean(obs.wy(static_idx)); mean(obs.wz(static_idx))];
                obj.bg = deg2rad(obj.bg);
            else
                obj.ba = zeros(3, 1);
                obj.bg = zeros(3, 1);
            end
            
            % 共分散初期化
            obj.P = eye(15) * 0.01;
            
            % プロセスノイズの初期推定
            if length(static_idx) > 10
                accel_static = [obs.ax(static_idx), obs.ay(static_idx), obs.az(static_idx)];
                accel_mean = mean(accel_static, 1);
                sigma_a = mean(std(accel_static - accel_mean, [], 1));
                
                gyro_static = [obs.wx(static_idx), obs.wy(static_idx), obs.wz(static_idx)];
                sigma_g = mean(std(gyro_static, [], 1));
                sigma_g = deg2rad(sigma_g);
            else
                sigma_a = 0.1;
                sigma_g = deg2rad(0.1);
            end
            
            obj.Q = zeros(15);
            obj.Q(4:6, 4:6) = eye(3) * (0.01^2);
            obj.Q(7:9, 7:9) = eye(3) * (0.01^2);
            obj.Q(10:12, 10:12) = eye(3) * (sigma_a^2 * 1e-4);
            obj.Q(13:15, 13:15) = eye(3) * (sigma_g^2 * 1e-5);
            
            % ノイズ推定器の初期化
            obj.noiseEstimator = NoiseEstimator(10);

            % --- センサーごとの初期ノイズ推定 (静止期間データに基づく) ---
            if length(static_idx) > 10
                % 磁気計ノイズ
                mag_static = [obs.mx(static_idx), obs.my(static_idx), obs.mz(static_idx)];
                mag_mean = mean(mag_static, 1);
                sigma_mag = mean(std(mag_static - mag_mean, [], 1));

                % 気圧計ノイズ (高度換算)
                P0 = 101325;
                pressure_static = obs.pressure(static_idx);
                alt_baro_static = 44330 * (1 - (pressure_static / P0).^0.1903);
                sigma_press = std(alt_baro_static - mean(alt_baro_static));

                % GPSノイズ (緯度経度->メートルに変換して分散を評価)
                lat_static = obs.lat(static_idx);
                lon_static = obs.lon(static_idx);
                alt_static = obs.alt(static_idx);
                lat0 = mean(lat_static);
                lon0 = mean(lon_static);
                y_m = (lat_static - lat0) / (9.0e-6);
                x_m = (lon_static - lon0) / (9.0e-6 / cosd(lat0));
                z_m = alt_static - mean(alt_static);
                sigma_gps = mean([std(x_m); std(y_m); std(z_m)]);
            end

            % NoiseEstimator に初期値をシード (分散で保存)
            obj.noiseEstimator.R_accel = ones(3,1) * (sigma_a^2);
            obj.noiseEstimator.R_gyro  = ones(3,1) * (sigma_g^2);
            obj.noiseEstimator.R_mag   = ones(3,1) * (sigma_mag^2);
            obj.noiseEstimator.R_baro  = (sigma_press^2);
            obj.noiseEstimator.R_gps   = ones(3,1) * (sigma_gps^2);

            % GPS原点の設定
            if length(static_idx) > 0
                obj.gps_origin = [mean(obs.lat(static_idx)); mean(obs.lon(static_idx)); mean(obs.alt(static_idx))];
            else
                obj.gps_origin = [obs.lat(1); obs.lon(1); obs.alt(1)];
            end
            
            % 更新頻度の設定
            obj.freq_mag = 4;
            obj.freq_baro = 8;
            obj.freq_gps = 10;
            
            % ジャイロノイズ閾値
            if length(static_idx) > 10
                wx_all = deg2rad(obs.wx(:));
                wy_all = deg2rad(obs.wy(:));
                wz_all = deg2rad(obs.wz(:));
                std_wx = std(wx_all);
                std_wy = std(wy_all);
                std_wz = std(wz_all);
                obj.gyro_noise_threshold = 2 * max([std_wx, std_wy, std_wz]);
            end
            
            % 初期化完了
            
            % 発散対策の初期化
            config = struct();
            config.max_velocity = 2.0;
            config.max_acceleration = 2.0;
            config.max_allowed_innov = 50.0;
            % イノベーションを limit/2 に縮小して更新する
            config.max_innov_cap_fraction = 0.5;
            config.max_gain_norm = 100; % clamp Kalman gain Frobenius norm to this value (Inf = no clamp)
            config.innov_change_ratio_threshold = 2.0;
            config.attenuation_factor = 0.5;
            % P行列の姿勢部分（7-9行）の上限設定（rad^2）
            config.max_attitude_variance = (deg2rad(10))^2;  % 10度の分散
            % MAG更新時のカルマンゲイン制限（各要素の絶対値）
            config.max_mag_gain_element = 0.15;  % 15%以下に制限
            obj.divergence_guard = DivergenceGuard(config);
            % dx の安全上限（magnitude）。必要に応じて調整してください。
            obj.max_dx_norm = 5.0;
            
            % 加速度フィルタの初期化
            % ema_alpha=0.3（平滑化の強さ）, history_size=20
            obj.accel_filter = AccelFilter(0.3, 20);
            
            % 統一フィルタシステムの初期化
            obj.sensor_filters = struct();
            obj.sensor_filters.accel = SensorFilter.createAccelFilter();
            obj.sensor_filters.gyro = SensorFilter.createGyroFilter();
            obj.sensor_filters.mag = SensorFilter.createMagFilter();
            obj.sensor_filters.gps = SensorFilter.createGPSFilter();
            obj.sensor_filters.baro = SensorFilter.createBaroFilter();
            
            % ジャイロフィルタ設定（Biquad導入済み）
            obj.gyro_filter_yaw_alpha = 0.08;  % 未使用（Biquadに移行）
            obj.enable_yaw_raw_gyro = false;   % false: Biquadフィルタ適用
            obj.enable_mag_update = false;     % 磁気計更新を無効化（Yaw干渉防止）
            obj.enable_gyro_filter = true;     % Biquadフィルタを有効化
        end
        
        function update_filter(obj, obs, k)
            % 1ステップ更新実行

            % センサーデータ取得
            a = [obs.ax(k); obs.ay(k); obs.az(k)];
            % 生成データの角速度: x=pitch, y=roll, z=yaw
            % ESKF内部: x=roll, y=pitch, z=yaw なので入れ替え
            w_raw = [obs.wx(k); obs.wy(k); obs.wz(k)];
            w = [w_raw(2); w_raw(1); w_raw(3)];
            w = deg2rad(w);

            % 予測ステップ
            obj.predict(a, w);

            % === 加速度・磁気計更新を無効化（純粋な積分のみ） ===
            obj.update_accel(a);
            
            % 周期的更新
            if mod(k, obj.freq_mag) == 0
                    % 磁気計更新はフラグで制御
                    if isprop(obj, 'enable_mag_update') && obj.enable_mag_update
                        obj.update_mag([obs.mx(k); obs.my(k); obs.mz(k)]);
                    end
            end
            if mod(k, obj.freq_baro) == 0
                obj.update_baro(obs.pressure(k));
            end
            if mod(k, obj.freq_gps) == 0 && ~isnan(obs.lat(k)) && ~isnan(obs.lon(k))
                obj.update_gps(obs.lat(k), obs.lon(k), obs.alt(k), k);
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
            % 加速度による姿勢更新
            
            % センサーフィルタ適用
            [a_corrected, is_outlier, ~] = obj.sensor_filters.accel.apply(a_meas, zeros(3,1));
            
            if is_outlier
                % 外れ値の場合は更新をスキップ
                return;
            end
            
            % 健全性チェック
            a_norm = norm(a_corrected);
            if a_norm < 0.1 || abs(a_norm - 9.81) > 3.0
                return;
            end
            
            % 現在のYawを取得（加速度計では観測不可能なため保持）
            euler_current = QuaternionLib.to_euler(obj.q);
            yaw_current = euler_current(3);
            
            % 加速度から直接Roll/Pitchを計算
            ax = a_corrected(1);
            ay = a_corrected(2);
            az = a_corrected(3);
            
            roll_measured = atan2d(ay, az);
            pitch_measured = atan2d(-ax, sqrt(ay^2 + az^2));
            
            % 現在のRoll/Pitchを取得
            euler_before = QuaternionLib.to_euler(obj.q);
            roll_current = euler_before(1);
            pitch_current = euler_before(2);
            
            % 変化量を計算
            roll_diff_raw = roll_measured - roll_current;
            roll_diff = mod((roll_diff_raw + 180), 360) - 180;
            roll_diff = abs(roll_diff);
            
            pitch_diff_raw = pitch_measured - pitch_current;
            pitch_diff = mod((pitch_diff_raw + 180), 360) - 180;
            pitch_diff = abs(pitch_diff);
            
            % 大きな変化をスケーリング
            scale_factor = 1.0;
            if roll_diff > 1.0 || pitch_diff > 1.0
                scale_factor = 0.1;
            end
            
            % スケーリングを適用
            roll_target = roll_current + roll_diff_raw * scale_factor;
            pitch_target = pitch_current + pitch_diff_raw * scale_factor;
            
            % 適応的ゲイン制限
            if roll_diff > 0.5 || pitch_diff > 0.5
                adaptive_gain = 1.0 / (1.0 + max(roll_diff, pitch_diff) / 2.0);
                roll_target = roll_current + (roll_target - roll_current) * adaptive_gain;
                pitch_target = pitch_current + (pitch_target - pitch_current) * adaptive_gain;
            end
            
            % 新しいEuler角からクォータニオンを生成（MEX使用）
            scale = 1.0;
            if roll_diff > 0.5 || pitch_diff > 0.5
                scale = adaptive_gain;
            end
            obj.q = eskf_core_mex('update_accel', obj.q, a_corrected, scale);
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
            
            z = m_filtered;
            h = h_mag;
            H = [zeros(3,6), RotationLib.skew_symmetric(h), zeros(3,6)];
            
            % 現在のノイズ推定値を使用
            R_est = obj.noiseEstimator.getRnoise('mag');
            
            [~, S, R_used] = kalman_filter_core('compute_innovation_and_S', z, h, H, obj.P, R_est, struct());

            % Use OutlierGuard to unify checks
            try
                K_prop = kalman_filter_core('compute_kalman_gain', obj.P, H, S);
            catch
                K_prop = [];
            end
            ctx = struct(); ctx.k = NaN; ctx.z = z; ctx.h = h; ctx.P_diag = diag(obj.P); ctx.R_diag = diag(R_used);
            [should_update, y_used, K_used, dx_used, ~] = OutlierGuard.checkAndApply('mag', z, h, H, obj.P, R_used, K_prop, [], obj.divergence_guard, obj.noiseEstimator, ctx);
            if ~should_update
                return;
            end

            obj.noiseEstimator.estimate('mag', y_used, H, obj.P);

            if isempty(K_used)
                K = kalman_filter_core('compute_kalman_gain', obj.P, H, S);
                K = obj.divergence_guard.clamp_gain(K);
            else
                K = K_used;
            end
            
            % MAG更新専用のゲイン制限（姿勢部分のみ）
            if isfield(obj.divergence_guard.config, 'max_mag_gain_element')
                max_gain = obj.divergence_guard.config.max_mag_gain_element;
                % K(7:9,:)の各要素を制限（姿勢部分）
                K(7:9,:) = max(min(K(7:9,:), max_gain), -max_gain);
            end
            
            if isempty(dx_used)
                dx = K * y_used;
            else
                dx = dx_used;
            end
            
            % dxのサイズ確認とベクトル化
            if numel(dx) < 9
                % dx が十分な要素を持っていない場合、ゼロパディング
                dx_full = zeros(15, 1);
                dx_full(1:numel(dx)) = dx(:);
                dx = dx_full;
            end

            dtheta = [0; 0; dx(9)];
            
            dq = QuaternionLib.small_angle_quat(dtheta);
            obj.q = QuaternionLib.multiply(obj.q, dq);
            obj.q = QuaternionLib.normalize(obj.q);

            x_pred = zeros(15,1);
            [~, obj.P] = kalman_filter_core('update_state_covariance', x_pred, obj.P, K, H, y_used, R_used);
        end
        
    function update_gps(obj, lat, lon, alt, k)
            % GPS位置更新
            
            lat0 = obj.gps_origin(1);
            lon0 = obj.gps_origin(2);
            alt0 = obj.gps_origin(3);
            
            % 座標変換
            y_m = (lat - lat0) / (9.0e-6);
            x_m = (lon - lon0) / (9.0e-6 / cosd(lat0));
            z_m = alt - alt0;
            
            z_gps = [x_m; y_m; z_m];
            
            % 新しいセンサーフィルタを使用
            [z_gps_filtered, is_outlier, ~] = obj.sensor_filters.gps.apply(z_gps);
            
            if is_outlier
                return;  % 外れ値の場合は更新をスキップ
            end

            R = obj.noiseEstimator.getRnoise('gps');
            
            % GPS更新前にPを正則化（UKFのシグマポイント生成の安定性向上）
            obj.P = obj.divergence_guard.regularize_for_ukf(obj.P);
            
            x_err = zeros(15, 1);
            h_func = @(dx) obj.p + dx(1:3);
            
            [dx, P_upd, ~, ~, y_innov] = ukf_update(x_err, obj.P, z_gps_filtered, h_func, R);

            % Use OutlierGuard to handle SensorFilter + Divergence checks
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
            obj.p = obj.p + dx(1:3);
            obj.v = obj.v + dx(4:6);
            obj.P = P_upd;
            
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

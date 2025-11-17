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
        % デバッグ用コールバック (関数ハンドル)。存在する場合、各ステップで
        % 情報構造体を渡して呼び出す。例: cb(info)
        debugCallback
        % 詳細ダンプを取得したい時刻インデックス (空 = 無効)
        debugDumpK
        
        % パルス検知
        pulse_detection_enabled  % パルス検知有効化フラグ
        pulse_threshold          % パルス検知閾値 (degree/m/s)
        pulse_log                % パルス検知ログ
        prev_position            % 前ステップの位置 [x; y; z]
        prev_velocity            % 前ステップの速度 [vx; vy; vz]
        prev_euler               % 前ステップのEuler角 [roll; pitch; yaw]
        

    end

    methods
        % コンストラクタ
        function obj = ESKF(obs, static_time, dt)
            % ESKF コンストラクタ
            %
            % 入力:
            %   obs         - 観測データ構造体
            %   static_time - 静止期間 (秒)
            %   dt          - サンプリング周期 (秒)
            
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
            obj.q = quat_lib('quatnormalize', [1; 0; 0; 0]);
            
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
            
            % 初期化情報表示
            fprintf('ESKF 初期化:\n');
            fprintf('  静止期間: %.1f秒 (%d サンプル)\n', static_time, length(static_idx));
            fprintf('  GPS原点: [%.6f, %.6f, %.2f]\n', obj.gps_origin(1), obj.gps_origin(2), obj.gps_origin(3));
            fprintf('  初期バイアス - 加速度: [%.4f, %.4f, %.4f], ジャイロ: [%.4f, %.4f, %.4f]\n', ...
                    obj.ba(1), obj.ba(2), obj.ba(3), obj.bg(1), obj.bg(2), obj.bg(3));
            fprintf('  推定ノイズレベル - 加速度: %.4f, ジャイロ: %.4f 地磁気: %.4f 気圧: %.4f GPS: %.4f\n', sigma_a, sigma_g, sigma_mag, sigma_press, sigma_gps);
            
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
            
            % パルス検知の初期化
            obj.pulse_detection_enabled = true;  % パルス検知を有効化
            obj.pulse_threshold = 2.0;  % 2.0以上の急変をパルスとして検知（姿勢:度、位置:m、速度:m/s）
            obj.pulse_log = struct('step', {}, 'time', {}, 'type', {}, ...
                'roll_change', {}, 'pitch_change', {}, 'yaw_change', {}, ...
                'px_change', {}, 'py_change', {}, 'pz_change', {}, ...
                'vx_change', {}, 'vy_change', {}, 'vz_change', {});
            obj.prev_position = [0; 0; 0];  % 初期位置
            obj.prev_velocity = [0; 0; 0];  % 初期速度
            obj.prev_euler = [0; 0; 0];     % 初期Euler角
            

        end
        
        function updateFilter(obj, obs, k)
            % UPDATEFILTER  1ステップの更新を実行
            %
            % 入力:
            %   obs - 観測データ構造体
            %   k   - タイムインデックス

            % センサーデータの取得
            a = [obs.ax(k); obs.ay(k); obs.az(k)];
            % 生成データの角速度: x=pitch, y=roll, z=yaw
            % ESKF内部: x=roll, y=pitch, z=yaw なので入れ替え
            w_raw = [obs.wx(k); obs.wy(k); obs.wz(k)];
            w = [w_raw(2); w_raw(1); w_raw(3)];
            w = deg2rad(w);

            % 予測ステップ
            obj.predict(a, w);

            % デバッグコール: 予測直後の P を渡す
            info = struct(); info.k = k; info.stage = 'post_predict'; info.P = obj.P; info.p = obj.p; info.v = obj.v;
            obj.callDebug(info);

            % (no per-step dump here)

            % (no per-step dump here)
            

            % === 加速度・磁気計更新を無効化（純粋な積分のみ） ===
            obj.updateAccel(a);
            
            % 周期的更新
            if mod(k, obj.freq_mag) == 0
                obj.updateMag([obs.mx(k); obs.my(k); obs.mz(k)]);
            end
            if mod(k, obj.freq_baro) == 0
                obj.updateBaro(obs.pressure(k));
            end
            if mod(k, obj.freq_gps) == 0 && ~isnan(obs.lat(k)) && ~isnan(obs.lon(k))
                obj.updateGPS(obs.lat(k), obs.lon(k), obs.alt(k), k);
            end
            
            % パルス検知（全ての推定値に対して）
            obj.detectPulse(k, obs.time(k));
        end

        function callDebug(obj, info)
            % callDebug - debugCallback が設定されていれば安全に呼び出す
            try
                if ~isempty(obj.debugCallback) && isa(obj.debugCallback, 'function_handle')
                    obj.debugCallback(info);
                end
            catch e
                warning('ESKF:debugCallback', 'debugCallback failed: %s', e.message);
            end
        end
        
        function predict(obj, a_meas, w_meas)
            % PREDICT  予測ステップ
            %
            % 入力:
            %   a_meas - 加速度測定値 (3x1)
            %   w_meas - 角速度測定値 (3x1, rad/s)

            % ジャイロの外れ値検出のみ実行（EMA平滑化は廃止）
            [~, w_is_outlier, ~] = obj.sensor_filters.gyro.apply(w_meas, obj.bg);
            if w_is_outlier
                % 外れ値の場合はバイアス推定値を使用
                w_meas = obj.bg;
            end
            % w_meas はそのまま integrate_nominal に渡す（平滑化なし）

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

            [obj.p, obj.v, obj.q, obj.ba, obj.bg] = integrate_nominal(...
                obj.p, obj.v, obj.q, obj.ba, obj.bg, a_meas, w_meas, obj.dt, obj.g, gyro_thr_vec, accel_thr_vec);

            % 共分散の予測
            obj.P = kalman_filter_core('predict_step', obj.P, obj.q, a_meas, obj.ba, w_meas, obj.bg, obj.Q, obj.dt);
            
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
        
        function updateAccel(obj, a_meas)
            % UPDATEACCEL  加速度による姿勢更新
            % 
            % 修正履歴 (2025/11/17):
            %   - SensorAccelFilterを使用した統一フィルタリング
            %   - EMA平滑化 + 外れ値検出 + 大きな変化スケーリング
            %
            % 入力:
            %   a_meas - 加速度測定値 (3x1)
            
            persistent test_counter
            if isempty(test_counter), test_counter = 0; end
            test_counter = test_counter + 1;
            
            % 新しいセンサーフィルタを使用
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
            euler_current = quat_lib('quat_to_euler', obj.q);
            yaw_current = euler_current(3);
            
            % 加速度から直接Roll/Pitchを計算
            ax = a_corrected(1);
            ay = a_corrected(2);
            az = a_corrected(3);
            
            roll_measured = atan2d(ay, az);
            pitch_measured = atan2d(-ax, sqrt(ay^2 + az^2));
            
            % 現在のRoll/Pitchを取得
            euler_before = quat_lib('quat_to_euler', obj.q);
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
                if mod(test_counter, 500) == 0
                    fprintf('  [ACCEL更新] 大きな変化を検出 - スケーリング適用: Roll差%.2f°, Pitch差%.2f° → 1/10\n', ...
                        roll_diff, pitch_diff);
                end
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
            
            % 新しいEuler角からクォータニオンを生成
            obj.q = quat_lib('euler_to_quat', [roll_target; pitch_target; yaw_current]);
            obj.q = quat_lib('quatnormalize', obj.q);
        end

        function updateMag(obj, m_meas)
            % UPDATEMAG  磁気計による姿勢更新
            %
            % 入力:
            %   m_meas - 磁気計測定値 (3x1)
            
            % 新しいセンサーフィルタを使用
            [m_filtered, is_outlier, ~] = obj.sensor_filters.mag.apply(m_meas);
            
            if is_outlier
                return;  % 外れ値の場合は更新をスキップ
            end
            
            m_world = [0; 50; 0];
            Rb = quat_lib('quat_to_rotm', obj.q);
            h_mag = Rb' * m_world;
            
            z = m_filtered;
            h = h_mag;
            H = [zeros(3,6), quat_lib('skew', h), zeros(3,6)];
            
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
            [should_update, y_used, K_used, dx_used, diag_info] = OutlierGuard.checkAndApply('mag', z, h, H, obj.P, R_used, K_prop, [], obj.divergence_guard, obj.noiseEstimator, ctx);
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

            dtheta = [0; 0; dx(9)];
            
            % デバッグ: 大きなYaw更新を検出
            if abs(rad2deg(dx(9))) > 2.0
                euler_before = quat_lib('quat_to_euler', obj.q);
                fprintf('  [MAG更新] 大きなYaw補正: %.3f° (before Yaw=%.2f°)\n', ...
                    rad2deg(dx(9)), euler_before(3));
                fprintf('    イノベーション: [%.3f, %.3f, %.3f]\n', y_used(1), y_used(2), y_used(3));
                fprintf('    K(9行): [%.4f, %.4f, %.4f]\n', K(9,1), K(9,2), K(9,3));
                fprintf('    測定値: [%.2f, %.2f, %.2f], 予測: [%.2f, %.2f, %.2f]\n', ...
                    z(1), z(2), z(3), h(1), h(2), h(3));
            end
            
            dq = quat_lib('small_angle_quat', dtheta);
            obj.q = quat_lib('quatmultiply', obj.q, dq);
            obj.q = quat_lib('quatnormalize', obj.q);

            x_pred = zeros(15,1);
            [~, obj.P] = kalman_filter_core('update_state_covariance', x_pred, obj.P, K, H, y_used, R_used);
            info = struct(); info.stage = 'baro_post'; info.k = NaN; info.P = obj.P; info.z = z; info.h = h; info.y = y_used; info.P_diag = diag(obj.P);
            try
                info.K_norm = norm(K, 'fro');
            catch
                info.K_norm = NaN;
            end
            obj.callDebug(info);
        end
        
    function updateGPS(obj, lat, lon, alt, k)
            % UPDATEGPS  GPS位置観測による更新
            %
            % 入力:
            %   lat, lon, alt - GPS観測値
            
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

            % Build context for dump/diagnostics
            H_gps = [eye(3), zeros(3, 12)];
            S = H_gps * obj.P * H_gps' + R;
            ctx = struct();
            ctx.k = k;
            ctx.z = z_gps;
            ctx.h = obj.p; % predicted measurement for dx=0
            ctx.y = y_innov;
            ctx.P_diag = diag(obj.P);
            ctx.R_diag = diag(R);
            ctx.S_rcond = rcond(S);

            % --- 自動ダンプ: GPS pre のタイミングでフル内部状態を安全に保存 ---
            if ~isempty(obj.debugDumpK) && isequal(k, obj.debugDumpK)
                try
                    try
                        K_approx = obj.P * H_gps' / S;
                    catch
                        K_approx = NaN;
                    end
                    dump = struct('k',k,'sensor','gps','z',z_gps,'h',obj.p,'y',y_innov,'P',obj.P,'R',R,'S',S,'K_approx',K_approx,'time',datestr(now,'yyyymmdd_HHMMSS'));
                    obj.saveDebugDump(dump);
                catch e
                    warning('ESKF:saveDebugDump','Could not auto-save debug dump: %s', e.message);
                end
            end

            % デバッグ: GPS 更新前情報
            info = struct(); info.k = k; info.stage = 'gps_pre'; info.P = obj.P; info.z = z_gps; info.h = obj.p; info.y = y_innov; info.S_rcond = ctx.S_rcond;
            % Kalman gain approximate (pre-update) for diagnostics: K = P*H' * inv(S)
            try
                H_gps = [eye(3), zeros(3, 12)];
                K_approx = obj.P * H_gps' / S; %#ok<NASGU>
                info.K_norm = norm(K_approx, 'fro');
            catch
                info.K_norm = NaN;
            end
            obj.callDebug(info);

            % Use OutlierGuard to handle SensorFilter + Divergence checks
            R_updated = obj.noiseEstimator.getRnoise('gps');
            ctx.gps = ctx;
            [should_update, y_used, K_used, dx_used, diag_info] = OutlierGuard.checkAndApply('gps', z_gps, obj.p, H_gps, obj.P, R_updated, [], dx, obj.divergence_guard, obj.noiseEstimator, ctx);
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
            % obj.ba = obj.ba + dx(10:12);
            obj.P = P_upd;
            % debug: gps post-update
            info = struct(); info.k = k; info.stage = 'gps_post'; info.P = obj.P; info.z = z_gps; info.h = obj.p; info.y = y_innov; info.P_diag = diag(obj.P);
            try
                H_gps = [eye(3), zeros(3, 12)];
                K_approx = (P_upd) * H_gps' / S;
                info.K_norm = norm(K_approx, 'fro');
            catch
                info.K_norm = NaN;
            end
            obj.callDebug(info);
            
            % 速度チェックとクリッピング
            [obj.v, obj.P, ~] = obj.divergence_guard.check_and_clip_velocity(obj.v, obj.P, 4:6);
        end
        
        function updateBaro(obj, pressure)
            % UPDATEBARO  気圧計による高度更新
            %
            % 入力:
            %   pressure - 気圧測定値 (Pa)
            
            % 新しいセンサーフィルタを使用
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
            dx = K * y_filtered;
            
            % 高度更新
            if abs(dx(3)) >= 0.1
                obj.p(3) = obj.p(3) + dx(3);
            end
            
            % 共分散更新
            x_pred = zeros(15,1);
            x_pred(1:3) = obj.p;
            [~, obj.P] = kalman_filter_core('update_state_covariance', x_pred, obj.P, K, H, y_filtered, R_used);
        end
        
        function euler = getEuler(obj)
            % GETEULER  オイラー角を取得
            %
            % 出力:
            %   euler - [roll; pitch; yaw] (度)
            euler = quat_lib('quat_to_euler', obj.q);  % now returns [roll; pitch; yaw]
        end
        
        function log = getPulseLog(obj)
            % GETPULSELOG  パルス検知ログを取得
            %
            % 出力:
            %   log - パルス検知ログ (struct配列)
            log = obj.pulse_log;
        end
        
        function detectPulse(obj, step, time)
            % DETECTPULSE  全ての推定値に対してパルス検知を実行
            %
            % 入力:
            %   step - ステップ番号
            %   time - 時刻 (秒)
            
            if ~obj.pulse_detection_enabled
                return;
            end
            
            % 現在の状態を取得
            current_position = obj.p;
            current_velocity = obj.v;
            current_euler = obj.getEuler();
            
            % 変化量を計算（角度差はラップ処理を行う）
            % wrapTo180 を使って角度差の最小代表を取得
            d_euler = mod((current_euler - obj.prev_euler) + 180, 360) - 180;  % deg
            roll_change = abs(d_euler(1));
            pitch_change = abs(d_euler(2));
            yaw_change = abs(d_euler(3));
            
            px_change = abs(current_position(1) - obj.prev_position(1));
            py_change = abs(current_position(2) - obj.prev_position(2));
            pz_change = abs(current_position(3) - obj.prev_position(3));
            
            vx_change = abs(current_velocity(1) - obj.prev_velocity(1));
            vy_change = abs(current_velocity(2) - obj.prev_velocity(2));
            vz_change = abs(current_velocity(3) - obj.prev_velocity(3));
            
            % 閾値判定（姿勢：度、位置：m、速度：m/s）
            attitude_pulse = roll_change > obj.pulse_threshold || ...
                             pitch_change > obj.pulse_threshold || ...
                             yaw_change > obj.pulse_threshold;
            position_pulse = px_change > obj.pulse_threshold || ...
                             py_change > obj.pulse_threshold || ...
                             pz_change > obj.pulse_threshold;
            velocity_pulse = vx_change > obj.pulse_threshold || ...
                             vy_change > obj.pulse_threshold || ...
                             vz_change > obj.pulse_threshold;
            
            % パルス検知時にログに記録
            if attitude_pulse || position_pulse || velocity_pulse
                pulse_entry = struct();
                pulse_entry.step = step;
                pulse_entry.time = time;
                
                % どのタイプのパルスか記録
                types = {};
                if attitude_pulse, types{end+1} = 'attitude'; end
                if position_pulse, types{end+1} = 'position'; end
                if velocity_pulse, types{end+1} = 'velocity'; end
                pulse_entry.type = strjoin(types, '+');
                
                % 各変化量を記録
                pulse_entry.roll_change = roll_change;
                pulse_entry.pitch_change = pitch_change;
                pulse_entry.yaw_change = yaw_change;
                pulse_entry.px_change = px_change;
                pulse_entry.py_change = py_change;
                pulse_entry.pz_change = pz_change;
                pulse_entry.vx_change = vx_change;
                pulse_entry.vy_change = vy_change;
                pulse_entry.vz_change = vz_change;
                
                obj.pulse_log(end+1) = pulse_entry;
                
                % コンソール出力
                fprintf('  ★ パルス検知 [Step %d, %.3fs] タイプ=%s\n', step, time, pulse_entry.type);
                if attitude_pulse
                    fprintf('     姿勢: Roll=%.3f° Pitch=%.3f° Yaw=%.3f°\n', ...
                        roll_change, pitch_change, yaw_change);
                end
                if position_pulse
                    fprintf('     位置: X=%.3fm Y=%.3fm Z=%.3fm\n', ...
                        px_change, py_change, pz_change);
                end
                if velocity_pulse
                    fprintf('     速度: VX=%.3fm/s VY=%.3fm/s VZ=%.3fm/s\n', ...
                        vx_change, vy_change, vz_change);
                end
            end
            
            % 前回値を更新
            obj.prev_position = current_position;
            obj.prev_velocity = current_velocity;
            obj.prev_euler = current_euler;
        end

        function saveDebugDump(obj, dump)
            % saveDebugDump  指定されたダンプ構造体を Results フォルダに保存
            try
                % mfilename('fullpath') -> .../kalman/ESKF/ESKF.m
                classPath = mfilename('fullpath');
                base = fileparts(fileparts(classPath)); % .../kalman
                resultsDir = fullfile(base, 'Results');
                if ~exist(resultsDir, 'dir')
                    mkdir(resultsDir);
                end
                tstr = datestr(now,'yyyymmdd_HHMMSS');
                if isfield(dump,'k')
                    kstr = sprintf('k%u',dump.k);
                else
                    kstr = 'kNaN';
                end
                if isfield(dump,'sensor')
                    sname = dump.sensor;
                else
                    sname = 'unknown';
                end
                fname = fullfile(resultsDir, sprintf('divergence_full_dump_%s_%s_%s.mat', sname, kstr, tstr));
                save(fname, 'dump');
                fprintf('ESKF: saved debug dump to %s\n', fname);
            catch e
                warning('ESKF:saveDebugDump', 'Failed to save debug dump: %s', e.message);
            end
        end
    end
end

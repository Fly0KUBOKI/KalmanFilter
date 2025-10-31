classdef ESKF < handle
    % ESKF  Error State Kalman Filter for IMU/GPS/Mag/Baro
    % 誤差状態カルマンフィルタ（ESKF）実装
    % 
    % 使用方法:
    %   eskf = ESKF(obs, static_time, dt);
    %   eskf.updateFilter(obs, k);
    %   euler = eskf.getEuler();

    properties
        % 状態変数
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
            obj.g = [0; 0; 9.81];
            
            % 静止期間のインデックス
            static_samples = round(static_time / dt);
            static_idx = 1:min(static_samples, length(obs.time));
            
            % 状態初期化
            obj.p = zeros(3, 1);
            obj.v = zeros(3, 1);
            obj.q = quat_lib('quatnormalize', [1; 0; 0; 0]);
            
            % バイアスの初期推定
            if length(static_idx) > 10
                accel_static_mean = [mean(obs.ax(static_idx)); mean(obs.ay(static_idx)); mean(obs.az(static_idx))];
                obj.ba = accel_static_mean - [0; 0; 9.81];
                
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
            else
                obj.gyro_noise_threshold = deg2rad(0.1);
            end
            
            % 初期化情報表示
            fprintf('ESKF 初期化:\n');
            fprintf('  静止期間: %.1f秒 (%d サンプル)\n', static_time, length(static_idx));
            fprintf('  GPS原点: [%.6f, %.6f, %.2f]\n', obj.gps_origin(1), obj.gps_origin(2), obj.gps_origin(3));
            fprintf('  初期バイアス - 加速度: [%.4f, %.4f, %.4f], ジャイロ: [%.4f, %.4f, %.4f]\n', ...
                    obj.ba(1), obj.ba(2), obj.ba(3), obj.bg(1), obj.bg(2), obj.bg(3));
            fprintf('  推定ノイズレベル - 加速度: %.4f, ジャイロ: %.4f\n', sigma_a, sigma_g);
        end
    end
    
    methods
        function updateFilter(obj, obs, k)
            % UPDATEFILTER  1ステップの更新を実行
            %
            % 入力:
            %   obs - 観測データ構造体
            %   k   - タイムインデックス
            
            % センサーデータの取得
            a = [obs.ax(k); obs.ay(k); obs.az(k)];
            w = [obs.wx(k); obs.wy(k); obs.wz(k)];
            w = deg2rad(w);
            
            % 予測ステップ
            obj.predict(a, w);
            
            % 加速度更新
            obj.updateAccel(a);
            
            % 周期的更新
            if mod(k, obj.freq_mag) == 0
                obj.updateMag([obs.mx(k); obs.my(k); obs.mz(k)]);
            end
            if mod(k, obj.freq_baro) == 0
                obj.updateBaro(obs.pressure(k));
            end
            if mod(k, obj.freq_gps) == 0 && ~isnan(obs.lat(k)) && ~isnan(obs.lon(k))
                obj.updateGPS(obs.lat(k), obs.lon(k), obs.alt(k));
            end
        end
        
        function predict(obj, a_meas, w_meas)
            % PREDICT  予測ステップ
            %
            % 入力:
            %   a_meas - 加速度測定値 (3x1)
            %   w_meas - 角速度測定値 (3x1, rad/s)
            
            % ノミナル状態の積分
            [obj.p, obj.v, obj.q, obj.ba, obj.bg] = integrate_nominal(...
                obj.p, obj.v, obj.q, obj.ba, obj.bg, a_meas, w_meas, obj.dt, obj.g, obj.gyro_noise_threshold);
            
            % 共分散の予測
            obj.P = kalman_filter_core('predict_step', obj.P, obj.q, a_meas, obj.ba, w_meas, obj.bg, obj.Q, obj.dt);
        end
        
        function updateAccel(obj, a_meas)
            % UPDATEACCEL  加速度による姿勢更新
            %
            % 入力:
            %   a_meas - 加速度測定値 (3x1)
            
            % 静止判定の持続処理
            persistent count accel_int dt_sum
            if isempty(count)
                count = 0;
                accel_int = zeros(3,1);
                dt_sum = 0;
            end
            
            count = count + 1;
            accel_int = accel_int + a_meas * obj.dt;
            dt_sum = dt_sum + obj.dt;
            
            if count < 4
                return;
            end
            
            a_meas = accel_int / dt_sum;
            count = 0;
            accel_int = zeros(3,1);
            dt_sum = 0;
            
            % 静止判定
            accel_norm = norm(a_meas);
            if abs(accel_norm - 9.81) > 0.5
                return;
            end
            
            % 観測モデル
            Rb = quat_lib('quat_to_rotm', obj.q);
            a_body_corrected = a_meas - obj.ba;
            a_world = Rb * a_body_corrected;
            
            z = a_world;
            h = obj.g;
            
            H_theta = -Rb * quat_lib('skew', a_body_corrected);
            H = [zeros(3,3), zeros(3,3), H_theta, -Rb, zeros(3,3)];
            
            % イノベーション
            y0 = z - h;
            
            % ノイズ推定
            obj.noiseEstimator.estimate('accel', y0, H, obj.P);
            R_est = obj.noiseEstimator.getRnoise('accel');
            
            % イノベーション計算
            [y, S, R_used] = kalman_filter_core('compute_innovation_and_S', z, h, H, obj.P, R_est, struct());
            
            % 2σフィルタリング
            [y_filtered, should_update] = SensorFilter.filterInnovation(y, R_used);
            if ~should_update
                return;
            end
            
            % カルマンゲインと更新
            K = kalman_filter_core('compute_kalman_gain', obj.P, H, S);
            dx = K * y_filtered;
            
            % バイアス更新
            obj.ba = obj.ba + Rb' * dx(10:12);
            
            % 共分散更新
            x_pred = zeros(15,1);
            x_pred(1:3) = obj.p; x_pred(4:6) = obj.v; x_pred(7:9) = zeros(3,1);
            x_pred(10:12) = obj.ba; x_pred(13:15) = obj.bg;
            [~, obj.P] = kalman_filter_core('update_state_covariance', x_pred, obj.P, K, H, y_filtered, R_used);
        end
        function updateMag(obj, m_meas)
            % UPDATEMAG  磁気計による姿勢更新
            %
            % 入力:
            %   m_meas - 磁気計測定値 (3x1)
            
            m_world = [0; 50; 0];
            Rb = quat_lib('quat_to_rotm', obj.q);
            h_mag = Rb' * m_world;
            
            z = m_meas;
            h = h_mag;
            H = [zeros(3,6), quat_lib('skew', h), zeros(3,6)];
            
            y0 = z - h;
            
            % ノイズ推定
            obj.noiseEstimator.estimate('mag', y0, H, obj.P);
            R_est = obj.noiseEstimator.getRnoise('mag');
            
            [y, S, R_used] = kalman_filter_core('compute_innovation_and_S', z, h, H, obj.P, R_est, struct());
            
            % 2σフィルタリング
            [y_filtered, should_update] = SensorFilter.filterInnovation(y, R_used);
            if ~should_update
                return;
            end
            
            K = kalman_filter_core('compute_kalman_gain', obj.P, H, S);
            dx = K * y_filtered;
            
            % 姿勢更新 (yawのみ)
            dtheta = [0; 0; dx(9)];
            dq = quat_lib('small_angle_quat', dtheta);
            obj.q = quat_lib('quatmultiply', obj.q, dq);
            obj.q = quat_lib('quatnormalize', obj.q);
            
            % 共分散更新
            x_pred = zeros(15,1);
            [~, obj.P] = kalman_filter_core('update_state_covariance', x_pred, obj.P, K, H, y_filtered, R_used);
        end
        
        function updateGPS(obj, lat, lon, alt)
            % UPDATEGPS  GPS位置観測による更新 (UKF)
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
            
            R = obj.noiseEstimator.getRnoise('gps');
            
            x_err = zeros(15, 1);
            h_func = @(dx) obj.p + dx(1:3);
            
            [dx, P_upd, ~, ~, y_innov] = ukf_update(x_err, obj.P, z_gps, h_func, R);
            
            % ノイズ推定
            H_gps = [eye(3), zeros(3, 12)];
            obj.noiseEstimator.estimate('gps', y_innov, H_gps, obj.P);
            
            % 2σフィルタリング
            R_updated = obj.noiseEstimator.getRnoise('gps');
            [~, should_update] = SensorFilter.filterInnovation(y_innov, R_updated);
            if ~should_update
                return;
            end
            
            % 状態更新
            obj.p = obj.p + dx(1:3);
            obj.v = obj.v + dx(4:6);
            obj.ba = obj.ba + dx(10:12);
            obj.P = P_upd;
        end
        
        function updateBaro(obj, pressure)
            % UPDATEBARO  気圧計による高度更新
            %
            % 入力:
            %   pressure - 気圧測定値 (Pa)
            
            P0 = 101325;
            alt_baro = 44330 * (1 - (pressure / P0)^0.1903);
            
            H = [0,0,1, zeros(1,12)];
            z = alt_baro;
            h = obj.p(3);
            y0 = z - h;
            
            % ノイズ推定
            obj.noiseEstimator.estimate('baro', y0, H, obj.P);
            R_est = obj.noiseEstimator.getRnoise('baro');
            
            [y, S, R_used] = kalman_filter_core('compute_innovation_and_S', z, h, H, obj.P, R_est, struct());
            
            % 2σフィルタリング
            [y_filtered, should_update] = SensorFilter.filterInnovation(y, R_used);
            if ~should_update
                return;
            end
            
            K = kalman_filter_core('compute_kalman_gain', obj.P, H, S);
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
            %   euler - [roll; pitch; yaw]
            
            euler_angles = quat_lib('quat_to_euler', obj.q);
            euler = [euler_angles(2); euler_angles(1); euler_angles(3)];
        end
    end
end

function params = config_params()
    % シミュレーションパラメータ設定

    params = struct();

    % タイミング
    params.T = 100;      % シミュレーション総時間 [秒]
    params.static_time = 5;
    
    % センサー更新頻度 [Hz]
    params.sensor_freq = struct();
    params.sensor_freq.system = 100;   % システム全体の周波数 [Hz]

    % システム全体の周期 dt を周波数から計算（秒）
    params.dt = 1.0 / double(params.sensor_freq.system);  % 例: 50 Hz -> 0.02 s
    params.sensor_freq.imu = 100;      % IMU (加速度・ジャイロ)
    params.sensor_freq.mag = 100;      % 磁気センサー
    params.sensor_freq.gps = 10;       % GPS
    params.sensor_freq.baro = 50;      % 気圧センサー

    % 運動タイプ
    params.motion_type = 'circular';

    % ヘディングモード
    params.heading_mode = 'align_velocity'; % 'fixed_north' or 'align_velocity'

    % センサーノイズ (1σ)
    params.noise = struct();
    
    % ノイズ有効/無効フラグ
    params.noise.enable = struct();
    params.noise.enable.accel = true;
    params.noise.enable.gyro = true;
    params.noise.enable.mag = true;
    params.noise.enable.baro = true;
    params.noise.enable.gps = true;
    params.noise.enable.outlier = true;
    params.noise.enable.pink = true;
    params.noise.enable.allan = true;
    
    % ベースノイズレベル (フラグがtrueの場合のみ適用)
    params.noise.base = struct();
    params.noise.base.accel_std = single(0.1);
    params.noise.base.gyro_std = single(0.5);
    % 磁気センサの標準偏差（単位: μT）
    params.noise.base.mag_std = single(5.0);
    params.noise.base.baro_std = single(2.0);
    params.noise.base.gps_std = single(1.0);
    
    % 実際のノイズ設定 (フラグに基づいて決定)
    params.noise.accel_std = params.noise.enable.accel * params.noise.base.accel_std;
    params.noise.gyro_std = params.noise.enable.gyro * params.noise.base.gyro_std;
    params.noise.mag_std = params.noise.enable.mag * params.noise.base.mag_std;
    params.noise.baro_std = params.noise.enable.baro * params.noise.base.baro_std;
    params.noise.gps_std = params.noise.enable.gps * params.noise.base.gps_std;
    % 外れ値設定
    params.noise.outlier = struct();
    if params.noise.enable.outlier
        params.noise.outlier.prob = single(0.01);
        params.noise.outlier.range = struct( ...
            'accel', single(2.0), ...  % m/s^2
            'gyro', single(2.0), ...   % deg/s
            'mag', single(50.0), ...    % μT
            'baro', single(10.0), ...   % meters
            'gps', single(10.0) ...     % meters
        );
    else
        params.noise.outlier.prob = single(0.0);
        params.noise.outlier.range = struct( ...
            'accel', single(0.0), ...
            'gyro', single(0.0), ...
            'mag', single(0.0), ...
            'baro', single(0.0), ...
            'gps', single(0.0) ...
        );
    end

    % ピンクノイズ
    if params.noise.enable.pink
        params.noise.accel_pink_std = single(0.2);
        params.noise.gyro_pink_std = single(0.2);    % Gyroscope pink noise (deg/s)
        params.noise.gps_pink_std = single(1.0);     % GPS pink noise (meters)
    else
        params.noise.accel_pink_std = single(0.5);
        params.noise.gyro_pink_std = single(0.0);
        params.noise.gps_pink_std = single(0.0);
    end

    % Allan偏差
    if params.noise.enable.allan
        params.noise.gyro_allan_std = single(0.5);   % Gyroscope Allan deviation (deg/s)
        params.noise.baro_allan_std = single(1.0);   % Barometer Allan deviation (meters)
    else
        params.noise.accel_allan_std = single(0.0);
        params.noise.gyro_allan_std = single(0.0);
        params.noise.baro_allan_std = single(0.0);
    end

    % 運動パラメータ
    params.motion = struct();

    % 円運動
    params.motion.circular = struct();
    params.motion.circular.radius = single(10);
    params.motion.circular.omega = single(4);
    params.motion.circular.altitude = single(0);
    params.motion.circular.accel_time = single(5);
    params.motion.circular.angular_std = single(2.0);
    params.motion.circular.angular_tau = single(5.0);
    % Pitch/Roll振動
    params.motion.oscillation = struct();
        params.motion.oscillation.roll_amplitude_deg = single(15); % 0 = disabled
        params.motion.oscillation.roll_period = single(50);
        params.motion.oscillation.pitch_amplitude_deg = single(20); % 0 = disabled
        params.motion.oscillation.pitch_period = single(45);
    params.motion.oscillation.soft_start_time = single(5);
    
    % 上下運動 (高度振動)
    params.motion.vertical = struct();
    params.motion.vertical.amplitude = single(2.0);        % 振幅 [m]
    params.motion.vertical.period = single(10.0);          % 周期 [s]
    params.motion.vertical.soft_start_time = single(5);    % ソフトスタート時間 [s]
    
    % ランダムウォーク
    params.motion.random_walk = struct();
    params.motion.random_walk.velocity_std = single(0.0);
    params.motion.random_walk.angular_std = single(0.0);
    params.motion.random_walk.altitude_std = single(0.0);

    % GPS原点
    params.gps_origin = struct();
    params.gps_origin.lat = 36.0;
    params.gps_origin.lon = 140.0;
    params.gps_origin.alt = 0;

    % 初期状態
    params.initial = struct();
    params.initial.gps_position = [params.gps_origin.lat, params.gps_origin.lon, 0];
    params.initial.velocity = [0, 0, 0];
    params.initial.attitude = [0, 0, 0];
    % 初期地磁気ベクトル（world frame, 単位: μT）
    % 例: [50, 0, 0] = 北向き 50 μT, [0, 50, 0] = 東向き 50 μT
    params.initial.mag_ref = [0, 50, 0];

    % 出力設定
    cfg_dir = fileparts(mfilename('fullpath'));
    params.output = struct();
    params.output.dir = cfg_dir;
    params.output.truth_filename = 'truth_data.csv';
    params.output.sensor_filename = 'sensor_data.csv';

    params.thresholds = 1.0e-6;  % Thresholds

end

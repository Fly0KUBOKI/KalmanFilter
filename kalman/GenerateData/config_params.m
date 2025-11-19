function params = config_params()
    % シミュレーションパラメータ設定

    params = struct();

    % タイミング
    params.dt = 0.0025;
    params.T = 90;
    params.static_time = 0.5;

    % 運動タイプ
    params.motion_type = 'circular';

    % ヘディングモード
    params.heading_mode = 'align_velocity'; % 'fixed_north' or 'align_velocity'

    % センサーノイズ (1σ)
    params.noise = struct();
    params.noise.accel_std = 0.1;
    params.noise.gyro_std = 0.5;
    params.noise.mag_std = 10.0;
    params.noise.baro_std = 0.0;
    params.noise.gps_std = 1.0;
    % 外れ値設定
    params.noise.outlier = struct();
    params.noise.outlier.prob = 0.01;
    params.noise.outlier.range = struct( ...
        'accel', 2.0, ...  % m/s^2
        'gyro', 2.0, ...   % deg/s
        'mag', 50.0, ...    % nT
        'baro', 0.0, ...   % meters
        'gps', 10.0 ...     % meters
    );

    % ピンクノイズ
    params.noise.accel_pink_std = 0.1;
    params.noise.gyro_pink_std = 0.1;    % Gyroscope pink noise (deg/s)
    params.noise.gps_pink_std = 1.0;     % GPS pink noise (meters)

    % Allan偏差
    params.noise.gyro_allan_std = 0.5;
    params.noise.baro_allan_std = 0.0;   % Barometer Allan deviation (meters)

    % 運動パラメータ
    params.motion = struct();

    % 円運動
    params.motion.circular = struct();
    params.motion.circular.radius = 10;
    params.motion.circular.omega = 4;
    params.motion.circular.altitude = 0;
    params.motion.circular.accel_time = 5;
    params.motion.circular.angular_std = 2.0;
    params.motion.circular.angular_tau = 5.0;
    % Pitch/Roll振動
    params.motion.oscillation = struct();
    params.motion.oscillation.roll_amplitude_deg = 5;
    params.motion.oscillation.roll_period = 5;
    params.motion.oscillation.pitch_amplitude_deg = 3;
    params.motion.oscillation.pitch_period = 5;
    params.motion.oscillation.soft_start_time = 5;
    % ランダムウォーク
    params.motion.random_walk = struct();
    params.motion.random_walk.velocity_std = 0.0;
    params.motion.random_walk.angular_std = 0.0;
    params.motion.random_walk.altitude_std = 0.0;

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

    % 出力設定
    cfg_dir = fileparts(mfilename('fullpath'));
    params.output = struct();
    params.output.dir = cfg_dir;
    params.output.truth_filename = 'truth_data.csv';
    params.output.sensor_filename = 'sensor_data.csv';

    params.thresholds = 1.0e-6;  % Thresholds

end

function save_simulation_data(t, pos_world, vel_world, attitude, accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt, params)
    % SAVE_SIMULATION_DATA シミュレーションデータをCSVファイルに保存
    %
    % 入力:
    %   t - 時間ベクトル
    %   pos_world, vel_world, attitude - 真値データ
    %   accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt - センサーデータ
    %   params - 設定パラメータ（出力先を含む）

    % 出力ディレクトリ設定
    out_dir = '.';
    if isfield(params, 'output') && isfield(params.output, 'dir') && ~isempty(params.output.dir)
        out_dir = params.output.dir;
    end

    % ファイルパス設定
    truth_path = fullfile(out_dir, 'truth_data.csv');
    sensor_path = fullfile(out_dir, 'sensor_data.csv');
    if isfield(params, 'output') && isfield(params.output, 'truth_filename')
        truth_path = fullfile(out_dir, params.output.truth_filename);
    end
    if isfield(params, 'output') && isfield(params.output, 'sensor_filename')
        sensor_path = fullfile(out_dir, params.output.sensor_filename);
    end

    % 真値データの準備（姿勢を度に変換）
    att_deg = rad2deg(attitude);
    truth_data = [t, pos_world, vel_world, att_deg];
    % attitude 内部配列は [roll, pitch, yaw] のため、ヘッダはそれに合わせる
    truth_headers = {'time', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw'};

    % センサーデータの準備
    % Export: keep GPS as double, but cast other sensor channels to single
    time_col = t;
    accel_x = single(accel_body(:,1));
    accel_y = single(accel_body(:,2));
    accel_z = single(accel_body(:,3));
    gyro_x  = single(gyro_body(:,1));
    gyro_y  = single(gyro_body(:,2));
    gyro_z  = single(gyro_body(:,3));
    mag_x   = single(mag_body(:,1));
    mag_y   = single(mag_body(:,2));
    mag_z   = single(mag_body(:,3));
    baro_c  = single(baro);
    gps_lat_c = double(gps_lat); % explicitly cast to double
    gps_lon_c = double(gps_lon); % explicitly cast to double
    gps_alt_c = double(gps_alt); % explicitly cast to double

    % CSVファイル保存
    truth_table = array2table(truth_data, 'VariableNames', truth_headers);
    writetable(truth_table, truth_path);

    % Manually write sensor CSV so we can control numeric formatting.
    % Requirement: only GPS columns should be written with high-precision (double),
    % other sensor columns written with single-like precision.
    fid = fopen(sensor_path, 'w');
    if fid < 0
        error('save_simulation_data: cannot open %s for writing', sensor_path);
    end

    % Header
    fprintf(fid, 'time, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, baro, gps_lat, gps_lon, gps_alt\n');

    n = length(time_col);
    % Formats: time double (6 decimals), sensors single-like (7 decimals), gps double (10 decimals)
    fmt = '%.6f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.6f, %.10f, %.10f, %.6f\n';
    for i = 1:n
        fprintf(fid, fmt, single(time_col(i)), single(accel_x(i)), single(accel_y(i)), single(accel_z(i)), ...
            single(gyro_x(i)), single(gyro_y(i)), single(gyro_z(i)), ...
            single(mag_x(i)), single(mag_y(i)), single(mag_z(i)), ...
            single(baro_c(i)), gps_lat_c(i), gps_lon_c(i), gps_alt_c(i));
    end
    fclose(fid);

    fprintf('sim_generate: wrote %s and %s (%d samples)\n', truth_path, sensor_path, length(t));

end
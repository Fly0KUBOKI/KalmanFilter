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
    sensor_data = [t, accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt];
    sensor_headers = {'time', 'accel_x', 'accel_y', 'accel_z', ...
                    'gyro_x', 'gyro_y', 'gyro_z', ...
                    'mag_x', 'mag_y', 'mag_z', ...
                    'baro', 'gps_lat', 'gps_lon', 'gps_alt'};

    % CSVファイル保存
    truth_table = array2table(truth_data, 'VariableNames', truth_headers);
    writetable(truth_table, truth_path);

    sensor_table = array2table(sensor_data, 'VariableNames', sensor_headers);
    writetable(sensor_table, sensor_path);

    fprintf('sim_generate: wrote %s and %s (%d samples)\n', truth_path, sensor_path, length(t));

end
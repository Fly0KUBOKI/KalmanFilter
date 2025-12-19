% シミュレーションデータ生成
% 座標系: Body: 前(+X), 右(+Y), 下(+Z), Geographic: GPS座標
function sim_generate(params)

    % 引数チェック（clear前に実行）
    if nargin < 1 || isempty(params)
        params = config_params();
    end
    
    % clearはローカル変数のみ（paramsを保護）
    close all;

    params = convert_angle_units(params);

    dt = params.dt;
    T = params.T;
    N = floor(T/dt)+1;
    t = (0:N-1)' * dt;

    motion_type = params.motion_type;
    heading_mode = 'follow_velocity';
    if isfield(params, 'heading_mode')
        heading_mode = params.heading_mode;
    end

    % 運動生成
    if strcmp(motion_type, 'circular')
        [pos_world, vel_world, attitude] = generate_circular_motion(params, t, N);
    elseif strcmp(motion_type, 'random_walk')
        [pos_world, vel_world, attitude] = generate_random_walk(params, t, N);
    elseif strcmp(motion_type, 'stationary')
        % 完全静止状態
        pos_world = zeros(N,3);
        vel_world = zeros(N,3);
        attitude = zeros(N,3);
    else
        error('未サポートの運動タイプ: %s', motion_type);
    end

    % NOTE: don't apply apply_random_walk_overlay here — it may overwrite
    % the intended oscillatory attitude generation. Previously this helper
    % could replace roll/pitch with a smooth random walk; keep it disabled
    % to preserve the sinusoidal pitch/roll behavior.
    % if exist('apply_random_walk_overlay', 'file') == 2
    %     [pos_world, vel_world, attitude] = apply_random_walk_overlay(pos_world, vel_world, attitude, params, t);
    % end

    % 速度計算
    [vel_world, ~] = compute_body_velocities(pos_world, vel_world, attitude, params, heading_mode);

    % センサー観測生成
    [accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt] = ...
        generate_sensor_observations(pos_world, vel_world, attitude, params, t, N);

    % ノイズ追加
    [accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt] = ...
        add_sensor_noise(accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt, params);

    % 保存
    save_simulation_data(t, pos_world, vel_world, attitude, accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt, params);

end

function params = convert_angle_units(params)
    % 角度単位変換（度→rad）
    if isfield(params, 'initial') && isfield(params.initial, 'attitude')
        params.initial.attitude = deg2rad(params.initial.attitude);
    end
    if isfield(params, 'motion') && isfield(params.motion, 'circular') && isfield(params.motion.circular, 'omega')
        params.motion.circular.omega = deg2rad(params.motion.circular.omega);
    end
    if isfield(params, 'motion') && isfield(params.motion, 'random_walk') && isfield(params.motion.random_walk, 'angular_std')
        params.motion.random_walk.angular_std = deg2rad(params.motion.random_walk.angular_std);
    end
    % convert oscillation amplitudes (degrees -> radians) if provided
    if isfield(params, 'motion') && isfield(params.motion, 'oscillation')
        if isfield(params.motion.oscillation, 'roll_amplitude_deg')
            params.motion.oscillation.roll_amplitude = deg2rad(params.motion.oscillation.roll_amplitude_deg);
        end
        if isfield(params.motion.oscillation, 'pitch_amplitude_deg')
            params.motion.oscillation.pitch_amplitude = deg2rad(params.motion.oscillation.pitch_amplitude_deg);
        end
        % ensure soft_start_time exists
        if ~isfield(params.motion.oscillation, 'soft_start_time')
            params.motion.oscillation.soft_start_time = 3;
        end
    end
end

function data_out = apply_update_frequency(data_in, freq, N)
    % センサーデータを更新周期に応じて補完
    % freq: 更新周期（サンプル数）
    % 例: freq=5なら、5サンプルごとに値を更新し、その間は同じ値を保持
    
    data_out = data_in;
    
    for i = 1:N
        % 更新タイミング: i が freq の倍数の時のみ新しい値
        if mod(i-1, freq) ~= 0
            % 更新しない: 直前の更新値をコピー
            update_idx = floor((i-1) / freq) * freq + 1;
            if update_idx >= 1 && update_idx <= N
                data_out(i, :) = data_out(update_idx, :);
            end
        end
    end
end
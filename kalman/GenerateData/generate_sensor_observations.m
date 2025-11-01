function [accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt] = generate_sensor_observations(pos_world, vel_world, attitude, params, t, N)
    % GENERATE_SENSOR_OBSERVATIONS センサー観測値を生成
    %
    % 入力:
    %   pos_world - 世界座標系での位置 [N x 3]
    %   vel_world - 世界座標系での速度 [N x 3]
    %   attitude - 姿勢（ロール、ピッチ、ヨー） [N x 3]
    %   params - 設定パラメータ
    %   t - 時間ベクトル
    %   N - サンプル数
    % 出力:
    %   accel_body - 加速度計出力（比力） [N x 3]
    %   gyro_body - ジャイロスコープ出力 [N x 3]
    %   mag_body - 磁気計出力 [N x 3]
    %   baro - 気圧計出力 [N x 1]
    %   gps_lat, gps_lon, gps_alt - GPS出力 [N x 1]

    dt = params.dt;
    motion_type = params.motion_type;

    % GPS原点
    lat0 = params.gps_origin.lat;
    lon0 = params.gps_origin.lon;
    alt0 = params.gps_origin.alt;

    % 物理定数
    mag_strength = 50;  % nT

    % 円運動パラメータ（必要な場合）
    if strcmp(motion_type, 'circular')
        radius = params.motion.circular.radius;
        omega = params.motion.circular.omega;
        center_x = radius;
        center_y = 0;
        
        static_time = 5;
        if isfield(params, 'static_time')
            static_time = params.static_time;
        end
        accel_time = 5;
        if isfield(params.motion.circular, 'accel_time')
            accel_time = params.motion.circular.accel_time;
        end
    end

    % 初期化
    accel_body = zeros(N,3);
    gyro_body = zeros(N,3);
    mag_body = zeros(N,3);
    baro = zeros(N,1);
    gps_lat = zeros(N,1);
    gps_lon = zeros(N,1);
    gps_alt = zeros(N,1);

    % 各時刻でセンサー観測を生成
    for i = 1:N
        roll = attitude(i,1);
        pitch = attitude(i,2);
        yaw = attitude(i,3);

        % 回転行列（世界→ボディ）
        R = eul2rotm([yaw, pitch, roll], 'ZYX');

        % 加速度計（比力 = 加速度 - 重力）
        if strcmp(motion_type, 'circular')
            a_world = compute_circular_acceleration(i, t, pos_world, params, center_x, center_y, static_time, accel_time, omega, radius);
        else
            a_world = compute_general_acceleration(i, vel_world, dt, params);
        end
        
        g_world = [0, 0, -9.81];
        specific_force_world = a_world - g_world;
        accel_body(i,:) = (R' * specific_force_world')';

        % ジャイロスコープ（角速度）
        if strcmp(motion_type, 'circular')
            omega_world = compute_circular_angular_velocity(i, t, static_time, accel_time, omega);
            gyro_body(i,:) = rad2deg(R' * omega_world')';
        else
            omega_world = compute_general_angular_velocity(i, vel_world, dt, params);
            gyro_body(i,:) = rad2deg(R' * omega_world')';
        end

        % 磁気計
        mag_world = [0, mag_strength, 0];
        mag_body(i,:) = (R' * mag_world')';

        % 気圧計
        alt = alt0 + pos_world(i,3);
        P0 = 101325;
        alt_clip = min(alt, 44330 - eps);
        baro(i) = P0 * (1 - (alt_clip / 44330))^(1/0.1903);

        % GPS
        north_m = pos_world(i,2);
        east_m = pos_world(i,1);
        dlat = north_m * 9.0e-6;
        dlon = east_m * (9.0e-6 / max(cosd(lat0), 1e-6));
        gps_lat(i) = lat0 + dlat;
        gps_lon(i) = lon0 + dlon;
        gps_alt(i) = alt;
        if mod(i, 10000) == 0
            fprintf('Sensor step %d / %d\n', i, N);
        end
    end

end

function a_world = compute_circular_acceleration(i, t, pos_world, params, center_x, center_y, static_time, accel_time, omega, radius)
    % 円運動の加速度計算
    if i == 1
        a_world = [0,0,0];
    else
        rel_x = pos_world(i,1) - center_x;
        rel_y = pos_world(i,2) - center_y;
        r_norm = sqrt(rel_x^2 + rel_y^2);
        
        if r_norm > params.thresholds
            % 角速度スケーリングと角加速度
            if t(i) < static_time
                omega_scale = 0.0;
                alpha = 0;
            elseif t(i) < static_time + accel_time
                t_motion = t(i) - static_time;
                omega_scale = 0.5 * (1 - cos(pi * t_motion / accel_time));
                alpha = -omega * 0.5 * (pi / accel_time) * sin(pi * t_motion / accel_time);
            else
                omega_scale = 1.0;
                alpha = 0;
            end
            
            theta_dot_current = -omega * omega_scale;
            
            % 向心加速度
            a_centripetal_mag = theta_dot_current^2 * r_norm;
            a_centripetal = [-rel_x/r_norm * a_centripetal_mag, -rel_y/r_norm * a_centripetal_mag, 0];
            
            % 接線加速度
            theta_angle = atan2(rel_y, rel_x);
            tangent_x = -sin(theta_angle);
            tangent_y = cos(theta_angle);
            a_tangential_mag = alpha * r_norm;
            a_tangential = [tangent_x * a_tangential_mag, tangent_y * a_tangential_mag, 0];
            
            a_world = a_centripetal + a_tangential;
        else
            a_world = [0,0,0];
        end
    end
end

function a_world = compute_general_acceleration(i, vel_world, dt, params)
    % 一般的な運動の加速度計算（差分）
    if i == 1
        a_world = [0,0,0];
    else
        dv = vel_world(i,:) - vel_world(i-1,:);
        for j = 1:3
            if abs(dv(j)) < params.thresholds
                dv(j) = 0;
            end
        end
        a_world = dv / dt;
    end
end

function omega_world = compute_circular_angular_velocity(i, t, static_time, accel_time, omega)
    % 円運動の角速度計算
    if t(i) < static_time
        omega_scale = 0.0;
    elseif t(i) < static_time + accel_time
        t_motion = t(i) - static_time;
        omega_scale = 0.5 * (1 - cos(pi * t_motion / accel_time));
    else
        omega_scale = 1.0;
    end

    theta_dot_gyro = -omega * omega_scale;
    omega_world = [0, 0, theta_dot_gyro];
end

function omega_world = compute_general_angular_velocity(i, vel_world, dt, params)
    % 一般的な運動の角速度計算
    if i > 1
        u = vel_world(i-1,:); 
        v = vel_world(i,:);
        nu = norm(u); 
        nv = norm(v);
        
        if nu < params.thresholds || nv < params.thresholds
            omega_world = [0,0,0];
        else
            u_n = u / nu; 
            v_n = v / nv;
            d = dot(u_n, v_n);
            d = min(1, max(-1, d));
            angle = acos(d);
            axis = cross(u_n, v_n);
            an = norm(axis);
            if an < params.thresholds
                omega_world = [0,0,0];
            else
                axis_n = axis / an;
                omega_world = (axis_n * (angle / dt));
            end
        end
    else
        omega_world = [0,0,0];
    end
end
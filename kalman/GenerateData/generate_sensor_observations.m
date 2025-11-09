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

    % pitch/rollの角速度を姿勢角の時間微分から直接計算
    pitch_rate = zeros(N,1);
    roll_rate = zeros(N,1);
    
    for i = 1:N
        if i == 1
            % 前進差分
            if N > 1
                roll_rate(i) = (attitude(2,1) - attitude(1,1)) / dt;
                pitch_rate(i) = (attitude(2,2) - attitude(1,2)) / dt;
            else
                roll_rate(i) = 0;
                pitch_rate(i) = 0;
            end
        elseif i == N
            % 後退差分
            roll_rate(i) = (attitude(N,1) - attitude(N-1,1)) / dt;
            pitch_rate(i) = (attitude(N,2) - attitude(N-1,2)) / dt;
        else
            % 中央差分（より正確）
            roll_rate(i) = (attitude(i+1,1) - attitude(i-1,1)) / (2*dt);
            pitch_rate(i) = (attitude(i+1,2) - attitude(i-1,2)) / (2*dt);
        end
    end

    % 各時刻でセンサー観測を生成
    for i = 1:N
        roll = attitude(i,1);
        pitch = attitude(i,2);
        yaw = attitude(i,3);

    % 回転行列（世界→ボディ）: pitch=x軸, roll=y軸, yaw=z軸周り
    R = eul2rotm([yaw, pitch, roll], 'ZXY');

        % 加速度計（比力 = 加速度 - 重力）
        if strcmp(motion_type, 'circular')
            a_world = compute_circular_acceleration(i, t, pos_world, params, center_x, center_y, static_time, accel_time, omega, radius);
        else
            a_world = compute_general_acceleration(i, vel_world, dt, params);
        end
        
        g_world = [0, 0, -9.81];
        specific_force_world = a_world - g_world;
    % 機体軸: x=roll, y=pitch, z=down となるように列を配置（直接マップ）
    accel_tmp = (R' * specific_force_world')';
    % x軸をroll方向、y軸をpitch方向としてそのまま割り当てる
    accel_body(i,1) = accel_tmp(1); % x: roll方向
    accel_body(i,2) = accel_tmp(2); % y: pitch方向
    accel_body(i,3) = accel_tmp(3); % z: down

        % ジャイロスコープ（角速度）
        % pitch/rollは姿勢角の時間微分から直接計算
        % yawは運動モデルから取得（位置・速度に依存）
        
        % yaw角速度を運動モデルから計算
        if strcmp(motion_type, 'circular')
            omega_yaw_world = compute_circular_yaw_rate(i, t, static_time, accel_time, omega);
        else
            omega_yaw_world = compute_general_yaw_rate(i, vel_world, dt, params);
        end
        
        % 体軸角速度：[p, q, r] = [roll_rate, pitch_rate, yaw_rate]
        % Euler角微分から体軸角速度への変換
        roll = attitude(i,1);
        pitch = attitude(i,2);
        
        cos_pitch = cos(pitch);
        sin_pitch = sin(pitch);
        cos_roll = cos(roll);
        sin_roll = sin(roll);
        
        % 体軸角速度計算（Euler角微分→体軸角速度変換）
        if abs(cos_pitch) > params.thresholds  % 特異点回避
            p = roll_rate(i) - sin_pitch * omega_yaw_world;
            q = cos_roll * pitch_rate(i) + sin_roll * cos_pitch * omega_yaw_world;
            r = -sin_roll * pitch_rate(i) + cos_roll * cos_pitch * omega_yaw_world;
        else
            % 特異点近傍では近似
            p = roll_rate(i);
            q = pitch_rate(i);
            r = omega_yaw_world;
        end
        
    % x=pitch(アップ正/ダウン負), y=roll(右回り正/左回り負), z=ヨー
    % pitch: q, roll: p, yaw: r
    % pitch(アップ正/ダウン負)はq, roll(右回り正/左回り負)はp
    gyro_tmp = rad2deg([p, q, r]);
    gyro_body(i,1) = gyro_tmp(2); % x: pitch (アップ正/ダウン負)
    gyro_body(i,2) = gyro_tmp(1); % y: roll (右回り正/左回り負)
    gyro_body(i,3) = gyro_tmp(3); % z: yaw

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

function yaw_rate = compute_circular_yaw_rate(i, t, static_time, accel_time, omega)
    % 円運動のyaw角速度計算
    if t(i) < static_time
        omega_scale = 0.0;
    elseif t(i) < static_time + accel_time
        t_motion = t(i) - static_time;
        omega_scale = 0.5 * (1 - cos(pi * t_motion / accel_time));
    else
        omega_scale = 1.0;
    end

    yaw_rate = -omega * omega_scale;  % 時計回り
end

function yaw_rate = compute_general_yaw_rate(i, vel_world, dt, params)
    % 一般的な運動のyaw角速度計算（速度ベクトルの変化から）
    if i > 1
        u = vel_world(i-1,:); 
        v = vel_world(i,:);
        nu = norm(u); 
        nv = norm(v);
        
        if nu < params.thresholds || nv < params.thresholds
            yaw_rate = 0;
        else
            u_n = u / nu; 
            v_n = v / nv;
            d = dot(u_n, v_n);
            d = min(1, max(-1, d));
            angle = acos(d);
            axis = cross(u_n, v_n);
            an = norm(axis);
            if an < params.thresholds
                yaw_rate = 0;
            else
                % ヨー成分のみ抽出（Z軸周りの回転）
                if axis(3) > 0
                    yaw_rate = angle / dt;
                else
                    yaw_rate = -angle / dt;
                end
            end
        end
    else
        yaw_rate = 0;
    end
end
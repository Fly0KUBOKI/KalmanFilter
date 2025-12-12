function update_filter(obj, obs, k)
    % 1ステップ更新実行
    
    % センサーデータ取得
    a = [obs.accel_x(k); obs.accel_y(k); obs.accel_z(k)];
    % 生成データの角速度: 既に [roll_rate, pitch_rate, yaw_rate] の順
    % ESKF内部: x=roll, y=pitch, z=yaw
    % 軸の入れ替えは不要
    w = deg2rad([obs.gyro_x(k); obs.gyro_y(k); obs.gyro_z(k)]);
    
    % 予測ステップ
    obj.predict(a, w);
    
    % 周期的更新
    if mod(k, obj.freq_accel) == 0
        obj.sensor_updates('accel', a);
    end
    if mod(k, obj.freq_mag) == 0
        obj.sensor_updates('mag', [obs.mag_x(k); obs.mag_y(k); obs.mag_z(k)]);
    end
    if mod(k, obj.freq_baro) == 0
        obj.sensor_updates('baro', obs.baro(k));
    end
    if mod(k, obj.freq_gps) == 0 && ~isnan(obs.gps_lat(k)) && ~isnan(obs.gps_lon(k))
        obj.sensor_updates('gps', obs.gps_lat(k), obs.gps_lon(k), obs.gps_alt(k), k);
    end
    
    % 発散チェックとリセット
    obj.reset('check', obs, k);
end

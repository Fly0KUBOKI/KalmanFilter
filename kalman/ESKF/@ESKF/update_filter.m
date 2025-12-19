function update_filter(obj, obs, k)
    % 1ステップ更新実行
    % 注: センサー更新周期はGenerateData段階で制御済み
    %     C++側で前回値との差分により自動的に更新判定される
    
    % センサーデータ取得
    a = [obs.accel_x(k); obs.accel_y(k); obs.accel_z(k)];
    w = deg2rad([obs.gyro_x(k); obs.gyro_y(k); obs.gyro_z(k)]);
    
    % 予測ステップ
    obj.predict(a, w);
    
    % センサー更新（すべて毎回実行、C++側で変更検知により自動更新）
    obj.sensor_updates('accel', a);
    obj.sensor_updates('mag', [obs.mag_x(k); obs.mag_y(k); obs.mag_z(k)]);
    obj.sensor_updates('baro', obs.baro(k));
    
    if ~isnan(obs.gps_lat(k)) && ~isnan(obs.gps_lon(k))
        obj.sensor_updates('gps', obs.gps_lat(k), obs.gps_lon(k), obs.gps_alt(k), k);
    end
    
    % 発散チェックとリセット
    obj.reset('check', obs, k);
end

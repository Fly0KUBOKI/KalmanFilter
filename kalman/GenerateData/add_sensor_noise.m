function [accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt] = add_sensor_noise(accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt, params)
% ADD_SENSOR_NOISE センサー観測値にノイズを追加
%
% 入力/出力:
%   accel_body, gyro_body, mag_body, baro, gps_lat, gps_lon, gps_alt - センサー観測値
%   params - ノイズパラメータを含む設定

if ~isfield(params, 'noise')
    return;  % ノイズ設定がない場合は何もしない
end

N = size(accel_body, 1);

% ホワイトノイズ
if isfield(params.noise, 'accel_std')
    accel_body = accel_body + randn(N,3) * params.noise.accel_std;
end
if isfield(params.noise, 'gyro_std')
    gyro_body = gyro_body + randn(N,3) * params.noise.gyro_std;
end
if isfield(params.noise, 'mag_std')
    mag_body = mag_body + randn(N,3) * params.noise.mag_std;
end
if isfield(params.noise, 'baro_std')
    baro = baro + randn(N,1) * params.noise.baro_std;
end
if isfield(params.noise, 'gps_std')
    gps_lat = gps_lat + randn(N,1) * params.noise.gps_std * 9.0e-6;
    gps_lon = gps_lon + randn(N,1) .* (params.noise.gps_std * 9.0e-6 ./ max(cosd(gps_lat), 1e-6));
    gps_alt = gps_alt + randn(N,1) * params.noise.gps_std;
end

% ピンクノイズ（1/fノイズ）
if isfield(params.noise, 'accel_pink_std') && params.noise.accel_pink_std > 0
    for j = 1:3
        pink = generate_pink_noise(N);
        accel_body(:,j) = accel_body(:,j) + pink * params.noise.accel_pink_std;
    end
end
if isfield(params.noise, 'gyro_pink_std') && params.noise.gyro_pink_std > 0
    for j = 1:3
        pink = generate_pink_noise(N);
        gyro_body(:,j) = gyro_body(:,j) + pink * params.noise.gyro_pink_std;
    end
end
if isfield(params.noise, 'gps_pink_std') && params.noise.gps_pink_std > 0
    pink = generate_pink_noise(N);
    gps_lat = gps_lat + pink * params.noise.gps_pink_std * 9.0e-6;
    pink = generate_pink_noise(N);
    gps_lon = gps_lon + pink .* (params.noise.gps_pink_std * 9.0e-6 ./ max(cosd(gps_lat), 1e-6));
    pink = generate_pink_noise(N);
    gps_alt = gps_alt + pink * params.noise.gps_pink_std;
end

% アラン偏差（バイアス不安定性）
if isfield(params.noise, 'gyro_allan_std') && params.noise.gyro_allan_std > 0
    dt = params.dt;
    for j = 1:3
        bias = cumsum(randn(N,1)) * params.noise.gyro_allan_std * sqrt(dt);
        gyro_body(:,j) = gyro_body(:,j) + bias;
    end
end
if isfield(params.noise, 'baro_allan_std') && params.noise.baro_allan_std > 0
    dt = params.dt;
    bias = cumsum(randn(N,1)) * params.noise.baro_allan_std * sqrt(dt);
    baro = baro + bias;
end

end

function pink = generate_pink_noise(N)
% ピンクノイズ生成（Voss-McCartney アルゴリズム）
white = randn(N, 1);
b = [0.049922035, -0.095993537, 0.050612699, -0.004408786];
a = 1;
pink = filter(b, a, white);
pink = pink / std(pink);
end
function obs = read_csv(filePath)
% READ_CSV  observation.csv を読み込み必須列を構造体で返す
% 必須列: time, ax, ay, az, wx, wy, wz, mx, my, mz, pressure, lat, lon, alt

if nargin<1 || ~exist(filePath,'file')
    error('read_csv: file not found: %s', filePath);
end

T = readtable(filePath);

% Accept either canonical names or dataset-specific names
pairs = {
    'time', {'time'};
    'ax',   {'ax','accel_x'};
    'ay',   {'ay','accel_y'};
    'az',   {'az','accel_z'};
    'wx',   {'wx','gyro_x'};
    'wy',   {'wy','gyro_y'};
    'wz',   {'wz','gyro_z'};
    'mx',   {'mx','mag_x'};
    'my',   {'my','mag_y'};
    'mz',   {'mz','mag_z'};
    'pressure', {'pressure','baro'};
    'lat',  {'lat','gps_lat'};
    'lon',  {'lon','gps_lon'};
    'alt',  {'alt','gps_alt'};
    };

for i=1:size(pairs,1)
    key = pairs{i,1}; names = pairs{i,2}; found = false;
    for j=1:numel(names)
        if ismember(names{j}, T.Properties.VariableNames)
            obs.(key) = T.(names{j}); found = true; break;
        end
    end
    if ~found
        error('read_csv: missing column for %s (tried: %s)', key, strjoin(names,', '));
    end
end
end

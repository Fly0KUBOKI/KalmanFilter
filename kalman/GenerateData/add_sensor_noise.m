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

    %% ホワイトノイズ
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

    %% ピンクノイズ（1/fノイズ）
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

    %% アラン偏差（バイアス不安定性）
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
    %% 外れ値 (outliers)
    % params.noise.outlier の想定フォーマット:
    %   .prob  - 各サンプルが外れ値になる確率 (0-1)
    %   .range - 外れ値の振幅を指定する構造体またはスカラー
    %            例: params.noise.outlier.range.accel = 5;  % m/s^2 など
    %            スカラーを与えるとすべてのセンサーに適用される
    % もし range に各センサー用のフィールドが無ければ、経験的なデフォルトを使う
    if isfield(params.noise, 'outlier') && isfield(params.noise.outlier, 'prob') && params.noise.outlier.prob > 0
        p = params.noise.outlier.prob;
        r = params.noise.outlier;

        % 実装しやすくするため、個別に取り出す（柔軟にスカラー/ベクトルを許容）
        if isfield(r, 'range') && isstruct(r.range)
            rr = r.range;
        else
            rr = struct();
        end

        % defaults: 推定値。パラメータが与えられない場合の安全な既定値
        def_accel = (isfield(params.noise, 'accel_std') && params.noise.accel_std>0) * (10*params.noise.accel_std) + (~(isfield(params.noise, 'accel_std') && params.noise.accel_std>0)) * 1.0;
        def_gyro  = (isfield(params.noise, 'gyro_std') && params.noise.gyro_std>0)  * (10*params.noise.gyro_std)  + (~(isfield(params.noise, 'gyro_std') && params.noise.gyro_std>0))  * 0.1;
        def_mag   = (isfield(params.noise, 'mag_std') && params.noise.mag_std>0)    * (10*params.noise.mag_std)   + (~(isfield(params.noise, 'mag_std') && params.noise.mag_std>0))    * 0.1;
        def_baro  = 5.0; % m
        def_gps_m = 50.0; % m (lat/lon/alt に対する外れ値の大きさのデフォルト)

        % accel
        if isfield(rr, 'accel')
            accel_range = rr.accel;
        elseif isfield(r, 'range') && ~isstruct(r.range)
            accel_range = r.range;
        else
            accel_range = def_accel;
        end

        % gyro
        if isfield(rr, 'gyro')
            gyro_range = rr.gyro;
        elseif isfield(r, 'range') && ~isstruct(r.range)
            gyro_range = r.range;
        else
            gyro_range = def_gyro;
        end

        % mag
        if isfield(rr, 'mag')
            mag_range = rr.mag;
        elseif isfield(r, 'range') && ~isstruct(r.range)
            mag_range = r.range;
        else
            mag_range = def_mag;
        end

        % baro
        if isfield(rr, 'baro')
            baro_range = rr.baro;
        elseif isfield(r, 'range') && ~isstruct(r.range)
            baro_range = r.range;
        else
            baro_range = def_baro;
        end

        % gps (レンジはメートルで与える想定)
        if isfield(rr, 'gps')
            gps_range_m = rr.gps;
        elseif isfield(r, 'range') && ~isstruct(r.range)
            gps_range_m = r.range;
        else
            gps_range_m = def_gps_m;
        end

        % マスクを作ってランダムに外れ値を挿入
        mask = rand(N,1) < p;
        n_out = nnz(mask);
        if n_out > 0
            idx = find(mask);

            % accel, 3軸 (スカラー -> 各軸同じ振幅、1x3 -> 各軸個別)
            if ~isempty(accel_range)
                if isscalar(accel_range)
                    ar = repmat(accel_range, 1, 3);
                else
                    ar = reshape(accel_range, 1, []);
                    if numel(ar) == 1, ar = repmat(ar,1,3); end
                end
                noise = (rand(n_out,3)*2 - 1) .* ar;
                accel_body(idx, :) = accel_body(idx, :) + noise;
            end

            % gyro
            if ~isempty(gyro_range)
                if isscalar(gyro_range)
                    gr = repmat(gyro_range, 1, 3);
                else
                    gr = reshape(gyro_range, 1, []);
                    if numel(gr) == 1, gr = repmat(gr,1,3); end
                end
                noise = (rand(n_out,3)*2 - 1) .* gr;
                gyro_body(idx, :) = gyro_body(idx, :) + noise;
            end

            % mag
            if ~isempty(mag_range)
                if isscalar(mag_range)
                    mr = repmat(mag_range, 1, 3);
                else
                    mr = reshape(mag_range, 1, []);
                    if numel(mr) == 1, mr = repmat(mr,1,3); end
                end
                noise = (rand(n_out,3)*2 - 1) .* mr;
                mag_body(idx, :) = mag_body(idx, :) + noise;
            end

            % baro
            if ~isempty(baro_range)
                noise = (rand(n_out,1)*2 - 1) * baro_range;
                baro(idx) = baro(idx) + noise;
            end

            % gps: lat/lon をメートルから度に変換して外れ値を加える
            if ~isempty(gps_range_m)
                % 緯度 (deg)
                dlat = (rand(n_out,1)*2 - 1) * (gps_range_m * 9.0e-6);
                % 経度は緯度に応じてスケール
                lon_scale = max(cosd(gps_lat(idx)), 1e-6);
                dlon = (rand(n_out,1)*2 - 1) .* (gps_range_m * 9.0e-6 ./ lon_scale);
                dalt = (rand(n_out,1)*2 - 1) * gps_range_m; % 高度はメートル
                gps_lat(idx) = gps_lat(idx) + dlat;
                gps_lon(idx) = gps_lon(idx) + dlon;
                gps_alt(idx) = gps_alt(idx) + dalt;
            end
        end
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
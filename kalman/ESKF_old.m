classdef ESKF < handle
    % ESKF  Error State Kalman Filter for IMU/GPS/Mag/Baro
    % 隱､蟾ｮ迥ｶ諷九き繝ｫ繝槭Φ繝輔ぅ繝ｫ繧ｿ・・SKF・牙ｮ溯｣・    % 
    % 菴ｿ逕ｨ譁ｹ豕・
    %   eskf = ESKF(obs, static_time, dt);
    %   eskf.updateFilter(obs, k);
    %   euler = eskf.getEuler();

    properties
        % Nominal / error-state components and configuration
        p
        v
        q
        ba
        bg
        P
        Q
        dt
        g
        noiseEstimator
        sensor_filters
        accel_filter
        divergence_guard
        max_dx_norm
        gyro_filter_yaw_alpha
        enable_yaw_raw_gyro
        enable_mag_update
        enable_gyro_filter
        freq_mag
        freq_baro
        freq_gps
        freq_accel
        gyro_noise_threshold
        gps_origin
    end

    methods
        function obj = ESKF(obs, static_time, dt)
            % Constructor
            % obs: observation structure
            % static_time: duration of static period (seconds)
            % dt: sampling interval (seconds)
            
            if nargin < 3 || isempty(dt)
                dt = 1/100;
            end
            obj.dt = dt;

            % Calculate static indices from static_time
            if nargin >= 2 && ~isempty(static_time) && static_time > 0
                N_static = floor(static_time / dt);
                if isfield(obs, 'accel_x') && length(obs.accel_x) >= N_static
                    static_idx = 1:N_static;
                else
                    static_idx = [];
                end
            else
                static_idx = [];
            end

            % default nominal states
            obj.p = zeros(3,1);
            obj.v = zeros(3,1);
            
            % gravity
            obj.g = [0;0;-9.80665];

            % Initialize noise parameters
            if ~isempty(static_idx) && length(static_idx) > 10
                % Estimate noise from static period
                accel_static = [obs.accel_x(static_idx), obs.accel_y(static_idx), obs.accel_z(static_idx)];
                accel_mean = mean(accel_static, 1);
                sigma_a = mean(std(accel_static - accel_mean, [], 1));
                
                % 蛻晄悄蟋ｿ蜍｢縺ｮ謗ｨ螳・(蜉騾溷ｺｦ蟷ｳ蝮・°繧・
                % 蜉騾溷ｺｦ險医・荳雁髄縺阪・蜉帙ｒ貂ｬ螳壹☆繧九◆繧√・撕豁｢譎ゅ・ [0, 0, g] (荳雁髄縺・ 繧呈欠縺・                % 驥榊鴨繝吶け繝医Ν縺ｯ [0, 0, -g] (荳句髄縺・
                % 縺励◆縺後▲縺ｦ縲∝刈騾溷ｺｦ險医・蜃ｺ蜉帙・繧ｯ繝医Ν繧呈ｭ｣隕丞喧縺励※縲√◎繧後′ [0, 0, 1] 縺ｫ縺ｪ繧九ｈ縺・↑蝗櫁ｻ｢繧呈ｱゅａ繧・                % 縺溘□縺励√％縺薙〒縺ｯ邁｡譏鍋噪縺ｫ Roll/Pitch 繧定ｨ育ｮ励☆繧・                % ax = g * sin(pitch)
                % ay = -g * cos(pitch) * sin(roll)
                % az = -g * cos(pitch) * cos(roll)
                % (NED蠎ｧ讓咏ｳｻ縲・㍾蜉帑ｸ句髄縺阪∝刈騾溷ｺｦ險亥・蜉・= -g_body)
                % g_body = R' * [0;0;-g] = R' * g_ned
                % a_meas = -g_body = -R' * [0;0;-g] = R' * [0;0;g]
                % a_meas = [  g * sin(theta)            ]
                %          [ -g * cos(theta) * sin(phi) ]
                %          [ -g * cos(theta) * cos(phi) ]
                
                phi = atan2(-accel_mean(2), -accel_mean(3));
                theta = atan2(accel_mean(1), sqrt(accel_mean(2)^2 + accel_mean(3)^2));
                
                obj.q = QuaternionLib.from_euler([phi; theta; 0]);
                
                gyro_static = [obs.gyro_x(static_idx), obs.gyro_y(static_idx), obs.gyro_z(static_idx)];
                sigma_g = mean(std(gyro_static, [], 1));
                sigma_g = deg2rad(sigma_g);
                
                if isfield(obs, 'mag_x')
                    mag_static = [obs.mag_x(static_idx), obs.mag_y(static_idx), obs.mag_z(static_idx)];
                    mag_mean = mean(mag_static, 1);
                    sigma_mag = mean(std(mag_static - mag_mean, [], 1));
                else
                    sigma_mag = 10.0;
                end
                
                if isfield(obs, 'baro')
                    P0 = 101325;
                    pressure_static = obs.baro(static_idx);
                    alt_baro_static = 44330 * (1 - (pressure_static / P0).^0.1903);
                    sigma_press = std(alt_baro_static - mean(alt_baro_static));
                else
                    sigma_press = 1.0;
                end
                
                if isfield(obs, 'gps_lat') && isfield(obs, 'gps_lon') && isfield(obs, 'gps_alt')
                    lat_static = obs.gps_lat(static_idx);
                    lon_static = obs.gps_lon(static_idx);
                    alt_static = obs.gps_alt(static_idx);
                    lat0 = mean(lat_static);
                    lon0 = mean(lon_static);
                    y_m = (lat_static - lat0) / (9.0e-6);
                    x_m = (lon_static - lon0) / (9.0e-6 / cosd(lat0));
                    z_m = alt_static - mean(alt_static);
                    sigma_gps = mean([std(x_m); std(y_m); std(z_m)]);
                else
                    sigma_gps = 1.0;
                end
                
                % Gyro noise threshold
                wx_all = deg2rad(obs.gyro_x(:));
                wy_all = deg2rad(obs.gyro_y(:));
                wz_all = deg2rad(obs.gyro_z(:));
                std_wx = std(wx_all);
                std_wy = std(wy_all);
                std_wz = std(wz_all);
                obj.gyro_noise_threshold = 2 * max([std_wx, std_wy, std_wz]);
            else
                % Default values
                sigma_a = 0.1;
                sigma_g = deg2rad(0.1);
                sigma_mag = 10.0;
                sigma_press = 1.0;
                sigma_gps = 1.0;
                obj.gyro_noise_threshold = deg2rad(0.1);
                obj.q = [1;0;0;0]; % 髱呎ｭ｢繝・・繧ｿ縺後↑縺・ｴ蜷医・豌ｴ蟷ｳ縺ｨ莉ｮ螳・            end
            
            obj.ba = zeros(3,1);
            obj.bg = zeros(3,1);

            % Process noise Q
            obj.Q = zeros(15);
            obj.Q(4:6, 4:6) = eye(3) * (0.005^2); % 騾溷ｺｦ縺ｮ繝ｩ繝ｳ繝繝繧ｦ繧ｩ繝ｼ繧ｯ繧貞ｰ上＆縺・(0.01 -> 0.005)
            obj.Q(7:9, 7:9) = eye(3) * (0.005^2); % 蟋ｿ蜍｢縺ｮ繝ｩ繝ｳ繝繝繧ｦ繧ｩ繝ｼ繧ｯ繧貞ｰ上＆縺・(0.01 -> 0.005)
            obj.Q(10:12, 10:12) = eye(3) * (sigma_a^2 * 1e-5); % 繝舌う繧｢繧ｹ螟牙虚繧貞ｰ上＆縺・            obj.Q(13:15, 13:15) = eye(3) * (sigma_g^2 * 1e-6);

            % Initial covariance
            % 菴咲ｽｮ繝ｻ騾溷ｺｦ縺ｮ蛻晄悄荳咲｢ｺ縺九＆繧貞､ｧ縺阪￥險ｭ螳夲ｼ・PS譖ｴ譁ｰ繧貞柑譫懃噪縺ｫ縺吶ｋ縺溘ａ・・            obj.P = eye(15) * 0.01;
            obj.P(1:3, 1:3) = eye(3) * 10.0;  % 菴咲ｽｮ縺ｮ蛻晄悄蛻・淵・・0 m^2・・            obj.P(4:6, 4:6) = eye(3) * 0.1;   % 騾溷ｺｦ縺ｮ蛻晄悄蛻・淵繧貞ｰ上＆縺・(1.0 -> 0.1) 蛻晄悄縺ｯ髱呎ｭ｢縺励※縺・ｋ縺溘ａ
            obj.P(10:12, 10:12) = eye(3) * 0.1; % 蜉騾溷ｺｦ繝舌う繧｢繧ｹ縺ｮ蛻晄悄蛻・淵繧貞ｰ代＠螟ｧ縺阪￥縺励※蜿取據繧呈掠繧√ｋ

            % Noise estimator
            try
                obj.noiseEstimator = NoiseEstimator(10);
                obj.noiseEstimator.R_accel = ones(3,1) * (sigma_a^2);
                obj.noiseEstimator.R_gyro  = ones(3,1) * (sigma_g^2);
                obj.noiseEstimator.R_mag   = ones(3,1) * (sigma_mag^2);
                obj.noiseEstimator.R_baro  = (sigma_press^2);
                obj.noiseEstimator.R_gps   = ones(3,1) * (sigma_gps^2);
            catch
                obj.noiseEstimator = [];
            end

            % Sensor filters
            obj.sensor_filters = struct();
            if exist('SensorFilter','class')
                try
                    obj.sensor_filters.accel = SensorFilter.createAccelFilter();
                catch
                end
                try
                    obj.sensor_filters.gyro  = SensorFilter.createGyroFilter();
                catch
                end
                try
                    obj.sensor_filters.mag   = SensorFilter.createMagFilter();
                catch
                end
                try
                    obj.sensor_filters.gps   = SensorFilter.createGPSFilter();
                catch
                end
                try
                    obj.sensor_filters.baro  = SensorFilter.createBaroFilter();
                catch
                end
            end

            % Accel filter
            try
                % 騾溷ｺｦ謗ｨ螳夂畑縺ｮ蠑ｷ蜉帙↑繝輔ぅ繝ｫ繧ｿ (alpha=0.05: 髱槫ｸｸ縺ｫ蠑ｷ縺・ｹｳ貊大喧)
                obj.accel_filter = AccelFilter(0.05, 50);
            catch
                obj.accel_filter = [];
            end

            % Filter settings
            obj.gyro_filter_yaw_alpha = 0.08;
            obj.enable_yaw_raw_gyro = false;
            obj.enable_mag_update = false;
            obj.enable_gyro_filter = true;

            obj.freq_mag = 4;
            obj.freq_baro = 8;
            obj.freq_gps = 10;
            obj.freq_accel = 4;

            obj.max_dx_norm = 5.0;

            % Divergence guard
            try
                config = struct();
                config.max_velocity = 2.0;
                config.max_acceleration = 2.0;
                config.max_allowed_innov = 50.0;
                config.max_innov_cap_fraction = 0.5;
                config.max_gain_norm = 100;
                config.innov_change_ratio_threshold = 2.0;
                config.attenuation_factor = 0.5;
                config.max_attitude_variance = (deg2rad(10))^2;
                config.max_mag_gain_element = 0.15;
                obj.divergence_guard = DivergenceGuard(config);
            catch
                obj.divergence_guard = [];
            end

            % GPS origin
            if ~isempty(static_idx) && isfield(obs,'gps_lat') && isfield(obs,'gps_lon') && isfield(obs,'gps_alt')
                obj.gps_origin = [mean(obs.gps_lat(static_idx)); mean(obs.gps_lon(static_idx)); mean(obs.gps_alt(static_idx))];
            elseif ~isempty(static_idx) && isfield(obs,'lat') && isfield(obs,'lon') && isfield(obs,'alt')
                obj.gps_origin = [mean(obs.lat(static_idx)); mean(obs.lon(static_idx)); mean(obs.alt(static_idx))];
            elseif isfield(obs,'gps_lat') && isfield(obs,'gps_lon') && isfield(obs,'gps_alt')
                obj.gps_origin = [obs.gps_lat(1); obs.gps_lon(1); obs.gps_alt(1)];
            elseif isfield(obs,'lat') && isfield(obs,'lon') && isfield(obs,'alt')
                obj.gps_origin = [obs.lat(1); obs.lon(1); obs.alt(1)];
            else
                obj.gps_origin = [0;0;0];
            end
        end
        
        function update_filter(obj, obs, k)
            % 1繧ｹ繝・ャ繝玲峩譁ｰ螳溯｡・
            % 繧ｻ繝ｳ繧ｵ繝ｼ繝・・繧ｿ蜿門ｾ・            a = [obs.accel_x(k); obs.accel_y(k); obs.accel_z(k)];
            % 逕滓・繝・・繧ｿ縺ｮ隗帝溷ｺｦ: 譌｢縺ｫ [roll_rate, pitch_rate, yaw_rate] 縺ｮ鬆・            % ESKF蜀・Κ: x=roll, y=pitch, z=yaw
            % 霆ｸ縺ｮ蜈･繧梧崛縺医・荳崎ｦ・            w = deg2rad([obs.gyro_x(k); obs.gyro_y(k); obs.gyro_z(k)]);

            % 莠域ｸｬ繧ｹ繝・ャ繝・            obj.predict(a, w);
        
            % 蜻ｨ譛溽噪譖ｴ譁ｰ
            if mod(k, obj.freq_accel) == 0
                obj.update_accel(a);
            end
            if mod(k, obj.freq_mag) == 0
                obj.update_mag([obs.mag_x(k); obs.mag_y(k); obs.mag_z(k)]);
            end
            if mod(k, obj.freq_baro) == 0
                obj.update_baro(obs.baro(k));
            end
            if mod(k, obj.freq_gps) == 0 && ~isnan(obs.gps_lat(k)) && ~isnan(obs.gps_lon(k))
                obj.update_gps(obs.gps_lat(k), obs.gps_lon(k), obs.gps_alt(k), k);
            end
        end
        
        function predict(obj, a_meas, w_meas)
            % 莠域ｸｬ繧ｹ繝・ャ繝・
            % 繧ｸ繝｣繧､繝ｭ繝輔ぅ繝ｫ繧ｿ驕ｩ逕ｨ
            if isprop(obj, 'enable_gyro_filter') && ~isempty(obj.enable_gyro_filter) && obj.enable_gyro_filter
                % 繝輔ぅ繝ｫ繧ｿ繧剃ｽｿ逕ｨ縺吶ｋ蝣ｴ蜷医・譌｢蟄倥・繧ｻ繝ｳ繧ｵ繝ｼ繝輔ぅ繝ｫ繧ｿ繧帝←逕ｨ
                % w_expected縺ｫ縺ｯ蜑榊屓縺ｮ繝輔ぅ繝ｫ繧ｿ貂医∩蛟､繧剃ｽｿ逕ｨ・・g縺ｯ髱呎ｭ｢譎ょｰら畑・・                w_expected = obj.sensor_filters.gyro.w_filtered;
                [w_filtered, w_is_outlier, ~] = obj.sensor_filters.gyro.apply(w_meas, w_expected);
                if w_is_outlier
                    % 螟悶ｌ蛟､縺ｮ蝣ｴ蜷医・蜑榊屓縺ｮ繝輔ぅ繝ｫ繧ｿ貂医∩蛟､繧剃ｽｿ逕ｨ
                    w_meas = w_expected;
                else
                    % Yaw 霆ｸ縺ｮ蛟句挨蜃ｦ逅・蠢・ｦ√↓蠢懊§縺ｦ逕溷､縺ｫ謌ｻ縺・
                    if obj.enable_yaw_raw_gyro
                        w_filtered(3) = w_meas(3);
                    end
                    w_meas = w_filtered;
                end
            else
                % 繝輔ぅ繝ｫ繧ｿ繧堤┌蜉ｹ縺ｫ縺吶ｋ蝣ｴ蜷・ 逕溘・隗帝溷ｺｦ繧剃ｽｿ逕ｨ(繝舌う繧｢繧ｹ陬懈ｭ｣縺ｯ integrate_nominal 蜀・〒陦後≧)
                % 縺薙％縺ｧ縺ｯ w_meas 繧偵◎縺ｮ縺ｾ縺ｾ貂｡縺・            end
            
            % 蜉騾溷ｺｦ繝輔ぅ繝ｫ繧ｿ驕ｩ逕ｨ・磯溷ｺｦ謗ｨ螳夂畑・・            % 繝ｦ繝ｼ繧ｶ繝ｼ隕∵悍: 蠑ｷ繧√↓繝輔ぅ繝ｫ繧ｿ繝ｪ繝ｳ繧ｰ繧偵＠縺ｦ縺昴ｌ繧堤ｩ榊・縺吶ｋ
            if ~isempty(obj.accel_filter)
                % 譛溷ｾ・､縺ｨ縺励※迴ｾ蝨ｨ縺ｮ謗ｨ螳壼刈騾溷ｺｦ・磯㍾蜉幃勁蜴ｻ蜑搾ｼ峨ｒ菴ｿ逕ｨ縺励◆縺・′縲・                % 邁｡譏鍋噪縺ｫ蜑榊屓縺ｮ繝輔ぅ繝ｫ繧ｿ蛟､繧剃ｽｿ逕ｨ
                a_expected = obj.accel_filter.a_filtered;
                if norm(a_expected) < 1e-3
                    a_expected = a_meas; % 蛻晄悄蛹・                end
                
                [a_filtered, is_outlier] = obj.accel_filter.filter(a_meas, a_expected);
                
                if is_outlier
                    % 螟悶ｌ蛟､縺ｮ蝣ｴ蜷医・譖ｴ譁ｰ縺励↑縺・ｼ亥燕蝗槭・蛟､繧剃ｽｿ縺・°縲√≠繧九＞縺ｯ菫｡鬆ｼ蠎ｦ繧剃ｸ九￡繧具ｼ・                    % 縺薙％縺ｧ縺ｯ繝輔ぅ繝ｫ繧ｿ蛟､繧偵◎縺ｮ縺ｾ縺ｾ菴ｿ縺・ｼ・ilter繝｡繧ｽ繝・ラ蜀・〒蜑榊屓蛟､繧定ｿ斐☆繧医≧縺ｫ縺ｪ縺｣縺ｦ縺・ｋ・・                end
                
                % 騾溷ｺｦ譖ｴ譁ｰ縺ｫ縺ｯ繝輔ぅ繝ｫ繧ｿ繝ｪ繝ｳ繧ｰ縺輔ｌ縺溷刈騾溷ｺｦ繧剃ｽｿ逕ｨ
                a_for_vel = a_filtered;
            else
                a_for_vel = a_meas;
            end

            % 繝弱Α繝翫Ν迥ｶ諷九・遨榊・
            % --- NoiseEstimator縺九ｉ髢ｾ蛟､繧貞叙蠕・---
            if ~isempty(obj.noiseEstimator)
                % 霆ｸ縺斐→縺ｮ髢ｾ蛟､繧貞叙蠕暦ｼ・ﾏ・ｒ菴ｿ逕ｨ・・                [accel_thr_vec, ~] = obj.noiseEstimator.getThreshold('accel', 2.0);
                [gyro_thr_vec, ~] = obj.noiseEstimator.getThreshold('gyro', 2.0);
              
                accel_thr_vec = max(accel_thr_vec, 0.001);  % 譛菴・0.001 m/s^2
                gyro_thr_vec = max(gyro_thr_vec, obj.gyro_noise_threshold);  % 蛻晄悄謗ｨ螳壼､繧剃ｸ矩剞縺ｫ
            else
                accel_thr_vec = ones(3,1) * 0.1;
                gyro_thr_vec = ones(3,1) * obj.gyro_noise_threshold;
            end

            % 繝弱Α繝翫Ν迥ｶ諷九・遨榊・
            % 騾溷ｺｦ譖ｴ譁ｰ縺ｫ縺ｯ a_for_vel 繧剃ｽｿ逕ｨ縺励∝ｧｿ蜍｢譖ｴ譁ｰ縺ｫ縺ｯ w_meas 繧剃ｽｿ逕ｨ
            % eskf_core_mex 縺ｯ蜊倅ｸ縺ｮ蜉騾溷ｺｦ蜈･蜉帙ｒ蜿励￠蜿悶ｋ縺溘ａ縲・            % 蟋ｿ蜍｢譖ｴ譁ｰ縺ｫ繧・a_for_vel 縺御ｽｿ繧上ｌ繧九％縺ｨ縺ｫ縺ｪ繧九′縲・            % 蟋ｿ蜍｢縺ｮ莠域ｸｬ繧ｹ繝・ャ繝励〒縺ｯ蜉騾溷ｺｦ縺ｯ菴ｿ繧上ｌ縺ｪ縺・ｼ郁ｧ帝溷ｺｦ縺ｮ縺ｿ・峨・縺ｧ蝠城｡後↑縺・・            % (蜉騾溷ｺｦ縺ｯ update_accel 縺ｧ蟋ｿ蜍｢陬懈ｭ｣縺ｫ菴ｿ繧上ｌ繧・
            
            [obj.p, obj.v, obj.q, obj.ba, obj.bg] = eskf_core_mex('integrate_nominal', ...
                obj.p, obj.v, obj.q, obj.ba, obj.bg, a_for_vel, w_meas, obj.dt, obj.g, gyro_thr_vec, accel_thr_vec);

            % 蜈ｱ蛻・淵縺ｮ莠域ｸｬ・・EX蛹厄ｼ・            % 縺薙％縺ｧ繧・a_for_vel 繧剃ｽｿ逕ｨ
            obj.P = eskf_core_mex('predict_covariance', obj.P, obj.q, a_for_vel, obj.ba, w_meas, obj.bg, obj.Q, obj.dt);
            
            % 蜈ｱ蛻・淵陦悟・縺ｮ豁｣蜑・喧
            obj.P = obj.divergence_guard.regularize_covariance(obj.P);
            
            % P陦悟・縺ｮ蟋ｿ蜍｢驛ｨ蛻・ｼ・-9陦鯉ｼ峨↓荳企剞繧帝←逕ｨ
            if isfield(obj.divergence_guard.config, 'max_attitude_variance')
                max_var = obj.divergence_guard.config.max_attitude_variance;
                for i = 7:9
                    if obj.P(i,i) > max_var
                        obj.P(i,i) = max_var;
                    end
                end
            end
            
            % 騾溷ｺｦ繝√ぉ繝・け縺ｨ繧ｯ繝ｪ繝・ヴ繝ｳ繧ｰ
            [obj.v, obj.P, ~] = obj.divergence_guard.check_and_clip_velocity(obj.v, obj.P, 4:6);
        end
        
        function update_accel(obj, a_meas)
            % 蜉騾溷ｺｦ縺ｫ繧医ｋ繧ｫ繝ｫ繝槭Φ繝輔ぅ繝ｫ繧ｿ蟋ｿ蜍｢譖ｴ譁ｰ・・oll/Pitch縺ｮ縺ｿ縲〆aw荳榊庄隕ｳ貂ｬ・・            % 繧ｮ繧ｶ繧ｮ繧ｶ謚大宛: 蠑ｷ蜉帙↑繧ｲ繧､繝ｳ蛻ｶ髯・+ 譎る俣逧・紛蜷域ｧ繝√ぉ繝・け
            
            % 繧ｻ繝ｳ繧ｵ繝ｼ繝輔ぅ繝ｫ繧ｿ驕ｩ逕ｨ
            [a_corrected, is_outlier, ~] = obj.sensor_filters.accel.apply(a_meas, zeros(3,1));
            
            if is_outlier
                return;
            end
            
            % 蛛･蜈ｨ諤ｧ繝√ぉ繝・け
            a_norm = norm(a_corrected);
            if a_norm < 0.1 || abs(a_norm - 9.81) > 3.0
                return;
            end
            
            % 莠域ｸｬ蛟､險育ｮ暦ｼ夂樟蝨ｨ縺ｮ蟋ｿ蜍｢縺九ｉ驥榊鴨縺ｮ繝懊ョ繧｣蠎ｧ讓咏ｳｻ陦ｨ迴ｾ繧定ｨ育ｮ・            Rb = QuaternionLib.to_rotation_matrix(obj.q);
            g_body = Rb' * obj.g;  % 驥榊鴨繝吶け繝医Ν繧偵・繝・ぅ蠎ｧ讓吶∈螟画鋤
            
            % 隕ｳ貂ｬ繝｢繝・Ν・壼刈騾溷ｺｦ險医・ -g・井ｸ雁髄縺搾ｼ峨ｒ貂ｬ螳・            h = -g_body;
            
            % 隕ｳ貂ｬ陦悟・・唏 = 竏Ｉ/竏ばｸ = -[g_body]ﾃ・            H_full = [zeros(3,6), -RotationLib.skew_symmetric(g_body), zeros(3,6)];
            
            % 繧ｸ繝｣繧､繝ｭ繝舌う繧｢繧ｹ陬懈ｭ｣繧定ｿｽ蜉・遺・h/竏Ｃ_g 竕・(竏Ｉ/竏ばｸ)*(竏ばｸ/竏Ｃ_g) = H_theta * (-dt)・・            H_full(:,13:15) = -H_full(:,7:9) * obj.dt;
            
            % x,y謌仙・縺ｮ縺ｿ繧剃ｽｿ逕ｨ・・aw蟷ｲ貂牙屓驕ｿ + 邱壼ｽ｢蛹冶ｪ､蟾ｮ菴取ｸ幢ｼ・            H = H_full(1:2, :);
            z = a_corrected(1:2);
            h_pred = h(1:2);
            
            % 繝弱う繧ｺ蜈ｱ蛻・淵・・rduPilot蠑・ 蜍慕噪R隱ｿ謨ｴ・・            R_est_full = obj.noiseEstimator.getRnoise('accel');
            R_est_2d = diag(R_est_full);
            R_est_2d = R_est_2d(1:2);
            
            % 蜍慕噪R隱ｿ謨ｴ: 驥榊鴨蛛丞ｷｮ縺ｫ蠢懊§縺ｦR繧貞｢玲ｸ・            gravity_deviation = abs(a_norm - 9.81);
            R_scale = 1.0 + (gravity_deviation / 1.0);  % 諢溷ｺｦ繧剃ｸ九￡繧・(2.0 -> 1.0)
            
            % 繝弱う繧ｺ荳矩剞・亥ｹｳ貊大喧蠑ｷ蛹・ 0.05 竊・0.1・・            R_floor = 0.1;  % 貂ｬ螳壹ヮ繧､繧ｺ繧偵ｈ繧贋ｿ晏ｮ育噪縺ｫ隕狗ｩ阪ｂ繧・            R = diag(max(R_est_2d, R_floor) * R_scale);
            
            % --- 繝悶Ο繝・け蛹匁怙驕ｩ蛹・ 髱槭ぞ繝ｭ蛻励・縺ｿ縺ｧ險育ｮ・---
            idx_nz = [7:9, 13:15];  % 蟋ｿ蜍｢縺ｨ繧ｸ繝｣繧､繝ｭ繝舌う繧｢繧ｹ・郁ｨ・蛻暦ｼ・            H_sub = H(:, idx_nz);    % 2x6
            P_sub = obj.P(idx_nz, idx_nz);  % 6x6 蟇ｾ遘ｰ驛ｨ蛻・            P_cross = obj.P(:, idx_nz);     % 15x6 (K險育ｮ礼畑)
            
            % 繧､繝弱・繝ｼ繧ｷ繝ｧ繝ｳ險育ｮ・            y = z - h_pred;
            
            % S = H_sub * P_sub * H_sub' + R (2x2)
            S = H_sub * (P_sub * H_sub') + R;
            
            % S豁｣蜑・喧・・holesky蜑阪↓蟇ｾ遘ｰ蛹悶→繧ｸ繝・ち霑ｽ蜉・・            S = (S + S') / 2;  % 蟇ｾ遘ｰ蛹・            try
                s_rcond = rcond(S);
            catch
                s_rcond = 0;
            end
            if isempty(s_rcond) || s_rcond < 1e-12
                jitter = max(1e-8, abs(trace(S)) * 1e-6);
                S = S + eye(size(S)) * jitter;
            end
            
            R_used = R;
            
            % ArduPilot蠑・蟷ｳ貊大喧謚陦・            
            % 1. 繧､繝弱・繝ｼ繧ｷ繝ｧ繝ｳ蛻ｶ髯・(Innovation Clamping)
            %    螟ｧ縺阪↑繧､繝弱・繝ｼ繧ｷ繝ｧ繝ｳ繧陳ｱ0.3rad (ﾂｱ17蠎ｦ) 縺ｫ蛻ｶ髯・(蠑ｷ蛹・
            max_innovation = 0.1;  % rad (0.5 竊・0.3 縺ｧ繧医ｊ貊代ｉ縺九↓)
            innov_norm = norm(y);
            if innov_norm > max_innovation
                y = y * (max_innovation / innov_norm);  % 豁｣隕丞喧縺励※蛻ｶ髯・            end
            
            % 2. 繝槭ワ繝ｩ繝弱ン繧ｹ霍晞屬險育ｮ・            mahalanobis_dist = sqrt(y' / S * y);
            
            % 3. 5-Sigma蝨ｧ邵ｮ繧ｹ繧ｱ繝ｼ繝ｫ (Compression Scale Factor)
            %    5-sigma莉･荳翫・繧､繝弱・繝ｼ繧ｷ繝ｧ繝ｳ繧偵せ繧ｱ繝ｼ繝ｫ繝繧ｦ繝ｳ
            if mahalanobis_dist > 5.0
                innov_comp_scale = 5.0 / mahalanobis_dist;
                y = y * innov_comp_scale;
                % 豕ｨ: S繧りｪｿ謨ｴ縺吶ｋ蠢・ｦ√′縺ゅｋ縺後∫ｰ｡譏鍋沿縺ｧ縺ｯ逵∫払
            end
            
            % 4. 螟悶ｌ蛟､蛻､螳夲ｼ育ｷｩ蜥・ 3.0 竊・5.0・・            if mahalanobis_dist > 5.0
                return;  % 5-sigma莉･荳翫・譽・唆
            end
            
            % --- 繧ｫ繝ｫ繝槭Φ繧ｲ繧､繝ｳ險育ｮ暦ｼ医ヶ繝ｭ繝・け蛹・+ Cholesky螳牙ｮ壼喧・・---
            % K = P * H' / S = P_cross * H_sub' * inv(S)
            % Cholesky蛻・ｧ｣縺ｧ螳牙ｮ壹↓隗｣縺・ S = U'*U => K = P_cross * H_sub' / (U'*U)
            try
                U = chol(S);  % 荳贋ｸ芽ｧ・                tmp = P_cross * H_sub';  % 15x2
                % solve tmp / S via: tmp = K * S => K = tmp / S
                % tmp' = S' * K' = S * K' (S縺ｯ蟇ｾ遘ｰ)
                % U' * U * K' = tmp' => K' = U \ (U' \ tmp')
                K = (U \ (U' \ tmp'))';  % 15x2
            catch
                % 繝輔か繝ｼ繝ｫ繝舌ャ繧ｯ: 逶ｴ謗･騾・｡悟・
                try
                    K = P_cross * (H_sub' / S);  % 15x2
                catch
                    return;
                end
            end
            
            K = obj.divergence_guard.clamp_gain(K);
            
            % 蟋ｿ蜍｢繧ｲ繧､繝ｳ蛻ｶ髯撰ｼ亥ｹｳ貊大喧蠑ｷ蛹・ 0.04 竊・0.02・・            % 繧医ｊ菫晏ｮ育噪縺ｪ繧ｲ繧､繝ｳ縺ｧ貊代ｉ縺九↑蠢懃ｭ斐ｒ螳溽樟
            max_attitude_gain = 0.02;  % 2%莉･荳具ｼ医う繝弱・繝ｼ繧ｷ繝ｧ繝ｳ蛻ｶ髯仙ｼｷ蛹悶→菴ｵ逕ｨ・・            if size(K,1) >= 9
                K(7:9,:) = max(min(K(7:9,:), max_attitude_gain), -max_attitude_gain);
            end
            
            % 迥ｶ諷倶ｿｮ豁｣驥剰ｨ育ｮ・            dx = K * y;
            if numel(dx) < 15
                dx_full = zeros(15,1);
                dx_full(1:numel(dx)) = dx(:);
                dx = dx_full;
            end
            
            % 譎る俣逧・紛蜷域ｧ繝√ぉ繝・け: dx 縺檎焚蟶ｸ縺ｫ螟ｧ縺阪＞蝣ｴ蜷医・繧ｹ繧ｱ繝ｼ繝ｫ繝繧ｦ繝ｳ
            % 蟷ｳ貊大喧蠑ｷ蛹悶・縺溘ａ髢ｾ蛟､繧剃ｸ九￡繧・            dx_attitude_norm = norm(dx(7:9));
            if dx_attitude_norm > deg2rad(0.5)  % 0.5蠎ｦ莉･荳翫・螟牙喧縺ｯ謚大宛 (1.0 -> 0.5)
                scale_down = deg2rad(0.5) / dx_attitude_norm;
                dx(7:9) = dx(7:9) * scale_down;
            end
            
            % 蟋ｿ蜍｢譖ｴ譁ｰ・亥ｾｮ蟆剰ｧ定ｿ台ｼｼ縲〆aw荳榊庄隕ｳ貂ｬ・・            dtheta = dx(7:9);
            dtheta(3) = 0;  % Yaw蠑ｷ蛻ｶ繧ｼ繝ｭ
            
            dq = QuaternionLib.small_angle_quat(dtheta);
            obj.q = QuaternionLib.multiply(obj.q, dq);
            obj.q = QuaternionLib.normalize(obj.q);
            
            % 蜈ｱ蛻・淵譖ｴ譁ｰ・医ヶ繝ｭ繝・け譛驕ｩ蛹也沿: 隕ｳ貂ｬ縺ｫ髢｢騾｣縺吶ｋ驛ｨ蛻・・縺ｿ譖ｴ譁ｰ・・            % accel 縺ｯ蟋ｿ蜍｢(7:9)縺ｨ繧ｸ繝｣繧､繝ｭ繝舌う繧｢繧ｹ(13:15)縺ｮ縺ｿ縺ｫ蠖ｱ髻ｿ
            idx_obs = [7:9, 13:15];
            
            % 蟆上ヶ繝ｭ繝・け縺ｧJoseph譖ｴ譁ｰ
            I_KH_block = eye(length(idx_obs)) - K(idx_obs,:) * H(:,idx_obs);
            P_block = obj.P(idx_obs, idx_obs);
            P_block_new = I_KH_block * P_block * I_KH_block' + K(idx_obs,:) * R_used * K(idx_obs,:)';
            
            % 譖ｴ譁ｰ縺輔ｌ縺溘ヶ繝ｭ繝・け繧呈綾縺・            obj.P(idx_obs, idx_obs) = P_block_new;
            
            % 繧ｯ繝ｭ繧ｹ鬆・峩譁ｰ: P(:, idx_obs) 縺ｮ蜈ｨ陦・            for i = 1:15
                if ~ismember(i, idx_obs)
                    obj.P(i, idx_obs) = obj.P(i, idx_obs) - K(i,:) * (H(:,idx_obs) * obj.P(idx_obs, idx_obs));
                    obj.P(idx_obs, i) = obj.P(i, idx_obs)';
                end
            end
            
            % P蟇ｾ遘ｰ蛹・            obj.P = (obj.P + obj.P') / 2;
            
            % 繝弱う繧ｺ謗ｨ螳壽峩譁ｰ・・隕∫ｴ縺ｫ繝代ョ繧｣繝ｳ繧ｰ・・            y_full = zeros(3,1);
            y_full(1:2) = y;
            H_full_for_estimator = [H; zeros(1,15)];
            obj.noiseEstimator.estimate('accel', y_full, H_full_for_estimator, obj.P);
        end


        function update_mag(obj, m_meas)
            % 逎∵ｰ苓ｨ医↓繧医ｋ蟋ｿ蜍｢譖ｴ譁ｰ
            
            % 繧ｻ繝ｳ繧ｵ繝ｼ繝輔ぅ繝ｫ繧ｿ驕ｩ逕ｨ
            [m_filtered, is_outlier, ~] = obj.sensor_filters.mag.apply(m_meas);
            
            if is_outlier
                return;  % 螟悶ｌ蛟､縺ｮ蝣ｴ蜷医・譖ｴ譁ｰ繧偵せ繧ｭ繝・・
            end
            
            m_world = [0; 50; 0];
            Rb = QuaternionLib.to_rotation_matrix(obj.q);
            h_mag = Rb' * m_world;
            
            % 逎∵ｰ苓ｨ医・豁｣隕丞喧縺輔ｌ縺溘・繧ｯ繝医Ν縺ｨ縺励※謇ｱ縺・ｼ・ensorFilter縺ｨ謨ｴ蜷茨ｼ・            h_mag_norm = norm(h_mag);
            if h_mag_norm > 1e-6
                h_mag = h_mag / h_mag_norm;
            end
            
            z = m_filtered;
            h = h_mag;
            H = [zeros(3,6), RotationLib.skew_symmetric(h), zeros(3,6)];
            
            % 繧ｸ繝｣繧､繝ｭ繝舌う繧｢繧ｹ陬懈ｭ｣繧定ｿｽ蜉・育｣∵ｰ励〒Yaw繝舌う繧｢繧ｹ繧呈耳螳夲ｼ・            H(:,13:15) = -H(:,7:9) * obj.dt;
            
            % 迴ｾ蝨ｨ縺ｮ繝弱う繧ｺ謗ｨ螳壼､繧剃ｽｿ逕ｨ
            R_est = obj.noiseEstimator.getRnoise('mag');
            
            % --- 繝悶Ο繝・け蛹匁怙驕ｩ蛹・ 髱槭ぞ繝ｭ蛻励・縺ｿ縺ｧ險育ｮ・---
            idx_nz = [7:9, 13:15];  % 蟋ｿ蜍｢縺ｨ繧ｸ繝｣繧､繝ｭ繝舌う繧｢繧ｹ・郁ｨ・蛻暦ｼ・            H_sub = H(:, idx_nz);    % 3x6
            P_sub = obj.P(idx_nz, idx_nz);  % 6x6 蟇ｾ遘ｰ驛ｨ蛻・            P_cross = obj.P(:, idx_nz);     % 15x6 (K險育ｮ礼畑)
            
            % 繧､繝弱・繝ｼ繧ｷ繝ｧ繝ｳ險育ｮ・            y = z - h;
            
            % S = H_sub * P_sub * H_sub' + R_est (3x3)
            S = H_sub * (P_sub * H_sub') + R_est;
            
            % S蟇ｾ遘ｰ蛹悶→Cholesky螳牙ｮ壼喧
            S = (S + S') / 2;
            try
                s_rcond = rcond(S);
            catch
                s_rcond = 0;
            end
            if isempty(s_rcond) || s_rcond < 1e-12
                jitter = max(1e-8, abs(trace(S)) * 1e-6);
                S = S + eye(size(S)) * jitter;
            end
            
            R_used = R_est;
            
            % 繧ｫ繝ｫ繝槭Φ繧ｲ繧､繝ｳ險育ｮ暦ｼ・holesky螳牙ｮ壼喧・・            try
                U = chol(S);  % 荳贋ｸ芽ｧ・                tmp = P_cross * H_sub';  % 15x3
                K = (U \ (U' \ tmp'))';  % 15x3
            catch
                % 繝輔か繝ｼ繝ｫ繝舌ャ繧ｯ
                try
                    K = P_cross * (H_sub' / S);
                catch
                    return;
                end
            end
            
            % 繝弱う繧ｺ謗ｨ螳壹ｒ譖ｴ譁ｰ
            obj.noiseEstimator.estimate('mag', y, H, obj.P);
            
            % 繧ｫ繝ｫ繝槭Φ繧ｲ繧､繝ｳ繧偵け繝ｩ繝ｳ繝・            K = obj.divergence_guard.clamp_gain(K);
            
            % MAG譖ｴ譁ｰ蟆ら畑縺ｮ繧ｲ繧､繝ｳ蛻ｶ髯撰ｼ亥ｧｿ蜍｢驛ｨ蛻・・縺ｿ・・            if isfield(obj.divergence_guard.config, 'max_mag_gain_element')
                max_gain = obj.divergence_guard.config.max_mag_gain_element;
                % K(7:9,:)縺ｮ蜷・ｦ∫ｴ繧貞宛髯撰ｼ亥ｧｿ蜍｢驛ｨ蛻・ｼ・                if size(K,1) >= 9
                    K(7:9,:) = max(min(K(7:9,:), max_gain), -max_gain);
                end
            end
            
            dx = K * y;
            
            % dx縺ｮ繧ｵ繧､繧ｺ遒ｺ隱阪→繝吶け繝医Ν蛹・            if numel(dx) < 9
                % dx 縺悟香蛻・↑隕∫ｴ繧呈戟縺｣縺ｦ縺・↑縺・ｴ蜷医√ぞ繝ｭ繝代ョ繧｣繝ｳ繧ｰ
                dx_full = zeros(15, 1);
                dx_full(1:numel(dx)) = dx(:);
                dx = dx_full;
            end

            % 逎∵ｰ苓ｨ医・3霆ｸ縺ｮ蟋ｿ蜍｢繧定ｦｳ貂ｬ縺吶ｋ縺ｮ縺ｧ縲∝・蟋ｿ蜍｢隱､蟾ｮdx(7:9)繧剃ｽｿ逕ｨ
            dtheta = dx(7:9);
            
            dq = QuaternionLib.small_angle_quat(dtheta);
            obj.q = QuaternionLib.multiply(obj.q, dq);
            obj.q = QuaternionLib.normalize(obj.q);

            % 蜈ｱ蛻・淵譖ｴ譁ｰ・医ヶ繝ｭ繝・け譛驕ｩ蛹也沿: 隕ｳ貂ｬ縺ｫ髢｢騾｣縺吶ｋ驛ｨ蛻・・縺ｿ譖ｴ譁ｰ・・            % mag 縺ｯ蟋ｿ蜍｢(7:9)縺ｨ繧ｸ繝｣繧､繝ｭ繝舌う繧｢繧ｹ(13:15)縺ｮ縺ｿ縺ｫ蠖ｱ髻ｿ
            idx_obs = [7:9, 13:15];
            
            I_KH_block = eye(length(idx_obs)) - K(idx_obs,:) * H(:,idx_obs);
            P_block = obj.P(idx_obs, idx_obs);
            P_block_new = I_KH_block * P_block * I_KH_block' + K(idx_obs,:) * R_used * K(idx_obs,:)';
            obj.P(idx_obs, idx_obs) = P_block_new;
            
            % 繧ｯ繝ｭ繧ｹ鬆・峩譁ｰ
            for i = 1:15
                if ~ismember(i, idx_obs)
                    obj.P(i, idx_obs) = obj.P(i, idx_obs) - K(i,:) * (H(:,idx_obs) * obj.P(idx_obs, idx_obs));
                    obj.P(idx_obs, i) = obj.P(i, idx_obs)';
                end
            end
            
            obj.P = (obj.P + obj.P') / 2;
        end
        
    function update_gps(obj, lat, lon, alt, k)
            % GPS菴咲ｽｮ譖ｴ譁ｰ・医ヶ繝ｭ繝・け蛹・EKF 繝吶・繧ｹ・・            
            lat0 = obj.gps_origin(1);
            lon0 = obj.gps_origin(2);
            alt0 = obj.gps_origin(3);
            
            % 蠎ｧ讓吝､画鋤
            y_m = (lat - lat0) / (9.0e-6);
            x_m = (lon - lon0) / (9.0e-6 / cosd(lat0));
            z_m = alt - alt0;
            
            z_gps = [x_m; y_m; z_m];
            
            % 繧ｻ繝ｳ繧ｵ繝ｼ繝輔ぅ繝ｫ繧ｿ菴ｿ逕ｨ・亥・譛溷喧蝠城｡後・縺溘ａ荳譎ら噪縺ｫ繧ｹ繧ｭ繝・・・・            % [z_gps_filtered, is_outlier, ~] = obj.sensor_filters.gps.apply(z_gps);
            % if is_outlier
            %     return;
            % end
            z_gps_filtered = z_gps;  % 繝輔ぅ繝ｫ繧ｿ繧偵ヰ繧､繝代せ

            R = obj.noiseEstimator.getRnoise('gps');
            
            % GPS譖ｴ譁ｰ蜑阪↓P繧呈ｭ｣蜑・喧
            obj.P = obj.divergence_guard.regularize_for_ukf(obj.P);
            
            % --- UKF 譖ｴ譁ｰ: 菴咲ｽｮ繝ｻ騾溷ｺｦ縺ｮ6谺｡蜈・し繝悶す繧ｹ繝・Β縺ｫ驕ｩ逕ｨ ---
            % 迥ｶ諷九し繝悶そ繝・ヨ: x_sub = [p; v] (6x1)
            idx_pv = 1:6;
            x_sub = [obj.p; obj.v];
            P_sub = obj.P(idx_pv, idx_pv);  % 6x6
            
            % 隕ｳ貂ｬ髢｢謨ｰ: 菴咲ｽｮ縺ｮ縺ｿ繧定ｿ斐☆ (3x1)
            h_func = @(x_pv) x_pv(1:3);
            
            % UKF 譖ｴ譁ｰ繧貞ｮ溯｡・            ukf_success = false;
            try
                % alpha=1e-3, beta=2, kappa=0 (讓呎ｺ冶ｨｭ螳・
                [x_sub_upd, P_sub_upd, K_ukf, S, y_innov] = ukf_update(x_sub, P_sub, z_gps_filtered, h_func, R, 1e-3, 2, 0);
                ukf_success = true;
            catch ME
                % UKF螟ｱ謨玲凾縺ｯEKF縺ｫ繝輔か繝ｼ繝ｫ繝舌ャ繧ｯ
                warning('ESKF:update_gps:UKF_Failed', 'UKF update failed (%s), using EKF fallback', ME.message);
                ukf_success = false;
            end
            
            % 隕ｳ貂ｬ陦悟・・井ｽ咲ｽｮ縺ｮ縺ｿ・・ H = [I3, 0_{3x12}]
            H_gps = [eye(3), zeros(3, 12)];
            
            if ukf_success
                % UKF謌仙粥譎・ S縺ｨy_innov縺ｯ譌｢縺ｫ險育ｮ玲ｸ医∩
                % 蜈ｨ迥ｶ諷九∈縺ｮ繧ｫ繝ｫ繝槭Φ繧ｲ繧､繝ｳ繧定ｨ育ｮ・                S = (S + S') / 2;
                if rcond(S) < 1e-12
                    S = S + eye(3) * 1e-8;
                end
                
                try
                    U = chol(S);
                    tmp = obj.P(:, 1:3);
                    K = (U \ (U' \ tmp'))';  % 15x3
                catch
                    K = obj.P(:, 1:3) / S;
                end
                
                K = obj.divergence_guard.clamp_gain(K);
                dx = K * y_innov;
            else
                % EKF繝輔か繝ｼ繝ｫ繝舌ャ繧ｯ
                idx_pos = 1:3;
                y_innov = z_gps_filtered - obj.p;
                P_pos = obj.P(idx_pos, idx_pos);
                S = P_pos + R;
                S = (S + S') / 2;
                
                try
                    U = chol(S);
                    tmp = obj.P(:, idx_pos);
                    K = (U \ (U' \ tmp'))';
                catch
                    K = obj.P(:, idx_pos) / S;
                end
                
                K = obj.divergence_guard.clamp_gain(K);
                dx = K * y_innov;
            end
            
            % OutlierGuard 繝√ぉ繝・け・・KF謌仙粥譎ゅ・菫｡鬆ｼ縺励※繧ｹ繧ｭ繝・・・・            if ukf_success
                % UKF縺梧・蜉溘＠縺溷ｴ蜷医・譌｢縺ｫsigma points縺ｧ繝ｭ繝舌せ繝医↓蜃ｦ逅・＆繧後※縺・ｋ縺溘ａ縲・                % OutlierGuard繧偵ヰ繧､繝代せ縺励※逶ｴ謗･譖ｴ譁ｰ繧帝←逕ｨ
                should_update = true;
                y_used = y_innov;
                dx_used = [];
                
                % UKF縺ｮ蝣ｴ蜷医〒繧ゅ√う繝弱・繝ｼ繧ｷ繝ｧ繝ｳ縺悟､ｧ縺阪☆縺弱ｋ蝣ｴ蜷医・譽・唆縺吶ｋ (螟悶ｌ蛟､蟇ｾ遲門ｼｷ蛹・
                % 菴咲ｽｮ縺ｮ繧､繝弱・繝ｼ繧ｷ繝ｧ繝ｳ縺・5m 莉･荳翫↑繧画｣・唆
                if norm(y_innov(1:3)) > 5.0
                    should_update = false;
                end
            else
                % EKF繝輔か繝ｼ繝ｫ繝舌ャ繧ｯ譎ゅ・縺ｿOutlierGuard繧剃ｽｿ逕ｨ
                H_gps = [eye(3), zeros(3, 12)];
                R_updated = obj.noiseEstimator.getRnoise('gps');
                ctx = struct();
                ctx.k = k;
                ctx.z = z_gps;
                ctx.h = obj.p;
                ctx.y = y_innov;
                ctx.P_diag = diag(obj.P);
                ctx.R_diag = diag(R);
                ctx.gps = ctx;
                [should_update, y_used, ~, dx_used, ~] = OutlierGuard.checkAndApply('gps', z_gps, obj.p, H_gps, obj.P, R_updated, [], dx, obj.divergence_guard, obj.noiseEstimator, ctx);
            end
            
            if ~should_update
                return;
            end

            % 繝弱う繧ｺ謗ｨ螳壹ｒ譖ｴ譁ｰ
            obj.noiseEstimator.estimate('gps', y_used, H_gps, obj.P);

            % 繧ｯ繝ｪ繝・・貂医∩ dx 縺後≠繧後・菴ｿ縺・            if ~isempty(dx_used)
                dx = dx_used;
            end

            % 迥ｶ諷区峩譁ｰ
            if ukf_success
                % UKF謌仙粥譎・ 譖ｴ譁ｰ縺輔ｌ縺溽憾諷九→縺ｮ蟾ｮ蛻・ｒ驕ｩ逕ｨ
                % x_sub_upd縺ｯ邨ｶ蟇ｾ蛟､縺ｪ縺ｮ縺ｧ縲∽ｺ句燕謗ｨ螳噎_sub縺ｨ縺ｮ蟾ｮ蛻・ｒ縺ｨ繧・                dx_ukf = x_sub_upd - x_sub;
                obj.p = obj.p + dx_ukf(1:3);
                obj.v = obj.v + dx_ukf(4:6);
            else
                % EKF繝輔か繝ｼ繝ｫ繝舌ャ繧ｯ譎・ dx陬懈ｭ｣繧帝←逕ｨ
                obj.p = obj.p + dx(1:3);
                obj.v = obj.v + dx(4:6);
            end
            
            % 蜈ｱ蛻・淵譖ｴ譁ｰ
            if ukf_success
                % UKF謌仙粥譎・ 繧ｵ繝悶す繧ｹ繝・Β縺ｮ譖ｴ譁ｰ貂医∩蜈ｱ蛻・淵繧剃ｽｿ逕ｨ
                idx_obs = 1:6;
                obj.P(idx_obs, idx_obs) = P_sub_upd;
                
                % 繧ｯ繝ｭ繧ｹ鬆・峩譁ｰ (蟋ｿ蜍｢繝ｻ繝舌う繧｢繧ｹ縺ｨ縺ｮ逶ｸ髢｢繧呈峩譁ｰ)
                for i = 7:15
                    obj.P(i, idx_obs) = obj.P(i, idx_obs) - K(i,:) * (H_gps(:,idx_obs) * obj.P(idx_obs, idx_obs));
                    obj.P(idx_obs, i) = obj.P(i, idx_obs)';
                end
            else
                % EKF繝輔か繝ｼ繝ｫ繝舌ャ繧ｯ譎・ Joseph蠖｢蠑上〒譖ｴ譁ｰ
                idx_obs = 1:6;
                I_KH_block = eye(length(idx_obs)) - K(idx_obs,:) * H_gps(:,idx_obs);
                P_block = obj.P(idx_obs, idx_obs);
                P_block_new = I_KH_block * P_block * I_KH_block' + K(idx_obs,:) * R_updated * K(idx_obs,:)';
                obj.P(idx_obs, idx_obs) = P_block_new;
                
                % 繧ｯ繝ｭ繧ｹ鬆・峩譁ｰ
                for i = 7:15
                    obj.P(i, idx_obs) = obj.P(i, idx_obs) - K(i,:) * (H_gps(:,idx_obs) * obj.P(idx_obs, idx_obs));
                    obj.P(idx_obs, i) = obj.P(i, idx_obs)';
                end
            end
            
            obj.P = (obj.P + obj.P') / 2;
            
            % 騾溷ｺｦ繝√ぉ繝・け縺ｨ繧ｯ繝ｪ繝・ヴ繝ｳ繧ｰ
            [obj.v, obj.P, ~] = obj.divergence_guard.check_and_clip_velocity(obj.v, obj.P, 4:6);
        end
        
        function update_baro(obj, pressure)
            % 豌怜悸險医↓繧医ｋ鬮伜ｺｦ譖ｴ譁ｰ
            
            % 繧ｻ繝ｳ繧ｵ繝ｼ繝輔ぅ繝ｫ繧ｿ驕ｩ逕ｨ
            [alt_baro, is_outlier, ~] = obj.sensor_filters.baro.apply(pressure);
            
            if is_outlier
                return;  % 螟悶ｌ蛟､縺ｮ蝣ｴ蜷医・譖ｴ譁ｰ繧偵せ繧ｭ繝・・
            end
            
            H = [0,0,1, zeros(1,12)];
            z = alt_baro;
            h = obj.p(3);
            
            % 迴ｾ蝨ｨ縺ｮ繝弱う繧ｺ謗ｨ螳壼､繧剃ｽｿ逕ｨ
            R_est = obj.noiseEstimator.getRnoise('baro');
            
            [y, S, R_used] = kalman_filter_core('compute_innovation_and_S', z, h, H, obj.P, R_est, struct());
            
            % 繝輔ぅ繝ｫ繧ｿ繝ｪ繝ｳ繧ｰ・亥､悶ｌ蛟､蛻､螳夲ｼ・            [y_filtered, should_update] = SensorFilter.filterInnovation(y, R_used);
            if ~should_update
                return;
            end
            
            % --- 螟悶ｌ蛟､縺ｧ縺ｪ縺・ｴ蜷医・縺ｿ繝弱う繧ｺ謗ｨ螳壹ｒ譖ｴ譁ｰ ---
            obj.noiseEstimator.estimate('baro', y_filtered, H, obj.P);
            
            K = kalman_filter_core('compute_kalman_gain', obj.P, H, S);
            K = obj.divergence_guard.clamp_gain(K);
            
            % dx險育ｮ・ K縺ｯ15x1縲【_filtered縺ｯ繧ｹ繧ｫ繝ｩ繝ｼ
            dx = K * y_filtered;
            
            % dx縺ｮ繧ｵ繧､繧ｺ遒ｺ隱阪→繝吶け繝医Ν蛹・            if numel(dx) == 1
                % 繧ｹ繧ｫ繝ｩ繝ｼ縺ｮ蝣ｴ蜷医∽ｽ咲ｽｮ縺ｮ鬮伜ｺｦ縺ｮ縺ｿ譖ｴ譁ｰ
                dz = dx;
                dx = zeros(15, 1);
                dx(3) = dz;
            end
            
            % 鬮伜ｺｦ譖ｴ譁ｰ・磯明蛟､繝√ぉ繝・け莉倥″・・            if abs(dx(3)) >= 0.1
                obj.p(3) = obj.p(3) + dx(3);
            end
            
            % 蜈ｱ蛻・淵譖ｴ譁ｰ
            x_pred = zeros(15,1);
            x_pred(1:3) = obj.p;
            [~, obj.P] = kalman_filter_core('update_state_covariance', x_pred, obj.P, K, H, y_filtered, R_used);
        end
        
        function euler = get_euler(obj)
            % 繧ｪ繧､繝ｩ繝ｼ隗貞叙蠕・            euler = QuaternionLib.to_euler(obj.q);
        end
    end
end

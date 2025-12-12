function varargout = sensor_updates(obj, method, varargin)
    % センサー更新統合メソッド
    % 使用例:
    %   obj.sensor_updates('accel', a_meas)
    %   obj.sensor_updates('mag', m_meas)
    %   obj.sensor_updates('gps', lat, lon, alt, k)
    %   obj.sensor_updates('baro', pressure)
    
    switch method
        case 'accel'
            update_accel_impl(obj, varargin{1});
        case 'mag'
            update_mag_impl(obj, varargin{1});
        case 'gps'
            update_gps_impl(obj, varargin{1}, varargin{2}, varargin{3}, varargin{4});
        case 'baro'
            update_baro_impl(obj, varargin{1});
        otherwise
            error('Unknown sensor update method: %s', method);
    end
end

function update_accel_impl(obj, a_meas)
    % 加速度更新
    if ~isempty(obj.w_body) && norm(obj.w_body) > 1.5; return; end
    [a_corrected, is_outlier, ~] = obj.sensor_filters.accel.apply(a_meas, zeros(3,1));
    if any(isnan(a_corrected)) || is_outlier; return; end
    a_norm = norm(a_corrected);
    if a_norm < 0.1 || abs(a_norm - 9.81) > 3.0; return; end
    call_cpp_update_impl(obj, 'accel', a_corrected);
end

function update_mag_impl(obj, m_meas)
    % 磁気計更新
    [m_filtered, is_outlier, ~] = obj.sensor_filters.mag.apply(m_meas);
    if any(isnan(m_filtered)) || is_outlier; return; end
    call_cpp_update_impl(obj, 'mag', m_filtered);
end

function update_gps_impl(obj, lat, lon, alt, k)
    % GPS更新
    lat0 = obj.gps_origin(1);
    lon0 = obj.gps_origin(2);
    alt0 = obj.gps_origin(3);
    y_m = (lat - lat0) / (9.0e-6);
    x_m = (lon - lon0) / (9.0e-6 / cosd(lat0));
    z_m = alt - alt0;
    z_gps = [y_m; x_m; -z_m];
    call_cpp_update_impl(obj, 'gps', z_gps);
end

function update_baro_impl(obj, pressure)
    % 気圧計更新
    [alt_baro, is_outlier, ~] = obj.sensor_filters.baro.apply(pressure);
    if any(isnan(alt_baro)) || is_outlier; return; end
    
    weight_factor = 1.0 / obj.baro_weight;
    obj.P(3,3) = obj.P(3,3) * weight_factor;
    call_cpp_update_impl(obj, 'baro', alt_baro);
    obj.P(3,3) = obj.P(3,3) / weight_factor;
end


function varargout = reset(obj, method, varargin)
    % リセット関連メソッド統合
    % 使用例:
    %   obj.reset('check', obs, k)
    %   obj.reset('filter', obs, k)
    
    switch method
        case 'check'
            check_and_reset_impl(obj, varargin{1}, varargin{2});
        case 'filter'
            reset_filter_impl(obj, varargin{1}, varargin{2});
        otherwise
            error('Unknown reset method: %s', method);
    end
end

function check_and_reset_impl(obj, obs, k)
    % 発散チェックとリセット
    if isempty(k); return; end
    
    reset_needed = false;
    reason = '';
    
    if any(isnan(obj.p)) || any(isnan(obj.v)) || any(isnan(obj.q))
        reset_needed = true;
        reason = 'NaN in state';
    elseif any(isinf(obj.p)) || any(isinf(obj.v))
        reset_needed = true;
        reason = 'Inf in state';
    end
    
    if reset_needed
        fprintf('Step %d: Reset - %s\n', k, reason);
        if ~isempty(obs)
            reset_filter_impl(obj, obs, k);
        else
            obj.last_reset_step = k;
            obj.P(1:3, 1:3) = eye(3) * 20.0;
            obj.P(4:6, 4:6) = eye(3) * 2.0;
            obj.P(7:9, 7:9) = eye(3) * (deg2rad(30))^2;
            obj.v = zeros(3,1);
        end
    end
end

function reset_filter_impl(obj, obs, k)
    % フィルタをリセット
    obj.last_reset_step = k;
    obj.P = eye(15) * 0.01;
    obj.P(1:3, 1:3) = eye(3) * 20.0;
    obj.P(4:6, 4:6) = eye(3) * 2.0;
    obj.P(7:9, 7:9) = eye(3) * (deg2rad(30))^2;
    obj.v = zeros(3,1);
    
    if isfield(obs, 'lat') && k <= length(obs.lat) && ~isnan(obs.lat(k))
        lat0 = obj.gps_origin(1);
        lon0 = obj.gps_origin(2);
        alt0 = obj.gps_origin(3);
        y_m = (obs.lat(k) - lat0) / (9.0e-6);
        x_m = (obs.lon(k) - lon0) / (9.0e-6 / cosd(lat0));
        z_m = obs.alt(k) - alt0;
        obj.p = [x_m; y_m; z_m];
    end
    
    if any(isnan(obj.q)); obj.q = [1;0;0;0]; end
    obj.ba = zeros(3,1);
    obj.bg = zeros(3,1);
end

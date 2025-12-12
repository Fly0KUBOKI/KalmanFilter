function varargout = zupt(obj, method, varargin)
    % ZUPT関連メソッド統合
    % 使用例:
    %   is_stat = obj.zupt('check', a_meas, w_meas)
    %   obj.zupt('update')
    
    switch method
        case 'check'
            varargout{1} = check_stationary_impl(obj, varargin{1}, varargin{2});
        case 'update'
            update_zupt_impl(obj);
        otherwise
            error('Unknown ZUPT method: %s', method);
    end
end

function is_stat = check_stationary_impl(obj, a_meas, w_meas)
    % 静止判定: 加速度と角速度の大きさをチェック
    a_norm = norm(a_meas);
    gravity_deviation = abs(a_norm - 9.81);
    w_norm = norm(w_meas);
    
    if gravity_deviation < obj.zupt_threshold_accel && w_norm < obj.zupt_threshold_gyro
        obj.zupt_counter = obj.zupt_counter + 1;
    else
        obj.zupt_counter = 0;
    end
    
    obj.is_stationary = (obj.zupt_counter >= obj.zupt_min_duration);
    is_stat = obj.is_stationary;
end

function update_zupt_impl(obj)
    % ZUPT (Zero Velocity Update): 速度をゼロに補正
    if ~obj.is_stationary; return; end
    try
        call_cpp_update_impl(obj, 'zupt', []);
    catch ME
        warning('ESKF:zupt:Failed', 'C++ ZUPT update failed: %s', ME.message);
    end
end

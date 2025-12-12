function varargout = utils(obj, method, varargin)
    % ユーティリティメソッド統合
    % 使用例:
    %   euler = obj.utils('get_euler')
    %   data = obj.utils('get_field', obs, field_names, idx, num_cols)
    
    switch method
        case 'get_euler'
            varargout{1} = get_euler_impl(obj);
        case 'get_field'
            varargout{1} = get_field_impl(obj, varargin{1}, varargin{2}, varargin{3}, varargin{4});
        otherwise
            error('Unknown utils method: %s', method);
    end
end

function euler = get_euler_impl(obj)
    % オイラー角取得 (degrees)
    euler = QuaternionLib.to_euler(obj.q);
end

function data = get_field_impl(obj, obs, field_names, idx, num_cols)
    % フィールド名の候補リストから最初に見つかったものを取得
    for i = 1:length(field_names)
        if isfield(obs, field_names{i})
            if num_cols == 1
                data = obs.(field_names{i})(idx);
            elseif num_cols == 3
                field_base = field_names{i};
                if endsWith(field_base, '_x')
                    field_base = field_base(1:end-2);
                end
                if startsWith(field_base, 'gyro') || startsWith(field_base, 'mag') || ...
                   startsWith(field_base, 'accel') || startsWith(field_base, 'gps')
                    data = [obs.([field_base '_x'])(idx), ...
                            obs.([field_base '_y'])(idx), ...
                            obs.([field_base '_z'])(idx)];
                else
                    % ax, wx, mx形式
                    suffix = field_base(1);
                    data = [obs.([suffix 'x'])(idx), ...
                            obs.([suffix 'y'])(idx), ...
                            obs.([suffix 'z'])(idx)];
                end
            end
            return;
        end
    end
    error('ESKF:utils:get_field', 'None of the fields found: %s', strjoin(field_names, ', '));
end

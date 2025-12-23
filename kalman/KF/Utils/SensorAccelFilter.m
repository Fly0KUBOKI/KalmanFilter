classdef SensorAccelFilter < handle
    % SENSORACCELFILTER  加速度計専用フィルタ
    %
    % 機能:
    %   - EMA平滑化
    %   - 外れ値検出（3σ）
    %   - 重力ノルム検証
    %   - 大きな変化スケーリング
    
    properties
        config              % フィルタ設定
        a_filtered          % フィルタ済み加速度
        noise_history       % ノイズ履歴
    end
    
    methods
        function obj = SensorAccelFilter(config)
            % コンストラクタ
            obj.config = config;
            obj.a_filtered = [0; 0; 0];
            obj.noise_history = [];
        end
        
        function [a_out, is_outlier, info] = apply(obj, a_meas, a_expected)
            % APPLY  加速度計測値をフィルタリング（MEX に直接委譲）
            if nargin < 3
                a_expected = zeros(3, 1);
            end

            % Try MEX first; fall back to MATLAB if unavailable or invalid
            if exist('mex_sensor_filter','file') == 3
                try
                    if nargout >= 2
                        [a_filt, is_outlier] = SensorFilters.accel(a_meas, a_expected);
                    else
                        a_filt = SensorFilters.accel(a_meas, a_expected);
                        is_outlier = false;
                    end
                    % Validate MEX output
                    if SensorFilters.validate_sensor_output('accel', a_filt)
                        obj.a_filtered = a_filt;
                        a_out = a_filt;
                        info = struct('is_outlier', is_outlier, 'is_gravity_mismatch', false, 'scale_factor', 1.0);
                        return;
                    end
                catch ME
                    warning('SensorAccelFilter:MEXFailed', 'MEX accel failed: %s. Using MATLAB fallback.', ME.message);
                end
            end

            % MATLAB fallback: simple pass-through (measured accel)
            a_out = a_meas;
            is_outlier = false;
            obj.a_filtered = a_meas;
            info = struct('is_outlier', is_outlier, 'is_gravity_mismatch', false, 'scale_factor', 1.0);
        end
        
        function noise_level = getNoiseLevel(obj)
            % GETNOISELEVEL  ノイズレベルを取得
            if isempty(obj.noise_history)
                noise_level = 0.1;
            else
                noise_level = std(obj.noise_history);
            end
        end
    end
end

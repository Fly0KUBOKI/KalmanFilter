classdef SensorBaroFilter < handle
    % SENSORBAROFILTER  気圧計専用フィルタ
    %
    % 機能:
    %   - スカラー値のフィルタリング
    %   - EMA平滑化（最も強い、α=0.1）
    %   - 時間変化が遅い特性を利用
    %   - 外れ値検出
    
    properties
        config              % フィルタ設定
        alt_filtered        % フィルタ済み高度
        noise_history       % ノイズ履歴
    end
    
    methods
        function obj = SensorBaroFilter(config)
            % Allow construction but prefer delegating to SensorFilters (MEX).
            if nargin < 1
                config = struct();
            end
            obj.config = config;
            obj.alt_filtered = [];
            obj.noise_history = [];

            if exist('mex_sensor_filter','file') ~= 3
                warning('SensorBaroFilter:MissingMEX', 'mex_sensor_filter not found. SensorFilters.baro will attempt to call MEX at runtime.');
            end
        end
        
        function [alt_out, is_outlier, info] = apply(obj, pressure)
            % APPLY  気圧計測値をフィルタリング (MEX delegation)
            try
                if nargout <= 1
                    alt_out = SensorFilters.baro(pressure);
                    is_outlier = false;
                    info = struct('delegated', true);
                else
                    [alt_out, is_outlier] = SensorFilters.baro(pressure);
                    info = struct('delegated', true);
                end
                if ~is_outlier
                    obj.alt_filtered = alt_out;
                end
                return;
            catch ME
                error('SensorBaroFilter:MissingMEX', 'Required MEX via SensorFilters.baro failed: %s', ME.message);
            end
        end
        
        function noise_level = getNoiseLevel(obj)
            if isempty(obj.noise_history)
                noise_level = 0.5;
            else
                noise_level = std(obj.noise_history);
            end
        end
    end
end

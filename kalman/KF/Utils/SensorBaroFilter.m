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
            % コンストラクタ
            obj.config = config;
            obj.alt_filtered = 0;
            obj.noise_history = [];
        end
        
        function [alt_out, is_outlier, info] = apply(obj, pressure)
            % APPLY  気圧計測値をフィルタリング（MEX に委譲）
            % Delegate to SensorFilters.baro and keep internal cache.

            info = struct();
            info.is_outlier = false;

            if nargout >= 2
                [alt_filt, is_outlier] = SensorFilters.baro(pressure);
            else
                alt_filt = SensorFilters.baro(pressure);
                is_outlier = false;
            end

            if ~is_outlier
                obj.alt_filtered = alt_filt;
            end

            alt_out = alt_filt;
            info.is_outlier = is_outlier;
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

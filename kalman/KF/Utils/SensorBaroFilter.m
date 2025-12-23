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
            error('SensorBaroFilter:disabled','SensorBaroFilter.apply is disabled. Use SensorFilters.baro(pressure).');
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

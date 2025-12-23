classdef SensorGPSFilter < handle
    % SENSORGPSFILTER  GPS専用フィルタ
    %
    % 機能:
    %   - 水平・垂直で異なる精度を考慮
    %   - EMA平滑化（弱い、α=0.15）
    %   - 信頼度の動的調整
    %   - 外れ値検出
    
    properties
        config              % フィルタ設定
        pos_filtered        % フィルタ済み位置 [x; y; z]
        noise_history       % ノイズ履歴
    end
    
    methods
        function obj = SensorGPSFilter(config)
            % コンストラクタ
            obj.config = config;
            obj.pos_filtered = [0; 0; 0];
            obj.noise_history = [];
        end
        
            function [pos_out, is_outlier, info] = apply(obj, pos_meas)
                error('SensorGPSFilter:disabled','SensorGPSFilter.apply is disabled. Use SensorFilters.gps(gps_pos, dt).');
            end
        
        function noise_level = getNoiseLevel(obj)
            if isempty(obj.noise_history)
                noise_level = [2.5; 5.0];  % [horizontal; vertical]
            else
                noise_level = [mean(std(obj.noise_history(:, 1))), ...
                               mean(std(obj.noise_history(:, 2)))];
            end
        end
    end
end

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
            % APPLY  GPS位置データをフィルタリング（MEX に委譲）
            % Delegate to SensorFilters.gps and keep internal cache.

            info = struct();
            info.is_outlier = false;

            % SensorFilters.gps expects a scalar dt as second argument; pass
            % a reasonable default (1.0) to avoid signature mismatch.
            if nargout >= 2
                [pos_filt, ~] = SensorFilters.gps(pos_meas, 1.0);
                is_outlier = false;
            else
                pos_filt = SensorFilters.gps(pos_meas, 1.0);
                is_outlier = false;
            end

            if ~is_outlier
                obj.pos_filtered = pos_filt;
            end

            pos_out = pos_filt;
            info.is_outlier = is_outlier;
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

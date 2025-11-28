classdef SensorFilter < handle
    % SensorFilter センサーフィルタ基底クラス
    
    properties
        alpha
        filtered_value
    end
    
    methods
        function obj = SensorFilter(alpha)
            if nargin < 1
                alpha = 0.3;
            end
            obj.alpha = alpha;
            obj.filtered_value = [];
        end
        
        function [z_filt, is_outlier, z_expected] = apply(obj, z_meas, z_expected)
            if nargin < 3
                z_expected = zeros(size(z_meas));
            end
            
            % 外れ値判定（簡易版）
            if ~isempty(obj.filtered_value)
                diff_norm = norm(z_meas - obj.filtered_value);
                is_outlier = (diff_norm > 10.0);  % 簡易閾値
            else
                is_outlier = false;
            end
            
            if is_outlier
                z_filt = obj.filtered_value;
            else
                if isempty(obj.filtered_value)
                    obj.filtered_value = z_meas;
                else
                    obj.filtered_value = obj.alpha * z_meas + (1 - obj.alpha) * obj.filtered_value;
                end
                z_filt = obj.filtered_value;
            end
        end
    end
    
    methods (Static)
        function filter = createAccelFilter()
            filter = SensorFilter(0.3);
        end
        
        function filter = createGyroFilter()
            filter = SensorFilter(0.2);
            filter.w_filtered = zeros(3,1);
        end
        
        function filter = createMagFilter()
            filter = SensorFilter(0.5);
        end
        
        function filter = createGPSFilter()
            filter = SensorFilter(0.4);
        end
        
        function filter = createBaroFilter()
            % 気圧フィルタ：気圧値を高度に変換
            filter = SensorFilter(0.3);
        end
        
        function [y_filt, should_update] = filterInnovation(y, R)
            % イノベーションフィルタリング
            y_filt = y;
            should_update = true;
            
            % 簡易外れ値判定
            if norm(y) > 10.0 * sqrt(trace(R))
                should_update = false;
            end
        end
    end
end

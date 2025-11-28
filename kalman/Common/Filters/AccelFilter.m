classdef AccelFilter < handle
    % AccelFilter 加速度フィルタ
    
    properties
        alpha
        threshold
        a_filtered
    end
    
    methods
        function obj = AccelFilter(alpha, threshold)
            obj.alpha = alpha;
            obj.threshold = threshold;
            obj.a_filtered = zeros(3,1);
        end
        
        function [a_filt, is_outlier] = filter(obj, a_meas, a_expected)
            % 外れ値チェック
            diff_norm = norm(a_meas - a_expected);
            is_outlier = (diff_norm > obj.threshold);
            
            if is_outlier
                % 外れ値の場合は前回値を返す
                a_filt = obj.a_filtered;
            else
                % EMAフィルタ適用
                if norm(obj.a_filtered) < 1e-6
                    obj.a_filtered = a_meas;
                else
                    obj.a_filtered = obj.alpha * a_meas + (1 - obj.alpha) * obj.a_filtered;
                end
                a_filt = obj.a_filtered;
            end
        end
    end
end

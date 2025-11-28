classdef OutlierGuard < handle
    % OutlierGuard 外れ値検出クラス
    
    methods (Static)
        function [should_update, y_used, R_updated, dx_used, info] = checkAndApply(sensor_type, z, h, H, P, R, K_in, dx_in, divergence_guard, noise_estimator, ctx)
            % 外れ値チェックと適用
            
            % デフォルト：更新を許可
            should_update = true;
            y_used = z - h;
            R_updated = R;
            dx_used = dx_in;
            info = struct();
            
            % マハラノビス距離チェック
            S = H * P * H' + R;
            try
                mahal_dist = sqrt(y_used' / S * y_used);
                
                % 5-sigma 以上は棄却
                if mahal_dist > 5.0
                    should_update = false;
                end
            catch
                % S が特異の場合
                should_update = false;
            end
        end
    end
end

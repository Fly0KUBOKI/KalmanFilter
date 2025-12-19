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
            % APPLY  GPS位置データをフィルタリング
            %
            % 入力:
            %   pos_meas - 計測位置 [x; y; z] (m)
            %
            % 出力:
            %   pos_out    - フィルタ済み位置 (3x1)
            %   is_outlier - 外れ値判定フラグ
            %   info       - デバッグ情報
            
            info = struct();
            info.is_outlier = false;
            
            % 残差を計算
            residual = pos_meas - obj.pos_filtered;
            
            % 水平・垂直で異なる精度
            residual_h = sqrt(residual(1)^2 + residual(2)^2);  % 水平
            residual_v = abs(residual(3));                      % 垂直
            
            % 残差ノルムを推定
            if isempty(obj.noise_history)
                noise_estimate_h = residual_h;
                noise_estimate_v = residual_v;
            else
                % 過去の水平・垂直残差をそれぞれ追跡
                if size(obj.noise_history, 2) >= 2
                    noise_history_h = obj.noise_history(:, 1);
                    noise_history_v = obj.noise_history(:, 2);
                    noise_estimate_h = std(noise_history_h);
                    noise_estimate_v = std(noise_history_v);
                else
                    noise_estimate_h = residual_h;
                    noise_estimate_v = residual_v;
                end
            end
            
            % 外れ値判定（3σ、水平・垂直で分離）
            h_outlier = (residual_h > 3.0 * max(noise_estimate_h, 1.0));
            v_outlier = (residual_v > 3.0 * max(noise_estimate_v, 1.0));
            is_outlier = h_outlier || v_outlier;
            
            if is_outlier
                pos_out = obj.pos_filtered;
                info.is_outlier = true;
                info.h_outlier = h_outlier;
                info.v_outlier = v_outlier;
                return;
            end
            
            % EMAフィルタを適用（弱い平滑化）
            pos_smooth = obj.config.ema_alpha * pos_meas + ...
                         (1 - obj.config.ema_alpha) * obj.pos_filtered;
            obj.pos_filtered = pos_smooth;
            
            % ノイズ履歴を更新
            obj.noise_history = [obj.noise_history; residual_h, residual_v];
            if size(obj.noise_history, 1) > obj.config.history_size
                obj.noise_history = obj.noise_history(2:end, :);
            end
            
            pos_out = pos_smooth;
            info.residual_h = residual_h;
            info.residual_v = residual_v;
            info.noise_estimate_h = noise_estimate_h;
            info.noise_estimate_v = noise_estimate_v;
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

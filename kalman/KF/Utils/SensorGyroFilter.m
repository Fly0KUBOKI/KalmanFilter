classdef SensorGyroFilter < handle
    % SENSORGYROFILTER  ジャイロ専用フィルタ
    %
    % 機能:
    %   - EMA平滑化（より強い、α=0.25）
    %   - 外れ値検出（3σ）
    %   - ドリフト推定
    
    properties
        config              % フィルタ設定
        w_filtered          % フィルタ済み角速度
        w_filtered_axis     % 軸別フィルタ済み角速度（α値の軸別制御用）
        bias_estimate       % バイアス推定
        noise_history       % ノイズ履歴
    end
    
    methods
        function obj = SensorGyroFilter(config)
            % コンストラクタ
            obj.config = config;
            obj.w_filtered = [0; 0; 0];
            obj.w_filtered_axis = [0; 0; 0];  % Roll, Pitch, Yaw のフィルタ済み値
            obj.bias_estimate = [0; 0; 0];
            obj.noise_history = [];
        end
        
        function [w_out, is_outlier, info] = apply(obj, w_meas, w_expected)
            % APPLY  ジャイロ計測値をフィルタリング
            %
            % 入力:
            %   w_meas     - 計測角速度 (3x1, rad/s)
            %   w_expected - 期待角速度 (3x1, オプション)
            %
            % 出力:
            %   w_out      - フィルタ済み角速度 (3x1)
            %   is_outlier - 外れ値判定フラグ
            %   info       - デバッグ情報
            
            if nargin < 3
                w_expected = zeros(3, 1);
            end
            
            info = struct();
            info.is_outlier = false;
            
            % 残差を計算
            residual = w_meas - w_expected;
            residual_norm = norm(residual);
            
            % ノイズレベルを推定
            if isempty(obj.noise_history)
                noise_estimate = residual_norm;
            else
                noise_std = std(obj.noise_history);
                noise_estimate = max(noise_std, residual_norm / 3.0);
            end
            
            % 外れ値判定（3σ）
            is_outlier = (residual_norm > 3.0 * max(noise_estimate, 0.01));
            if is_outlier
                w_out = obj.w_filtered;
                info.is_outlier = true;
                return;
            end
            
            % EMAフィルタを軸別に適用
            % Roll (1軸): α=0.25（標準）
            % Pitch (2軸): α=0.25（標準）
            % Yaw (3軸): α=0.08（弱い平滑化、角速度積分を重視）
            alpha_roll = obj.config.ema_alpha;           % 0.25
            alpha_pitch = obj.config.ema_alpha;          % 0.25
            alpha_yaw = 0.08;  % ← Yaw は弱めの平滑化
            
            w_smooth = [0; 0; 0];
            w_smooth(1) = alpha_roll * w_meas(1) + (1 - alpha_roll) * obj.w_filtered(1);
            w_smooth(2) = alpha_pitch * w_meas(2) + (1 - alpha_pitch) * obj.w_filtered(2);
            w_smooth(3) = alpha_yaw * w_meas(3) + (1 - alpha_yaw) * obj.w_filtered(3);
            
            obj.w_filtered = w_smooth;
            obj.w_filtered_axis = w_smooth;
            
            % ドリフト推定（静止時の偏り）
            % α を使用したドリフト学習
            obj.bias_estimate = obj.bias_estimate + ...
                obj.config.drift_learning_rate * residual;
            
            % ノイズ履歴を更新
            obj.noise_history = [obj.noise_history; residual_norm];
            if length(obj.noise_history) > obj.config.history_size
                obj.noise_history = obj.noise_history(2:end);
            end
            
            w_out = w_smooth;
            info.residual_norm = residual_norm;
            info.bias_estimate = obj.bias_estimate;
        end
        
        function noise_level = getNoiseLevel(obj)
            if isempty(obj.noise_history)
                noise_level = 0.01;
            else
                noise_level = std(obj.noise_history);
            end
        end
        
        function bias = getBiasEstimate(obj)
            bias = obj.bias_estimate;
        end
    end
end

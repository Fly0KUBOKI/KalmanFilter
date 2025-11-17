classdef SensorAccelFilter < handle
    % SENSORACCELFILTER  加速度計専用フィルタ
    %
    % 機能:
    %   - EMA平滑化
    %   - 外れ値検出（3σ）
    %   - 重力ノルム検証
    %   - 大きな変化スケーリング
    
    properties
        config              % フィルタ設定
        a_filtered          % フィルタ済み加速度
        noise_history       % ノイズ履歴
    end
    
    methods
        function obj = SensorAccelFilter(config)
            % コンストラクタ
            obj.config = config;
            obj.a_filtered = [0; 0; 0];
            obj.noise_history = [];
        end
        
        function [a_out, is_outlier, info] = apply(obj, a_meas, a_expected)
            % APPLY  加速度計測値をフィルタリング
            %
            % 入力:
            %   a_meas     - 計測加速度 (3x1)
            %   a_expected - 期待加速度 (3x1、オプション)
            %
            % 出力:
            %   a_out      - フィルタ済み加速度 (3x1)
            %   is_outlier - 外れ値判定フラグ
            %   info       - デバッグ情報 (struct)
            
            if nargin < 3
                a_expected = zeros(3, 1);
            end
            
            info = struct();
            info.is_outlier = false;
            info.is_gravity_mismatch = false;
            info.scale_factor = 1.0;
            
            % 重力ノルム検証
            a_norm = norm(a_meas);
            gravity_range = obj.config.gravity_range;
            if a_norm < gravity_range(1) || a_norm > gravity_range(2)
                % 重力以外の加速度成分が大きい
                is_outlier = true;
                a_out = obj.a_filtered;
                info.is_outlier = true;
                info.a_norm = a_norm;
                return;
            end
            
            % 残差を計算
            residual = a_meas - a_expected;
            residual_norm = norm(residual);
            
            % ノイズレベルを推定
            if isempty(obj.noise_history)
                noise_estimate = residual_norm;
            else
                noise_std = std(obj.noise_history);
                noise_estimate = max(noise_std, residual_norm / 3.0);
            end
            
            % 外れ値判定（3σ）
            is_outlier = (residual_norm > 3.0 * max(noise_estimate, 0.1));
            if is_outlier
                a_out = obj.a_filtered;
                info.is_outlier = true;
                info.residual_norm = residual_norm;
                info.noise_estimate = noise_estimate;
                return;
            end
            
            % EMAフィルタを適用
            a_smooth = obj.config.ema_alpha * a_meas + ...
                       (1 - obj.config.ema_alpha) * obj.a_filtered;
            obj.a_filtered = a_smooth;
            
            % ノイズ履歴を更新
            obj.noise_history = [obj.noise_history; residual_norm];
            if length(obj.noise_history) > obj.config.history_size
                obj.noise_history = obj.noise_history(2:end);
            end
            
            a_out = a_smooth;
            info.residual_norm = residual_norm;
            info.noise_estimate = noise_estimate;
        end
        
        function noise_level = getNoiseLevel(obj)
            % GETNOISELEVEL  ノイズレベルを取得
            if isempty(obj.noise_history)
                noise_level = 0.1;
            else
                noise_level = std(obj.noise_history);
            end
        end
    end
end

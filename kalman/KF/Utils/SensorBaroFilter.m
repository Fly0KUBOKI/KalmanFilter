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
            % APPLY  気圧計測値をフィルタリング
            %
            % 入力:
            %   pressure - 気圧 (Pa)
            %
            % 出力:
            %   alt_out    - フィルタ済み高度 (m)
            %   is_outlier - 外れ値判定フラグ
            %   info       - デバッグ情報
            
            info = struct();
            info.is_outlier = false;
            
            % 気圧から高度に変換（バロメトリック公式）
            P0 = 101325;  % 標準気圧
            alt_meas = obj.config.altitude_per_pressure * ...
                       (1 - (pressure / P0)^0.1903);
            
            % 残差を計算
            residual = alt_meas - obj.alt_filtered;
            residual_abs = abs(residual);
            
            % ノイズレベル推定
            if isempty(obj.noise_history)
                noise_estimate = residual_abs;
            else
                noise_std = std(obj.noise_history);
                noise_estimate = max(noise_std, residual_abs / 3.0);
            end
            
            % 外れ値判定（3σ）
            % 気圧計は時間変化が遅いため、大きな変化は外れ値の可能性が高い
            is_outlier = (residual_abs > 3.0 * max(noise_estimate, 0.5));
            
            if is_outlier
                alt_out = obj.alt_filtered;
                info.is_outlier = true;
                info.residual = residual;
                info.noise_estimate = noise_estimate;
                return;
            end
            
            % EMAフィルタを適用（最も強い平滑化）
            alt_smooth = obj.config.ema_alpha * alt_meas + ...
                         (1 - obj.config.ema_alpha) * obj.alt_filtered;
            obj.alt_filtered = alt_smooth;
            
            % ノイズ履歴を更新
            obj.noise_history = [obj.noise_history; residual_abs];
            if length(obj.noise_history) > obj.config.history_size
                obj.noise_history = obj.noise_history(2:end);
            end
            
            alt_out = alt_smooth;
            info.residual = residual;
            info.noise_estimate = noise_estimate;
            info.pressure = pressure;
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

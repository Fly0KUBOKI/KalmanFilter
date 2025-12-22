classdef AccelFilter < handle
    % ACCELFILTER  加速度計測値の平滑化とノイズ低減
    %
    % 指数移動平均（EMA）フィルタと外れ値検出を提供
    
    properties
        a_filtered          % フィルタ済み加速度 (3x1)
        ema_alpha           % EMA係数 (0.0 ~ 1.0)
        outlier_threshold   % 外れ値検出閾値（σ）
        noise_history       % ノイズ履歴（分散の推定用）
        history_size        % 履歴サイズ
    end
    
    methods
        function obj = AccelFilter(ema_alpha, history_size)
            % コンストラクタ
            %
            % 入力:
            %   ema_alpha    - EMA係数 (デフォルト: 0.3, 小さいほど平滑化が強い)
            %   history_size - 履歴サイズ (デフォルト: 20)
            
            if nargin < 1 || isempty(ema_alpha)
                ema_alpha = 0.3;
            end
            if nargin < 2 || isempty(history_size)
                history_size = 20;
            end
            
            obj.a_filtered = [0; 0; 0];
            obj.ema_alpha = ema_alpha;
            obj.outlier_threshold = 3.0;  % 3σを超える値を外れ値とする
            obj.noise_history = [];
            obj.history_size = history_size;
        end
        
        function [a_smooth, is_outlier] = filter(obj, a_meas, a_expected)
            % FILTER  加速度計測値をフィルタリング
            %
            % 入力:
            %   a_meas     - 計測加速度 (3x1)
            %   a_expected - 期待加速度 (3x1、オプション)
            %
            % 出力:
            %   a_smooth   - フィルタ済み加速度 (3x1)
            %   is_outlier - 外れ値判定フラグ (boolean)
            
            if nargin < 3
                a_expected = zeros(3, 1);
            end
            
            % Delegate to compiled MEX implementation (MATLAB fallback removed)
            try
                if nargout >= 2
                    [a_filt, is_outlier] = SensorFilters.accel(a_meas, a_expected);
                else
                    a_filt = SensorFilters.accel(a_meas, a_expected);
                    is_outlier = false;
                end
                a_smooth = a_filt;
                obj.a_filtered = a_smooth;
                % update noise history with residual if available
                try
                    residual = a_smooth - a_expected;
                    residual_norm = norm(residual);
                    obj.noise_history = [obj.noise_history; residual_norm];
                    if length(obj.noise_history) > obj.history_size
                        obj.noise_history = obj.noise_history(2:end);
                    end
                catch
                end
                return;
            catch ME
                error('AccelFilter:MissingMEX', 'Required MEX ''mex_sensor_filter'' not available or failed: %s', ME.message);
            end

            % 現在のノイズレベルを推定
            if isempty(obj.noise_history)
                noise_estimate = residual_norm;
            else
                noise_std = std(obj.noise_history);
                noise_estimate = max(noise_std, residual_norm / 3.0);  % 保守的に推定
            end

            % 外れ値判定
            is_outlier = (residual_norm > obj.outlier_threshold * max(noise_estimate, 0.1));

            % 外れ値の場合は前回値を保持（更新しない）
            if is_outlier
                a_smooth = obj.a_filtered;
                return;
            end

            % EMAフィルタを適用
            % y_new = alpha * x + (1 - alpha) * y_old
            a_smooth = obj.ema_alpha * a_meas + (1 - obj.ema_alpha) * obj.a_filtered;
            obj.a_filtered = a_smooth;

            % ノイズ履歴を更新（キューイング）
            obj.noise_history = [obj.noise_history; residual_norm];
            if length(obj.noise_history) > obj.history_size
                obj.noise_history = obj.noise_history(2:end);
            end
        end
        
        function noise_level = getNoiseLevel(obj)
            % GETNOISELEVEL  現在のノイズレベルを取得
            %
            % 出力:
            %   noise_level - ノイズの標準偏差推定値
            
            if isempty(obj.noise_history)
                noise_level = 0.1;
            else
                noise_level = std(obj.noise_history);
            end
        end
        
        function setEMAAlpha(obj, alpha)
            % SETEMALPHA  EMA係数を設定
            % （ノイズレベルに応じて動的に調整するために使用）
            %
            % 入力:
            %   alpha - EMA係数 (0.0 ~ 1.0)
            
            obj.ema_alpha = max(0.05, min(1.0, alpha));
        end
    end
end

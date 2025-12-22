classdef SensorFilter
    % SENSORFILTER  センサー観測のフィルタリング共通クラス
    %
    % σ閾値によるイノベーションフィルタリングと
    % センサー固有のフィルタリングを提供
    
    properties (Constant)
        SIGMA_THRESHOLD = 1.0;  % σ閾値
    end
    
    methods (Static)
        function filter = createAccelFilter(varargin)
            % CREATEACCELFILTER  加速度計フィルタを生成
            %
            % 特性:
            %   - EMA平滑化 (α=0.3)
            %   - 外れ値検出 (3σ)
            %   - 重力ノルム検証 (9.81 m/s^2)
            %   - 大きな変化スケーリング (>1.0°)
            
            p = inputParser;
            addParameter(p, 'ema_alpha', 0.3);
            addParameter(p, 'history_size', 20);
            addParameter(p, 'gravity_range', [8.5, 10.5]);  % 許容重力ノルム範囲
            addParameter(p, 'large_change_threshold', 1.0);  % 度
            addParameter(p, 'scale_factor', 0.1);  % 大きな変化時のスケーリング
            parse(p, varargin{:});
            
            config = struct();
            config.type = 'accel';
            config.ema_alpha = p.Results.ema_alpha;
            config.history_size = p.Results.history_size;
            config.gravity_range = p.Results.gravity_range;
            config.large_change_threshold = p.Results.large_change_threshold;
            config.scale_factor = p.Results.scale_factor;
            
            filter = SensorAccelFilter(config);
            % If MEX exists and not forced to use MATLAB, sync config to C++ side
            force_matlab_env = getenv('FORCE_MATLAB_FILTERS');
            force_matlab = ~isempty(force_matlab_env) && strcmp(force_matlab_env, '1');
            if exist('mex_sensor_filter','file') == 3 && ~force_matlab
                try
                    SensorFilters.accel_config(config);
                catch
                    % ignore mex config failures; MATLAB filter still works
                end
            end
        end
        
        function filter = createGyroFilter(varargin)
            % CREATEGYROFILTER  ジャイロフィルタを生成
            %
            % 特性:
            %   - EMA平滑化 (α=0.25, より強い)
            %   - 外れ値検出 (3σ)
            %   - ドリフト補正
            
            p = inputParser;
            addParameter(p, 'ema_alpha', 0.25);
            addParameter(p, 'history_size', 30);
            addParameter(p, 'drift_learning_rate', 0.0001);
            parse(p, varargin{:});
            
            config = struct();
            config.type = 'gyro';
            config.ema_alpha = p.Results.ema_alpha;
            config.history_size = p.Results.history_size;
            config.drift_learning_rate = p.Results.drift_learning_rate;
            
            error('SensorGyroFilter has been removed. Gyro filtering is discontinued; do not call createGyroFilter.');
        end
        
        function filter = createMagFilter(varargin)
            % CREATEMAGFILTER  磁気計フィルタを生成
            %
            % 特性:
            %   - ベクトル正規化必須
            %   - EMA平滑化 (α=0.2, より強い)
            %   - 地場ノルム検証
            
            p = inputParser;
            addParameter(p, 'ema_alpha', 0.2);
            addParameter(p, 'history_size', 20);
            addParameter(p, 'mag_norm_expected', 50);  % nT
            parse(p, varargin{:});
            
            config = struct();
            config.type = 'mag';
            config.ema_alpha = p.Results.ema_alpha;
            config.history_size = p.Results.history_size;
            config.mag_norm_expected = p.Results.mag_norm_expected;
            
            filter = SensorMagFilter(config);
        end
        
        function filter = createGPSFilter(varargin)
            % CREATEGPSFILTER  GPSフィルタを生成
            %
            % 特性:
            %   - 水平・垂直で異なる精度
            %   - EMA平滑化 (α=0.15)
            %   - DOP値に基づいた信頼度調整
            
            p = inputParser;
            addParameter(p, 'ema_alpha', 0.15);
            addParameter(p, 'history_size', 10);
            addParameter(p, 'horizontal_accuracy', 2.5);  % m
            addParameter(p, 'vertical_accuracy', 5.0);    % m
            parse(p, varargin{:});
            
            config = struct();
            config.type = 'gps';
            config.ema_alpha = p.Results.ema_alpha;
            config.history_size = p.Results.history_size;
            config.horizontal_accuracy = p.Results.horizontal_accuracy;
            config.vertical_accuracy = p.Results.vertical_accuracy;
            
            filter = SensorGPSFilter(config);
        end
        
        function filter = createBaroFilter(varargin)
            % CREATEBAROFILTER  気圧計フィルタを生成
            %
            % 特性:
            %   - スカラー値
            %   - EMA平滑化 (α=0.1, 最も強い)
            %   - 時間変化が遅い
            
            p = inputParser;
            addParameter(p, 'ema_alpha', 0.1);
            addParameter(p, 'history_size', 50);
            addParameter(p, 'altitude_per_pressure', 44330);  % Barometric formula constant
            parse(p, varargin{:});
            
            config = struct();
            config.type = 'baro';
            config.ema_alpha = p.Results.ema_alpha;
            config.history_size = p.Results.history_size;
            config.altitude_per_pressure = p.Results.altitude_per_pressure;
            
            filter = SensorBaroFilter(config);
        end
        
        function [filtered_innovation, should_update] = filterInnovation(innovation, R_noise)
            % FILTERINNOVATION  イノベーションをσ閾値でフィルタリング
            %
            % 入力:
            %   innovation - イノベーションベクトル (nx1)
            %   R_noise    - 観測ノイズ共分散の対角要素 (nx1) または (nxn)
            %
            % 出力:
            %   filtered_innovation - フィルタリング後のイノベーション
            %   should_update       - 更新すべきかどうか (boolean)
            
            filtered_innovation = innovation;
            n = length(innovation);
            
            % R_noiseが行列の場合は対角要素を取得
            if size(R_noise, 1) == size(R_noise, 2) && size(R_noise, 1) > 1
                R_diag = diag(R_noise);
            else
                R_diag = R_noise(:);
            end
            
            % 各要素ごとにσ閾値でフィルタリング
            for i = 1:n
                noise_std = sqrt(max(R_diag(i), eps));
                threshold = SensorFilter.SIGMA_THRESHOLD * noise_std;
                
                if abs(innovation(i)) < threshold
                    filtered_innovation(i) = 0;
                end
            end
            
            % 全てのイノベーションが0なら更新不要
            should_update = (norm(filtered_innovation) > eps);
        end
        
        function should_update = shouldUpdate(innovation, R_noise)
            % SHOULDUPDATE  更新すべきかどうかを判定
            %
            % 入力:
            %   innovation - イノベーションベクトル (nx1)
            %   R_noise    - 観測ノイズ共分散の対角要素 (nx1)
            %
            % 出力:
            %   should_update - 更新すべきかどうか (boolean)
            
            [~, should_update] = SensorFilter.filterInnovation(innovation, R_noise);
        end
        
        function filtered_dx = filterStateCorrection(dx, R_noise, H, component_indices)
            % FILTERSTATECORRECTION  状態修正量をフィルタリング
            %
            % 入力:
            %   dx                - 状態修正量ベクトル (15x1)
            %   R_noise          - 観測ノイズ共分散
            %   H                - 観測行列
            %   component_indices - フィルタ対象の状態インデックス
            %
            % 出力:
            %   filtered_dx - フィルタリング後の状態修正量
            
            filtered_dx = dx;
            
            if nargin < 4 || isempty(component_indices)
                return;
            end
            
            % R_noiseから標準偏差を取得
            if size(R_noise, 1) == size(R_noise, 2)
                R_diag = diag(R_noise);
            else
                R_diag = R_noise(:);
            end
            
            % 各成分について閾値判定
            for i = 1:length(component_indices)
                idx = component_indices(i);
                
                % 観測行列から感度を推定
                if ~isempty(H) && size(H, 2) >= idx
                    sensitivity = max(norm(H(:, idx)), eps);
                    noise_std = sqrt(max(mean(R_diag), eps));
                    threshold = SensorFilter.SIGMA_THRESHOLD * noise_std / sensitivity;
                else
                    % デフォルト閾値
                    threshold = 0.001;
                end
                
                if abs(filtered_dx(idx)) < threshold
                    filtered_dx(idx) = 0;
                end
            end
        end
    end
end
classdef NoiseEstimator < handle
    % NOISEESTIMATOR  ノイズ推定クラス
    %
    % 初期N回は平均で初期値を決定、その後はEMAで逐次更新
    
    properties
        % ノイズ推定値 (分散)
        R_accel     % 加速度計 (3x1)
        R_gyro      % ジャイロ (3x1)
        R_mag       % 磁気計 (3x1)
        R_gps       % GPS (3x1)
        R_baro      % 気圧計 (scalar)
        
        % 内部状態
        count_accel
        count_gyro
        count_mag
        count_gps
        count_baro
        
        sum_accel
        sum_gyro
        sum_mag
        sum_gps
        sum_baro
        
        alpha           % EMA更新率
        alpha_count     % α値更新カウンタ
        warmup_samples  % ウォームアップサンプル数
    end
    
    properties (Constant)
        R_ABS_MAX = 1e6;
        R_ABS_MIN = eps;
        OUTLIER_FACTOR = 50;  % 外れ値判定係数（現在のR値のN倍まで許容）
    end
    
    methods
        function obj = NoiseEstimator(warmup_samples)
            % コンストラクタ
            if nargin < 1
                warmup_samples = 10;
            end
            
            obj.warmup_samples = warmup_samples;
            
            % カウンタ初期化
            obj.count_accel = 0;
            obj.count_gyro = 0;
            obj.count_mag = 0;
            obj.count_gps = 0;
            obj.count_baro = 0;
            
            % 累積和初期化
            obj.sum_accel = zeros(3, 1);
            obj.sum_gyro = zeros(3, 1);
            obj.sum_mag = zeros(3, 1);
            obj.sum_gps = zeros(3, 1);
            obj.sum_baro = 0;
            
            % デフォルトノイズ値
            obj.R_accel = ones(3, 1) * 0.01;
            obj.R_gyro = ones(3, 1) * deg2rad(0.1)^2;
            obj.R_mag = ones(3, 1) * 5.0^2;
            obj.R_gps = [3^2; 3^2; 5^2];
            obj.R_baro = 1.0^2;
            
            % α値（EMA更新率: 小さいほど滑らかで変動に鈍感）
            obj.alpha = 0.01;  % 0.1 から 0.01 に変更（より滑らかな推定）
            obj.alpha_count = 0;
        end
        
        function estimate(obj, sensor_type, innovation, H, P_pred)
            % ESTIMATE  ノイズを推定・更新
            %
            % 入力:
            %   sensor_type - センサータイプ ('accel', 'gyro', 'mag', 'gps', 'baro')
            %   innovation  - イノベーション
            %   H           - 観測行列
            %   P_pred      - 予測共分散
            
            % イノベーション共分散成分の計算
            HPHT = H * P_pred * H';
            HPHT_diag = diag(HPHT);
            
            % イノベーションからノイズ分散を推定
            innov_sq = innovation.^2;
            innov_var = max(innov_sq - HPHT_diag, obj.R_ABS_MIN);
            
            % センサータイプごとの処理
            switch sensor_type
                case 'accel'
                    obj.updateNoise('accel', innov_var);
                case 'gyro'
                    obj.updateNoise('gyro', innov_var);
                case 'mag'
                    obj.updateNoise('mag', innov_var);
                case 'gps'
                    obj.updateNoise('gps', innov_var);
                case 'baro'
                    obj.updateNoise('baro', innov_var(1));
                otherwise
                    error('Unknown sensor type: %s', sensor_type);
            end
        end
        
        function R = getRnoise(obj, sensor_type)
            % GETRNOISE  ノイズ共分散行列を取得
            %
            % 入力:
            %   sensor_type - センサータイプ
            %
            % 出力:
            %   R - ノイズ共分散行列
            
            switch sensor_type
                case 'accel'
                    R = diag(obj.R_accel);
                case 'gyro'
                    R = diag(obj.R_gyro);
                case 'mag'
                    R = diag(obj.R_mag);
                case 'gps'
                    R = diag(obj.R_gps);
                case 'baro'
                    R = obj.R_baro;
                otherwise
                    error('Unknown sensor type: %s', sensor_type);
            end
        end
        
        function [threshold_vec, threshold_scalar] = getThreshold(obj, sensor_type, sigma_multiplier)
            % GETTHRESHOLD  閾値を取得（軸ごとのベクトルとスカラー）
            %
            % 入力:
            %   sensor_type      - センサータイプ ('accel', 'gyro', etc.)
            %   sigma_multiplier - σの倍数（デフォルト: 2.0）
            %
            % 出力:
            %   threshold_vec    - 軸ごとの閾値ベクトル (nx1)
            %   threshold_scalar - スカラー閾値（全軸の平均）
            
            if nargin < 3
                sigma_multiplier = 2.0;
            end
            
            switch sensor_type
                case 'accel'
                    R_var = obj.R_accel;
                case 'gyro'
                    R_var = obj.R_gyro;
                case 'mag'
                    R_var = obj.R_mag;
                case 'gps'
                    R_var = obj.R_gps;
                case 'baro'
                    R_var = obj.R_baro;
                otherwise
                    error('Unknown sensor type: %s', sensor_type);
            end
            
            % 標準偏差を計算
            std_vec = sqrt(max(R_var, eps));
            
            % 閾値 = σ × multiplier
            threshold_vec = sigma_multiplier * std_vec;
            threshold_scalar = mean(threshold_vec);
        end
    end
    
    methods (Access = private)
        function updateNoise(obj, sensor_type, innov_var)
            % UPDATENOISE  ノイズ推定値を更新 (内部メソッド)
            %
            % 外れ値検出とクリッピングを実装
            
            % フィールド名を生成
            field_count = ['count_' sensor_type];
            field_sum = ['sum_' sensor_type];
            field_R = ['R_' sensor_type];
            
            % カウンタ更新
            obj.(field_count) = obj.(field_count) + 1;
            count = obj.(field_count);
            
            % 初期期間: 平均を計算
            if count <= obj.warmup_samples
                obj.(field_sum) = obj.(field_sum) + innov_var;
                obj.(field_R) = obj.(field_sum) / count;
            else
                % 外れ値検出: 現在のR値の OUTLIER_FACTOR 倍を超える場合はクリップ
                R_prev = obj.(field_R);
                max_allowed = R_prev * obj.OUTLIER_FACTOR;
                innov_var_clipped = min(innov_var, max_allowed);
                
                % EMAで逐次更新
                alpha_val = obj.alpha;
                obj.(field_R) = (1 - alpha_val) * R_prev + alpha_val * innov_var_clipped;
                
                % % α値の逐次更新(徐々に小さくする)
                % obj.alpha_count = obj.alpha_count + 1;
                % if obj.alpha_count > obj.warmup_samples
                %     obj.alpha = max(0.01, obj.alpha * 0.9995);
                % end
            end
            
            % 数値安定性のためのクランプ
            obj.(field_R) = min(max(obj.(field_R), obj.R_ABS_MIN), obj.R_ABS_MAX);
            
            % 無効値の除去
            obj.(field_R)(~isfinite(obj.(field_R))) = obj.R_ABS_MIN;
        end
    end
end

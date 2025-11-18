classdef NoiseEstimatorLib
    % NOISEESTIMATORLIB ノイズ推定の静的ライブラリ
    %
    % イノベーション統計に基づくセンサーノイズ推定
    % C++互換設計: すべて静的メソッド
    
    methods (Static)
        %% ノイズ状態の初期化
        function state = create_state(warmup_samples)
            % CREATE_STATE ノイズ推定状態の作成
            %
            % 入力:
            %   warmup_samples - ウォームアップサンプル数 (デフォルト: 10)
            %
            % 出力:
            %   state - ノイズ推定状態構造体
            
            if nargin < 1
                warmup_samples = 10;
            end
            
            state = struct();
            
            % 推定値 (分散)
            state.R_accel = ones(3,1) * 0.01;
            state.R_gyro = ones(3,1) * deg2rad(0.1)^2;
            state.R_mag = ones(3,1) * 5.0^2;
            state.R_gps = [3^2; 3^2; 5^2];
            state.R_baro = 1.0^2;
            
            % カウンタ
            state.count_accel = 0;
            state.count_gyro = 0;
            state.count_mag = 0;
            state.count_gps = 0;
            state.count_baro = 0;
            
            % 累積和
            state.sum_accel = zeros(3,1);
            state.sum_gyro = zeros(3,1);
            state.sum_mag = zeros(3,1);
            state.sum_gps = zeros(3,1);
            state.sum_baro = 0;
            
            % パラメータ
            state.warmup_samples = warmup_samples;
            state.alpha = 0.01;  % EMA更新率
            state.R_MAX = 1e6;
            state.R_MIN = eps;
            state.OUTLIER_FACTOR = 20;
        end
        
        %% ノイズ推定更新
        function state = estimate(state, sensor_type, innovation, S)
            % ESTIMATE イノベーションからノイズを推定・更新
            %
            % 入力:
            %   state       - ノイズ推定状態
            %   sensor_type - センサータイプ ('accel', 'gyro', 'mag', 'gps', 'baro')
            %   innovation  - イノベーション [nx1]
            %   S           - イノベーション共分散 [nxn]
            %
            % 出力:
            %   state - 更新後の状態
            
            % イノベーション2乗ノルム
            innov_sq = innovation.^2;
            
            switch sensor_type
                case 'accel'
                    state = NoiseEstimatorLib.update_sensor_noise(...
                        state, 'accel', innov_sq, S);
                    
                case 'gyro'
                    state = NoiseEstimatorLib.update_sensor_noise(...
                        state, 'gyro', innov_sq, S);
                    
                case 'mag'
                    state = NoiseEstimatorLib.update_sensor_noise(...
                        state, 'mag', innov_sq, S);
                    
                case 'gps'
                    state = NoiseEstimatorLib.update_sensor_noise(...
                        state, 'gps', innov_sq, S);
                    
                case 'baro'
                    state = NoiseEstimatorLib.update_sensor_noise(...
                        state, 'baro', innov_sq, S);
            end
        end
        
        %% 内部: センサー別ノイズ更新
        function state = update_sensor_noise(state, sensor_type, innov_sq, S)
            % UPDATE_SENSOR_NOISE センサー別のノイズ更新
            %
            % イノベーション統計: E[vv'] = HPH' + R = S
            % → R_est = diag(vv') - diag(HPH')
            
            % フィールド名
            count_field = ['count_' sensor_type];
            sum_field = ['sum_' sensor_type];
            R_field = ['R_' sensor_type];
            
            count = state.(count_field);
            
            % 外れ値チェック
            current_R = state.(R_field);
            if any(innov_sq > state.OUTLIER_FACTOR * current_R)
                % 外れ値はスキップ
                return;
            end
            
            % S行列から予測分散を差し引いてノイズ推定
            if isvector(S)
                % ベクトル形式（対角要素のみ）
                S_diag = S(:);
            else
                % 行列形式
                S_diag = diag(S);
            end
            
            % R推定値 = innovation^2 - (HPH')の対角要素
            % 簡易的に: R_est ≈ innovation^2
            R_est = max(innov_sq, state.R_MIN);
            R_est = min(R_est, state.R_MAX);
            
            if count < state.warmup_samples
                % ウォームアップ期間: 累積平均
                state.(sum_field) = state.(sum_field) + R_est;
                state.(count_field) = count + 1;
                
                if state.(count_field) == state.warmup_samples
                    % ウォームアップ完了
                    state.(R_field) = state.(sum_field) / state.warmup_samples;
                end
            else
                % 通常期間: EMA更新
                alpha = state.alpha;
                R_new = alpha * R_est + (1 - alpha) * state.(R_field);
                state.(R_field) = R_new;
                state.(count_field) = count + 1;
            end
        end
        
        %% ノイズ行列の取得
        function R = get_R_matrix(state, sensor_type)
            % GET_R_MATRIX センサーノイズ行列を取得
            %
            % 入力:
            %   state       - ノイズ推定状態
            %   sensor_type - センサータイプ
            %
            % 出力:
            %   R - ノイズ共分散行列
            
            R_field = ['R_' sensor_type];
            R_vec = state.(R_field);
            
            if isscalar(R_vec)
                R = R_vec;
            else
                R = diag(R_vec);
            end
        end
        
        %% 手動設定
        function state = set_noise(state, sensor_type, R_value)
            % SET_NOISE ノイズ値を手動設定
            %
            % 入力:
            %   state       - ノイズ推定状態
            %   sensor_type - センサータイプ
            %   R_value     - ノイズ値 (分散)
            %
            % 出力:
            %   state - 更新後の状態
            
            R_field = ['R_' sensor_type];
            state.(R_field) = R_value;
        end
        
        %% リセット
        function state = reset(state, sensor_type)
            % RESET 特定センサーのノイズ推定をリセット
            %
            % 入力:
            %   state       - ノイズ推定状態
            %   sensor_type - センサータイプ (省略時は全センサー)
            %
            % 出力:
            %   state - リセット後の状態
            
            if nargin < 2
                % 全センサーリセット
                sensors = {'accel', 'gyro', 'mag', 'gps', 'baro'};
            else
                sensors = {sensor_type};
            end
            
            for i = 1:length(sensors)
                sensor = sensors{i};
                count_field = ['count_' sensor];
                sum_field = ['sum_' sensor];
                
                state.(count_field) = 0;
                state.(sum_field) = zeros(size(state.(sum_field)));
            end
        end
    end
end

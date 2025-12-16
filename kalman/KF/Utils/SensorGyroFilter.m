classdef SensorGyroFilter < handle
    % SENSORGYROFILTER  ジャイロ専用フィルタ
    %
    % 機能:
    %   - Biquad ローパスフィルタ（低遅延、明示的なカットオフ周波数）
    %   - ドリフト推定
    
    properties
        config              % フィルタ設定
        w_filtered          % フィルタ済み角速度
        w_filtered_axis     % 軸別フィルタ済み角速度
        bias_estimate       % バイアス推定
        noise_history       % ノイズ履歴
        biquad_filters      % Biquadフィルタ（3軸分）
    end
    
    methods
        function obj = SensorGyroFilter(config)
            % コンストラクタ
            obj.config = config;
            obj.w_filtered = [0; 0; 0];
            obj.w_filtered_axis = [0; 0; 0];
            obj.bias_estimate = [0; 0; 0];
            obj.noise_history = [];
            
            % Biquadフィルタを初期化（3軸分）
            % サンプリング周波数: 200Hz, カットオフ: 30Hz（ジャイロ用）
            sample_rate = 200;  % Hz
            cutoff_freq = 30;   % Hz（ナイキスト周波数100Hzの30%）
            
            obj.biquad_filters = cell(3, 1);
            for i = 1:3
                obj.biquad_filters{i} = BiquadFilter(sample_rate, cutoff_freq);
            end
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
            is_outlier = false;
            
            % まず MEX 実装を試す（存在すれば C++ Biquad を使用）
            % ただし環境変数 FORCE_MATLAB_FILTERS=1 が設定されている場合は強制的にMATLAB実装を使う
            force_matlab_env = getenv('FORCE_MATLAB_FILTERS');
            force_matlab = ~isempty(force_matlab_env) && strcmp(force_matlab_env, '1');
            use_mex = (exist('mex_sensor_filter','file') == 3) && ~force_matlab;
            if use_mex
                try
                    % mex_sensor_filter expects dt; derive from sample_rate used at construction
                    sample_rate = 200; % matches constructor
                    dt = 1.0 / sample_rate;
                    cutoff_freq = obj.biquad_filters{1}.cutoff_freq;
                    w_filt = mex_sensor_filter('gyro', w_meas, dt, cutoff_freq);
                    w_smooth = w_filt;
                    obj.w_filtered = w_smooth;
                    obj.w_filtered_axis = w_smooth;
                catch
                    % MEX failed — fallback to MATLAB per-axis Biquad
                    w_smooth = zeros(3, 1);
                    for i = 1:3
                        w_smooth(i) = obj.biquad_filters{i}.apply(w_meas(i));
                    end
                    obj.w_filtered = w_smooth;
                    obj.w_filtered_axis = w_smooth;
                end
            else
                % MEX が無ければ MATLAB 実装
                w_smooth = zeros(3, 1);
                for i = 1:3
                    w_smooth(i) = obj.biquad_filters{i}.apply(w_meas(i));
                end
                obj.w_filtered = w_smooth;
                obj.w_filtered_axis = w_smooth;
            end
            
            % 残差を計算（フィルタ後の値で）
            residual = w_smooth - w_expected;
            residual_norm = norm(residual);
            
            % ドリフト推定（静止時の偏り）
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

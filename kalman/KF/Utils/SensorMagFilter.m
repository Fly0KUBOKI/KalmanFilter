classdef SensorMagFilter < handle
    % SENSORMAGFILTER  磁気計専用フィルタ
    %
    % 機能:
    %   - ベクトル正規化
    %   - EMA平滑化（強い、α=0.2）
    %   - 地場ノルム検証
    %   - 外れ値検出
    
    properties
        config              % フィルタ設定
        m_filtered          % フィルタ済み磁気計測値
        noise_history       % ノイズ履歴
        is_initialized      % 初期化フラグ
    end
    
    methods
        function obj = SensorMagFilter(config)
            % コンストラクタ
            obj.config = config;
            obj.m_filtered = [1; 0; 0];  % デフォルトは北
            obj.noise_history = [];
            obj.is_initialized = false;
        end
        
        function [m_out, is_outlier, info] = apply(obj, m_meas)
            % APPLY  磁気計測値をフィルタリング
            %
            % 入力:
            %   m_meas - 計測磁気計測値 (3x1, nT)
            %
            % 出力:
            %   m_out      - フィルタ済み磁気計測値 (3x1)
            %   is_outlier - 外れ値判定フラグ
            %   info       - デバッグ情報
            
            info = struct();
            info.is_outlier = false;
            info.is_norm_mismatch = false;
            
            % ノルム検証
            m_norm = norm(m_meas);
            expected_norm = obj.config.mag_norm_expected;
            norm_tolerance = expected_norm * 0.3;  % ±30%
            
            if m_norm < (expected_norm - norm_tolerance) || ...
               m_norm > (expected_norm + norm_tolerance)
                % ノルムが大きく異なる（干渉など）
                is_outlier = true;
                m_out = obj.m_filtered;
                info.is_outlier = true;
                info.is_norm_mismatch = true;
                info.m_norm = m_norm;
                info.expected_norm = expected_norm;
                %fprintf('Mag Outlier: Norm mismatch. Norm=%.2f, Expected=%.2f\n', m_norm, expected_norm);
                return;
            end
            
            % 正規化
            if m_norm < 1e-9
                m_norm_meas = obj.m_filtered;  % 前回値を使用
            else
                m_norm_meas = m_meas / m_norm;
            end

            % 初回初期化
            if ~obj.is_initialized
                obj.m_filtered = m_norm_meas;
                obj.is_initialized = true;
                m_out = obj.m_filtered;
                is_outlier = false;
                return;
            end
            
            % 残差角度を計算（ベクトル間の角度）
            dot_product = dot(m_norm_meas, obj.m_filtered);
            dot_product = max(-1, min(1, dot_product));  % [-1, 1] にクリップ
            residual_angle = acos(dot_product);  % rad
            
            % ノイズレベル推定（角度）
            if isempty(obj.noise_history)
                noise_estimate = residual_angle;
            else
                noise_std = std(obj.noise_history);
                noise_estimate = max(noise_std, residual_angle / 3.0);
            end
            
            % 外れ値判定（3σ、ラジアン）
            % 高速回転時の誤検知を防ぐため、下限を緩和 (0.01 -> 0.1 rad approx 5.7 deg)
            is_outlier = (residual_angle > 3.0 * max(noise_estimate, 0.1));
            if is_outlier
                m_out = obj.m_filtered;
                info.is_outlier = true;
                info.residual_angle = residual_angle;
                %fprintf('Mag Outlier: Angle mismatch. Residual=%.4f rad, Threshold=%.4f rad\n', residual_angle, 3.0 * max(noise_estimate, 0.1));
                return;
            end
            
            % EMAフィルタを適用（ベクトル空間で）
            m_smooth = obj.config.ema_alpha * m_norm_meas + ...
                       (1 - obj.config.ema_alpha) * obj.m_filtered;
            
            % 正規化して保存
            m_smooth_norm = norm(m_smooth);
            if m_smooth_norm > 1e-9
                m_smooth = m_smooth / m_smooth_norm;
            end
            obj.m_filtered = m_smooth;
            
            % ノイズ履歴を更新
            obj.noise_history = [obj.noise_history; residual_angle];
            if length(obj.noise_history) > obj.config.history_size
                obj.noise_history = obj.noise_history(2:end);
            end
            
            m_out = m_smooth;
            info.residual_angle = residual_angle;
            info.noise_estimate = noise_estimate;
        end
        
        function noise_level = getNoiseLevel(obj)
            if isempty(obj.noise_history)
                noise_level = 0.01;
            else
                noise_level = std(obj.noise_history);
            end
        end
    end
end

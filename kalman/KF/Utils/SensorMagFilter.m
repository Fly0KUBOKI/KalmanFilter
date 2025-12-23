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
            % APPLY  磁気計測値をフィルタリング（MEX に委譲）
            % Delegate the core logic to SensorFilters.mag and keep
            % internal cached state for parity.

            info = struct();
            info.is_outlier = false;

            % Call SensorFilters.mag (MEX wrapper). Preserve behavior
            % when caller only requests a single output.
            if nargout >= 2
                [m_filt, is_outlier] = SensorFilters.mag(m_meas, obj.m_filtered);
            else
                m_filt = SensorFilters.mag(m_meas, obj.m_filtered);
                is_outlier = false;
            end

            % Update internal cached value when not an outlier
            if ~is_outlier
                obj.m_filtered = m_filt;
                obj.is_initialized = true;
            end

            m_out = m_filt;
            info.is_outlier = is_outlier;
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

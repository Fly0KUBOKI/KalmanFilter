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
            error('SensorMagFilter:disabled','SensorMagFilter.apply is disabled. Use SensorFilters.mag(m_meas,m_expected).');
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

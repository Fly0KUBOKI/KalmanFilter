classdef SensorFilterFactory < handle
    % SENSORFILTERFACTORY  センサーごとの統一フィルタファクトリー
    %
    % 各センサーの特性に応じた最適なフィルタを生成
    
    properties (Constant)
        % センサー種別
        SENSOR_ACCEL = 'accel'
        SENSOR_GYRO = 'gyro'
        SENSOR_MAG = 'mag'
        SENSOR_GPS = 'gps'
        SENSOR_BARO = 'baro'
    end
    
    methods (Static)
        function filter = createFilter(sensor_type, varargin)
            % CREATEFILTER  センサー種別に応じたフィルタを生成
            %
            % 使用例:
            %   filter = SensorFilterFactory.createFilter('accel');
            %   [data, is_outlier] = filter.apply(measurement);
            %
            % 入力:
            %   sensor_type - 'accel', 'gyro', 'mag', 'gps', 'baro'
            %   varargin    - センサー固有のパラメータ
            %
            % 出力:
            %   filter - SensorFilterインスタンス
            
            switch lower(sensor_type)
                case 'accel'
                    % 加速度計: 重力方向の推定が重要
                    % ノルムが9.81 m/s^2 付近であることを利用
                    filter = SensorFilter.createAccelFilter(varargin{:});
                    
                case 'gyro'
                    % ジャイロフィルタは廃止済み — フィルタを返さない
                    warning('SensorFilterFactory:gyroRemoved', 'Gyro filter implementation removed; returning empty.');
                    filter = [];
                    
                case 'mag'
                    % 磁気計: 方向ベクトル、ノルムは保存すべき
                    filter = SensorFilter.createMagFilter(varargin{:});
                    
                case 'gps'
                    % GPS: 水平・垂直で精度が異なる
                    filter = SensorFilter.createGPSFilter(varargin{:});
                    
                case 'baro'
                    % 気圧計: スカラー値、時間変化が遅い
                    filter = SensorFilter.createBaroFilter(varargin{:});
                    
                otherwise
                    error('Unknown sensor type: %s', sensor_type);
            end
        end
    end
end

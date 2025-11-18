classdef SensorModels
    % SENSORMODELS センサー観測モデル集
    % C++移行を想定した設計
    
    methods (Static)
        %% 加速度計モデル
        function [z_pred, H] = accel_model(q, g)
            % 加速度計の観測モデル
            % 入力:
            %   q: クォータニオン [4x1]
            %   g: 重力ベクトル (world frame) [3x1]
            % 出力:
            %   z_pred: 予測観測値 [3x1]
            %   H: 観測ヤコビ行列 [3x15] (誤差状態に対して)
            
            % 回転行列 (world -> body)
            R = QuaternionLib.to_rotation_matrix(q);
            R_body_to_world = R;
            R_world_to_body = R';
            
            % 予測観測値: 重力を機体座標系に変換
            z_pred = R_world_to_body * g;
            
            % ヤコビ行列 (誤差状態に対して)
            H = zeros(3, 15);
            % 姿勢誤差に対する偏微分
            H(:, 7:9) = -RotationLib.skew_symmetric(z_pred);
        end
        
        %% 磁気計モデル
        function [z_pred, H] = mag_model(q, m_world)
            % 磁気計の観測モデル
            % 入力:
            %   q: クォータニオン [4x1]
            %   m_world: 地磁気ベクトル (world frame) [3x1]
            % 出力:
            %   z_pred: 予測観測値 [3x1]
            %   H: 観測ヤコビ行列 [3x15]
            
            % 回転行列
            R = QuaternionLib.to_rotation_matrix(q);
            R_world_to_body = R';
            
            % 予測観測値: 地磁気を機体座標系に変換
            z_pred = R_world_to_body * m_world;
            
            % ヤコビ行列
            H = zeros(3, 15);
            H(:, 7:9) = -RotationLib.skew_symmetric(z_pred);
        end
        
        %% GPSモデル
        function [z_pred, H] = gps_model(p)
            % GPSの観測モデル（位置）
            % 入力:
            %   p: 位置 (ENU frame) [3x1]
            % 出力:
            %   z_pred: 予測観測値 [3x1]
            %   H: 観測ヤコビ行列 [3x15]
            
            % 線形モデル: z = p
            z_pred = p;
            
            % ヤコビ行列
            H = zeros(3, 15);
            H(1:3, 1:3) = eye(3);
        end
        
        %% 気圧計モデル
        function [z_pred, H] = baro_model(p, pressure_to_altitude)
            % 気圧計の観測モデル（高度）
            % 入力:
            %   p: 位置 [3x1]
            %   pressure_to_altitude: 気圧→高度変換関数 (オプション)
            % 出力:
            %   z_pred: 予測高度 [1x1]
            %   H: 観測ヤコビ行列 [1x15]
            
            if nargin < 2
                % 線形モデル: z = p(3) (高度)
                z_pred = p(3);
            else
                % 非線形モデル
                z_pred = pressure_to_altitude(p(3));
            end
            
            % ヤコビ行列
            H = zeros(1, 15);
            H(1, 3) = 1;  % 高度成分のみ
        end
        
        %% 速度計モデル（GPS速度など）
        function [z_pred, H] = velocity_model(v)
            % 速度観測モデル
            % 入力:
            %   v: 速度 [3x1]
            % 出力:
            %   z_pred: 予測観測値 [3x1]
            %   H: 観測ヤコビ行列 [3x15]
            
            z_pred = v;
            
            H = zeros(3, 15);
            H(1:3, 4:6) = eye(3);
        end
        
        %% 姿勢観測モデル（視覚オドメトリなど）
        function [z_pred, H] = attitude_model(q)
            % 姿勢観測モデル
            % 入力:
            %   q: クォータニオン [4x1]
            % 出力:
            %   z_pred: 予測姿勢 (オイラー角) [3x1] (度)
            %   H: 観測ヤコビ行列 [3x15]
            
            z_pred = QuaternionLib.to_euler(q);
            
            % 簡易ヤコビ（小角度近似）
            H = zeros(3, 15);
            H(1:3, 7:9) = eye(3);
        end
        
        %% 高度と気圧の変換
        function altitude = pressure_to_altitude_standard(pressure_pa)
            % 標準大気モデルによる気圧→高度変換
            % 入力: pressure_pa - 気圧 (Pa)
            % 出力: altitude - 高度 (m)
            
            P0 = 101325;  % 海面気圧 (Pa)
            altitude = 44330 * (1 - (pressure_pa / P0)^0.1903);
        end
        
        function pressure = altitude_to_pressure_standard(altitude)
            % 標準大気モデルによる高度→気圧変換
            % 入力: altitude - 高度 (m)
            % 出力: pressure - 気圧 (Pa)
            
            P0 = 101325;
            pressure = P0 * (1 - altitude / 44330)^5.255;
        end
    end
end

classdef GSF_YawEstimator < handle
    % GSF_YAWESTI MATOR  ArduPilot風のGaussian Sum Filterによるヨー推定
    %
    % ArduPilot NavEKF2/3のEKFGSF_yaw実装を参考
    % 複数のEKFモデルをガウス混合で統合し、ロバストなヨー推定を実現
    %
    % 参考: https://github.com/ArduPilot/ardupilot/libraries/AP_NavEKF/EKFGSF_yaw.cpp
    
    properties
        N_MODELS = 5;           % EKFモデル数（ArduPilotと同じ）
        
        % 各EKFモデルの状態
        % X: [vx; vy; yaw] (3x1)
        % P: 共分散行列 (3x3)
        % weight: モデル重み
        EKF_models
        
        % GSFパラメータ
        vel_innovation_test_ratio = 1.0;  % 速度イノベーション検定比
        yaw_variance_max = deg2rad(45)^2; % ヨー分散の上限
        
        % AHRS（姿勢推定）補完フィルタ
        AHRS_models  % 各モデルの回転行列 R (3x3)
        
        % 初期化フラグ
        initialized = false;
        yaw_aligned = false;
        
        % 統計
        update_count = 0;
    end
    
    methods
        function obj = GSF_YawEstimator()
            % コンストラクタ
            obj.reset();
        end
        
        function reset(obj)
            % GSFのリセット
            obj.EKF_models = cell(obj.N_MODELS, 1);
            obj.AHRS_models = cell(obj.N_MODELS, 1);
            
            % 各モデルを異なるヨー初期値で初期化
            yaw_span = deg2rad(180);  % ±90度の範囲
            for i = 1:obj.N_MODELS
                % ヨー初期値を等間隔に配置
                yaw_init = (i - (obj.N_MODELS + 1) / 2) * yaw_span / obj.N_MODELS;
                
                % EKF状態の初期化
                obj.EKF_models{i}.X = [0; 0; yaw_init];
                obj.EKF_models{i}.P = diag([1.0, 1.0, deg2rad(45)^2]);
                obj.EKF_models{i}.weight = 1.0 / obj.N_MODELS;
                
                % AHRS回転行列の初期化（ロール=0、ピッチ=0、ヨー=yaw_init）
                obj.AHRS_models{i}.R = eul2rotm([yaw_init, 0, 0], 'ZXY');
            end
            
            obj.initialized = true;
            obj.yaw_aligned = false;
            obj.update_count = 0;
        end
        
        function update(obj, del_ang, del_vel, dt)
            % 予測ステップと姿勢推定の更新
            %
            % 入力:
            %   del_ang - 角度増分 [roll; pitch; yaw] (rad)
            %   del_vel - 速度増分 [vx; vy; vz] (m/s)
            %   dt      - 時間刻み (s)
            
            if ~obj.initialized
                obj.reset();
            end
            
            obj.update_count = obj.update_count + 1;
            
            % 各モデルでAHRS更新とEKF予測
            for i = 1:obj.N_MODELS
                % AHRS更新（補完フィルタによる姿勢推定）
                obj.predictAHRS(i, del_ang, del_vel, dt);
                
                % EKF予測（速度とヨーの予測）
                obj.predictEKF(i, del_vel, dt);
            end
        end
        
        function fuseVelocity(obj, vel_meas, vel_var)
            % 速度測定値の融合
            %
            % 入力:
            %   vel_meas - 測定速度 [vx; vy] (m/s)
            %   vel_var  - 測定分散 (m^2/s^2)
            
            if ~obj.initialized
                return;
            end
            
            % 各モデルで速度更新
            for i = 1:obj.N_MODELS
                success = obj.correctEKF(i, vel_meas, vel_var);
                
                if ~success
                    % 更新失敗時は重みを減らす
                    obj.EKF_models{i}.weight = obj.EKF_models{i}.weight * 0.5;
                end
            end
            
            % 重みの正規化
            total_weight = 0;
            for i = 1:obj.N_MODELS
                total_weight = total_weight + obj.EKF_models{i}.weight;
            end
            
            if total_weight > 1e-6
                for i = 1:obj.N_MODELS
                    obj.EKF_models{i}.weight = obj.EKF_models{i}.weight / total_weight;
                end
            else
                % 全モデルが失敗した場合はリセット
                obj.reset();
            end
        end
        
        function [yaw, yaw_var, valid] = getYaw(obj)
            % 統合されたヨー推定値を取得
            %
            % 出力:
            %   yaw     - 推定ヨー角 (rad)
            %   yaw_var - ヨー分散 (rad^2)
            %   valid   - 推定が有効かどうか
            
            if ~obj.initialized || obj.update_count < 10
                yaw = 0;
                yaw_var = deg2rad(90)^2;
                valid = false;
                return;
            end
            
            % 重み付き平均
            yaw_sum = 0;
            yaw_var_sum = 0;
            
            for i = 1:obj.N_MODELS
                w = obj.EKF_models{i}.weight;
                yaw_i = obj.EKF_models{i}.X(3);
                var_i = obj.EKF_models{i}.P(3,3);
                
                yaw_sum = yaw_sum + w * yaw_i;
                yaw_var_sum = yaw_var_sum + w * (var_i + yaw_i^2);
            end
            
            yaw = yaw_sum;
            yaw_var = yaw_var_sum - yaw^2;
            
            % 分散が妥当な範囲かチェック
            valid = (yaw_var > 0) && (yaw_var < obj.yaw_variance_max);
        end
    end
    
    methods (Access = private)
        function predictAHRS(obj, model_idx, del_ang, del_vel, dt)
            % AHRS補完フィルタの予測
            % 簡略版：ジャイロ積分のみ
            
            R = obj.AHRS_models{model_idx}.R;
            
            % 小角度近似で回転行列を更新
            omega = del_ang / dt;
            skew_omega = [0, -omega(3), omega(2);
                         omega(3), 0, -omega(1);
                         -omega(2), omega(1), 0];
            
            R_new = R * (eye(3) + skew_omega * dt);
            
            % 直交性を保つために再正規化（簡略版）
            obj.AHRS_models{model_idx}.R = obj.orthonormalize(R_new);
        end
        
        function predictEKF(obj, model_idx, del_vel, dt)
            % EKF予測ステップ
            % 状態: [vx; vy; yaw]
            
            X = obj.EKF_models{model_idx}.X;
            P = obj.EKF_models{model_idx}.P;
            R_ahrs = obj.AHRS_models{model_idx}.R;
            
            % AHRSからヨー角を抽出
            yaw = atan2(R_ahrs(2,1), R_ahrs(1,1));
            X(3) = yaw;
            
            % 速度の予測（世界座標系での速度増分）
            dv_body = del_vel(1:2);  % [dvx; dvy] in body frame
            dv_world = [cos(yaw), -sin(yaw);
                       sin(yaw),  cos(yaw)] * dv_body;
            
            X(1:2) = X(1:2) + dv_world;
            
            % 共分散の予測（簡略版）
            % プロセスノイズ
            Q = diag([0.1 * dt, 0.1 * dt, deg2rad(0.1)^2 * dt]);
            
            % F = I + F_c * dt
            F = eye(3);
            % 速度-ヨーのカップリング（簡略化のため省略）
            
            P = F * P * F' + Q;
            
            % 対称化
            P = 0.5 * (P + P');
            
            % 更新
            obj.EKF_models{model_idx}.X = X;
            obj.EKF_models{model_idx}.P = P;
        end
        
        function success = correctEKF(obj, model_idx, vel_meas, vel_var)
            % EKF更新ステップ
            
            X = obj.EKF_models{model_idx}.X;
            P = obj.EKF_models{model_idx}.P;
            
            % 観測行列 H = [I_2, 0]
            H = [eye(2), zeros(2,1)];
            
            % イノベーション
            z = vel_meas;
            h = X(1:2);
            y = z - h;
            
            % イノベーション共分散
            R = eye(2) * vel_var;
            S = H * P * H' + R;
            
            % カルマンゲイン
            try
                K = P * H' / S;
            catch
                success = false;
                return;
            end
            
            % 状態更新
            X = X + K * y;
            P = (eye(3) - K * H) * P;
            
            % ガウス尤度でモデル重みを更新
            % log-likelihood = -0.5 * (y' * inv(S) * y + log(det(S)))
            try
                log_likelihood = -0.5 * (y' / S * y + log(det(S) + 1e-12));
                obj.EKF_models{model_idx}.weight = obj.EKF_models{model_idx}.weight * exp(log_likelihood);
            catch
                % 失敗時は重みを減らす
                obj.EKF_models{model_idx}.weight = obj.EKF_models{model_idx}.weight * 0.1;
            end
            
            % 更新
            obj.EKF_models{model_idx}.X = X;
            obj.EKF_models{model_idx}.P = P;
            
            success = true;
        end
        
        function R_ortho = orthonormalize(~, R)
            % 回転行列の直交正規化（Gram-Schmidt）
            v1 = R(:,1);
            v1 = v1 / norm(v1);
            
            v2 = R(:,2);
            v2 = v2 - (v1' * v2) * v1;
            v2 = v2 / norm(v2);
            
            v3 = cross(v1, v2);
            
            R_ortho = [v1, v2, v3];
        end
    end
end

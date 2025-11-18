classdef ESKFCovariancePrediction
    % ESKFCOVARIANCEPREDICTION ESKF共分散予測関数集
    % C++移行を想定した設計
    
    methods (Static)
        function P_new = predict(P, q, a_meas, ba, w_meas, bg, Q, dt)
            % ESKF共分散予測
            % 入力:
            %   P: 現在の共分散行列 [15x15]
            %   q: 現在のクォータニオン [4x1]
            %   a_meas: 加速度測定値 [3x1]
            %   ba: 加速度バイアス [3x1]
            %   w_meas: 角速度測定値 [3x1]
            %   bg: 角速度バイアス [3x1]
            %   Q: プロセスノイズ共分散 [15x15]
            %   dt: 時間刻み (s)
            % 出力:
            %   P_new: 予測された共分散行列 [15x15]
            
            % 状態遷移行列Fの構築
            F = ESKFCovariancePrediction.compute_state_transition_matrix(...
                q, a_meas, ba, w_meas, bg, dt);
            
            % 共分散予測: P = F*P*F' + Q
            P_new = F * P * F' + Q;
            
            % 対称性の強制
            P_new = CovarianceRegularizer.enforce_symmetry(P_new);
            
            % 正定値性の保証
            min_var = 1e-9;
            for i = 1:15
                if P_new(i,i) < min_var
                    P_new(i,i) = min_var;
                end
            end
        end
        
        function F = compute_state_transition_matrix(q, a_meas, ba, w_meas, bg, dt)
            % 状態遷移行列Fの計算
            % F = I + F_continuous * dt
            
            F = eye(15);
            
            % 位置-速度カップリング
            F(1:3, 4:6) = eye(3) * dt;
            
            % 速度-姿勢カップリング
            R = QuaternionLib.to_rotation_matrix(q);
            a_corrected = a_meas - ba;
            skew_a = RotationLib.skew_symmetric(a_corrected);
            F(4:6, 7:9) = -R * skew_a * dt;
            
            % 速度-加速度バイアスカップリング
            F(4:6, 10:12) = -R * dt;
            
            % 姿勢-角速度バイアスカップリング
            F(7:9, 13:15) = -eye(3) * dt;
        end
        
        function Q_discrete = discretize_process_noise(Q_continuous, dt, R)
            % プロセスノイズの離散化
            % 入力:
            %   Q_continuous: 連続時間プロセスノイズ
            %   dt: 時間刻み
            %   R: 回転行列 (body to world)
            % 出力:
            %   Q_discrete: 離散化されたプロセスノイズ
            
            % 簡易版: Q_discrete = Q_continuous + 数値安定性
            Q_discrete = Q_continuous + eye(size(Q_continuous)) * 1e-12;
            
            % より正確な離散化が必要な場合:
            % G行列の構築
            G = zeros(15, 12);
            G(4:6, 1:3) = R * dt;           % 加速度ノイズ
            G(7:9, 4:6) = eye(3) * dt;      % 角速度ノイズ
            G(10:12, 7:9) = eye(3) * dt;    % 加速度バイアスランダムウォーク
            G(13:15, 10:12) = eye(3) * dt;  % 角速度バイアスランダムウォーク
            
            % Q_c: 連続時間ノイズ密度（12x12）
            Q_c = diag([
                Q_continuous(4,4), Q_continuous(5,5), Q_continuous(6,6),   % accel
                Q_continuous(7,7), Q_continuous(8,8), Q_continuous(9,9),   % gyro
                Q_continuous(10,10), Q_continuous(11,11), Q_continuous(12,12),  % ba
                Q_continuous(13,13), Q_continuous(14,14), Q_continuous(15,15)   % bg
            ]);
            
            % Q_discrete = G * Q_c * G'
            % ただし、簡易版として元のQを使用
        end
    end
end

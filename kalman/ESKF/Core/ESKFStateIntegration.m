classdef ESKFStateIntegration
    % ESKFSTATEINTEGRATION ESKF固有の状態積分関数集
    % C++移行を想定した設計
    
    methods (Static)
        function nominal = integrate_nominal_state(nominal, a_meas, w_meas, dt, g, gyro_threshold, accel_threshold)
            % ノミナル状態の積分（位置・速度・姿勢・バイアス）
            % 入力:
            %   nominal: struct with .p, .v, .q, .ba, .bg
            %   a_meas: 加速度測定値 [3x1] (m/s^2)
            %   w_meas: 角速度測定値 [3x1] (rad/s)
            %   dt: 時間刻み (s)
            %   g: 重力ベクトル [3x1] (m/s^2)
            %   gyro_threshold: ジャイロ閾値 [3x1] (rad)
            %   accel_threshold: 加速度閾値 [3x1] (m/s^2)
            % 出力:
            %   nominal: 更新された状態
            
            % バイアス補正
            a = a_meas - nominal.ba;
            w = w_meas - nominal.bg;
            
            % 加速度の閾値処理
            for i = 1:3
                if abs(a(i)) < accel_threshold(i) * dt
                    a(i) = 0;
                end
            end
            
            % 姿勢積分
            nominal.q = ESKFStateIntegration.integrate_attitude(nominal.q, w, dt);
            
            % 回転行列
            R = QuaternionLib.to_rotation_matrix(nominal.q);
            a_world = R * a;
            
            % 速度・位置積分（Adams-Bashforth 2）
            [nominal.p, nominal.v] = ESKFStateIntegration.integrate_position_velocity(...
                nominal.p, nominal.v, a_world, g, dt);
        end
        
        function q_new = integrate_attitude(q, omega, dt)
            % 姿勢（クォータニオン）積分
            % 入力:
            %   q: 現在のクォータニオン [4x1]
            %   omega: 角速度 [3x1] (rad/s)
            %   dt: 時間刻み (s)
            % 出力:
            %   q_new: 更新されたクォータニオン
            
            q_new = QuaternionLib.integrate(q, omega, dt);
        end
        
        function [p_new, v_new] = integrate_position_velocity(p, v, a_world, g, dt)
            % 位置・速度の積分（Adams-Bashforth 2）
            % persistent変数で前ステップの値を保持
            
            persistent prev_a_world prev_v prev_initialized
            
            if isempty(prev_initialized)
                % 初回ステップ: Forward Euler
                v_prev = v;
                v_candidate = v + (a_world + g) * dt;
                
                % 速度発散防止
                max_accel = 2.0;
                dv = v_candidate - v;
                dv_norm = norm(dv);
                max_dv = max_accel * dt;
                if dv_norm > max_dv
                    dv = dv * (max_dv / dv_norm);
                end
                v_new = v + dv;
                
                % 速度クリップ
                max_velocity = 50;
                v_new = max(min(v_new, max_velocity), -max_velocity);
                
                % 位置更新
                p_new = p + v * dt + 0.5 * (a_world + g) * dt * dt;
                
                prev_a_world = a_world;
                prev_v = v_prev;
                prev_initialized = true;
            else
                % Adams-Bashforth 2
                a_prev = prev_a_world;
                v_old = v;
                v_candidate = v + dt * (1.5 * (a_world + g) - 0.5 * (a_prev + g));
                
                % 速度発散防止
                max_accel = 2.0;
                dv = v_candidate - v;
                dv_norm = norm(dv);
                max_dv = max_accel * dt;
                if dv_norm > max_dv
                    dv = dv * (max_dv / dv_norm);
                end
                v_new = v + dv;
                
                % 速度クリップ
                max_velocity = 50;
                v_new = max(min(v_new, max_velocity), -max_velocity);
                
                % 位置更新（AB2）
                p_new = p + dt * (1.5 * v_old - 0.5 * prev_v);
                
                prev_v = v_old;
                prev_a_world = a_world;
            end
        end
        
        function nominal = reset_integration_state()
            % 積分状態のリセット（persistentクリア用）
            persistent prev_a_world prev_v prev_initialized %#ok<REDEF>
            prev_a_world = [];
            prev_v = [];
            prev_initialized = [];
            nominal = [];
        end
    end
end

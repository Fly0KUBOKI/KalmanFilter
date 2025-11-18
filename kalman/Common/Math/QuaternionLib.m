classdef QuaternionLib
    % クォータニオン演算ライブラリ
    
    properties (Constant)
        EPS = 1.0e-9;  % 小さい値の閾値
    end
    
    methods (Static)
        function q_out = multiply(q1, q2)
            % クォータニオン積
            a = q1(:); b = q2(:);
            
            if numel(a) ~= 4 || numel(b) ~= 4
                error('QuaternionLib:multiply', 'Input must be 4-element quaternions');
            end
            if any(~isfinite(a)) || any(~isfinite(b))
                error('QuaternionLib:multiply', 'Non-finite quaternion detected');
            end
            
            aw = a(1); av = a(2:4);
            bw = b(1); bv = b(2:4);
            
            s = aw*bw - av'*bv;
            v = aw*bv + bw*av + cross(av, bv);
            q_out = [s; v];
        end
        
        function q_out = conjugate(q)
            % 共役
            q = q(:);
            q_out = [q(1); -q(2:4)];
        end
        
        function q_out = normalize(q)
            % 正規化
            q = q(:);
            
            if any(~isfinite(q))
                warning('QuaternionLib:normalize', 'Non-finite input, resetting to identity');
                q_out = [1; 0; 0; 0];
                return;
            end
            
            n = norm(q);
            if n < QuaternionLib.EPS
                q_out = [1; 0; 0; 0];
            else
                q_out = q / n;
                q_out(abs(q_out) < QuaternionLib.EPS) = 0;
            end
        end
        
        function q_out = inverse(q)
            % 逆元
            q = q(:);
            q_conj = QuaternionLib.conjugate(q);
            norm_sq = q' * q;
            q_out = q_conj / norm_sq;
        end
        
        function R = to_rotation_matrix(q)
            % 回転行列へ変換
            q = q(:);
            q = QuaternionLib.normalize(q);
            
            qw = q(1); qx = q(2); qy = q(3); qz = q(4);
            
            R = [1-2*(qy^2+qz^2), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw);
                 2*(qx*qy+qz*qw), 1-2*(qx^2+qz^2),   2*(qy*qz-qx*qw);
                 2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw),   1-2*(qx^2+qy^2)];
            
            % 小さい値をゼロに、1に近い値を±1に
            EPS = QuaternionLib.EPS;
            for i = 1:3
                for j = 1:3
                    if abs(R(i,j)) < EPS
                        R(i,j) = 0;
                    elseif abs(R(i,j)-1) < EPS
                        R(i,j) = sign(R(i,j));
                    end
                end
            end
        end
        
        function euler = to_euler(q, sequence)
            % オイラー角へ変換 (度)
            if nargin < 2
                sequence = 'ZYX';
            end
            
            q = q(:);
            q = QuaternionLib.normalize(q);
            
            qw = q(1); qx = q(2); qy = q(3); qz = q(4);
            
            if strcmp(sequence, 'ZYX')
                % Roll (X軸回転)
                sinr_cosp = 2 * (qw * qx + qy * qz);
                cosr_cosp = 1 - 2 * (qx^2 + qy^2);
                roll = atan2(sinr_cosp, cosr_cosp);
                
                % Pitch (Y軸回転)
                sinp = 2 * (qw * qy - qz * qx);
                if abs(sinp) >= 1
                    pitch = sign(sinp) * pi / 2;
                else
                    pitch = asin(sinp);
                end
                
                % Yaw (Z軸回転)
                siny_cosp = 2 * (qw * qz + qx * qy);
                cosy_cosp = 1 - 2 * (qy^2 + qz^2);
                yaw = atan2(siny_cosp, cosy_cosp);
            else
                error('QuaternionLib:to_euler', 'Unsupported sequence: %s', sequence);
            end
            
            euler = rad2deg([roll; pitch; yaw]);
        end
        
        function q = from_euler(euler, sequence)
            % オイラー角から変換
            if nargin < 2
                sequence = 'ZYX';
            end
            
            e = euler(:);
            if numel(e) ~= 3
                error('QuaternionLib:from_euler', 'Input must be 3-element vector');
            end
            
            e_rad = deg2rad(e);
            roll = e_rad(1); pitch = e_rad(2); yaw = e_rad(3);
            
            if strcmp(sequence, 'ZYX')
                cy = cos(yaw * 0.5); sy = sin(yaw * 0.5);
                cp = cos(pitch * 0.5); sp = sin(pitch * 0.5);
                cr = cos(roll * 0.5); sr = sin(roll * 0.5);
                
                qw = cr * cp * cy + sr * sp * sy;
                qx = sr * cp * cy - cr * sp * sy;
                qy = cr * sp * cy + sr * cp * sy;
                qz = cr * cp * sy - sr * sp * cy;
            else
                error('QuaternionLib:from_euler', 'Unsupported sequence: %s', sequence);
            end
            
            q = QuaternionLib.normalize([qw; qx; qy; qz]);
        end
        
        function q_new = integrate(q, omega, dt)
            % 積分
            q = q(:);
            omega = omega(:);
            
            w_dt = omega * dt;
            w_dt_norm = norm(w_dt);
            
            threshold = 1e-15;
            if w_dt_norm > threshold
                half_angle = w_dt_norm / 2;
                if half_angle > 1e-6
                    sin_half = sin(half_angle);
                    cos_half = cos(half_angle);
                    w_unit = w_dt / w_dt_norm;
                    delta_q = [cos_half; w_unit * sin_half];
                else
                    % Taylor展開
                    w_norm_sq = w_dt_norm * w_dt_norm;
                    delta_q = [1.0 - w_norm_sq/8.0; w_dt * 0.5 * (1.0 - w_norm_sq/24.0)];
                end
                
                q_new = QuaternionLib.multiply(q, delta_q);
                q_new = QuaternionLib.normalize(q_new);
            else
                q_new = q;
            end
        end
        
        function q = small_angle_quat(theta)
            % 小角度回転ベクトルから変換
            th = theta(:);
            th2 = th' * th;
            EPS = QuaternionLib.EPS;
            
            if th2 < EPS^2
                q = [1; 0.5*th];
            else
                angle = sqrt(th2);
                axis = th / angle;
                q = [cos(angle/2); axis*sin(angle/2)];
            end
            
            q = QuaternionLib.normalize(q);
        end
        
        function d = distance(q1, q2)
            % 角度距離計算
            q1 = QuaternionLib.normalize(q1(:));
            q2 = QuaternionLib.normalize(q2(:));
            
            dot_product = abs(q1' * q2);
            dot_product = min(dot_product, 1.0);
            
            d = 2 * acos(dot_product);
        end
        
        function q = slerp(q1, q2, t)
            % 球面線形補間
            q1 = QuaternionLib.normalize(q1(:));
            q2 = QuaternionLib.normalize(q2(:));
            
            dot_product = q1' * q2;
            
            % 最短経路を選択
            if dot_product < 0
                q2 = -q2;
                dot_product = -dot_product;
            end
            
            dot_product = min(dot_product, 1.0);
            theta = acos(dot_product);
            
            if abs(theta) < QuaternionLib.EPS
                q = q1;
            else
                q = (sin((1-t)*theta) / sin(theta)) * q1 + ...
                    (sin(t*theta) / sin(theta)) * q2;
                q = QuaternionLib.normalize(q);
            end
        end
        
        function q = from_two_vectors(v1, v2)
            % 2ベクトル間の回転生成
            v1 = v1(:); v2 = v2(:);
            
            n1 = norm(v1); n2 = norm(v2);
            EPS = QuaternionLib.EPS;
            
            if n1 < EPS
                v1 = [1; 0; 0];
            else
                v1 = v1 / n1;
            end
            
            if n2 < EPS
                v2 = [1; 0; 0];
            else
                v2 = v2 / n2;
            end
            
            dot_product = dot(v1, v2);
            cross_product = cross(v1, v2);
            
            qw = 1 + dot_product;
            qxyz = cross_product;
            
            q = QuaternionLib.normalize([qw; qxyz]);
        end
    end
end

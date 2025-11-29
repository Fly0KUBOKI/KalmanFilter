classdef KalmanCompute
    % 状態非依存のカルマンフィルタ計算ライブラリ
    % 
    % C++実装が利用可能な場合は自動的にMEXを使用
    % すべての関数は純粋な計算関数で、状態管理はMATLAB側で行う
    %
    % 設計方針:
    % - 関数インターフェース: output = compute_xxx(input)
    % - input: 計算に必要なすべてのデータを含む行列/ベクトル
    % - output: 計算結果の行列/ベクトル
    % - float型を基本使用
    
    properties (Constant)
        MEX_FUNCTION = 'mex_kalman_compute';
    end
    
    methods (Static)
        
        % ===== MEX利用可能性チェック =====
        
        function available = check_mex_available()
            % MEX実装の利用可能性をチェック
            persistent mex_available;
            if isempty(mex_available)
                mex_available = exist(KalmanCompute.MEX_FUNCTION, 'file') == 3;
                if mex_available
                    fprintf('[KalmanCompute] MEX available: %s\n', ...
                        KalmanCompute.MEX_FUNCTION);
                else
                    warning('[KalmanCompute] MEX not available, using MATLAB fallback');
                end
            end
            available = mex_available;
        end
        
        % ===== Quaternion計算 =====
        
        function q_out = quat_multiply(q1, q2)
            % クォータニオン積
            % input: q1(4x1), q2(4x1)
            % output: q_out(4x1)
            
            input = [q1(:); q2(:)];  % 8x1
            
            if KalmanCompute.check_mex_available()
                try
                    q_out = mex_kalman_compute('quat_multiply', single(input));
                    return;
                catch ME
                    warning('MEX failed, using MATLAB: %s', ME.message);
                end
            end
            
            % MATLAB fallback
            q_out = KalmanCompute.quat_multiply_matlab(q1, q2);
        end
        
        function q_out = quat_normalize(q)
            % クォータニオン正規化
            % input: q(4x1)
            % output: q_out(4x1)
            
            if KalmanCompute.check_mex_available()
                try
                    q_out = mex_kalman_compute('quat_normalize', single(q(:)));
                    return;
                catch ME
                    warning('MEX failed, using MATLAB: %s', ME.message);
                end
            end
            
            % MATLAB fallback
            q_out = KalmanCompute.quat_normalize_matlab(q);
        end
        
        function q_out = quat_conjugate(q)
            % クォータニオン共役
            % input: q(4x1)
            % output: q_out(4x1)
            
            if KalmanCompute.check_mex_available()
                try
                    q_out = mex_kalman_compute('quat_conjugate', single(q(:)));
                    return;
                catch ME
                    warning('MEX failed, using MATLAB: %s', ME.message);
                end
            end
            
            % MATLAB fallback
            q_out = [q(1); -q(2:4)];
        end
        
        function q_out = quat_inverse(q)
            % クォータニオン逆元
            % input: q(4x1)
            % output: q_out(4x1)
            
            if KalmanCompute.check_mex_available()
                try
                    q_out = mex_kalman_compute('quat_inverse', single(q(:)));
                    return;
                catch ME
                    warning('MEX failed, using MATLAB: %s', ME.message);
                end
            end
            
            % MATLAB fallback
            q_conj = KalmanCompute.quat_conjugate(q);
            norm_sq = q' * q;
            q_out = q_conj / norm_sq;
        end
        
        function R = quat_to_rotation_matrix(q)
            % クォータニオン→回転行列
            % input: q(4x1)
            % output: R(3x3)
            
            if KalmanCompute.check_mex_available()
                try
                    R = mex_kalman_compute('quat_to_rotation_matrix', single(q(:)));
                    return;
                catch ME
                    warning('MEX failed, using MATLAB: %s', ME.message);
                end
            end
            
            % MATLAB fallback
            R = KalmanCompute.quat_to_rotation_matrix_matlab(q);
        end
        
        function euler = quat_to_euler(q)
            % クォータニオン→オイラー角 (度)
            % input: q(4x1)
            % output: euler(3x1) - [roll, pitch, yaw] in degrees
            
            if KalmanCompute.check_mex_available()
                try
                    euler = mex_kalman_compute('quat_to_euler', single(q(:)));
                    return;
                catch ME
                    warning('MEX failed, using MATLAB: %s', ME.message);
                end
            end
            
            % MATLAB fallback
            euler = KalmanCompute.quat_to_euler_matlab(q);
        end
        
        function q = quat_from_euler(euler)
            % オイラー角→クォータニオン (度)
            % input: euler(3x1) - [roll, pitch, yaw] in degrees
            % output: q(4x1)
            
            if KalmanCompute.check_mex_available()
                try
                    q = mex_kalman_compute('quat_from_euler', single(euler(:)));
                    return;
                catch ME
                    warning('MEX failed, using MATLAB: %s', ME.message);
                end
            end
            
            % MATLAB fallback
            q = KalmanCompute.quat_from_euler_matlab(euler);
        end
        
        function q = quat_from_small_angle(theta)
            % 小角度回転ベクトル→クォータニオン
            % input: theta(3x1) - [theta_x, theta_y, theta_z] in radians
            % output: q(4x1)
            
            if KalmanCompute.check_mex_available()
                try
                    q = mex_kalman_compute('quat_from_small_angle', single(theta(:)));
                    return;
                catch ME
                    warning('MEX failed, using MATLAB: %s', ME.message);
                end
            end
            
            % MATLAB fallback
            q = KalmanCompute.quat_from_small_angle_matlab(theta);
        end
        
        function q_new = quat_integrate(q, omega, dt)
            % 角速度積分によるクォータニオン更新
            % input: q(4x1), omega(3x1), dt(scalar)
            % output: q_new(4x1)
            
            input = [q(:); omega(:); dt];  % 8x1
            
            if KalmanCompute.check_mex_available()
                try
                    q_new = mex_kalman_compute('quat_integrate', single(input));
                    return;
                catch ME
                    warning('MEX failed, using MATLAB: %s', ME.message);
                end
            end
            
            % MATLAB fallback
            q_new = KalmanCompute.quat_integrate_matlab(q, omega, dt);
        end
        
        % ===== Rotation計算 =====
        
        function skew = rot_skew_symmetric(v)
            % 歪対称行列生成
            % input: v(3x1)
            % output: skew(3x3)
            
            if KalmanCompute.check_mex_available()
                try
                    skew = mex_kalman_compute('rot_skew_symmetric', single(v(:)));
                    return;
                catch ME
                    warning('MEX failed, using MATLAB: %s', ME.message);
                end
            end
            
            % MATLAB fallback
            vx = v(1); vy = v(2); vz = v(3);
            skew = [  0, -vz,  vy;
                      vz,   0, -vx;
                     -vy,  vx,   0];
        end
        
        function R = rot_from_euler(euler)
            % オイラー角→回転行列 (度)
            % input: euler(3x1) - [roll, pitch, yaw] in degrees
            % output: R(3x3)
            
            if KalmanCompute.check_mex_available()
                try
                    R = mex_kalman_compute('rot_from_euler', single(euler(:)));
                    return;
                catch ME
                    warning('MEX failed, using MATLAB: %s', ME.message);
                end
            end
            
            % MATLAB fallback
            R = KalmanCompute.rot_from_euler_matlab(euler);
        end
        
        function euler = rot_to_euler(R)
            % 回転行列→オイラー角 (度)
            % input: R(3x3)
            % output: euler(3x1) - [roll, pitch, yaw] in degrees
            
            if KalmanCompute.check_mex_available()
                try
                    euler = mex_kalman_compute('rot_to_euler', single(R(:)));
                    return;
                catch ME
                    warning('MEX failed, using MATLAB: %s', ME.message);
                end
            end
            
            % MATLAB fallback
            euler = KalmanCompute.rot_to_euler_matlab(R);
        end
        
        function R_ortho = rot_orthonormalize(R)
            % 回転行列の正規直交化
            % input: R(3x3)
            % output: R_ortho(3x3)
            
            if KalmanCompute.check_mex_available()
                try
                    R_ortho = mex_kalman_compute('rot_orthonormalize', single(R(:)));
                    return;
                catch ME
                    warning('MEX failed, using MATLAB: %s', ME.message);
                end
            end
            
            % MATLAB fallback
            R_ortho = KalmanCompute.rot_orthonormalize_matlab(R);
        end
        
        function v_rot = rot_apply_rotation(R, v)
            % ベクトルに回転を適用
            % input: R(3x3), v(3x1)
            % output: v_rot(3x1) = R * v
            
            input = [R(:); v(:)];  % 12x1
            
            if KalmanCompute.check_mex_available()
                try
                    v_rot = mex_kalman_compute('rot_apply_rotation', single(input));
                    return;
                catch ME
                    warning('MEX failed, using MATLAB: %s', ME.message);
                end
            end
            
            % MATLAB fallback
            v_rot = R * v;
        end
        
    end
    
    % ===== Private MATLAB Fallback実装 =====
    
    methods (Static, Access = private)
        
        function q_out = quat_multiply_matlab(q1, q2)
            a = q1(:); b = q2(:);
            aw = a(1); av = a(2:4);
            bw = b(1); bv = b(2:4);
            s = aw*bw - av'*bv;
            v = aw*bv + bw*av + cross(av, bv);
            q_out = [s; v];
        end
        
        function q_out = quat_normalize_matlab(q)
            q = q(:);
            n = norm(q);
            if n < 1e-9
                q_out = [1; 0; 0; 0];
            else
                q_out = q / n;
            end
        end
        
        function R = quat_to_rotation_matrix_matlab(q)
            q = q(:);
            q = KalmanCompute.quat_normalize_matlab(q);
            qw = q(1); qx = q(2); qy = q(3); qz = q(4);
            R = [1-2*(qy^2+qz^2), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw);
                 2*(qx*qy+qz*qw), 1-2*(qx^2+qz^2),   2*(qy*qz-qx*qw);
                 2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw),   1-2*(qx^2+qy^2)];
        end
        
        function euler = quat_to_euler_matlab(q)
            q = q(:);
            q = KalmanCompute.quat_normalize_matlab(q);
            qw = q(1); qx = q(2); qy = q(3); qz = q(4);
            
            % Roll (x-axis)
            sinr_cosp = 2 * (qw*qx + qy*qz);
            cosr_cosp = 1 - 2 * (qx^2 + qy^2);
            roll = atan2(sinr_cosp, cosr_cosp);
            
            % Pitch (y-axis)
            sinp = 2 * (qw*qy - qz*qx);
            if abs(sinp) >= 1
                pitch = sign(sinp) * pi/2;
            else
                pitch = asin(sinp);
            end
            
            % Yaw (z-axis)
            siny_cosp = 2 * (qw*qz + qx*qy);
            cosy_cosp = 1 - 2 * (qy^2 + qz^2);
            yaw = atan2(siny_cosp, cosy_cosp);
            
            euler = [roll; pitch; yaw] * 180/pi;
        end
        
        function q = quat_from_euler_matlab(euler)
            roll  = euler(1) * pi/180;
            pitch = euler(2) * pi/180;
            yaw   = euler(3) * pi/180;
            
            cy = cos(yaw/2);   sy = sin(yaw/2);
            cp = cos(pitch/2); sp = sin(pitch/2);
            cr = cos(roll/2);  sr = sin(roll/2);
            
            q = [cr*cp*cy + sr*sp*sy;  % w
                 sr*cp*cy - cr*sp*sy;  % x
                 cr*sp*cy + sr*cp*sy;  % y
                 cr*cp*sy - sr*sp*cy]; % z
            
            q = KalmanCompute.quat_normalize_matlab(q);
        end
        
        function q = quat_from_small_angle_matlab(theta)
            theta_sq = sum(theta.^2);
            if theta_sq < 1e-18
                q = [1; 0.5*theta];
            else
                angle = sqrt(theta_sq);
                half_angle = angle / 2;
                s = sin(half_angle) / angle;
                q = [cos(half_angle); theta * s];
            end
            q = KalmanCompute.quat_normalize_matlab(q);
        end
        
        function q_new = quat_integrate_matlab(q, omega, dt)
            w_dt = omega * dt;
            w_dt_norm = norm(w_dt);
            
            if w_dt_norm < 1e-15
                q_new = q;
                return;
            end
            
            half_angle = w_dt_norm / 2;
            if half_angle > 1e-6
                sin_half = sin(half_angle);
                cos_half = cos(half_angle);
                w_unit = w_dt / w_dt_norm;
                delta_q = [cos_half; w_unit * sin_half];
            else
                w_sq = w_dt_norm^2;
                delta_q = [1 - w_sq/8; w_dt * 0.5 * (1 - w_sq/24)];
            end
            
            q_new = KalmanCompute.quat_multiply_matlab(q, delta_q);
            q_new = KalmanCompute.quat_normalize_matlab(q_new);
        end
        
        function R = rot_from_euler_matlab(euler)
            roll  = euler(1) * pi/180;
            pitch = euler(2) * pi/180;
            yaw   = euler(3) * pi/180;
            
            cr = cos(roll);  sr = sin(roll);
            cp = cos(pitch); sp = sin(pitch);
            cy = cos(yaw);   sy = sin(yaw);
            
            R = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr;
                 sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr;
                 -sp,   cp*sr,            cp*cr];
        end
        
        function euler = rot_to_euler_matlab(R)
            sp = -R(3,1);
            if abs(sp) >= 1
                pitch = sign(sp) * pi/2;
                roll = 0;
                yaw = atan2(-R(1,2), R(2,2));
            else
                pitch = asin(sp);
                roll = atan2(R(3,2), R(3,3));
                yaw = atan2(R(2,1), R(1,1));
            end
            euler = [roll; pitch; yaw] * 180/pi;
        end
        
        function R_ortho = rot_orthonormalize_matlab(R)
            % Gram-Schmidt orthonormalization
            r1 = R(1,:)'; r1 = r1 / norm(r1);
            r2 = R(2,:)'; r2 = r2 - (r2'*r1)*r1; r2 = r2 / norm(r2);
            r3 = cross(r1, r2); r3 = r3 / norm(r3);
            R_ortho = [r1'; r2'; r3'];
        end
        
    end
end

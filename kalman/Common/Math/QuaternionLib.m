classdef QuaternionLib
    % クォータニオン演算ライブラリ
    % C++実装が利用可能な場合は自動的にMEXを使用
    
    properties (Constant)
        EPS = 1.0e-9;  % 小さい値の閾値
    end
    
    methods (Static)
        function use_mex = check_mex_available()
            % MEX実装の利用可能性をチェック
            persistent mex_available;
            if isempty(mex_available)
                mex_available = exist('mex_quaternion_lib', 'file') == 3;
            end
            use_mex = mex_available;
        end
        
        function q_out = multiply(q1, q2)
            % クォータニオン積
            % MEX利用可能時は自動的に使用
            % MEX 専用
            persistent use_mex;
            if isempty(use_mex)
                use_mex = QuaternionLib.check_mex_available();
            end

            if use_mex
                try
                    q_out = mex_quaternion_lib('multiply', q1(:), q2(:));
                    return;
                catch ME
                    error('QuaternionLib:multiply:mexFailed', 'MEX call failed: %s. MATLAB fallback removed. Build mex_quaternion_lib.', ME.message);
                end
            end
            error('QuaternionLib:multiply:mexMissing', 'mex_quaternion_lib not available. Build the MEX to use this method.');
        end
        
        function q_out = conjugate(q)
            % 共役 (MEX 専用)
            persistent use_mex;
            if isempty(use_mex)
                use_mex = QuaternionLib.check_mex_available();
            end

            if use_mex
                try
                    q_out = mex_quaternion_lib('conjugate', q(:));
                    return;
                catch ME
                    error('QuaternionLib:conjugate:mexFailed', 'MEX call failed: %s. MATLAB fallback removed. Build mex_quaternion_lib.', ME.message);
                end
            end
            error('QuaternionLib:conjugate:mexMissing', 'mex_quaternion_lib not available. Build the MEX to use this method.');
        end
        
        function q_out = normalize(q)
            % 正規化
            % MEX利用可能時は自動的に使用
            % MEX 専用
            persistent use_mex;
            if isempty(use_mex)
                use_mex = QuaternionLib.check_mex_available();
            end

            if use_mex
                try
                    q_out = mex_quaternion_lib('normalize', q(:));
                    return;
                catch ME
                    error('QuaternionLib:normalize:mexFailed', 'MEX call failed: %s. MATLAB fallback removed. Build mex_quaternion_lib.', ME.message);
                end
            end
            error('QuaternionLib:normalize:mexMissing', 'mex_quaternion_lib not available. Build the MEX to use this method.');
        end
        
        function q_out = inverse(q)
            % 逆元 (MEX 専用)
            persistent use_mex;
            if isempty(use_mex)
                use_mex = QuaternionLib.check_mex_available();
            end

            if use_mex
                try
                    q_out = mex_quaternion_lib('inverse', q(:));
                    return;
                catch ME
                    error('QuaternionLib:inverse:mexFailed', 'MEX call failed: %s. MATLAB fallback removed. Build mex_quaternion_lib.', ME.message);
                end
            end
            error('QuaternionLib:inverse:mexMissing', 'mex_quaternion_lib not available. Build the MEX to use this method.');
        end
        
        function R = to_rotation_matrix(q)
            % 回転行列へ変換
            % MEX利用可能時は自動的に使用
            % MEX 専用
            persistent use_mex;
            if isempty(use_mex)
                use_mex = QuaternionLib.check_mex_available();
            end

            if use_mex
                try
                    q_norm = mex_quaternion_lib('normalize', q(:));
                    R = mex_quaternion_lib('to_rotation_matrix', q_norm(:));
                    return;
                catch ME
                    error('QuaternionLib:to_rotation_matrix:mexFailed', 'MEX call failed: %s. MATLAB fallback removed. Build mex_quaternion_lib.', ME.message);
                end
            end
            error('QuaternionLib:to_rotation_matrix:mexMissing', 'mex_quaternion_lib not available. Build the MEX to use this method.');
        end
        
        function R = quat2rotm(q)
            % to_rotation_matrixのエイリアス（MATLAB互換）
            R = QuaternionLib.to_rotation_matrix(q);
        end
        
        function euler = to_euler(q, sequence)
            % オイラー角へ変換 (度)
            if nargin < 2
                sequence = 'ZYX';
            end
            % MEX 専用: シーケンスは現状 'ZYX' のみサポート
            if ~strcmp(sequence, 'ZYX')
                error('QuaternionLib:to_euler', 'Unsupported sequence: %s', sequence);
            end

            persistent use_mex;
            if isempty(use_mex)
                use_mex = QuaternionLib.check_mex_available();
            end

            if use_mex
                try
                    euler = mex_quaternion_lib('to_euler', q(:));
                    return;
                catch ME
                    error('QuaternionLib:to_euler:mexFailed', 'MEX call failed: %s. MATLAB fallback removed. Build mex_quaternion_lib.', ME.message);
                end
            end
            error('QuaternionLib:to_euler:mexMissing', 'mex_quaternion_lib not available. Build the MEX to use this method.');
        end
        
        function q = from_euler(euler, sequence)
            % オイラー角から変換
            if nargin < 2
                sequence = 'ZYX';
            end
            if ~strcmp(sequence, 'ZYX')
                error('QuaternionLib:from_euler', 'Unsupported sequence: %s', sequence);
            end

            persistent use_mex;
            if isempty(use_mex)
                use_mex = QuaternionLib.check_mex_available();
            end

            if use_mex
                try
                    q = mex_quaternion_lib('from_euler', euler(:));
                    return;
                catch ME
                    error('QuaternionLib:from_euler:mexFailed', 'MEX call failed: %s. MATLAB fallback removed. Build mex_quaternion_lib.', ME.message);
                end
            end
            error('QuaternionLib:from_euler:mexMissing', 'mex_quaternion_lib not available. Build the MEX to use this method.');
        end
        
        function q_new = integrate(q, omega, dt)
            % 積分
            % MEX利用可能時は自動的に使用
            % MEX 専用
            persistent use_mex;
            if isempty(use_mex)
                use_mex = QuaternionLib.check_mex_available();
            end

            if use_mex
                try
                    q_new = mex_quaternion_lib('integrate', q(:), omega(:), dt);
                    return;
                catch ME
                    error('QuaternionLib:integrate:mexFailed', 'MEX call failed: %s. MATLAB fallback removed. Build mex_quaternion_lib.', ME.message);
                end
            end
            error('QuaternionLib:integrate:mexMissing', 'mex_quaternion_lib not available. Build the MEX to use this method.');
        end
        
        function q = small_angle_quat(theta)
            % 小角度回転ベクトルから変換 (MEX 専用)
            persistent use_mex;
            if isempty(use_mex)
                use_mex = QuaternionLib.check_mex_available();
            end

            if use_mex
                try
                    q = mex_quaternion_lib('small_angle_quat', theta(:));
                    return;
                catch ME
                    error('QuaternionLib:small_angle_quat:mexFailed', 'MEX call failed: %s. MATLAB fallback removed. Build mex_quaternion_lib.', ME.message);
                end
            end
            error('QuaternionLib:small_angle_quat:mexMissing', 'mex_quaternion_lib not available. Build the MEX to use this method.');
        end
        
        function d = distance(q1, q2)
            % 角度距離計算 (MEX 専用)
            persistent use_mex;
            if isempty(use_mex)
                use_mex = QuaternionLib.check_mex_available();
            end

            if use_mex
                try
                    d = mex_quaternion_lib('distance', q1(:), q2(:));
                    return;
                catch ME
                    error('QuaternionLib:distance:mexFailed', 'MEX call failed: %s. MATLAB fallback removed. Build mex_quaternion_lib.', ME.message);
                end
            end
            error('QuaternionLib:distance:mexMissing', 'mex_quaternion_lib not available. Build the MEX to use this method.');
        end
        
        function q = slerp(q1, q2, t)
            % 球面線形補間 (MEX 専用)
            persistent use_mex;
            if isempty(use_mex)
                use_mex = QuaternionLib.check_mex_available();
            end

            if use_mex
                try
                    q = mex_quaternion_lib('slerp', q1(:), q2(:), t);
                    return;
                catch ME
                    error('QuaternionLib:slerp:mexFailed', 'MEX call failed: %s. MATLAB fallback removed. Build mex_quaternion_lib.', ME.message);
                end
            end
            error('QuaternionLib:slerp:mexMissing', 'mex_quaternion_lib not available. Build the MEX to use this method.');
        end
        
        function q = from_two_vectors(v1, v2)
            % 2ベクトル間の回転生成 (MEX 専用)
            persistent use_mex;
            if isempty(use_mex)
                use_mex = QuaternionLib.check_mex_available();
            end

            if use_mex
                try
                    q = mex_quaternion_lib('from_two_vectors', v1(:), v2(:));
                    return;
                catch ME
                    error('QuaternionLib:from_two_vectors:mexFailed', 'MEX call failed: %s. MATLAB fallback removed. Build mex_quaternion_lib.', ME.message);
                end
            end
            error('QuaternionLib:from_two_vectors:mexMissing', 'mex_quaternion_lib not available. Build the MEX to use this method.');
        end
    end
end

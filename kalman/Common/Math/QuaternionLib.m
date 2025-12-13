classdef QuaternionLib
    % QuaternionLib - C++版QuaternionLibへのラッパー
    % mex_quaternion_lib.mexw64を使用してQuaternionLib互換インターフェースを提供
    
    methods (Static)
        function q = from_euler(euler_deg)
            % euler_deg: [roll, pitch, yaw] in degrees (3x1)
            % q: [qw, qx, qy, qz] (4x1)
            if size(euler_deg, 1) ~= 3 || size(euler_deg, 2) ~= 1
                error('euler_deg must be 3x1');
            end
            q = mex_quaternion_lib('from_euler', euler_deg(1), euler_deg(2), euler_deg(3));
        end
        
        function euler_deg = to_euler(q)
            % q: [qw, qx, qy, qz] (4x1)
            % euler_deg: [roll, pitch, yaw] in degrees (3x1)
            if size(q, 1) ~= 4 || size(q, 2) ~= 1
                error('q must be 4x1');
            end
            euler_deg = mex_quaternion_lib('to_euler', q);
        end
        
        function R = to_rotation_matrix(q)
            % q: [qw, qx, qy, qz] (4x1)
            % R: 3x3 rotation matrix
            if size(q, 1) ~= 4 || size(q, 2) ~= 1
                error('q must be 4x1');
            end
            R = mex_quaternion_lib('to_rotation_matrix', q);
        end
        
        function q = normalize(q)
            % q: [qw, qx, qy, qz] (4x1)
            if size(q, 1) ~= 4 || size(q, 2) ~= 1
                error('q must be 4x1');
            end
            q = mex_quaternion_lib('normalize', q);
        end
        
        function q = multiply(q1, q2)
            % q1, q2: [qw, qx, qy, qz] (4x1)
            if size(q1, 1) ~= 4 || size(q2, 1) ~= 4
                error('q1 and q2 must be 4x1');
            end
            q = mex_quaternion_lib('multiply', q1, q2);
        end
    end
end

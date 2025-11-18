classdef RotationLib
    % ROTATIONLIB 回転行列演算の静的メソッド集
    % C++移行を想定した設計
    
    properties (Constant)
        EPS = 1.0e-9;
    end
    
    methods (Static)
        %% 変換
        function R = from_euler(euler, sequence)
            % オイラー角から回転行列への変換
            % euler: [roll; pitch; yaw] (度)
            % sequence: 'ZYX' (デフォルト), 'XYZ' 等
            if nargin < 2
                sequence = 'ZYX';
            end
            
            e = deg2rad(euler(:));
            
            switch upper(sequence)
                case 'ZYX'
                    % R = Rz(yaw) * Ry(pitch) * Rx(roll)
                    Rx = RotationLib.rotation_x(e(1));
                    Ry = RotationLib.rotation_y(e(2));
                    Rz = RotationLib.rotation_z(e(3));
                    R = Rz * Ry * Rx;
                case 'XYZ'
                    Rx = RotationLib.rotation_x(e(1));
                    Ry = RotationLib.rotation_y(e(2));
                    Rz = RotationLib.rotation_z(e(3));
                    R = Rx * Ry * Rz;
                otherwise
                    error('RotationLib:from_euler', 'Unsupported sequence: %s', sequence);
            end
        end
        
        function euler = to_euler(R, sequence)
            % 回転行列からオイラー角への変換 (度)
            if nargin < 2
                sequence = 'ZYX';
            end
            
            switch upper(sequence)
                case 'ZYX'
                    % pitch
                    sinp = -R(3,1);
                    if abs(sinp) >= 1
                        pitch = sign(sinp) * pi/2;
                        % gimbal lock
                        roll = 0;
                        yaw = atan2(-R(1,2), R(2,2));
                    else
                        pitch = asin(sinp);
                        roll = atan2(R(3,2), R(3,3));
                        yaw = atan2(R(2,1), R(1,1));
                    end
                otherwise
                    error('RotationLib:to_euler', 'Unsupported sequence: %s', sequence);
            end
            
            euler = rad2deg([roll; pitch; yaw]);
        end
        
        %% 基本回転行列
        function Rx = rotation_x(angle)
            % X軸周りの回転行列 (rad)
            c = cos(angle);
            s = sin(angle);
            Rx = [1, 0, 0;
                  0, c, -s;
                  0, s, c];
        end
        
        function Ry = rotation_y(angle)
            % Y軸周りの回転行列 (rad)
            c = cos(angle);
            s = sin(angle);
            Ry = [c, 0, s;
                  0, 1, 0;
                  -s, 0, c];
        end
        
        function Rz = rotation_z(angle)
            % Z軸周りの回転行列 (rad)
            c = cos(angle);
            s = sin(angle);
            Rz = [c, -s, 0;
                  s, c, 0;
                  0, 0, 1];
        end
        
        %% ユーティリティ
        function S = skew_symmetric(v)
            % ベクトルから歪対称行列への変換
            % S = [v]× (cross product matrix)
            v = v(:);
            S = [0,    -v(3),  v(2);
                 v(3),  0,    -v(1);
                -v(2),  v(1),  0];
        end
        
        function v = from_skew(S)
            % 歪対称行列からベクトルへの変換
            v = [S(3,2); S(1,3); S(2,1)];
        end
        
        function R_ortho = orthonormalize(R)
            % 回転行列の直交正規化 (Gram-Schmidt)
            % 数値誤差で直交性が失われた回転行列を修正
            
            % 第1列を正規化
            c1 = R(:,1);
            c1 = c1 / norm(c1);
            
            % 第2列を直交化・正規化
            c2 = R(:,2);
            c2 = c2 - (c1' * c2) * c1;
            c2 = c2 / norm(c2);
            
            % 第3列は外積で計算（自動的に直交）
            c3 = cross(c1, c2);
            
            R_ortho = [c1, c2, c3];
        end
        
        function R_dot = derivative(R, omega)
            % 回転行列の時間微分
            % R_dot = R * [omega]×
            omega = omega(:);
            S = RotationLib.skew_symmetric(omega);
            R_dot = R * S;
        end
        
        function is_valid = is_rotation_matrix(R, tol)
            % 回転行列の妥当性チェック
            if nargin < 2
                tol = 1e-6;
            end
            
            % サイズチェック
            if ~isequal(size(R), [3, 3])
                is_valid = false;
                return;
            end
            
            % 直交性チェック: R' * R = I
            I = R' * R;
            if norm(I - eye(3), 'fro') > tol
                is_valid = false;
                return;
            end
            
            % 行列式チェック: det(R) = 1
            if abs(det(R) - 1) > tol
                is_valid = false;
                return;
            end
            
            is_valid = true;
        end
    end
end

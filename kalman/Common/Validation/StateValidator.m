classdef StateValidator
    % STATEVALIDATOR 状態検証の静的メソッド集
    % C++移行を想定した設計
    
    methods (Static)
        %% 速度検証
        function [v_clipped, clipped] = clip_velocity(v, max_vel)
            % 速度ベクトルのクリッピング
            % 入力:
            %   v: 速度ベクトル [3x1]
            %   max_vel: 最大速度 (m/s)
            % 出力:
            %   v_clipped: クリップされた速度
            %   clipped: クリップされたか
            
            v_norm = norm(v);
            if v_norm > max_vel
                v_clipped = v * (max_vel / v_norm);
                clipped = true;
            else
                v_clipped = v;
                clipped = false;
            end
        end
        
        function [a_limited, limited] = limit_acceleration(a, max_accel)
            % 加速度ベクトルの制限
            % 入力:
            %   a: 加速度ベクトル [3x1]
            %   max_accel: 最大加速度 (m/s^2)
            % 出力:
            %   a_limited: 制限された加速度
            %   limited: 制限されたか
            
            a_norm = norm(a);
            if a_norm > max_accel
                a_limited = a * (max_accel / a_norm);
                limited = true;
            else
                a_limited = a;
                limited = false;
            end
        end
        
        function is_valid = check_velocity_bounds(v, max_vel)
            % 速度が範囲内かチェック
            is_valid = norm(v) <= max_vel;
        end
        
        %% 姿勢検証
        function [q_valid, normalized] = validate_quaternion(q)
            % クォータニオンの検証と正規化
            % 入力:
            %   q: クォータニオン [4x1]
            % 出力:
            %   q_valid: 検証済みクォータニオン
            %   normalized: 正規化されたか
            
            q = q(:);
            
            % 有限値チェック
            if any(~isfinite(q))
                q_valid = [1; 0; 0; 0];
                normalized = true;
                return;
            end
            
            % ノルムチェック
            q_norm = norm(q);
            if abs(q_norm - 1.0) > 1e-6
                q_valid = q / q_norm;
                normalized = true;
            else
                q_valid = q;
                normalized = false;
            end
        end
        
        function is_valid = is_valid_quaternion(q, tol)
            % クォータニオンの妥当性チェック
            if nargin < 2
                tol = 1e-6;
            end
            
            q = q(:);
            
            % サイズチェック
            if numel(q) ~= 4
                is_valid = false;
                return;
            end
            
            % 有限値チェック
            if any(~isfinite(q))
                is_valid = false;
                return;
            end
            
            % ノルムチェック
            q_norm = norm(q);
            is_valid = abs(q_norm - 1.0) < tol;
        end
        
        %% 位置検証
        function is_valid = check_position_bounds(p, bounds)
            % 位置が範囲内かチェック
            % bounds: struct with .min [3x1] and .max [3x1]
            
            if isempty(bounds)
                is_valid = true;
                return;
            end
            
            is_valid = all(p >= bounds.min) && all(p <= bounds.max);
        end
        
        function [p_clipped, clipped] = clip_position(p, bounds)
            % 位置のクリッピング
            p_clipped = p;
            clipped = false;
            
            if isempty(bounds)
                return;
            end
            
            for i = 1:3
                if p(i) < bounds.min(i)
                    p_clipped(i) = bounds.min(i);
                    clipped = true;
                elseif p(i) > bounds.max(i)
                    p_clipped(i) = bounds.max(i);
                    clipped = true;
                end
            end
        end
        
        %% 状態全体の検証
        function is_valid = check_state_bounds(state, bounds)
            % 状態全体の範囲チェック
            % state: struct with .p, .v, .q, .ba, .bg
            % bounds: struct with corresponding bounds
            
            is_valid = true;
            
            if isfield(bounds, 'position')
                is_valid = is_valid && StateValidator.check_position_bounds(state.p, bounds.position);
            end
            
            if isfield(bounds, 'velocity')
                is_valid = is_valid && StateValidator.check_velocity_bounds(state.v, bounds.velocity);
            end
            
            if isfield(bounds, 'quaternion')
                is_valid = is_valid && StateValidator.is_valid_quaternion(state.q);
            end
        end
        
        %% バイアス検証
        function [bias_clipped, clipped] = clip_bias(bias, max_bias)
            % バイアスのクリッピング
            bias_norm = norm(bias);
            if bias_norm > max_bias
                bias_clipped = bias * (max_bias / bias_norm);
                clipped = true;
            else
                bias_clipped = bias;
                clipped = false;
            end
        end
        
        function is_valid = check_bias_bounds(bias, max_bias)
            % バイアスが範囲内かチェック
            is_valid = norm(bias) <= max_bias;
        end
        
        %% 数値検証
        function is_valid = check_finite(x)
            % 有限値チェック
            is_valid = all(isfinite(x(:)));
        end
        
        function [x_safe, sanitized] = sanitize_vector(x, default_value)
            % ベクトルの健全性確保
            if nargin < 2
                default_value = 0;
            end
            
            x_safe = x;
            sanitized = false;
            
            if any(~isfinite(x))
                x_safe = ones(size(x)) * default_value;
                sanitized = true;
            end
        end
    end
end

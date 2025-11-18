classdef ESKFErrorInjection
    % ESKFERRORINJECTION ESKF誤差状態注入関数集
    % C++移行を想定した設計
    
    methods (Static)
        function nominal = inject_error_state(nominal, dx)
            % 誤差状態をノミナル状態に注入
            % 入力:
            %   nominal: struct with .p, .v, .q, .ba, .bg
            %   dx: 誤差状態 [15x1]
            %       dx(1:3)   - 位置誤差
            %       dx(4:6)   - 速度誤差
            %       dx(7:9)   - 姿勢誤差 (小角度)
            %       dx(10:12) - 加速度バイアス誤差
            %       dx(13:15) - 角速度バイアス誤差
            % 出力:
            %   nominal: 更新された状態
            
            % 位置更新
            nominal.p = nominal.p + dx(1:3);
            
            % 速度更新
            nominal.v = nominal.v + dx(4:6);
            
            % 姿勢更新（クォータニオン積）
            dtheta = dx(7:9);
            dq = QuaternionLib.small_angle_quat(dtheta);
            nominal.q = QuaternionLib.multiply(nominal.q, dq);
            nominal.q = QuaternionLib.normalize(nominal.q);
            
            % バイアス更新
            nominal.ba = nominal.ba + dx(10:12);
            nominal.bg = nominal.bg + dx(13:15);
        end
        
        function nominal = inject_with_constraints(nominal, dx, config)
            % 制約付き誤差状態注入
            % 入力:
            %   nominal: ノミナル状態
            %   dx: 誤差状態 [15x1]
            %   config: 制約設定 struct
            % 出力:
            %   nominal: 更新された状態
            
            % dx の制限
            if isfield(config, 'max_dx_norm') && ~isempty(config.max_dx_norm)
                dx_norm = norm(dx);
                if dx_norm > config.max_dx_norm
                    dx = dx * (config.max_dx_norm / dx_norm);
                end
            end
            
            % 基本的な注入
            nominal = ESKFErrorInjection.inject_error_state(nominal, dx);
            
            % 速度制限
            if isfield(config, 'max_velocity') && ~isempty(config.max_velocity)
                [nominal.v, ~] = StateValidator.clip_velocity(nominal.v, config.max_velocity);
            end
            
            % バイアス制限
            if isfield(config, 'max_accel_bias') && ~isempty(config.max_accel_bias)
                [nominal.ba, ~] = StateValidator.clip_bias(nominal.ba, config.max_accel_bias);
            end
            
            if isfield(config, 'max_gyro_bias') && ~isempty(config.max_gyro_bias)
                [nominal.bg, ~] = StateValidator.clip_bias(nominal.bg, config.max_gyro_bias);
            end
        end
        
        function [nominal, dx_used] = inject_with_validation(nominal, dx, config)
            % 検証付き誤差状態注入
            % 大きすぎる修正を段階的に適用
            
            dx_used = dx;
            
            % 各成分を個別にチェック
            % 位置修正の制限
            if isfield(config, 'max_position_correction')
                pos_correction_norm = norm(dx(1:3));
                if pos_correction_norm > config.max_position_correction
                    scale = config.max_position_correction / pos_correction_norm;
                    dx_used(1:3) = dx(1:3) * scale;
                end
            end
            
            % 速度修正の制限
            if isfield(config, 'max_velocity_correction')
                vel_correction_norm = norm(dx(4:6));
                if vel_correction_norm > config.max_velocity_correction
                    scale = config.max_velocity_correction / vel_correction_norm;
                    dx_used(4:6) = dx(4:6) * scale;
                end
            end
            
            % 姿勢修正の制限（角度）
            if isfield(config, 'max_attitude_correction')
                att_correction_norm = norm(dx(7:9));
                if att_correction_norm > deg2rad(config.max_attitude_correction)
                    scale = deg2rad(config.max_attitude_correction) / att_correction_norm;
                    dx_used(7:9) = dx(7:9) * scale;
                end
            end
            
            % 制限付き注入
            nominal = ESKFErrorInjection.inject_with_constraints(nominal, dx_used, config);
        end
        
        function dx_reset = compute_error_state_reset()
            % 誤差状態のリセット（注入後はゼロに）
            dx_reset = zeros(15, 1);
        end
    end
end

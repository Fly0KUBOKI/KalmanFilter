function [z_pos, z_vel, z_acc] = compute_vertical_motion_unified(t_point, params)
    % COMPUTE_VERTICAL_MOTION_UNIFIED 垂直運動の位置・速度・加速度を統一的に計算
    % 真値データとセンサーデータで同じ式を使用して整合性を保証
    %
    % 上昇=正の座標系 (Z-up convention)
    % z(t) = +A*sin(ωt)*s(t)        [m]     (z正=上昇、z負=下降)
    % vz(t) = +A*ω*cos(ωt)*s(t) + A*sin(ωt)*ds/dt     [m/s]
    % az(t) = -A*ω²*sin(ωt)*s(t) + [ソフトスタート補正項]   [m/s²]
    
    % デフォルト値（垂直運動無効）
    z_pos = 0.0;
    z_vel = 0.0;
    z_acc = 0.0;
    
    if ~isfield(params, 'motion') || ~isfield(params.motion, 'vertical')
        return;
    end
    
    vert_param = params.motion.vertical;
    if ~isfield(vert_param, 'amplitude') || ~isfield(vert_param, 'period')
        return;
    end
    
    amplitude = vert_param.amplitude;
    period = vert_param.period;
    soft_start_time = 5; % デフォルト
    if isfield(vert_param, 'soft_start_time')
        soft_start_time = vert_param.soft_start_time;
    end
    
    % ソフトスタート関数とその導関数
    if t_point < soft_start_time
        phase = t_point / soft_start_time;
        scale = phase^2 * (3 - 2*phase);              % スムースステップ
        d_scale = (6*phase*(1-phase)) / soft_start_time;  % 1階導関数
        d2_scale = (6 - 24*phase) / (soft_start_time^2);  % 2階導関数
    else
        scale = 1.0;
        d_scale = 0.0;
        d2_scale = 0.0;
    end
    
    % 位相時間（運動開始からの経過時間）
    t_motion = t_point - soft_start_time;
    if t_motion < 0
        t_motion = 0;
    end
    
    omega = 2 * pi / period;
    sin_term = sin(omega * t_motion);
    cos_term = cos(omega * t_motion);
    
    % 位置: z = +A*sin(ωt)*s(t)  [Z-up: 上昇=正]
    z_pos = +amplitude * sin_term * scale;
    
    % 速度: vz = +A*ω*cos(ωt)*s(t) + A*sin(ωt)*ds/dt
    z_vel = +amplitude * omega * cos_term * scale + amplitude * sin_term * d_scale;
    
    % 加速度: az = -A*ω²*sin(ωt)*s(t) + 補正項
    % 完全な2階微分:
    % az = d²z/dt² = -A*ω²*sin(ωt)*s + A*ω*cos(ωt)*ds/dt 
    %              + A*ω*cos(ωt)*ds/dt + A*sin(ωt)*d²s/dt²
    z_acc = -amplitude * omega^2 * sin_term * scale ...
          + amplitude * omega * cos_term * d_scale ...
          + amplitude * omega * cos_term * d_scale ...
          + amplitude * sin_term * d2_scale;
end

function [pos_world, vel_world, attitude] = generate_random_walk(params, t, N)
    % GENERATE_RANDOM_WALK ランダムウォーク運動の位置、速度、姿勢を生成
    % ロール／ピッチはソフトスタート付き単振動で生成します。

    % パラメータ取得
    vel_std = 0;
    ang_std = 0;
    if isfield(params.motion, 'random_walk')
        if isfield(params.motion.random_walk, 'velocity_std')
            vel_std = params.motion.random_walk.velocity_std;
        end
        if isfield(params.motion.random_walk, 'angular_std')
            ang_std = params.motion.random_walk.angular_std;
        end
    end

    % 静止時間
    static_time = 0.5;
    if isfield(params, 'static_time')
        static_time = params.static_time;
    end

    dt = params.dt;

    % 初期化
    pos_world = zeros(N,3);
    vel_world = zeros(N,3);
    attitude = zeros(N,3);

    % 初期条件
    pos_world(1,:) = [0, 0, 0];
    v_forward = params.initial.velocity(1);
    yaw = 0;

    % ランダムシード設定（再現性）
    rng(42);

    % --- 単振動でロール・ピッチを生成（ソフトスタート対応） ---
    if isfield(params.motion, 'oscillation')
        roll_amp = params.motion.oscillation.roll_amplitude;
        roll_period = params.motion.oscillation.roll_period;
        pitch_amp = params.motion.oscillation.pitch_amplitude;
        pitch_period = params.motion.oscillation.pitch_period;
        soft_start_time = params.motion.oscillation.soft_start_time;
    else
        roll_amp = deg2rad(15);
        roll_period = 10;
        pitch_amp = deg2rad(10);
        pitch_period = 8;
        soft_start_time = 3;
    end

    roll_seq = generate_sinusoidal_oscillation_with_soft_start(t, roll_amp, roll_period, static_time, soft_start_time);
    pitch_seq = generate_sinusoidal_oscillation_with_soft_start(t, pitch_amp, pitch_period, static_time, soft_start_time);

    % 高度の滑らかなランダムウォーク（簡易）
    if isfield(params.motion.random_walk, 'altitude_range_m')
        alt_range = params.motion.random_walk.altitude_range_m;
    else
        alt_range = [0, 0];
    end
    if isfield(params.motion.random_walk, 'altitude_tau')
        alt_tau = params.motion.random_walk.altitude_tau;
    else
        alt_tau = 20.0;
    end
    seed_base = 100;
    alt_seq = smooth_bounded_random_walk(t, alt_range(1), alt_range(2), alt_tau, seed_base + 3, static_time, pos_world(1,3));
    vel_z = zeros(N,1);
    vel_z(2:end) = diff(alt_seq) / dt;

    %% ランダムウォーク生成ループ（主移動）
    for i = 2:N
        if t(i) < static_time
            % 静止期間：位置・速度・姿勢を維持
            pos_world(i,:) = pos_world(1,:);
            vel_world(i,:) = [0, 0, 0];
            attitude(i,:) = [0, 0, 0];
        else
            % 前方速度はランダム変動（従来ロジックを維持）
            dv = randn() * vel_std * sqrt(dt);
            v_forward = max(0, v_forward + dv);
            yaw = yaw + randn() * ang_std * dt;

            vel_world(i,1) = v_forward * cos(yaw);
            vel_world(i,2) = v_forward * sin(yaw);
            % 垂直速度は alt_seq から導出
            vel_world(i,3) = vel_z(i);

            % 位置更新（x,east; y,north; z,altitude）
            pos_world(i,:) = pos_world(i-1,:) + vel_world(i,:) * dt;

            % 姿勢：ロール・ピッチは単振動で生成
            attitude(i,:) = [ roll_seq(i), pitch_seq(i), yaw ];
            % 高度は pos_world(:,3) に alt_seq を反映
            pos_world(i,3) = alt_seq(i);
        end
        if mod(i, 10000) == 0
            fprintf('Motion step %d / %d\n', i, N);
        end
    end

end

function seq = generate_sinusoidal_oscillation_with_soft_start(t, amplitude, period, static_time, soft_start_time)
    omega = 2 * pi / period;
    seq = zeros(size(t));
    for i = 1:length(t)
        if t(i) < static_time
            seq(i) = 0;
        elseif t(i) < static_time + soft_start_time
            t_soft = t(i) - static_time;
            amplitude_scale = 0.5 * (1 - cos(pi * t_soft / soft_start_time));
            t_phase = t(i) - static_time;
            seq(i) = amplitude * amplitude_scale * sin(omega * t_phase);
        else
            t_phase = t(i) - static_time;
            seq(i) = amplitude * sin(omega * t_phase);
        end
    end
end

function seq = smooth_bounded_random_walk(t, vmin, vmax, tau, seed, static_time, init_value)
    if nargin < 7 || isempty(init_value)
        init_value = 0;
    end
    if nargin < 6 || isempty(static_time)
        static_time = 0;
    end
    if nargin < 5 || isempty(seed)
        seed = sum(100*clock);
    end
    rng(seed);
    N = numel(t);
    seq = zeros(N,1);
    T = t(end);
    if static_time >= T
        seq(:) = init_value; return;
    end
    n_ctrl = max(2, ceil((T-static_time)/max(tau, eps)) + 1);
    ctrl_t = linspace(static_time, T, n_ctrl);
    ctrl_v = zeros(1,n_ctrl);
    ctrl_v(1) = init_value;
    frac_ctrl = 0.05;
    step_max_ctrl = frac_ctrl * (vmax - vmin);
    step_max_ctrl = max(step_max_ctrl, 1e-10);
    for j=2:n_ctrl
        delta = (2*rand()-1) * step_max_ctrl;
        ctrl_v(j) = min(max(ctrl_v(j-1) + delta, vmin), vmax);
    end
    seq(t < static_time) = init_value;
    idx = find(t >= static_time);
    if ~isempty(idx)
        seq(idx) = interp1(ctrl_t, ctrl_v, t(idx), 'pchip');
    end
end
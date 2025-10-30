function [p, v, q, ba, bg] = integrate_nominal(p, v, q, ba, bg, a_meas, w_meas, dt, g)
% INTEGRATE_NOMINAL  ノミナル状態の数値積分（first-order）
% a_meas, w_meas は生センサ（機体座標系）

persistent count accel_sum dt_sum
if isempty(count)
    count = 0;
    accel_sum = zeros(3,1);
    dt_sum = 0;
end

% バイアス補正
a = a_meas - ba;
w = w_meas - bg;

%fprintf('ba: [%.2f, %.2f, %.2f], a_meas: [%.2f, %.2f, %.2f]\n', ba(1), ba(2), ba(3), a_meas(1), a_meas(2), a_meas(3));

% update quaternion
% delta_q = quat_lib('small_angle_quat', w * dt);
% q = quat_lib('quatmultiply', q, delta_q);
% q = quat_lib('quatnormalize', q);

% rotation matrix body <- world
Rb = quat_lib('quat_to_rotm', q);

% 世界座標系での加速度
a_world = Rb * a;

% 加速度を積算
count = count + 1;
accel_sum = accel_sum + a_world * dt;
dt_sum = dt_sum + dt;

% 4回に1回、平均加速度で速度・位置を更新
if count >= 4
    a_avg = accel_sum / dt_sum;
    %fprintf('a_avg: [%.2f, %.2f, %.2f]\n', a_avg(1), a_avg(2), a_avg(3));
    
    % integrate (台形則で精度向上)
    v_prev = v;
    v = v + (a_avg - g) * dt_sum;
    % update position using average velocity
    p = p + (v_prev + v) * 0.5 * dt_sum;
    
    % リセット
    count = 0;
    accel_sum = zeros(3,1);
    dt_sum = 0;
end

% fprintf('p: [%.2f, %.2f, %.2f], v: [%.2f, %.2f, %.2f]\n', p(1), p(2), p(3), v(1), v(2), v(3));
end
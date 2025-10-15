function [p, v, q, ba, bg] = integrate_nominal(p, v, q, ba, bg, a_meas, w_meas, dt, g)
% INTEGRATE_NOMINAL  ノミナル状態の数値積分（first-order）
% a_meas, w_meas は生センサ（機体座標系）

% バイアス補正
a = a_meas - ba;
w = w_meas - bg;

% fprintf('bg: [%.2f, %.2f, %.2f]\n', bg(1), bg(2), bg(3));

% update quaternion
delta_q = quat_lib('small_angle_quat', w * dt);
q = quat_lib('quatmultiply', q, delta_q);
q = quat_lib('quatnormalize', q);

% rotation matrix body <- world
Rb = quat_lib('quat_to_rotm', q);

% 世界座標系での加速度
a_world = Rb * a;
% fprintf('a_world: [%.2f, %.2f, %.2f]\n', a_world(1), a_world(2), a_world(3));

% integrate
v = v + (a_world - g) * dt;
% update position using new velocity
p = p + v * dt;
% fprintf('p: [%.2f, %.2f, %.2f], v: [%.2f, %.2f, %.2f]\n', p(1), p(2), p(3), v(1), v(2), v(3));
end
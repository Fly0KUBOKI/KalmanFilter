function [p, v, q, ba, bg] = integrate_nominal(p, v, q, ba, bg, a_meas, w_meas, dt, g)
% INTEGRATE_NOMINAL  ノミナル状態の数値積分（first-order）
% a_meas, w_meas は生センサ（機体座標系）

% バイアス補正
a = a_meas - ba;
w = w_meas - bg;

% update quaternion
delta_q = quat_lib('small_angle_quat', w * dt);
q = quat_lib('quatmultiply', q, delta_q);
q = quat_lib('quatnormalize', q);

% rotation matrix body <- world
Rb = quat_lib('quat_to_rotm', q);

% 世界座標系での加速度
a_world = Rb * a;
fprintf('a_world: [%f, %f, %f]\n', a_world(1), a_world(2), a_world(3));

% integrate
v = v + (a_world - g) * dt;
fprintf('v: [%f, %f, %f]\n', v(1), v(2), v(3));
p = p + v * dt;
end
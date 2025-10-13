function F = compute_jacobian(q, a_meas, ba, dt)
% COMPUTE_JACOBIAN  15x15 の誤差伝播ヤコビアン F_δ を作成する
% q: クォータニオン（world->body），a_meas: 機体座標系加速度，ba: 加速度バイアス
% 使う式はプロンプトの簡易形式に従う。

% rotation matrix
R = quat_lib('quat_to_rotm', q);

% a_nom: 世界座標系の加速度（b-a補正適用）
a_nom = R * (a_meas - ba);

I3 = eye(3);

% skew of a_nom
S_a = quat_lib('skew', a_nom);

% Build block matrix
F = eye(15);
% δp row: p <- p + v*dt => dp/dv = I*dt
F(1:3,4:6) = I3 * dt;

% δv row: v <- v + a_nom*dt, linearized w.r.t δθ and δb_a
F(4:6,7:9) = -S_a * dt;        % effect of attitude error on accel (approx)
F(4:6,10:12) = -R * dt;        % effect of accel bias error

% δθ row: q <- q ⊗ exp(ω*dt/2) => attitude error propagation approx: I - [ω]×*dt
% For simplicity use identity minus small gyro cross term (approximate ω=0)
% If needed, compute from gyro measurement but we keep simple form
% no cross-coupling to p/v
F(7:9,7:9) = eye(3); % placeholder for small-angle propagation
F(7:9,13:15) = -eye(3) * dt;  % bias_g affects theta over time

% biases: random walk
F(10:12,10:12) = eye(3);
F(13:15,13:15) = eye(3);
end
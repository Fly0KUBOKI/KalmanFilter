function P_next = covariance_prediction_optimized(P, q, a_meas, ba, w_meas, bg, Q, dt)
% COVARIANCE_PREDICTION_OPTIMIZED  最適化された共分散予測

% 補正済みIMU測定値
dvx = (a_meas(1) - ba(1)) * dt;
dvy = (a_meas(2) - ba(2)) * dt;
dvz = (a_meas(3) - ba(3)) * dt;
dax = (w_meas(1) - bg(1)) * dt;
day = (w_meas(2) - bg(2)) * dt;
daz = (w_meas(3) - bg(3)) * dt;

% クォータニオン成分
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

% バイアス
dax_b = 0;
day_b = 0;
daz_b = 0;

% 中間変数
SF = zeros(24, 1);
SF(1) = dvz - dax_b*q1 + day_b*q0 - daz_b*q2;
SF(2) = dvx + dax_b*q2 + day_b*q3 - daz_b*q0;
SF(3) = dvy - dax_b*q3 + day_b*q2 + daz_b*q1;
SF(4) = dax*0.5;
SF(5) = day*0.5;
SF(6) = daz*0.5;
SF(7) = dax_b*0.5;
SF(8) = day_b*0.5;
SF(9) = daz_b*0.5;

% 回転行列要素
SF(10) = q0*q0;
SF(11) = q1*q1;
SF(12) = q2*q2;
SF(13) = q3*q3;
SF(14) = q0*q1*2.0;
SF(15) = q0*q2*2.0;
SF(16) = q0*q3*2.0;
SF(17) = q1*q2*2.0;
SF(18) = q1*q3*2.0;
SF(19) = q2*q3*2.0;

% 正規化項
SF(20) = SF(10) - SF(11) + SF(12) - SF(13);
SF(21) = SF(10) + SF(11) - SF(12) - SF(13);
SF(22) = SF(10) - SF(11) - SF(12) + SF(13);

% 状態遷移ヤコビアン F
F = eye(15);

% 位置-速度カップリング
F(1:3, 4:6) = eye(3) * dt;

% 速度-姿勢カップリング
R_current = quat_to_rotm_internal(q);
skew_a = [0, -(a_meas(3)-ba(3)), (a_meas(2)-ba(2));
          (a_meas(3)-ba(3)), 0, -(a_meas(1)-ba(1));
          -(a_meas(2)-ba(2)), (a_meas(1)-ba(1)), 0];
F(4:6, 7:9) = -R_current * skew_a * dt;

% 速度-加速度バイアスカップリング
F(4:6, 10:12) = -R_current * dt;

% 姿勢-角速度バイアスカップリング
F(7:9, 13:15) = -eye(3) * dt;

% プロセスノイズの離散化
G = zeros(15, 12);
G(4:6, 1:3) = R_current * dt;
G(7:9, 4:6) = eye(3) * dt;
G(10:12, 7:9) = eye(3) * dt;
G(13:15, 10:12) = eye(3) * dt;

Q_discrete = Q + eye(15) * 1e-12;

% 共分散予測: P_next = F * P * F' + Q_discrete
P_next = F * P * F' + Q_discrete;

% 対称性の強制
P_next = 0.5 * (P_next + P_next');

% 正定値性の保証
min_var = 1e-9;
for i = 1:15
    if P_next(i,i) < min_var
        P_next(i,i) = min_var;
    end
end

end

function R = quat_to_rotm_internal(q)
q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
R = [1-2*(q2^2+q3^2), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3), 1-2*(q1^2+q3^2), 2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1^2+q2^2)];
end
